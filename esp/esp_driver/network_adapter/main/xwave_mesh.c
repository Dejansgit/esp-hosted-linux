/* xWave: ESP-WIFI-MESH mode of the hosted firmware (docs/WiFi-Plan.md 7.1).
 *
 * The mesh network interfaces follow ESP-IDF's examples/mesh/ip_internal_network
 * (mesh_netif.c), which is where the H743 sensors' mesh.bin comes from, so an
 * Edge node and an H743 node look the same to a root and vice versa:
 *   - a node has one "mesh station" netif whose frames travel to the root with
 *     esp_mesh_send(MESH_DATA_TODS) and get an address from the root's DHCP
 *     server over the mesh;
 *   - the root has one "mesh AP" netif (10.0.0.1/16, DHCP server) whose frames
 *     travel to the nodes with esp_mesh_send(MESH_DATA_P2P) by the routing
 *     table, and, with a router at the site, the real Wi-Fi station netif.
 * Added here: the link to Linux, an Ethernet netif whose frames go through the
 * hosted transport as ESP_MESH_IF (mesh0 on Linux), with a /30 and NAPT
 * marking it as "inside": a node translates Linux's traffic to its mesh
 * address; a root forwards between Linux, the mesh subnet and the router
 * (Linux's uplink traffic translated to the router address, mesh and Linux
 * routed to each other untranslated).
 *
 * Leaving mesh mode restarts the ESP: the firmware boots into the hosted
 * service and the Linux driver recovers the link by itself, which is simpler
 * and safer than unwinding the mesh and Wi-Fi state by hand.
 */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_netif.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "lwip/lwip_napt.h"
#include "dhcpserver/dhcpserver.h"
#include "interface.h"
#include "adapter.h"
#include "xwave_mesh.h"

static const char *TAG = "XWAVE_MESH";

/* cmd.c wraps esp_supplicant_init() with the host-driven supplicant; this is
 * IDF's own, and the unregister of whatever is installed (esp_private) */
esp_err_t __real_esp_supplicant_init(void);
int esp_wifi_unregister_wpa_cb_internal(void);

#define MESH_RX_SIZE            1560
#define LINK_MAX_FRAME          1600
#define ROUTE_TABLE_MAX         CONFIG_XWAVE_MESH_ROUTE_TABLE_SIZE
#define MAC_EQUAL(a, b)         (memcmp((a), (b), 6) == 0)

typedef struct {
    esp_netif_driver_base_t base;
    uint8_t sta_mac[6];
} mesh_drv_t;

/* the mesh subnet, as in the example (and in the H743's mesh.bin) */
static const esp_netif_ip_info_t s_mesh_subnet = {
    .ip      = { .addr = ESP_IP4TOADDR(10, 0, 0, 1) },
    .gw      = { .addr = ESP_IP4TOADDR(10, 0, 0, 1) },
    .netmask = { .addr = ESP_IP4TOADDR(255, 255, 0, 0) },
};

static struct cmd_xwave_mesh s_cfg;
static bool s_active, s_root, s_router;
static esp_netif_t *s_link, *s_sta, *s_ap;
static esp_netif_ip_info_t s_link_ip;
static volatile bool s_rx_run;
static mesh_addr_t s_route[ROUTE_TABLE_MAX];
static esp_event_handler_instance_t s_mesh_evt, s_ip_evt;
static uint8_t s_layer;
static mesh_addr_t s_parent;
static bool s_parent_connected, s_tods;
static uint32_t s_router_ip4, s_mesh_ip4;

bool xwave_mesh_active(void) { return s_active; }

/* ── the link to Linux (ESP_MESH_IF) ──────────────────────────────────── */

static esp_err_t link_transmit(void *h, void *buffer, size_t len)
{
    interface_buffer_handle_t bh = {0};
    uint8_t *copy;

    if (!s_active || !len || len > LINK_MAX_FRAME)
        return ESP_ERR_INVALID_SIZE;
    copy = malloc(len);
    if (!copy)
        return ESP_ERR_NO_MEM;
    memcpy(copy, buffer, len);
    bh.if_type = ESP_MESH_IF;
    bh.if_num = 0;
    bh.payload = copy;
    bh.payload_len = len;
    bh.pkt_type = PACKET_TYPE_DATA;
    bh.priv_buffer_handle = copy;
    bh.free_buf_handle = free;
    if (send_to_host_timeout(PRIO_Q_LOW, &bh, pdMS_TO_TICKS(200)) != pdTRUE) {
        free(copy);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t link_transmit_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
{
    return link_transmit(h, buffer, len);
}

/* every buffer handed to esp_netif_receive() here is copied by the stack
 * (LWIP_L2_TO_L3_COPY) and owned by its producer: nothing to free */
static void drv_free_rx(void *h, void *buffer) { }

static esp_err_t link_post_attach(esp_netif_t *netif, void *args)
{
    mesh_drv_t *d = args;
    esp_netif_driver_ifconfig_t ifc = {
        .handle = d,
        .transmit = link_transmit,
        .transmit_wrap = link_transmit_wrap,
        .driver_free_rx_buffer = drv_free_rx,
    };
    d->base.netif = netif;
    return esp_netif_set_driver_config(netif, &ifc);
}

static esp_err_t link_start(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config_t cfg;
    mesh_drv_t *d;
    uint8_t mac[6];

    base.flags = ESP_NETIF_FLAG_AUTOUP;        /* static address, no DHCP server */
    base.ip_info = &s_link_ip;
    base.if_key = "XWLINK";
    base.if_desc = "xwave_link";
    base.route_prio = 20;                      /* below a router station (100): Linux is the gateway only without one */
    cfg.base = &base;
    cfg.driver = NULL;
    cfg.stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_AP;
    s_link = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_link, ESP_FAIL, TAG, "link netif");
    d = calloc(1, sizeof(*d));
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "link driver");
    d->base.post_attach = link_post_attach;
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_link, d), TAG, "link attach");
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    mac[0] |= 0x02;                            /* locally administered, distinct from the radio's */
    mac[5] ^= 0x5A;
    esp_netif_set_mac(s_link, mac);
    esp_netif_action_start(s_link, NULL, 0, NULL);
    esp_netif_action_connected(s_link, NULL, 0, NULL);
    ip_napt_enable(s_link_ip.ip.addr, 1);      /* Linux is inside */
    ESP_LOGI(TAG, "link to Linux up: " IPSTR "/%u, Linux " IPSTR,
             IP2STR(&s_link_ip.ip), s_cfg.link_prefix, IP2STR(&s_link_ip.gw));
    return ESP_OK;
}

esp_err_t xwave_mesh_rx_from_host(const uint8_t *frame, uint16_t len)
{
    if (!s_active || !s_link || len < 14)
        return ESP_ERR_INVALID_STATE;
    return esp_netif_receive(s_link, (void *)frame, len, NULL);
}

/* ── the mesh netifs, after the IDF example ───────────────────────────── */

static void mesh_rx_task(void *arg)
{
    static uint8_t rx_buf[MESH_RX_SIZE];
    mesh_addr_t from;
    mesh_data_t data;
    int flag;

    while (s_rx_run) {
        data.data = rx_buf;
        data.size = MESH_RX_SIZE;
        if (esp_mesh_recv(&from, &data, pdMS_TO_TICKS(1000), &flag, NULL, 0) != ESP_OK)
            continue;
        if (s_root) {
            if (data.proto == MESH_PROTO_AP && s_ap)
                esp_netif_receive(s_ap, data.data, data.size, NULL);
        } else {
            if (data.proto == MESH_PROTO_STA && s_sta)
                esp_netif_receive(s_sta, data.data, data.size, NULL);
        }
    }
    vTaskDelete(NULL);
}

/* root AP -> a node's station (or every node for a broadcast) */
static esp_err_t ap_transmit(void *h, void *buffer, size_t len)
{
    static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    mesh_drv_t *d = h;
    mesh_addr_t dest;
    mesh_data_t data = { .data = buffer, .size = len, .proto = MESH_PROTO_STA, .tos = MESH_TOS_P2P };
    int n = 0;

    memcpy(dest.addr, buffer, 6);
    if (MAC_EQUAL(dest.addr, bcast)) {
        esp_mesh_get_routing_table(s_route, ROUTE_TABLE_MAX * 6, &n);
        for (int i = 0; i < n; i++) {
            if (MAC_EQUAL(s_route[i].addr, d->sta_mac))
                continue;
            esp_mesh_send(&s_route[i], &data, MESH_DATA_P2P, NULL, 0);
        }
        return ESP_OK;
    }
    return esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

static esp_err_t ap_transmit_wrap(void *h, void *b, size_t len, void *nb) { return ap_transmit(h, b, len); }

/* a node's station -> the root's AP */
static esp_err_t sta_transmit(void *h, void *buffer, size_t len)
{
    mesh_data_t data = { .data = buffer, .size = len, .proto = MESH_PROTO_AP, .tos = MESH_TOS_P2P };
    return esp_mesh_send(NULL, &data, MESH_DATA_TODS, NULL, 0);
}

static esp_err_t sta_transmit_wrap(void *h, void *b, size_t len, void *nb) { return sta_transmit(h, b, len); }

static esp_err_t ap_post_attach(esp_netif_t *netif, void *args)
{
    mesh_drv_t *d = args;
    esp_netif_driver_ifconfig_t ifc = { .handle = d, .transmit = ap_transmit,
                                        .transmit_wrap = ap_transmit_wrap, .driver_free_rx_buffer = drv_free_rx };
    d->base.netif = netif;
    return esp_netif_set_driver_config(netif, &ifc);
}

static esp_err_t sta_post_attach(esp_netif_t *netif, void *args)
{
    mesh_drv_t *d = args;
    esp_netif_driver_ifconfig_t ifc = { .handle = d, .transmit = sta_transmit,
                                        .transmit_wrap = sta_transmit_wrap, .driver_free_rx_buffer = drv_free_rx };
    d->base.netif = netif;
    return esp_netif_set_driver_config(netif, &ifc);
}

static mesh_drv_t *mesh_driver_new(bool ap)
{
    mesh_drv_t *d = calloc(1, sizeof(*d));
    if (!d)
        return NULL;
    d->base.post_attach = ap ? ap_post_attach : sta_post_attach;
    esp_wifi_get_mac(WIFI_IF_STA, d->sta_mac);
    if (!s_rx_run) {
        s_rx_run = true;
        xTaskCreate(mesh_rx_task, "xwave_mesh_rx", 3072, NULL, 5, NULL);
    }
    return d;
}

/* root: the mesh AP netif with its DHCP server; dns = what the nodes are told */
static esp_err_t root_ap_start(uint32_t dns)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config_t cfg = { .base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_AP };
    esp_netif_dns_info_t dnsinfo = { .ip.type = IPADDR_TYPE_V4 };
    dhcps_offer_t offer = OFFER_DNS;
    mesh_drv_t *d;
    uint8_t mac[6];

    if (s_ap)
        return ESP_OK;
    base.if_desc = "mesh_link_ap";
    base.ip_info = &s_mesh_subnet;
    s_ap = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_ap, ESP_FAIL, TAG, "mesh ap netif");
    d = mesh_driver_new(true);
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "mesh ap driver");
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_ap, d), TAG, "mesh ap attach");
    dnsinfo.ip.u_addr.ip4.addr = dns;
    esp_netif_dhcps_option(s_ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer, sizeof(offer));
    esp_netif_set_dns_info(s_ap, ESP_NETIF_DNS_MAIN, &dnsinfo);
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    esp_netif_set_mac(s_ap, mac);
    esp_netif_action_start(s_ap, NULL, 0, NULL);
    ip_napt_enable(s_mesh_subnet.ip.addr, 1);  /* the mesh is inside too */
    s_mesh_ip4 = s_mesh_subnet.ip.addr;
    ESP_LOGI(TAG, "root: mesh AP netif up, " IPSTR "/16, DNS " IPSTR, IP2STR(&s_mesh_subnet.ip), IP2STR(&dnsinfo.ip.u_addr.ip4));
    return ESP_OK;
}

/* root with a router: the real Wi-Fi station netif, driven by the mesh stack's
 * own association to the router */
static esp_err_t root_sta_start(void)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    uint8_t mac[6];
    void *drv;

    s_sta = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_sta, ESP_FAIL, TAG, "wifi sta netif");
    ESP_RETURN_ON_ERROR(esp_netif_attach_wifi_station(s_sta), TAG, "wifi sta attach");
    ESP_RETURN_ON_ERROR(esp_wifi_set_default_wifi_sta_handlers(), TAG, "wifi sta handlers");
    drv = esp_netif_get_io_driver(s_sta);
    ESP_RETURN_ON_ERROR(esp_wifi_register_if_rxcb(drv, esp_netif_receive, s_sta), TAG, "wifi sta rxcb");
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_netif_set_mac(s_sta, mac);
    esp_netif_action_start(s_sta, NULL, 0, NULL);
    return ESP_OK;
}

/* node: the mesh station netif, once a parent is there */
static esp_err_t node_sta_start(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config_t cfg = { .base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA };
    mesh_drv_t *d;
    uint8_t mac[6];

    if (s_sta)
        return ESP_OK;
    base.if_desc = "mesh_link_sta";
    s_sta = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_sta, ESP_FAIL, TAG, "mesh sta netif");
    d = mesh_driver_new(false);
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "mesh sta driver");
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_sta, d), TAG, "mesh sta attach");
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_netif_set_mac(s_sta, mac);
    esp_netif_action_start(s_sta, NULL, 0, NULL);
    esp_netif_action_connected(s_sta, NULL, 0, NULL);     /* DHCP client over the mesh */
    ESP_LOGI(TAG, "node: mesh station netif up, asking the root for an address");
    return ESP_OK;
}

static void node_sta_stop(void)
{
    if (!s_sta)
        return;
    esp_netif_action_disconnected(s_sta, NULL, 0, NULL);
    free(esp_netif_get_io_driver(s_sta));
    esp_netif_destroy(s_sta);
    s_sta = NULL;
    s_mesh_ip4 = 0;
}

/* ── events ───────────────────────────────────────────────────────────── */

/* A root with a router: the manual designation of the IDF example
 * manual_networking. The mesh stack does not take a fixed root to its router
 * by itself (seen on the bench 2026-11-03), so the root scans for the SSID
 * and names the router as its parent; without the router in range it falls
 * back to the router-less root so the group still works, and tries again
 * every minute. Runs in its own task: no scanning from an event handler. */
static void root_router_task(void *arg)
{
    wifi_scan_config_t scan = { .ssid = (uint8_t *)s_cfg.router_ssid, .show_hidden = true };
    wifi_ap_record_t *recs = NULL;
    uint16_t n = 0;
    bool fallback = false;

    while (s_active && s_root && s_router) {
        if (esp_wifi_scan_start(&scan, true) == ESP_OK) {
            esp_wifi_scan_get_ap_num(&n);
            if (n) {
                recs = calloc(n, sizeof(*recs));
                if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
                    int best = -1;
                    for (int i = 0; i < n; i++)
                        if (!strcmp((char *)recs[i].ssid, s_cfg.router_ssid) &&
                            (best < 0 || recs[i].rssi > recs[best].rssi))
                            best = i;
                    if (best >= 0) {
                        wifi_config_t parent = { 0 };
                        parent.sta.channel = recs[best].primary;
                        memcpy(parent.sta.ssid, recs[best].ssid, sizeof(parent.sta.ssid));
                        parent.sta.bssid_set = 1;
                        memcpy(parent.sta.bssid, recs[best].bssid, 6);
                        if (recs[best].authmode != WIFI_AUTH_OPEN)
                            memcpy(parent.sta.password, s_cfg.router_password,
                                   strlen(s_cfg.router_password) > sizeof(parent.sta.password) ?
                                   sizeof(parent.sta.password) : strlen(s_cfg.router_password));
                        ESP_LOGI(TAG, "root: router %s " MACSTR " channel %u rssi %d: naming it the parent",
                                 recs[best].ssid, MAC2STR(recs[best].bssid), recs[best].primary, recs[best].rssi);
                        esp_err_t ret = esp_mesh_set_parent(&parent, NULL, MESH_ROOT, MESH_ROOT_LAYER);
                        if (ret == ESP_OK) {
                            free(recs);
                            vTaskDelete(NULL);
                            return;
                        }
                        ESP_LOGW(TAG, "root: esp_mesh_set_parent: %s", esp_err_to_name(ret));
                    }
                }
                free(recs);
                recs = NULL;
            }
            esp_mesh_flush_scan_result();
        }
        if (!fallback) {
            ESP_LOGW(TAG, "root: router \"%s\" not in range; running as root without router meanwhile", s_cfg.router_ssid);
            esp_mesh_set_self_organized(true, false);
            esp_mesh_set_type(MESH_ROOT);
            esp_mesh_post_toDS_state(true);
            fallback = true;
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
    vTaskDelete(NULL);
}

static void mesh_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case MESH_EVENT_STARTED:
        s_layer = esp_mesh_get_layer();
        ESP_LOGI(TAG, "mesh started, %s", s_root ? "root" : "node");
        if (s_root) {
            root_ap_start(s_router ? 0 : s_link_ip.gw.addr);
            if (s_router)
                xTaskCreate(root_router_task, "xwave_root_router", 4096, NULL, 5, NULL);
            else
                esp_mesh_post_toDS_state(true);   /* Linux is the DS */
        }
        break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *c = data;
        s_layer = c->self_layer;
        memcpy(s_parent.addr, c->connected.bssid, 6);
        s_parent_connected = true;
        ESP_LOGI(TAG, "parent connected, layer %u, parent " MACSTR "%s", s_layer, MAC2STR(s_parent.addr),
                 esp_mesh_is_root() ? " (root)" : "");
        if (!s_root)
            node_sta_start();
        break;
    }
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *d = data;
        s_parent_connected = false;
        s_layer = esp_mesh_get_layer();
        ESP_LOGW(TAG, "parent lost, reason %u", d->reason);
        if (!s_root)
            node_sta_stop();
        break;
    }
    case MESH_EVENT_LAYER_CHANGE:
        s_layer = ((mesh_event_layer_change_t *)data)->new_layer;
        break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *c = data;
        ESP_LOGI(TAG, "child " MACSTR " connected (aid %u)", MAC2STR(c->mac), c->aid);
        break;
    }
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *c = data;
        ESP_LOGI(TAG, "child " MACSTR " left (aid %u)", MAC2STR(c->mac), c->aid);
        break;
    }
    case MESH_EVENT_TODS_STATE:
        s_tods = *(mesh_event_toDS_state_t *)data == MESH_TODS_REACHABLE;
        ESP_LOGI(TAG, "toDS %s", s_tods ? "reachable" : "unreachable");
        break;
    case MESH_EVENT_NO_PARENT_FOUND:
        ESP_LOGW(TAG, "no parent found (scan %u)", ((mesh_event_no_parent_found_t *)data)->scan_times);
        break;
    case MESH_EVENT_ROOT_ADDRESS:
        ESP_LOGI(TAG, "root is " MACSTR, MAC2STR(((mesh_event_root_address_t *)data)->addr));
        break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *f = data;
        ESP_LOGI(TAG, "network found on channel %u, router " MACSTR, f->channel, MAC2STR(f->router_bssid));
        break;
    }
    case MESH_EVENT_ROUTER_SWITCH:
    case MESH_EVENT_ROUTING_TABLE_ADD:
    case MESH_EVENT_ROUTING_TABLE_REMOVE:
    case MESH_EVENT_SCAN_DONE:
    case MESH_EVENT_NETWORK_STATE:
    case MESH_EVENT_ROOT_FIXED:
    case MESH_EVENT_VOTE_STARTED:
    case MESH_EVENT_VOTE_STOPPED:
    case MESH_EVENT_CHANNEL_SWITCH:
        break;
    default:
        ESP_LOGD(TAG, "mesh event %ld", (long)id);
        break;
    }
}

static void ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ip_event_got_ip_t *e = data;

    if (id != IP_EVENT_STA_GOT_IP)
        return;
    if (s_root) {
        esp_netif_dns_info_t dns;
        s_router_ip4 = e->ip_info.ip.addr;
        ESP_LOGI(TAG, "root: router gave " IPSTR, IP2STR(&e->ip_info.ip));
        esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN, &dns);
        root_ap_start(dns.ip.u_addr.ip4.addr ? dns.ip.u_addr.ip4.addr : s_link_ip.gw.addr);
    } else {
        s_mesh_ip4 = e->ip_info.ip.addr;
        ESP_LOGI(TAG, "node: mesh address " IPSTR ", gateway " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
    }
}

/* ── start / stop ─────────────────────────────────────────────────────── */

static esp_err_t mesh_start(const struct cmd_xwave_mesh *req)
{
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    esp_err_t ret;
    size_t n;

    s_cfg = *req;
    s_cfg.mesh_password[XWAVE_MESH_PW_LEN - 1] = 0;
    s_cfg.router_ssid[XWAVE_MESH_SSID_LEN - 1] = 0;
    s_cfg.router_password[XWAVE_MESH_PW_LEN - 1] = 0;
    s_root = req->mode == XWAVE_MESH_ROOT;
    s_router = s_cfg.router_ssid[0] != 0;
    s_layer = 0;
    s_parent_connected = s_tods = false;
    s_router_ip4 = s_mesh_ip4 = 0;
    memset(&s_parent, 0, sizeof(s_parent));

    memset(&s_link_ip, 0, sizeof(s_link_ip));
    memcpy(&s_link_ip.ip.addr, req->link_ip, 4);
    memcpy(&s_link_ip.gw.addr, req->host_ip, 4);
    s_link_ip.netmask.addr = req->link_prefix >= 32 ? 0xffffffffu :
                             htonl(~((1u << (32 - (req->link_prefix ? req->link_prefix : 30))) - 1));

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        return ret;
    /* the hosted service may have left the radio in any mode; the mesh
     * stack sets AP+STA itself once started */
    esp_wifi_disconnect();
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_ps(WIFI_PS_NONE);
    ret = esp_wifi_start();
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "esp_wifi_start: %s", esp_err_to_name(ret));

    /* The hosted firmware wraps esp_supplicant_init() (cmd.c, --wrap) so that
     * Linux's wpa_supplicant does authentication and association through the
     * host; the mesh needs the ESP's own supplicant for its station and its
     * AP (seen 2026-11-04: "auth timeout" against the router). Put IDF's back
     * for the life of the mesh; leaving mesh mode restarts the ESP anyway. */
    esp_wifi_unregister_wpa_cb_internal();
    ret = __real_esp_supplicant_init();
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "internal supplicant: %s", esp_err_to_name(ret));

    ESP_RETURN_ON_ERROR(esp_mesh_init(), TAG, "mesh init");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(MESH_EVENT, ESP_EVENT_ANY_ID, mesh_event, NULL, &s_mesh_evt), TAG, "mesh events");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event, NULL, &s_ip_evt), TAG, "ip events");
    ESP_RETURN_ON_ERROR(esp_mesh_set_max_layer(req->max_layer ? req->max_layer : 6), TAG, "max layer");
    ESP_RETURN_ON_ERROR(esp_mesh_set_vote_percentage(1), TAG, "vote");
    ESP_RETURN_ON_ERROR(esp_mesh_set_ap_assoc_expire(10), TAG, "assoc expire");
    ESP_RETURN_ON_ERROR(esp_mesh_send_block_time(5000), TAG, "block time");

    memcpy(cfg.mesh_id.addr, req->mesh_id, 6);
    cfg.channel = req->channel;
    n = strlen(s_cfg.router_ssid);
    if (n) {
        cfg.router.ssid_len = n;
        memcpy(cfg.router.ssid, s_cfg.router_ssid, n);
        memcpy(cfg.router.password, s_cfg.router_password, strlen(s_cfg.router_password));
    } else {
        /* no router at the site: the root never finds this one and stays
         * on `channel`; Linux is the DS (toDS posted reachable) */
        static const char none[] = "xwave-no-router";
        cfg.router.ssid_len = sizeof(none) - 1;
        memcpy(cfg.router.ssid, none, sizeof(none) - 1);
        if (!cfg.channel)
            cfg.channel = 6;
    }
    ESP_RETURN_ON_ERROR(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA2_PSK), TAG, "ap authmode");
    cfg.mesh_ap.max_connection = req->max_connection ? req->max_connection : 6;
    cfg.mesh_ap.nonmesh_max_connection = 0;
    n = strlen(s_cfg.mesh_password);
    memcpy(cfg.mesh_ap.password, s_cfg.mesh_password, n > sizeof(cfg.mesh_ap.password) - 1 ? sizeof(cfg.mesh_ap.password) - 1 : n);
    ESP_RETURN_ON_ERROR(esp_mesh_set_config(&cfg), TAG, "mesh config");

    if (s_root) {
        ESP_RETURN_ON_ERROR(esp_mesh_fix_root(true), TAG, "fix root");
        if (s_router) {
            /* the router is named as the parent after the start (root_router_task) */
            ESP_RETURN_ON_ERROR(esp_mesh_set_self_organized(false, false), TAG, "manual networking");
            ESP_RETURN_ON_ERROR(root_sta_start(), TAG, "router station");
        } else {
            ESP_RETURN_ON_ERROR(esp_mesh_set_type(MESH_ROOT), TAG, "set root");
        }
    } else {
        esp_mesh_fix_root(false);          /* self-organized, as the H743 nodes */
    }
    ESP_RETURN_ON_ERROR(link_start(), TAG, "link");
    s_active = true;
    ESP_RETURN_ON_ERROR(esp_mesh_start(), TAG, "mesh start");
    ESP_LOGI(TAG, "mesh %s started: id %02x%02x%02x%02x%02x%02x, channel %u, %s, heap %lu",
             s_root ? "root" : "node", req->mesh_id[0], req->mesh_id[1], req->mesh_id[2],
             req->mesh_id[3], req->mesh_id[4], req->mesh_id[5], cfg.channel,
             s_router ? s_cfg.router_ssid : "no router", (unsigned long)esp_get_free_heap_size());
    return ESP_OK;
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(300));            /* let the response leave */
    ESP_LOGW(TAG, "leaving mesh mode: restarting into the hosted service");
    esp_restart();
}

esp_err_t xwave_mesh_apply(const struct cmd_xwave_mesh *req)
{
    switch (req->mode) {
    case XWAVE_MESH_QUERY:
        return ESP_OK;
    case XWAVE_MESH_OFF:
        if (s_active) {
            s_active = false;
            xTaskCreate(restart_task, "xwave_mesh_off", 2048, NULL, 5, NULL);
        }
        return ESP_OK;
    case XWAVE_MESH_SWITCH_IMAGE: {
        /* the other OTA slot holds the other variant (hosted <-> mesh) when
         * Linux keeps both there: booting it is a 1 s role switch instead of
         * a 35 s write; Linux checks `image` after the boot event and writes
         * the right one if the slot held something else */
        const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
        esp_app_desc_t desc;
        if (!other || esp_ota_get_partition_description(other, &desc) != ESP_OK) {
            ESP_LOGW(TAG, "switch image: the other OTA slot holds no valid app");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGW(TAG, "switch image: booting %s (%s, %s)", other->label, desc.project_name, desc.version);
        if (esp_ota_set_boot_partition(other) != ESP_OK)
            return ESP_FAIL;
        s_active = false;
        xTaskCreate(restart_task, "xwave_mesh_switch", 2048, NULL, 5, NULL);
        return ESP_OK;
    }
    case XWAVE_MESH_NODE:
    case XWAVE_MESH_ROOT:
        if (s_active) {
            /* a different configuration: the clean way is the same restart;
             * Linux sends the new one after the boot event */
            ESP_LOGW(TAG, "mesh already running; restart first (mode off)");
            return ESP_ERR_INVALID_STATE;
        }
        return mesh_start(req);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

void xwave_mesh_get_status(struct xwave_mesh_status *st)
{
    int n = 0;

    memset(st, 0, sizeof(*st));
    st->mode = s_active ? (s_root ? XWAVE_MESH_ROOT : XWAVE_MESH_NODE) : XWAVE_MESH_OFF;
    st->free_heap = esp_get_free_heap_size();
#if CONFIG_XWAVE_MESH_IMAGE
    st->image = 1;
#else
    st->image = 0;
#endif
    if (!s_active)
        return;
    st->is_root = s_root;
    st->layer = s_layer;
    st->parent_connected = s_parent_connected;
    memcpy(st->parent, s_parent.addr, 6);
    memcpy(st->mesh_ip, &s_mesh_ip4, 4);
    memcpy(st->router_ip, &s_router_ip4, 4);
    st->tods_reachable = s_tods;
    if (s_root && esp_mesh_get_routing_table(s_route, ROUTE_TABLE_MAX * 6, &n) == ESP_OK) {
        st->route_table_size = n;
        st->children = n > 0 ? n - 1 : 0;
    }
}
