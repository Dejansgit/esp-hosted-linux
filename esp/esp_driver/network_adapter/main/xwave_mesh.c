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
 *
 * Who is what (2026-11-12, ODYSSEY docs/WiFi-Plan.md W11):
 *   - mode root: the unit is the group's root and stays it (a VoyagerLite
 *     master: the group's broker sits behind it on Linux). With a router
 *     configured it also joins the router, looks for it all the time while it
 *     is away, and carries the group without it meanwhile (root_router_task).
 *   - mode node with a router configured: a sensor, which takes part in the
 *     group's election exactly as the H743 sensors do (self-organized
 *     networking, the same router on every device). It is the root while it
 *     wins and holds the router, a node otherwise, and changes sides when the
 *     mesh tells it to (role_set, at every parent-connected event).
 *   - mode node without a router: a node and nothing else.
 * Every mesh device admits one plain Wi-Fi station on its mesh AP, which it
 * names EDGE_xxxxxx (the guest: a phone or a laptop that wants the unit's
 * pages), unless the option says no.
 */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_netif.h"
#include "esp_wifi_default.h"               /* the default station handlers, set and cleared as the role changes */
#include "esp_private/wifi.h"               /* the AP interface's raw transmit and receive, for the guest netif */
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "lwip/lwip_napt.h"
#include "lwip/prot/ip.h"                   /* IP_PROTO_TCP for the NAT port map */
#include "dhcpserver/dhcpserver.h"
#include "interface.h"
#include "adapter.h"
#include "xwave_mesh.h"

static const char *TAG = "XWAVE_MESH";

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
/* s_fixed_root: the unit was told to be the root (mode root). s_router: a router
 * is configured. s_root: what the unit is at this moment, which for a sensor
 * with a router is the election's business (role_set). */
static bool s_active, s_fixed_root, s_root, s_router;
static esp_netif_t *s_link, *s_sta, *s_ap;
static bool s_sta_wifi;                 /* s_sta is the real Wi-Fi station netif (a root's link to its router),
                                         * not a node's mesh station; the two never exist together */
static SemaphoreHandle_t s_netif_lock;  /* s_sta and s_ap come and go with the role while mesh_rx_task hands
                                         * frames to them: held for the pointers, never across a destroy */
static uint32_t s_router_dns;
static esp_netif_ip_info_t s_link_ip;
static volatile bool s_rx_run;
static mesh_addr_t s_route[ROUTE_TABLE_MAX];
static esp_event_handler_instance_t s_mesh_evt, s_ip_evt;
static uint8_t s_layer;
static mesh_addr_t s_parent;
static bool s_parent_connected, s_tods;
static uint32_t s_router_ip4, s_mesh_ip4;

bool xwave_mesh_active(void) { return s_active; }

/* ── options (2026-11-11) ─────────────────────────────────────────────────
 * Carried in the high nibble of the command's max_connection, for which the
 * Linux driver has the key maxconn=N on the mode line (the low nibble is the
 * number of mesh children, 6 by default; a mesh AP takes ten stations at
 * most). Without them (no maxconn on the line, what xwave-wifi writes) a
 * root admits one plain station on its AP, which it names EDGE_xxxxxx and
 * shows (guest_ap_start), and logs to its console only. They want a key of
 * their own in the driver with its next change. */
#define XWAVE_OPT_NO_GUEST      0x10    /* root: no plain (non-mesh) station on the mesh AP; the AP as the mesh stack makes it */
#define XWAVE_OPT_STOCK_AP      0x40    /* root: a plain station is admitted, but the AP keeps the mesh stack's
                                         * name and stays hidden (ESPM_xxxxxx) */
#define XWAVE_OPT_LOG           0x80    /* this firmware's log goes to Linux too, UDP port 5514 on the link */
static uint8_t s_opts;

/* The ESP's log on the Linux side without the console adapter
 * (XWAVE_OPT_LOG): the log's vprintf hook copies every piece of text into a
 * small buffer, a task sends what gathered there to the link's host address
 * ten times a second (on Linux: tcpdump -ni mesh0 -A udp port 5514, or
 * ODYSSEY tools/dev/mesh-diag.sh, which decodes it). Text that does not fit
 * is dropped; the console keeps getting everything. It found, on
 * 2026-11-11, why a plain station on the mesh AP stopped the mesh. */
#define XLOG_PORT               5514
#define XLOG_BUF                6144
static char s_xlog_buf[XLOG_BUF];
static size_t s_xlog_len;
static char s_xlog_line[256];
static portMUX_TYPE s_xlog_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_xlog_fmt;            /* one formatter at a time: s_xlog_line is shared */
static vprintf_like_t s_xlog_prev;
static uint32_t s_xlog_host;

static int xlog_vprintf(const char *fmt, va_list ap)
{
    if (s_xlog_fmt && !xPortInIsrContext() && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
        xSemaphoreTake(s_xlog_fmt, 0) == pdTRUE) {
        va_list cp;
        int n;

        va_copy(cp, ap);
        n = vsnprintf(s_xlog_line, sizeof(s_xlog_line), fmt, cp);
        va_end(cp);
        if (n > 0) {
            if (n >= (int)sizeof(s_xlog_line))
                n = sizeof(s_xlog_line) - 1;
            portENTER_CRITICAL(&s_xlog_mux);
            if (s_xlog_len + n <= XLOG_BUF) {
                memcpy(s_xlog_buf + s_xlog_len, s_xlog_line, n);
                s_xlog_len += n;
            }
            portEXIT_CRITICAL(&s_xlog_mux);
        }
        xSemaphoreGive(s_xlog_fmt);
    }
    return s_xlog_prev ? s_xlog_prev(fmt, ap) : 0;
}

static void xlog_task(void *arg)
{
    static char out[1200];
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(XLOG_PORT) };
    int s = -1;
    size_t n;

    to.sin_addr.s_addr = s_xlog_host;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_xlog_len)
            continue;
        portENTER_CRITICAL(&s_xlog_mux);
        n = s_xlog_len > sizeof(out) ? sizeof(out) : s_xlog_len;
        memcpy(out, s_xlog_buf, n);
        memmove(s_xlog_buf, s_xlog_buf + n, s_xlog_len - n);
        s_xlog_len -= n;
        portEXIT_CRITICAL(&s_xlog_mux);
        if (s < 0)
            s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s >= 0)
            sendto(s, out, n, 0, (struct sockaddr *)&to, sizeof(to));
    }
}

static void xlog_start(uint32_t host)
{
    if (s_xlog_fmt)
        return;
    s_xlog_host = host;
    s_xlog_fmt = xSemaphoreCreateMutex();
    if (!s_xlog_fmt)
        return;
    s_xlog_prev = esp_log_set_vprintf(xlog_vprintf);
    xTaskCreate(xlog_task, "xwave_xlog", 3072, NULL, 3, NULL);
}

/* who joins and leaves the AP, and why (the mesh events name mesh children only) */
static esp_event_handler_instance_t s_wifi_evt;
static void guest_station(const uint8_t *mac, bool joined);

static void wifi_ap_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "AP: station " MACSTR " joined, aid %u, %s", MAC2STR(e->mac), e->aid,
                 e->is_mesh_child ? "mesh child" : "not a mesh station");
        if (!e->is_mesh_child)
            guest_station(e->mac, true);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "AP: station " MACSTR " left, aid %u, %s, reason %u", MAC2STR(e->mac), e->aid,
                 e->is_mesh_child ? "mesh child" : "not a mesh station", e->reason);
        guest_station(e->mac, false);
    } else if (id == WIFI_EVENT_AP_START || id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG, "AP: %s", id == WIFI_EVENT_AP_START ? "started" : "stopped");
    }
}

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
        xSemaphoreTake(s_netif_lock, portMAX_DELAY);
        if (s_root) {
            if (data.proto == MESH_PROTO_AP && s_ap)
                esp_netif_receive(s_ap, data.data, data.size, NULL);
        } else {
            if (data.proto == MESH_PROTO_STA && s_sta && !s_sta_wifi)
                esp_netif_receive(s_sta, data.data, data.size, NULL);
        }
        xSemaphoreGive(s_netif_lock);
    }
    vTaskDelete(NULL);
}

/* ── the sensors' routing-table message ──────────────────────────────────
 * The H743 sensors' node firmware (mesh.bin, ESP-IDF 4.3.2's
 * ip_internal_network) expects the root to send it the routing table as a
 * binary mesh message, 0x56 followed by the table's station MACs, and
 * reboots its ESP after 75 s without one (its rtableWD_task). The example's
 * root sends it from its MQTT task; ours sends it every
 * ROUTE_TABLE_PERIOD_MS and as soon as a node enters the table. Nodes of
 * our own firmware ignore binary messages (mesh_rx_task). */
#define XWAVE_CMD_ROUTE_TABLE   0x56
#define ROUTE_TABLE_PERIOD_MS   10000

static TaskHandle_t s_route_task;

static void route_table_send(void)
{
    static uint8_t payload[1 + ROUTE_TABLE_MAX * 6];
    mesh_data_t data = { .data = payload, .proto = MESH_PROTO_BIN, .tos = MESH_TOS_P2P };
    uint8_t self[6];
    int n = 0;

    if (esp_mesh_get_routing_table(s_route, ROUTE_TABLE_MAX * 6, &n) != ESP_OK || n <= 0)
        return;
    esp_wifi_get_mac(WIFI_IF_STA, self);
    payload[0] = XWAVE_CMD_ROUTE_TABLE;
    memcpy(payload + 1, s_route, n * 6);
    data.size = 1 + n * 6;
    for (int i = 0; i < n; i++) {
        if (MAC_EQUAL(s_route[i].addr, self))
            continue;
        esp_mesh_send(&s_route[i], &data, MESH_DATA_P2P, NULL, 0);
    }
}

/* runs on every mesh device and sends while the device is the root: a sensor
 * becomes one, and stops being one, with the election */
static void route_table_task(void *arg)
{
    while (s_active) {
        if (s_root)
            route_table_send();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ROUTE_TABLE_PERIOD_MS));   /* a new node wakes it */
    }
    s_route_task = NULL;
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
    esp_netif_t *n;
    mesh_drv_t *d;
    uint8_t mac[6];

    if (s_ap)
        return ESP_OK;
    base.if_desc = "mesh_link_ap";
    base.ip_info = &s_mesh_subnet;
    n = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(n, ESP_FAIL, TAG, "mesh ap netif");
    d = mesh_driver_new(true);
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "mesh ap driver");
    ESP_RETURN_ON_ERROR(esp_netif_attach(n, d), TAG, "mesh ap attach");
    dnsinfo.ip.u_addr.ip4.addr = dns;
    esp_netif_dhcps_option(n, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer, sizeof(offer));
    esp_netif_set_dns_info(n, ESP_NETIF_DNS_MAIN, &dnsinfo);
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    esp_netif_set_mac(n, mac);
    esp_netif_action_start(n, NULL, 0, NULL);
    ip_napt_enable(s_mesh_subnet.ip.addr, 1);  /* the mesh is inside too */
    s_mesh_ip4 = s_mesh_subnet.ip.addr;
    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    s_ap = n;
    xSemaphoreGive(s_netif_lock);
    ESP_LOGI(TAG, "root: mesh AP netif up, " IPSTR "/16, DNS " IPSTR, IP2STR(&s_mesh_subnet.ip), IP2STR(&dnsinfo.ip.u_addr.ip4));
    return ESP_OK;
}

/* a sensor that is the root no longer: its DHCP server and the nodes' gateway go */
static void root_ap_stop(void)
{
    esp_netif_t *n;
    void *d;

    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    n = s_ap;
    s_ap = NULL;
    xSemaphoreGive(s_netif_lock);
    if (!n)
        return;
    esp_netif_dhcps_stop(n);
    d = esp_netif_get_io_driver(n);
    esp_netif_destroy(n);
    free(d);
    s_mesh_ip4 = 0;
}

/* ── a phone on the unit's mesh AP (the guest) ───────────────────────────
 * One ordinary Wi-Fi station (a phone, a laptop) may join the unit's mesh AP,
 * so a master without router and without cable can still be reached, and
 * since 2026-11-12 any mesh device, root or node: a node's pages are behind
 * the mesh's NAT for everybody else
 * (mesh_ap.nonmesh_max_connection = 1; WPA2 with the mesh password). Such a
 * station is no mesh child: its frames do not travel by esp_mesh_send/recv
 * but as plain frames of the Wi-Fi AP interface, so it gets a netif of its
 * own on that interface, with its own subnet and DHCP server. NAPT marks it
 * "inside" like the mesh and the link: the phone reaches Linux at the link's
 * host address (10.44.255.2: VoyagerLite on 80, the sensor page on 8080;
 * Linux needs the route back, xwave-wifi adds it) and, through a router, the
 * outside.
 * The AP's name. The mesh stack calls its AP "ESPM_" and the last three
 * bytes of the ESP's station MAC and hides it. The mesh children do not go
 * by that name (both H743 sensors joined an AP renamed in the started event,
 * as mesh children, at each of five starts), so the root calls it "EDGE_"
 * and the same six digits and shows it: a phone lists it like any network.
 * Bench 2026-11-11, two H743 sensors as children, no router: the PC as the
 * guest for 84 s, 2.4 MB of pages loaded at about 650 kB/s, 10 of 10 pings;
 * both sensors answered 118 of 118 pings meanwhile and streamed at their
 * rate. What it took is in the comments below. */
static const esp_netif_ip_info_t s_guest_subnet = {
    .ip      = { .addr = ESP_IP4TOADDR(10, 45, 0, 1) },
    .gw      = { .addr = ESP_IP4TOADDR(10, 45, 0, 1) },
    .netmask = { .addr = ESP_IP4TOADDR(255, 255, 255, 0) },
};
static esp_netif_t *s_guest;

#if CONFIG_XWAVE_MESH_IMAGE
/* No plain group-addressed frame may leave the mesh AP: 20 plain broadcast
 * frames from the root, two a second, and both H743 children stopped
 * answering for those ten seconds and rejoined (bench 2026-11-11). The Wi-Fi
 * library sends such frames by itself once the AP interface has a receive
 * callback: hostap_deliver_data() (libnet80211, ieee80211_hostap.o) forwards
 * every frame a plain station sends to a group address, or to another
 * station, back into the BSS with ieee80211_output() before it hands the
 * frame to the callback (without a callback it drops the frame, which is why
 * a plain station that merely associates does no harm). A phone sends such
 * frames all the time (ARP, DHCP, mDNS; 692 in 84 s from a Windows PC), so
 * with one on the AP the mesh was dead until it left. A mesh child's own
 * plain DHCP discover at its association is forwarded the same way, to the
 * other children: the likely cause of the two sensors leaving and rejoining
 * in turn, every 10-20 s, that was seen whenever the callback was registered
 * and never without it. There is no switch for that forwarding, so the call
 * is taken over at link time (-Wl,--wrap=ieee80211_output in CMakeLists.txt,
 * mesh image only).
 * Only ieee80211_hostap.o, ieee80211_ht.o (the same forwarding for A-MSDU
 * subframes) and wapi.o refer to ieee80211_output from outside its own object;
 * the mesh stack sends with esp_wifi_mesh_tx() and the netifs with
 * esp_wifi_internal_tx(), both defined next to it and not affected. While the
 * mesh runs the forwarded copy is dropped: stations on the mesh AP do not
 * reach each other through the AP, which nothing here needs. */
static uint32_t s_ap_forward_dropped;
int __real_ieee80211_output(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f);

int __wrap_ieee80211_output(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f)
{
    if (s_active) {
        s_ap_forward_dropped++;
        return 0;
    }
    return __real_ieee80211_output(a, b, c, d, e, f);
}

/* The AP's beacon interval stays at 100 ms on a root that admits a guest. The mesh
 * stack stretches it by itself: 100 ms after a station came or went, 300 ms ten seconds
 * later, 1000 ms once the network has been quiet for about 100 s ("[scan]new scanning
 * time ..., beacon interval ..." in its log), through esp_wifi_set_beacon_interval()
 * (libmesh's esp_mesh_set_beacon_interval and esp_mesh_map_change_beacon_interval call
 * it; libnet80211 defines it). A plain station does not survive a stretch: the AP let the
 * PC go about one second after each (the log's "leave ... reason = 0"), it came back at
 * once, its leaving set the interval to 100 ms again, and ten seconds later the next
 * stretch dropped it again: a station dropped and taken back every ten seconds for as
 * long as it stayed (bench 2026-11-11, seen on the visible AP). With the interval held
 * the PC stayed 148 s, 130 of them idle, through one refused stretch to 1000 ms, without
 * a drop; the two H743 children streamed on (they do not mind more beacons than the
 * stack would send). Taken over at link time like the call above
 * (-Wl,--wrap=esp_wifi_set_beacon_interval, mesh image only); the argument is the
 * interval in milliseconds. A device that admits no guest (XWAVE_OPT_NO_GUEST) keeps
 * the stack's own intervals. */
static uint32_t s_beacon_stretch_refused;
int __real_esp_wifi_set_beacon_interval(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f);

int __wrap_esp_wifi_set_beacon_interval(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e, uint32_t f)
{
    if (s_active && !(s_opts & XWAVE_OPT_NO_GUEST) && a > 100) {
        s_beacon_stretch_refused++;
        a = 100;
    }
    return __real_esp_wifi_set_beacon_interval(a, b, c, d, e, f);
}
#endif

/* The guest netif talks to the one non-mesh station and to nobody else, in
 * both directions:
 *   - receive: only frames whose source is the guest. A mesh child's ESP
 *     (the sensors' mesh.bin) runs a DHCP client on its plain Wi-Fi station
 *     for a moment at every association, one DHCP discover as a plain frame;
 *     this netif's DHCP server must not answer it (it gave the sensors a
 *     second address, 10.45.0.x, at every association);
 *   - transmit: only unicast frames to the guest. A broadcast of the stack
 *     (a DHCP offer, an ARP request) leaves as a unicast frame to the guest's
 *     MAC, for the reason above: no plain group-addressed frame on the mesh
 *     AP. Without a guest nothing leaves.
 * The guest is the station the AP reports as "not a mesh child". */
static uint8_t s_guest_mac[6];
static volatile bool s_guest_present;
static uint32_t s_ap_rx_frames;         /* plain frames the AP interface's receive callback was handed */

static void guest_station(const uint8_t *mac, bool joined)
{
    if (joined) {
        s_guest_present = false;
        memcpy(s_guest_mac, mac, 6);
        s_guest_present = true;
    } else if (MAC_EQUAL(mac, s_guest_mac)) {
        s_guest_present = false;
#if CONFIG_XWAVE_MESH_IMAGE
        ESP_LOGI(TAG, "guest left: so far %lu plain frames received on the AP, %lu forwardings into the BSS dropped, %lu beacon stretches refused",
                 (unsigned long)s_ap_rx_frames, (unsigned long)s_ap_forward_dropped, (unsigned long)s_beacon_stretch_refused);
#endif
    }
}

static esp_err_t guest_transmit(void *h, void *buffer, size_t len)
{
    static uint8_t frame[LINK_MAX_FRAME];
    const uint8_t *dst = buffer;

    if (!s_guest_present || len < 14 || len > sizeof(frame))
        return ESP_OK;
    if (dst[0] & 0x01) {
        memcpy(frame, buffer, len);
        memcpy(frame, s_guest_mac, 6);
        return esp_wifi_internal_tx(WIFI_IF_AP, frame, len);
    }
    if (!MAC_EQUAL(dst, s_guest_mac))
        return ESP_OK;
    return esp_wifi_internal_tx(WIFI_IF_AP, buffer, len);
}

static esp_err_t guest_transmit_wrap(void *h, void *b, size_t len, void *nb) { return guest_transmit(h, b, len); }

static void guest_free_rx(void *h, void *eb)
{
    if (eb)
        esp_wifi_internal_free_rx_buffer(eb);
}

/* the Wi-Fi AP interface's receive callback: plain (non-mesh) frames of any associated station */
static esp_err_t guest_wifi_rx(void *buffer, uint16_t len, void *eb)
{
    s_ap_rx_frames++;
    if (s_guest && s_guest_present && len >= 14 && MAC_EQUAL((const uint8_t *)buffer + 6, s_guest_mac))
        return esp_netif_receive(s_guest, buffer, len, eb);
    if (eb)
        esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

static esp_err_t guest_post_attach(esp_netif_t *netif, void *args)
{
    mesh_drv_t *d = args;
    esp_netif_driver_ifconfig_t ifc = { .handle = d, .transmit = guest_transmit,
                                        .transmit_wrap = guest_transmit_wrap, .driver_free_rx_buffer = guest_free_rx };
    d->base.netif = netif;
    return esp_netif_set_driver_config(netif, &ifc);
}

/* The name a phone sees. Written in the started event, before any child is
 * there (the AP restarts for it), and looked at again whenever a parent is
 * connected: only an AP that carries the stack's name again is rewritten, so
 * a device with children is not shaken for nothing. */
static void guest_ap_name(void)
{
    wifi_config_t ap;

    if ((s_opts & (XWAVE_OPT_NO_GUEST | XWAVE_OPT_STOCK_AP)) || esp_wifi_get_config(WIFI_IF_AP, &ap) != ESP_OK)
        return;
    if (memcmp(ap.ap.ssid, "ESPM_", 5) && !ap.ap.ssid_hidden)
        return;
    if (!memcmp(ap.ap.ssid, "ESPM_", 5))
        memcpy(ap.ap.ssid, "EDGE_", 5);
    ap.ap.ssid_hidden = 0;
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "the AP keeps the mesh stack's name: %s", esp_err_to_name(ret));
    if (esp_wifi_get_config(WIFI_IF_AP, &ap) == ESP_OK)
        ESP_LOGI(TAG, "mesh AP \"%s\"%s, channel %u, up to %u stations", (char *)ap.ap.ssid,
                 ap.ap.ssid_hidden ? " (hidden)" : "", ap.ap.channel, ap.ap.max_connection);
}

static esp_err_t guest_ap_start(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config_t cfg = { .base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_AP };
    uint8_t mac[6];
    mesh_drv_t *d;

    if (s_opts & XWAVE_OPT_NO_GUEST)
        return ESP_OK;
    guest_ap_name();
    if (s_guest) {
        /* a second started event: the AP interface may have been made anew, and
         * its receive callback wants registering again */
        return esp_wifi_internal_reg_rxcb(WIFI_IF_AP, guest_wifi_rx);
    }
    base.if_key = "XWGUEST";
    base.if_desc = "xwave_guest_ap";
    base.ip_info = &s_guest_subnet;
    base.route_prio = 5;
    s_guest = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_guest, ESP_FAIL, TAG, "guest ap netif");
    d = calloc(1, sizeof(*d));
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "guest ap driver");
    d->base.post_attach = guest_post_attach;
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_guest, d), TAG, "guest ap attach");
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    esp_netif_set_mac(s_guest, mac);
    esp_netif_action_start(s_guest, NULL, 0, NULL);
    ip_napt_enable(s_guest_subnet.ip.addr, 1);
    ESP_RETURN_ON_ERROR(esp_wifi_internal_reg_rxcb(WIFI_IF_AP, guest_wifi_rx), TAG, "guest ap rxcb");
    ESP_LOGI(TAG, "guest netif up on the mesh AP, " IPSTR "/24 (one non-mesh station)", IP2STR(&s_guest_subnet.ip));
    return ESP_OK;
}

/* A root's link to its router: the real Wi-Fi station netif, driven by the mesh
 * stack's own association to the router. The key is the default station's, as
 * the node's mesh station below: the two never exist together.
 * `connected`: the association is there already (a sensor that has just won
 * the election after having been a node: the default handlers, which start
 * the DHCP client at the association, were not registered when it happened). */
static esp_err_t root_sta_start(bool connected)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *n;
    uint8_t mac[6];
    void *drv;

    if (s_sta && s_sta_wifi)
        return ESP_OK;
    n = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(n, ESP_FAIL, TAG, "wifi sta netif");
    ESP_RETURN_ON_ERROR(esp_netif_attach_wifi_station(n), TAG, "wifi sta attach");
    ESP_RETURN_ON_ERROR(esp_wifi_set_default_wifi_sta_handlers(), TAG, "wifi sta handlers");
    drv = esp_netif_get_io_driver(n);
    ESP_RETURN_ON_ERROR(esp_wifi_register_if_rxcb(drv, esp_netif_receive, n), TAG, "wifi sta rxcb");
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_netif_set_mac(n, mac);
    esp_netif_action_start(n, NULL, 0, NULL);
    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    s_sta = n;
    s_sta_wifi = true;
    xSemaphoreGive(s_netif_lock);
    if (connected)
        esp_netif_action_connected(n, NULL, 0, NULL);     /* the DHCP client on the router */
    return ESP_OK;
}

static void root_sta_stop(void)
{
    esp_netif_t *n;

    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    n = (s_sta && s_sta_wifi) ? s_sta : NULL;
    if (n) {
        s_sta = NULL;
        s_sta_wifi = false;
    }
    xSemaphoreGive(s_netif_lock);
    if (!n)
        return;
    esp_netif_action_disconnected(n, NULL, 0, NULL);
    esp_wifi_clear_default_wifi_driver_and_handlers(n);
    esp_netif_destroy(n);
}

/* node: the mesh station netif, once a parent is there */
static esp_err_t node_sta_start(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config_t cfg = { .base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA };
    esp_netif_t *n;
    mesh_drv_t *d;
    uint8_t mac[6];

    if (s_sta && !s_sta_wifi)
        return ESP_OK;
    root_sta_stop();            /* a sensor that started out ready to be the root and found a parent instead */
    base.if_desc = "mesh_link_sta";
    n = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(n, ESP_FAIL, TAG, "mesh sta netif");
    d = mesh_driver_new(false);
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "mesh sta driver");
    ESP_RETURN_ON_ERROR(esp_netif_attach(n, d), TAG, "mesh sta attach");
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_netif_set_mac(n, mac);
    esp_netif_action_start(n, NULL, 0, NULL);
    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    s_sta = n;
    s_sta_wifi = false;
    xSemaphoreGive(s_netif_lock);
    esp_netif_action_connected(n, NULL, 0, NULL);     /* DHCP client over the mesh */
    ESP_LOGI(TAG, "node: mesh station netif up, asking the root for an address");
    return ESP_OK;
}

static void node_sta_stop(void)
{
    esp_netif_t *n;
    void *d;

    xSemaphoreTake(s_netif_lock, portMAX_DELAY);
    n = (s_sta && !s_sta_wifi) ? s_sta : NULL;
    if (n)
        s_sta = NULL;
    xSemaphoreGive(s_netif_lock);
    if (!n)
        return;
    esp_netif_action_disconnected(n, NULL, 0, NULL);
    d = esp_netif_get_io_driver(n);
    esp_netif_destroy(n);
    free(d);
    s_mesh_ip4 = 0;
}

/* ── events ───────────────────────────────────────────────────────────── */

/* ── the root's uplink ────────────────────────────────────────────────── */

/* The root holds an address on its router. The Linux side becomes reachable
 * from the router's network: the router's address belongs to this station,
 * nothing listens on the ESP, so a connection arriving there on the UIs'
 * ports (the VoyagerLite on 80, the sensor UI on 8080) or ssh is forwarded
 * through the NAT to Linux; keyed by port, a new address from the router
 * replaces the entry. A sensor that has just become the root starts serving
 * the nodes here (their DHCP server, with the router's DNS for them). */
static const uint16_t s_fwd_ports[] = { 22, 80, 8080 };

static void root_uplink_ready(void)
{
    for (size_t i = 0; i < sizeof(s_fwd_ports) / sizeof(s_fwd_ports[0]); i++)
        if (!ip_portmap_add(IP_PROTO_TCP, s_router_ip4, s_fwd_ports[i], s_link_ip.gw.addr, s_fwd_ports[i]))
            ESP_LOGW(TAG, "root: no room in the NAT port map for port %u", s_fwd_ports[i]);
    root_ap_start(s_router_dns ? s_router_dns : s_link_ip.gw.addr);
    esp_mesh_post_toDS_state(true);
}

/* the router is gone, or the unit is the root no longer */
static void root_uplink_lost(void)
{
    if (!s_router_ip4)
        return;
    for (size_t i = 0; i < sizeof(s_fwd_ports) / sizeof(s_fwd_ports[0]); i++)
        ip_portmap_remove(IP_PROTO_TCP, s_fwd_ports[i]);
    s_router_ip4 = 0;
}

/* A sensor with a router changes sides with the group's election, as the H743
 * sensors do; called whenever a parent is connected, which for the root is
 * the router. The master's root never comes here. */
static void role_set(bool root)
{
    if (root) {
        if (!s_root) {
            ESP_LOGW(TAG, "election: this unit is the group's root now");
            node_sta_stop();
            s_root = true;
        }
        /* with the netif there since the start the default handlers have seen the
         * association and the DHCP client runs; made now, it is told */
        root_sta_start(true);
        if (s_router_ip4)
            root_uplink_ready();
        if (s_route_task)
            xTaskNotifyGive(s_route_task);
    } else {
        if (s_root) {
            ESP_LOGW(TAG, "election: this unit is a node now");
            s_root = false;
            root_uplink_lost();
            root_ap_stop();
        }
        node_sta_start();
    }
}

/* ── the master's root and its router ─────────────────────────────────────
 * The router has the priority (W11): it is looked for the whole time, with
 * children and a phone on the AP as well. The mesh stack does not take a
 * fixed root to its router by itself (bench 2026-11-03), and with
 * self-organized networking on it hunts the router every 3 s with the radio
 * off the mesh AP each time (2026-11-05: the nodes lost one packet in three),
 * so the root stays in manual networking and this task does the looking, one
 * channel at a time: about 120 ms off the AP's channel every four seconds,
 * the channel the router was last seen on at every pass and one more of the
 * thirteen in turn (the first sweep runs through at once: nobody is on the AP
 * yet). Seen: the router is named the parent, as the IDF example
 * manual_networking does; on another channel than the mesh's the mesh moves
 * there first (one radio: a root cannot hold its group on one channel and its
 * router on another). Lost again (the parent-disconnected event): the root
 * carries the group on without it, as a root that never had one does, and
 * the looking goes on. */
static TaskHandle_t s_router_task;
static volatile bool s_router_joined;       /* the root's station is on the router */
static volatile bool s_router_drop;         /* the stack reported the parent lost and is trying it again by itself */
static volatile bool s_router_joining;      /* this task named the router the parent and waits for the outcome */
static volatile bool s_router_ever;         /* the root has been on the router since the mesh started */
static uint8_t s_router_chan;               /* where the router was last seen */

static bool router_scan(uint8_t chan, wifi_ap_record_t *best)
{
    wifi_scan_config_t scan = { .ssid = (uint8_t *)s_cfg.router_ssid, .channel = chan, .show_hidden = true,
                                .scan_type = WIFI_SCAN_TYPE_ACTIVE, .scan_time.active = { .min = 60, .max = 120 } };
    wifi_ap_record_t *recs;
    uint16_t n = 0;
    bool found = false;

    if (esp_wifi_scan_start(&scan, true) != ESP_OK)
        return false;
    esp_wifi_scan_get_ap_num(&n);
    if (n) {
        recs = calloc(n, sizeof(*recs));
        if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
            for (int i = 0; i < n; i++)
                if (!strcmp((char *)recs[i].ssid, s_cfg.router_ssid) && (!found || recs[i].rssi > best->rssi)) {
                    *best = recs[i];
                    found = true;
                }
        }
        free(recs);
    }
    esp_mesh_flush_scan_result();
    return found;
}

static void root_router_task(void *arg)
{
    uint8_t next = 0, scans = 0;
    bool said = false;

    while (s_active && s_fixed_root && s_router) {
        wifi_ap_record_t ap;
        bool found = false;

        if (s_router_joined) {                  /* on the router: nothing to look for */
            said = false;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
            continue;
        }
        if (s_router_drop) {
            /* The router went away. Left alone the mesh stack tries the parent it
             * had again every three seconds, for ever ("disconnected reason:201,
             * continuous:13/max:12"), the radio off the group's channel each time:
             * 25 s after the router was switched off one child answered 2 pings of
             * 5, after 4 to 5 seconds, a phone on the AP lost one ping in four, and
             * the scans below were refused ("STA is connecting"), bench 2026-11-12.
             * So the root lets go of the parent and this task alone looks for the
             * router. esp_mesh_disconnect() alone does that only when it comes
             * before the stack has started its next try ("parent is set<stop
             * reconnect>"); a moment later it changes nothing and the tries go on
             * (two runs of three). Disabling self-organized networking once more is
             * what the stack itself logs as "<stop reconnect>", so both are called,
             * for every loss the stack reports while this task is not itself
             * joining. */
            s_router_drop = false;
            esp_mesh_disconnect();
            esp_mesh_set_self_organized(false, false);
            if (!said && s_router_ever) {
                ESP_LOGW(TAG, "root: router \"%s\" lost: the root carries the group without it and keeps looking", s_cfg.router_ssid);
                said = true;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        if (s_router_chan)
            found = router_scan(s_router_chan, &ap);
        /* two more of the thirteen at every pass: a sweep in half a minute. With
         * one a router on a new channel took 112 s to be seen (one 120 ms look at
         * its channel every 52 s, and the first look missed it), 2026-11-12 */
        for (int k = 0; k < 2 && !found; k++) {
            next = next % 13 + 1;
            if (next == s_router_chan)
                next = next % 13 + 1;
            found = router_scan(next, &ap);
            if (scans < 13)
                scans++;
        }
        if (found) {
            wifi_config_t parent = { 0 };
            uint8_t cur = 0;
            wifi_second_chan_t sec;

            s_router_chan = ap.primary;
            /* The router on another channel than the group's: the root's station
             * goes there and its AP with it (one radio), so the whole group moves;
             * the mesh is configured to allow that (allow_channel_switch), and the
             * children, which look on every channel, find the root again. Stopping
             * the mesh to start it on the new channel is no way: esp_mesh_stop()
             * leaves the stack de-initialized ("mesh is deinited", 2026-11-12). */
            esp_wifi_get_channel(&cur, &sec);
            if (cur != ap.primary)
                ESP_LOGW(TAG, "root: router \"%s\" is on channel %u, the group on %u: the group moves with the root",
                         s_cfg.router_ssid, ap.primary, cur);
            parent.sta.channel = ap.primary;
            memcpy(parent.sta.ssid, ap.ssid, sizeof(parent.sta.ssid));
            parent.sta.bssid_set = 1;
            memcpy(parent.sta.bssid, ap.bssid, 6);
            if (ap.authmode != WIFI_AUTH_OPEN)
                memcpy(parent.sta.password, s_cfg.router_password,
                       strlen(s_cfg.router_password) > sizeof(parent.sta.password) ?
                       sizeof(parent.sta.password) : strlen(s_cfg.router_password));
            ESP_LOGI(TAG, "root: router %s " MACSTR " channel %u rssi %d: naming it the parent",
                     ap.ssid, MAC2STR(ap.bssid), ap.primary, ap.rssi);
            s_router_joining = true;
            esp_err_t ret = esp_mesh_set_parent(&parent, NULL, MESH_ROOT, MESH_ROOT_LAYER);
            if (ret != ESP_OK)
                ESP_LOGW(TAG, "root: esp_mesh_set_parent: %s", esp_err_to_name(ret));
            for (int i = 0; i < 40 && !s_router_joined && s_active; i++)     /* the association: 20 s */
                vTaskDelay(pdMS_TO_TICKS(500));
            s_router_joining = false;
            if (!s_router_joined) {
                /* seen but not joined (its password?): not at every pass, the radio
                 * leaves the group for each try, and the stack's own retries are
                 * stopped for the same reason */
                esp_mesh_disconnect();
                esp_mesh_set_self_organized(false, false);
                ESP_LOGW(TAG, "root: router \"%s\" is there but did not let the root in; again in a minute", s_cfg.router_ssid);
                for (int i = 0; i < 60 && s_active && !s_router_joined; i++) {
                    if (s_router_drop) {                /* the stack still tries on its own */
                        s_router_drop = false;
                        esp_mesh_disconnect();
                        esp_mesh_set_self_organized(false, false);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
            said = false;
            continue;
        }
        if (!said && scans >= 13) {
            ESP_LOGW(TAG, "root: router \"%s\" is not in range: the root carries the group without it and keeps looking",
                     s_cfg.router_ssid);
            esp_mesh_post_toDS_state(true);     /* Linux is the DS meanwhile */
            said = true;
        }
        /* a loss reported by the stack wakes the task: its retries are stopped at once */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(scans < 13 ? 100 : 4000));
    }
    s_router_task = NULL;
    vTaskDelete(NULL);
}

static void mesh_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case MESH_EVENT_STARTED:
        s_layer = esp_mesh_get_layer();
        ESP_LOGI(TAG, "mesh started, %s", s_fixed_root ? "root" : s_router ? "sensor (root or node by the election)" : "node");
        if (s_fixed_root) {
            /* esp_mesh_start() re-enables self-organized networking: set
             * before it, the root still hunted its (placeholder) router
             * every 2.9 s, the radio off the mesh AP each time, and the
             * nodes lost one packet in three (bench 2026-11-05: 1 kB/s for
             * two sensors, pings to a node all lost). The manual-networking
             * example disables it here, in the started event. */
            esp_mesh_set_self_organized(false, false);
            esp_wifi_scan_stop();
            /* the master's nodes stream to an address (Linux, the link's host): that is
             * their DNS server too, with or without a router */
            root_ap_start(s_link_ip.gw.addr);
            esp_mesh_post_toDS_state(true);       /* Linux is the DS */
            if (s_router && !s_router_task)
                xTaskCreate(root_router_task, "xwave_root_router", 4096, NULL, 5, &s_router_task);
        }
        guest_ap_start();                         /* a phone on this unit's AP, root or node */
        break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *c = data;
        s_layer = c->self_layer;
        memcpy(s_parent.addr, c->connected.bssid, 6);
        s_parent_connected = true;
        ESP_LOGI(TAG, "parent connected, layer %u, parent " MACSTR "%s", s_layer, MAC2STR(s_parent.addr),
                 esp_mesh_is_root() ? " (root)" : "");
        if (s_fixed_root) {
            s_router_joined = true;               /* the master's root: its parent is the router */
            s_router_ever = true;
        } else {
            role_set(esp_mesh_is_root());         /* a sensor: the election has spoken */
            guest_ap_name();
        }
        break;
    }
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *d = data;
        s_parent_connected = false;
        s_layer = esp_mesh_get_layer();
        ESP_LOGW(TAG, "parent lost, reason %u", d->reason);
        if (s_root) {
            /* the router is gone. The master's root carries its group on and looks
             * for the router again (root_router_task); a sensor that was the root
             * leaves the rest to the mesh: the group votes, and the next parent
             * connected says what this unit is then */
            root_uplink_lost();
            if (s_fixed_root) {
                if (!s_router_joining)
                    s_router_drop = true;         /* root_router_task stops the stack's own retries */
                s_router_joined = false;
                esp_mesh_post_toDS_state(true);   /* Linux is the DS meanwhile */
                if (s_router_task)
                    xTaskNotifyGive(s_router_task);
            }
        } else {
            node_sta_stop();
        }
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
    case MESH_EVENT_ROUTING_TABLE_ADD:
        if (s_route_task)
            xTaskNotifyGive(s_route_task);     /* the new node gets the table at once */
        break;
    case MESH_EVENT_ROUTER_SWITCH:
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
    if (s_sta_wifi && e->esp_netif == s_sta) {
        /* an address from the router: this unit is the root, or is about to be
         * told so (a sensor's parent-connected event can come after this one;
         * role_set then finishes what is left) */
        esp_netif_dns_info_t dns;
        s_router_ip4 = e->ip_info.ip.addr;
        ESP_LOGI(TAG, "root: router gave " IPSTR, IP2STR(&e->ip_info.ip));
        esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN, &dns);
        s_router_dns = dns.ip.u_addr.ip4.addr;
        if (s_root)
            root_uplink_ready();
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
    s_fixed_root = req->mode == XWAVE_MESH_ROOT;
    s_root = s_fixed_root;                  /* a sensor starts as no root; the election may make it one */
    s_router = s_cfg.router_ssid[0] != 0;
    s_layer = 0;
    s_parent_connected = s_tods = false;
    s_router_ip4 = s_mesh_ip4 = s_router_dns = 0;
    s_router_joined = s_router_drop = s_router_joining = s_router_ever = false;
    if (!s_netif_lock)
        s_netif_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_netif_lock, ESP_ERR_NO_MEM, TAG, "netif lock");
    memset(&s_parent, 0, sizeof(s_parent));

    memset(&s_link_ip, 0, sizeof(s_link_ip));
    memcpy(&s_link_ip.ip.addr, req->link_ip, 4);
    memcpy(&s_link_ip.gw.addr, req->host_ip, 4);
    s_link_ip.netmask.addr = req->link_prefix >= 32 ? 0xffffffffu :
                             htonl(~((1u << (32 - (req->link_prefix ? req->link_prefix : 30))) - 1));

    s_opts = req->max_connection & 0xF0;
    if (s_opts & XWAVE_OPT_LOG)
        xlog_start(s_link_ip.gw.addr);
    if (s_opts)
        ESP_LOGW(TAG, "options 0x%02x:%s%s%s", s_opts,
                 s_opts & XWAVE_OPT_NO_GUEST ? " no plain station on the mesh AP" : "",
                 s_opts & XWAVE_OPT_STOCK_AP ? " the AP hidden under the mesh stack's name" : "",
                 s_opts & XWAVE_OPT_LOG ? " log to Linux" : "");

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

#if !CONFIG_XWAVE_MESH_IMAGE
    /* the hosted image's patched Wi-Fi libraries defer authentication to the
     * host: no station or AP of the mesh can authenticate (2026-11-04); the
     * mesh roles need the mesh image (xwave-build.ps1 mesh) */
    ESP_LOGE(TAG, "this is the hosted image: the mesh needs the mesh image");
    return ESP_ERR_NOT_SUPPORTED;
#endif
    ESP_RETURN_ON_ERROR(esp_mesh_init(), TAG, "mesh init");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(MESH_EVENT, ESP_EVENT_ANY_ID, mesh_event, NULL, &s_mesh_evt), TAG, "mesh events");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event, NULL, &s_ip_evt), TAG, "ip events");
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_ap_event, NULL, &s_wifi_evt);
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
        /* no router at the site. The stack refuses an empty router SSID
         * (ESP_ERR_MESH_ARGUMENT), so a placeholder stays; the root's station
         * then tries it every 3 s on the fixed channel only (no channel
         * switch, no full scan: the AP stays on `channel`). Linux is the DS
         * (toDS posted reachable). */
        static const char none[] = "xwave-no-router";
        cfg.router.ssid_len = sizeof(none) - 1;
        memcpy(cfg.router.ssid, none, sizeof(none) - 1);
        cfg.router.allow_router_switch = false;
    }
    /* The channel. The master's root opens the group itself and needs one (6
     * when none is given); it moves the group to its router's channel when it
     * finds the router on another (root_router_task). A sensor looks for its
     * group: 0 = on every channel, as the H743 sensors do (the group lives on
     * its router's channel); a given channel is where it starts, and after
     * the attempts there have failed it looks on all of them. */
    if (s_fixed_root) {
        if (!cfg.channel)
            cfg.channel = 6;
        cfg.allow_channel_switch = s_router;    /* to its router's channel, when that is another */
    } else {
        cfg.allow_channel_switch = true;
    }
    ESP_RETURN_ON_ERROR(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA2_PSK), TAG, "ap authmode");
    cfg.mesh_ap.max_connection = (req->max_connection & 0x0F) ? (req->max_connection & 0x0F) : 6;
    cfg.mesh_ap.nonmesh_max_connection = (s_opts & XWAVE_OPT_NO_GUEST) ? 0 : 1;   /* a phone on this unit's AP, root or node */
    n = strlen(s_cfg.mesh_password);
    memcpy(cfg.mesh_ap.password, s_cfg.mesh_password, n > sizeof(cfg.mesh_ap.password) - 1 ? sizeof(cfg.mesh_ap.password) - 1 : n);
    ESP_RETURN_ON_ERROR(esp_mesh_set_config(&cfg), TAG, "mesh config");

    /* the same fixed-root setting on every device of a mesh: the H743
     * sensors' mesh.bin runs without it, so a root for them is not fixed */
    ESP_RETURN_ON_ERROR(esp_mesh_fix_root((req->flags & XWAVE_MESH_FLAG_FIXED_ROOT) != 0), TAG, "fix root");
    if (s_fixed_root) {
        /* manual networking for the master's root either way: with a router it
         * is named as the parent after the start (root_router_task); without
         * one the root must not hunt for the placeholder SSID (a self-organized
         * root scanned every 3 s, its AP off-channel each time, 2026-11-05) */
        ESP_RETURN_ON_ERROR(esp_mesh_set_self_organized(false, false), TAG, "manual networking");
        ESP_RETURN_ON_ERROR(esp_mesh_set_type(MESH_ROOT), TAG, "set root");
    }
    /* A unit that may come to hold the router (the master's root; a sensor,
     * should it win the election) gets the station netif before the mesh
     * starts, as the IDF example does: the default handlers then see the
     * association and start the DHCP client by themselves. A sensor that
     * finds a parent instead swaps it for the mesh station (node_sta_start).
     * A sensor is otherwise left to the mesh: self-organized networking, the
     * H743 sensors' way. */
    if (s_router)
        ESP_RETURN_ON_ERROR(root_sta_start(false), TAG, "router station");
    ESP_RETURN_ON_ERROR(link_start(), TAG, "link");
    s_active = true;
    ESP_RETURN_ON_ERROR(esp_mesh_start(), TAG, "mesh start");
    if (!s_route_task)
        xTaskCreate(route_table_task, "xwave_mesh_rt", 2048, NULL, 5, &s_route_task);
    ESP_LOGI(TAG, "mesh %s started: id %02x%02x%02x%02x%02x%02x, channel %u, %s, heap %lu",
             s_fixed_root ? "root" : s_router ? "sensor" : "node", req->mesh_id[0], req->mesh_id[1], req->mesh_id[2],
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
    if (s_parent_connected) {
        /* the station link's signal: a node's parent, a root's router (the sensors' heartbeat
         * carries it as M_Signal and Voyager draws it on the mesh tree's edges). Asked only
         * with a link: without one the Wi-Fi library answers each question with a warning
         * in the log ("Haven't to connect to a suitable AP now!"), and Linux asks often */
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            st->rssi = ap.rssi;
    }
    esp_wifi_get_mac(WIFI_IF_AP, st->self);     /* the mesh AP a child connects to: its parent's BSSID */
    if (s_root && esp_mesh_get_routing_table(s_route, ROUTE_TABLE_MAX * 6, &n) == ESP_OK) {
        st->route_table_size = n;
        st->children = n > 0 ? n - 1 : 0;
    }
}
