/* xWave mesh image: the hosted command handlers in cmd.c call private
 * 802.11 entry points that only Espressif's patched Wi-Fi archives export
 * (host-driven authentication, association, management frames, AP station
 * table, raw frame buffers). The mesh image links ESP-IDF's stock archives,
 * where the ESP does all of that itself and nothing on the host side drives
 * them (Linux talks to the mesh through mesh0 only), so the handlers are
 * dead code there. These stubs keep them linkable; every one fails loudly
 * if it is ever reached.
 */
#include "sdkconfig.h"
#if CONFIG_XWAVE_MESH_IMAGE

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi_types.h"

static const char *TAG = "XWAVE_MESH";

#define STUB() ESP_LOGE(TAG, "%s: hosted-only entry point called in the mesh image", __func__)

uint32_t esp_wifi_get_eb_data_len(void *eb_buf)
{
    (void)eb_buf;
    STUB();
    return 0;
}

uint8_t *esp_wifi_get_eb_data(void *eb_buf)
{
    (void)eb_buf;
    STUB();
    return NULL;
}

int ieee80211_add_node(wifi_interface_t interface, uint8_t *mac, uint16_t aid, uint8_t *rates,
                       uint8_t *htcap, uint8_t *vhtcap, uint8_t *hecap, bool is_pmf)
{
    (void)interface; (void)mac; (void)aid; (void)rates; (void)htcap; (void)vhtcap; (void)hecap; (void)is_pmf;
    STUB();
    return -1;
}

int ieee80211_delete_node(uint8_t *mac)
{
    (void)mac;
    STUB();
    return -1;
}

int ieee80211_send_mgmt_internal(wifi_interface_t interface, uint8_t *buf, size_t len)
{
    (void)interface; (void)buf; (void)len;
    STUB();
    return -1;
}

esp_err_t esp_wifi_send_auth_internal(uint8_t ifx, uint8_t *bssid, uint8_t algo, uint8_t seq,
                                      uint32_t status)
{
    (void)ifx; (void)bssid; (void)algo; (void)seq; (void)status;
    STUB();
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_wifi_send_assoc_internal(uint8_t ifx, uint8_t *bssid, uint8_t type, uint8_t status)
{
    (void)ifx; (void)bssid; (void)type; (void)status;
    STUB();
    return ESP_ERR_NOT_SUPPORTED;
}

#endif /* CONFIG_XWAVE_MESH_IMAGE */
