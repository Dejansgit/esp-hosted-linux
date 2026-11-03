/* xWave: the mesh0 network device and the mesh-mode control of the ESP
 * (docs/WiFi-Plan.md 7.1). The ESP runs ESP-WIFI-MESH with its own TCP/IP
 * stack; mesh0 is the Ethernet link between Linux and that stack, carried
 * over the hosted transport as ESP_MESH_IF. Control: the debugfs file
 * esp32/xwave_mesh (write key=value settings, read the status), used by
 * xwave-wifi on the Edge. */
#ifndef _ESP_XWAVE_MESH_H_
#define _ESP_XWAVE_MESH_H_

#include <linux/fs.h>
#include "esp.h"

int esp_xwave_mesh_init(struct esp_adapter *adapter);
void esp_xwave_mesh_deinit(struct esp_adapter *adapter);
/* a frame from the ESP (skb->data at the Ethernet frame) */
void esp_xwave_mesh_rx(struct esp_adapter *adapter, struct sk_buff *skb);

extern const struct file_operations esp_xwave_mesh_debugfs_ops;

#endif
