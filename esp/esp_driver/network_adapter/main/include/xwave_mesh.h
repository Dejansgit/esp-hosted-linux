/* xWave: ESP-WIFI-MESH mode of the hosted firmware (docs/WiFi-Plan.md 7.1).
 *
 * In mesh mode the ESP runs the mesh stack with its own TCP/IP stack and NAT;
 * Linux sees one Ethernet link (ESP_MESH_IF -> mesh0) with a /30 between the
 * ESP (link_ip) and Linux (host_ip). As a node the ESP joins the group and
 * Linux's traffic leaves through the node's mesh address. As a root the ESP
 * owns the group; Linux is its upstream when there is no router, and the mesh
 * subnet (10.0.0.0/16, the root's AP) is reachable from Linux through the link.
 * The mesh stack takes the radio: the hosted station/AP service is off until
 * XWAVE_MESH_OFF, and the hosted Wi-Fi event handler ignores the mesh's
 * events while xwave_mesh_active() is true.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "adapter.h"

/* Apply a CMD_XWAVE_MESH request: OFF stops the mesh, NODE/ROOT (re)start it
 * with the request's parameters, QUERY changes nothing. */
esp_err_t xwave_mesh_apply(const struct cmd_xwave_mesh *req);

/* The status answered to every CMD_XWAVE_MESH. */
void xwave_mesh_get_status(struct xwave_mesh_status *st);

/* Mesh mode in force (node or root), even while not yet connected. */
bool xwave_mesh_active(void);

/* An Ethernet frame from Linux (mesh0) into the ESP's TCP/IP stack. The
 * buffer belongs to the caller; the stack copies it (LWIP_L2_TO_L3_COPY). */
esp_err_t xwave_mesh_rx_from_host(const uint8_t *frame, uint16_t len);
