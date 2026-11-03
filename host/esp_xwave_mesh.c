/* xWave: mesh0, the Ethernet link to the ESP's own TCP/IP stack while the ESP
 * runs ESP-WIFI-MESH, and the debugfs control of that mode
 * (docs/WiFi-Plan.md 7.1, firmware side in network_adapter/main/xwave_mesh.c).
 *
 *   /sys/kernel/debug/esp32/xwave_mesh
 *     write: key=value settings separated by spaces, one line:
 *            mode=off|node|root|query  id=<12 hex>  pw=<mesh password>
 *            channel=<1-13>  maxconn=<n>  maxlayer=<n>
 *            link=<ESP address>  host=<Linux address>  prefix=<30>
 *            router=<SSID>  rpw=<router password>
 *            mode=off restarts the ESP into the hosted service; the driver
 *            recovers the link by itself.
 *     read:  the status the ESP answered last (a query is sent first).
 */
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/debugfs.h>
#include <linux/inet.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include "utils.h"
#include "esp_api.h"
#include "esp_if.h"
#include "esp_cmd.h"
#include "esp_kernel_port.h"
#include "esp_xwave_mesh.h"

struct mesh_ndev_priv {
	struct esp_adapter *adapter;
	struct net_device_stats stats;
};

static struct net_device *s_mesh_ndev;
static struct xwave_mesh_status s_status;
static DEFINE_MUTEX(s_lock);

static int mesh_open(struct net_device *ndev)
{
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int mesh_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	return 0;
}

static struct net_device_stats *mesh_get_stats(struct net_device *ndev)
{
	struct mesh_ndev_priv *p = netdev_priv(ndev);
	return &p->stats;
}

static netdev_tx_t mesh_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct mesh_ndev_priv *p = netdev_priv(ndev);
	struct esp_adapter *adapter = p->adapter;
	struct esp_payload_header *hdr;
	struct sk_buff *out;
	size_t total;
	u8 pad;

	if (!skb)
		return NETDEV_TX_OK;
	if (!adapter || !skb->len || skb->len > ETH_FRAME_LEN) {
		p->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	pad = sizeof(*hdr);
	total = skb->len + sizeof(*hdr);
	pad += (SKB_DATA_ADDR_ALIGNMENT - (total % SKB_DATA_ADDR_ALIGNMENT)) % SKB_DATA_ADDR_ALIGNMENT;
	out = esp_if_alloc_skb(adapter, skb->len + pad);
	if (!out) {
		p->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	skb_put(out, skb->len + pad);
	memset(out->data, 0, pad);
	memcpy(out->data + pad, skb->data, skb->len);
	hdr = (struct esp_payload_header *)out->data;
	hdr->if_type = ESP_MESH_IF;
	hdr->if_num = 0;
	hdr->len = esp_wire_cpu_to_le16(skb->len);
	hdr->offset = esp_wire_cpu_to_le16(pad);
	hdr->packet_type = PACKET_TYPE_DATA;
	if (adapter->capabilities & ESP_CHECKSUM_ENABLED)
		hdr->checksum = esp_wire_cpu_to_le16(compute_checksum(out->data, skb->len + pad));
	if (esp_send_packet(adapter, out)) {
		p->stats.tx_dropped++;
	} else {
		p->stats.tx_packets++;
		p->stats.tx_bytes += skb->len;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mesh_ndev_ops = {
	.ndo_open = mesh_open,
	.ndo_stop = mesh_stop,
	.ndo_start_xmit = mesh_xmit,
	.ndo_get_stats = mesh_get_stats,
	.ndo_validate_addr = eth_validate_addr,
};

void esp_xwave_mesh_rx(struct esp_adapter *adapter, struct sk_buff *skb)
{
	struct net_device *ndev = s_mesh_ndev;
	struct mesh_ndev_priv *p;

	if (!ndev || !netif_running(ndev) || skb->len < ETH_HLEN) {
		dev_kfree_skb_any(skb);
		return;
	}
	p = netdev_priv(ndev);
	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);
	skb->ip_summed = CHECKSUM_NONE;
	p->stats.rx_packets++;
	p->stats.rx_bytes += skb->len;
	netif_rx(skb);
}

int esp_xwave_mesh_init(struct esp_adapter *adapter)
{
	struct net_device *ndev;
	struct mesh_ndev_priv *p;
	int ret;

	if (s_mesh_ndev) {
		p = netdev_priv(s_mesh_ndev);
		p->adapter = adapter;
		return 0;
	}
	ndev = alloc_netdev(sizeof(*p), "mesh0", NET_NAME_UNKNOWN, ether_setup);
	if (!ndev)
		return -ENOMEM;
	p = netdev_priv(ndev);
	p->adapter = adapter;
	ndev->netdev_ops = &mesh_ndev_ops;
	ndev->needed_headroom = roundup(sizeof(struct esp_payload_header) + INTERFACE_HEADER_PADDING, 4);
	ndev->mtu = ETH_DATA_LEN;
	eth_hw_addr_random(ndev);
	ndev->priv_flags |= IFF_NO_QUEUE;
	ret = register_netdev(ndev);
	if (ret) {
		free_netdev(ndev);
		return ret;
	}
	netif_carrier_off(ndev);
	s_mesh_ndev = ndev;
	esp_info("xwave mesh0 registered\n");
	return 0;
}

void esp_xwave_mesh_deinit(struct esp_adapter *adapter)
{
	struct net_device *ndev = s_mesh_ndev;

	if (!ndev)
		return;
	s_mesh_ndev = NULL;
	unregister_netdev(ndev);
	free_netdev(ndev);
}

/* ── debugfs control ──────────────────────────────────────────────────── */

static int parse_ip(const char *s, u8 *out)
{
	const char *end;
	__be32 a;

	if (!in4_pton(s, -1, (u8 *)&a, -1, &end))
		return -EINVAL;
	memcpy(out, &a, 4);
	return 0;
}

static int parse_hex_id(const char *s, u8 *id)
{
	int i;

	if (strlen(s) != 2 * XWAVE_MESH_ID_LEN)
		return -EINVAL;
	for (i = 0; i < XWAVE_MESH_ID_LEN; i++) {
		int hi = hex_to_bin(s[2 * i]), lo = hex_to_bin(s[2 * i + 1]);
		if (hi < 0 || lo < 0)
			return -EINVAL;
		id[i] = (hi << 4) | lo;
	}
	return 0;
}

static ssize_t mesh_debugfs_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct esp_adapter *adapter = esp_get_adapter();
	struct cmd_xwave_mesh req;
	char *buf, *cur, *tok;
	int ret = 0;

	if (!adapter)
		return -ENODEV;
	if (count == 0 || count > 512)
		return -EINVAL;
	buf = kzalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, ubuf, count)) {
		kfree(buf);
		return -EFAULT;
	}
	memset(&req, 0, sizeof(req));
	req.mode = XWAVE_MESH_QUERY;
	req.max_connection = 6;
	req.max_layer = 6;
	req.link_prefix = 30;
	cur = buf;
	while ((tok = strsep(&cur, " \t\r\n")) != NULL) {
		char *val;
		unsigned int n;

		if (!*tok)
			continue;
		val = strchr(tok, '=');
		if (!val) {
			ret = -EINVAL;
			break;
		}
		*val++ = '\0';
		if (!strcmp(tok, "mode")) {
			if (!strcmp(val, "off")) req.mode = XWAVE_MESH_OFF;
			else if (!strcmp(val, "node")) req.mode = XWAVE_MESH_NODE;
			else if (!strcmp(val, "root")) req.mode = XWAVE_MESH_ROOT;
			else if (!strcmp(val, "query")) req.mode = XWAVE_MESH_QUERY;
			else ret = -EINVAL;
		} else if (!strcmp(tok, "id")) {
			ret = parse_hex_id(val, req.mesh_id);
		} else if (!strcmp(tok, "pw")) {
			strscpy(req.mesh_password, val, sizeof(req.mesh_password));
		} else if (!strcmp(tok, "router")) {
			strscpy(req.router_ssid, val, sizeof(req.router_ssid));
		} else if (!strcmp(tok, "rpw")) {
			strscpy(req.router_password, val, sizeof(req.router_password));
		} else if (!strcmp(tok, "channel")) {
			ret = kstrtouint(val, 10, &n); if (!ret) req.channel = n;
		} else if (!strcmp(tok, "maxconn")) {
			ret = kstrtouint(val, 10, &n); if (!ret) req.max_connection = n;
		} else if (!strcmp(tok, "maxlayer")) {
			ret = kstrtouint(val, 10, &n); if (!ret) req.max_layer = n;
		} else if (!strcmp(tok, "prefix")) {
			ret = kstrtouint(val, 10, &n); if (!ret) req.link_prefix = n;
		} else if (!strcmp(tok, "link")) {
			ret = parse_ip(val, req.link_ip);
		} else if (!strcmp(tok, "host")) {
			ret = parse_ip(val, req.host_ip);
		} else {
			esp_err("xwave_mesh: unknown setting '%s'\n", tok);
			ret = -EINVAL;
		}
		if (ret)
			break;
	}
	kfree(buf);
	if (ret)
		return ret;
	if ((req.mode == XWAVE_MESH_NODE || req.mode == XWAVE_MESH_ROOT) &&
	    (!req.link_ip[0] || !req.host_ip[0])) {
		esp_err("xwave_mesh: node/root need link= and host= addresses\n");
		return -EINVAL;
	}
	mutex_lock(&s_lock);
	ret = cmd_xwave_mesh(adapter, &req, &s_status);
	mutex_unlock(&s_lock);
	if (ret) {
		esp_err("xwave_mesh: command failed (%d)\n", ret);
		return ret < 0 ? ret : -EIO;
	}
	if (s_mesh_ndev) {
		if (req.mode == XWAVE_MESH_NODE || req.mode == XWAVE_MESH_ROOT)
			netif_carrier_on(s_mesh_ndev);
		else if (req.mode == XWAVE_MESH_OFF)
			netif_carrier_off(s_mesh_ndev);
	}
	return count;
}

static ssize_t mesh_debugfs_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	struct esp_adapter *adapter = esp_get_adapter();
	struct cmd_xwave_mesh req;
	struct xwave_mesh_status st;
	char out[320];
	int n, ret;

	if (*ppos)
		return 0;
	if (!adapter)
		return -ENODEV;
	memset(&req, 0, sizeof(req));
	req.mode = XWAVE_MESH_QUERY;
	mutex_lock(&s_lock);
	ret = cmd_xwave_mesh(adapter, &req, &s_status);
	st = s_status;
	mutex_unlock(&s_lock);
	if (ret)
		n = scnprintf(out, sizeof(out), "mode=unknown error=%d\n", ret);
	else
		n = scnprintf(out, sizeof(out),
			      "mode=%s root=%u layer=%u parent_connected=%u parent=%pM mesh_ip=%pI4 router_ip=%pI4 children=%u route_table=%u tods=%u free_heap=%u\n",
			      st.mode == XWAVE_MESH_OFF ? "off" : st.mode == XWAVE_MESH_NODE ? "node" :
			      st.mode == XWAVE_MESH_ROOT ? "root" : "?",
			      st.is_root, st.layer, st.parent_connected, st.parent, st.mesh_ip, st.router_ip,
			      st.children, le16_to_cpu(st.route_table_size), st.tods_reachable,
			      le32_to_cpu(st.free_heap));
	return simple_read_from_buffer(ubuf, count, ppos, out, n);
}

const struct file_operations esp_xwave_mesh_debugfs_ops = {
	.owner = THIS_MODULE,
	.read = mesh_debugfs_read,
	.write = mesh_debugfs_write,
	.llseek = default_llseek,
};
