#include "esp_xwave_mesh.h"
#include "utils.h"
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#ifdef ESP_SPI_HD_DEBUGFS
#include "esp_spi_hd.h"
#define SPI_HD_WORK_QUOTAS "spi_hd_work_quotas"
#endif

#define DEBUGFS_DIR_NAME "esp32"
#define LOG_LEVEL "log_level"
#define VERSION "version"

#define DEBUGFS_TODO 0

#if DEBUGFS_TODO
#define DEBUGFS_LOG_LEVEL "debugfs_log_level"
#define HOST_LOGS "logs"
#define FW_LOGS "fw_logs"
#define FW_LOGS_LEVEL "fw_logs_level"
#endif

#define LOG_BUFFER_SIZE 2048

struct esp32_debugfs {
	struct dentry *debugfs_dir;
	struct dentry *log_level_file; /* log level for host dmesg */
	struct dentry *version;
#ifdef ESP_SPI_HD_DEBUGFS
	struct dentry *spi_hd_work_quotas_file;
#endif
#if DEBUGFS_TODO
	struct dentry *host_log_level_file; /* log level for host logs in debugfs logger */
	struct dentry *host_log_file; /* debugfs host logger */

	struct dentry *fw_log_level_file; /* debugfs firmware log level */
	struct dentry *fw_log_file; /* debugfs firmware logger */
#endif
};
static struct esp32_debugfs drv_debugfs;


#if DEBUGFS_TODO
int debugfs_log_level;

static char log_buffer[LOG_BUFFER_SIZE] = "";
static size_t log_length = 0;
static size_t write_pos = 0;
#endif

// Read operation for the debugfs file
static ssize_t log_level_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	// Convert the log level integer to a string
	char level_str[32];
	snprintf(level_str, sizeof(level_str), "%d\n", log_level);

	// Copy the string to userspace
	return simple_read_from_buffer(buf, count, ppos, level_str, strlen(level_str));
}

static ssize_t version_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, version_str, strlen(version_str));
}

// Write operation for the debugfs file
static ssize_t log_level_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char level_str[32];

	// Read the user input
	if (count >= sizeof(level_str))
		return -EINVAL;

	if (copy_from_user(level_str, buf, count))
		return -EFAULT;

	level_str[count] = '\0';

	// Convert the input string to an integer
	if (kstrtoint(level_str, 10, &log_level))
		return -EINVAL;

	esp_err("Updated log level to %d\n", log_level);

	return count;
}

#if DEBUGFS_TODO
// Read operation for the debugfs file
static ssize_t debugfs_log_level_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	// Convert the log level integer to a string
	char level_str[32];
	snprintf(level_str, sizeof(level_str), "%d\n", debugfs_log_level);

	// Copy the string to userspace
	return simple_read_from_buffer(buf, count, ppos, level_str, strlen(level_str));
}

// Write operation for the debugfs file
static ssize_t debugfs_log_level_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char level_str[32];

	// Read the user input
	if (count >= sizeof(level_str))
		return -EINVAL;

	if (copy_from_user(level_str, buf, count))
		return -EFAULT;

	level_str[count] = '\0';

	// Convert the input string to an integer
	if (kstrtoint(level_str, 10, &debugfs_log_level))
		return -EINVAL;

	esp_err("Updated debugfs log level to %d\n", debugfs_log_level);

	return count;
}

// Read operation for the log output file
static ssize_t log_output_read(struct file *file, char __user *user_buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	if (log_length > 0) {
		ret = simple_read_from_buffer(user_buffer, count, ppos, log_buffer, log_length);
		if (ret > 0) {
			// Update read position
			*ppos += ret;
			// Clear buffer if no more data is available
			if (*ppos >= log_length) {
				log_length = 0;
				write_pos = 0;
			}
		}
	}
	return ret;
}

static int va_format_to_char(char *buf, size_t size, const char *fmt, va_list args)
{
	return vsnprintf(buf, size, fmt, args);
}

void write_to_buffer(const char *fmt, ...)
{
	size_t space_left = LOG_BUFFER_SIZE - log_length;
	va_list args;
	int len;

	va_start(args, fmt);
	len = va_format_to_char(log_buffer + write_pos, LOG_BUFFER_SIZE - write_pos, fmt, args);
	va_end(args);

	if (len > space_left)
		len = space_left;

	if (len > 0) {
		write_pos += len;
		log_length += len;
		if (write_pos >= LOG_BUFFER_SIZE)
			write_pos = 0;  // Wrap around if buffer is full
	}
}

// File operations for the debugfs file
static const struct file_operations debugfs_log_level_ops = {
	.read = debugfs_log_level_read,
	.write = debugfs_log_level_write,
};

// File operations for the debugfs file
static const struct file_operations debugfs_log_output_ops = {
	.read = log_output_read,
};

// File operations for the debugfs file
static const struct file_operations debugfs_fw_log_level_ops = {
	.read = debugfs_fw_log_level_read,
	.write = debugfs_fw_log_level_write,
};

static const struct file_operations debugfs_fw_log_output_ops = {
	.read = log_output_read,
};
#endif

// File operations for the debugfs file
static const struct file_operations log_level_ops = {
	.read = log_level_read,
	.write = log_level_write,
};

static const struct file_operations version_ops = {
	.read = version_read,
};

#ifdef ESP_SPI_HD_DEBUGFS
/*
 * SPI-HD-only host arbitration: atomic pair update, not two uncoordinated
 * debugfs integers. A single ordered SPI worker observes the new pair at
 * the start of each receive/send pass, even under continuous load.
 *
 *   cat /sys/kernel/debug/esp32/spi_hd_work_quotas
 *   echo '3:2' > /sys/kernel/debug/esp32/spi_hd_work_quotas
 *   echo '2 2' > /sys/kernel/debug/esp32/spi_hd_work_quotas
 *
 * Debugfs is a development interface, not a stable production ABI.
 */
static ssize_t spi_hd_work_quotas_read(struct file *file, char __user *buf,
				      size_t count, loff_t *ppos)
{
	char result[32];
	unsigned int e2h, h2e;
	int n;

	esp_spi_hd_get_work_quotas(&e2h, &h2e);
	n = scnprintf(result, sizeof(result), "%u:%u\n", e2h, h2e);
	return simple_read_from_buffer(buf, count, ppos, result, n);
}

static ssize_t spi_hd_work_quotas_write(struct file *file,
				       const char __user *buf,
				       size_t count, loff_t *ppos)
{
	char input[32], extra;
	unsigned int e2h, h2e;
	int ret;

	if (!count || count >= sizeof(input) || *ppos)
		return -EINVAL;
	if (copy_from_user(input, buf, count))
		return -EFAULT;
	/* Counted debugfs input must not hide trailing text after NUL. */
	if (memchr(input, '\0', count))
		return -EINVAL;
	input[count] = '\0';

	/* Permit "E2H:H2E" or "E2H H2E"; reject trailing garbage. */
	if (sscanf(input, "%u:%u %c", &e2h, &h2e, &extra) != 2 &&
	    sscanf(input, "%u %u %c", &e2h, &h2e, &extra) != 2)
		return -EINVAL;

	ret = esp_spi_hd_set_work_quotas(e2h, h2e);
	if (ret)
		return ret;
	*ppos += count;
	esp_info("SPI-HD debugfs work quotas updated E2H=%u H2E=%u\n",
		 e2h, h2e);
	return count;
}

static const struct file_operations spi_hd_work_quotas_ops = {
	.owner = THIS_MODULE,
	.read = spi_hd_work_quotas_read,
	.write = spi_hd_work_quotas_write,
	.llseek = default_llseek,
};
#endif

// Module initialization function
int debugfs_init(void)
{
	struct esp32_debugfs *debugfs = &drv_debugfs;

	debugfs->debugfs_dir = debugfs_create_dir(DEBUGFS_DIR_NAME, NULL);
	if (IS_ERR_OR_NULL(debugfs->debugfs_dir)) {
		esp_warn("Failed to create debugfs %s directory\n", DEBUGFS_DIR_NAME);
		debugfs->debugfs_dir = NULL;
		return 0;
	}

	debugfs->log_level_file = debugfs_create_file(LOG_LEVEL, 0644, debugfs->debugfs_dir, NULL, &log_level_ops);
	if (IS_ERR_OR_NULL(debugfs->log_level_file)) {
		esp_warn("Failed to create debugfs %s file\n", LOG_LEVEL);
		debugfs->log_level_file = NULL;
		goto cleanup;
	}

	debugfs->version = debugfs_create_file(VERSION, 0644, debugfs->debugfs_dir, NULL, &version_ops);
	debugfs_create_file("xwave_mesh", 0600, debugfs->debugfs_dir, NULL, &esp_xwave_mesh_debugfs_ops);   /* xWave: mesh mode control */
	if (IS_ERR_OR_NULL(debugfs->version)) {
		esp_warn("Failed to create debugfs %s file\n", VERSION);
		debugfs->version = NULL;
		goto cleanup;
	}

#ifdef ESP_SPI_HD_DEBUGFS
	debugfs->spi_hd_work_quotas_file =
		debugfs_create_file(SPI_HD_WORK_QUOTAS, 0600,
				    debugfs->debugfs_dir, NULL,
				    &spi_hd_work_quotas_ops);
	if (IS_ERR_OR_NULL(debugfs->spi_hd_work_quotas_file)) {
		esp_warn("Failed to create SPI-HD work quota debugfs file\n");
		debugfs->spi_hd_work_quotas_file = NULL;
		goto cleanup;
	}
#endif

#if DEBUGFS_TODO
	debugfs->host_log_level_file = debugfs_create_file(DEBUGFS_LOG_LEVEL, 0644, debugfs_dir, NULL, &debugfs_log_level_ops);
	if (IS_ERR_OR_NULL(debugfs->host_log_level_file)) {
		esp_warn("Failed to create debugfs %s file\n", DEBUGFS_LOG_LEVEL);
		debugfs->host_log_level_file = NULL;
		goto cleanup;
	}

	debugfs->host_log_file = debugfs_create_file(HOST_LOGS, 0644, debugfs_dir, NULL, &debugfs_log_output_ops);
	if (IS_ERR_OR_NULL(debugfs->host_log_file)) {
		esp_warn("Failed to create debugfs %s file\n", HOST_LOGS);
		debugfs->host_log_file = NULL;
		goto cleanup;
	}

	debugfs->fw_log_file = debugfs_create_file(FW_LOGS, 0644, debugfs_dir, NULL, &debugfs_fw_log_output_ops);
	if (IS_ERR_OR_NULL(debugfs->fw_log_file)) {
		esp_warn("Failed to create debugfs %s file\n", FW_LOGS);
		debugfs->fw_log_file = NULL;
		goto cleanup;
	}

	debugfs->fw_log_level_file = debugfs_create_file(FW_LOGS_LEVEL, 0644, debugfs_dir, NULL, &debugfs_fw_log_output_ops);
	if (IS_ERR_OR_NULL(debugfs->fw_log_level_file)) {
		esp_warn("Failed to create debugfs %s file\n", FW_LOGS_LEVEL);
		debugfs->fw_log_level_file = NULL;
		goto cleanup;
	}

#endif
	return 0;
cleanup:
	debugfs_exit();
	return 0;
}

void debugfs_exit(void)
{
	struct esp32_debugfs *debugfs = &drv_debugfs;
#ifdef ESP_SPI_HD_DEBUGFS
	if (debugfs->spi_hd_work_quotas_file &&
	    !IS_ERR(debugfs->spi_hd_work_quotas_file)) {
		debugfs_remove(debugfs->spi_hd_work_quotas_file);
		debugfs->spi_hd_work_quotas_file = NULL;
	}
#endif
#if DEBUGFS_TODO
	// Remove debugfs file and directory
	if (debugfs->fw_log_file)
		debugfs_remove(debugfs->fw_log_file);
	if (debugfs->fw_log_level_file)
		debugfs_remove(debugfs->fw_log_level_file);
	if (debugfs->host_log_file)
		debugfs_remove(debugfs->host_log_file);
	if (debugfs->host_log_level_file)
		debugfs_remove(debugfs->host_log_level_file);
#endif
	if (debugfs->log_level_file && !IS_ERR(debugfs->log_level_file)) {
		debugfs_remove(debugfs->log_level_file);
		debugfs->log_level_file = NULL;
	}
	if (debugfs->version && !IS_ERR(debugfs->version)) {
		debugfs_remove(debugfs->version);
		debugfs->version = NULL;
	}
	if (debugfs->debugfs_dir && !IS_ERR(debugfs->debugfs_dir)) {
		debugfs_remove(debugfs->debugfs_dir);
		debugfs->debugfs_dir = NULL;
	}
}
