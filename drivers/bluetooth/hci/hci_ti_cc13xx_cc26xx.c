/*
 * Copyright (c) 2026 Ayush Singh <ayush@beagleboard.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * HCI driver for the TI BLE5-Stack running as Bluetooth LE controller on
 * TI SimpleLink CC13xx/CC26xx SoCs.
 */

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/kernel.h>

#include <ti_ble_zephyr.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_ti);

#define DT_DRV_COMPAT ti_cc13xx_cc26xx_bt_hci

/* The BLE5-Stack ROM keeps its data at the start of SRAM */
#define TI_BLE_ROM_RAM_END 0x20000FE0

BUILD_ASSERT(DT_REG_ADDR(DT_CHOSEN(zephyr_sram)) >= TI_BLE_ROM_RAM_END,
	     "SRAM used by the BLE ROM is not reserved, build with the ti-cc13x2x7-ble snippet");

static K_KERNEL_STACK_DEFINE(hci_ti_app_stack, CONFIG_BT_TI_CC13XX_CC26XX_APP_THREAD_STACK_SIZE);
static struct k_thread hci_ti_app_thread;

static K_KERNEL_STACK_DEFINE(hci_ti_rx_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread hci_ti_rx_thread;

static K_FIFO_DEFINE(hci_ti_rx_fifo);

static bool hci_ti_is_evt_discardable(const uint8_t *evt, uint16_t len)
{
	if ((len < 3U) || (evt[0] != BT_HCI_EVT_LE_META_EVENT)) {
		return false;
	}

	switch (evt[2]) {
	case BT_HCI_EVT_LE_ADVERTISING_REPORT:
		return true;
	case BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT:
		/* Multi-part reports must not be split up */
		return !IS_ENABLED(CONFIG_BT_EXT_ADV);
	default:
		return false;
	}
}

/* Called from the BLE5-Stack HCI transport thread */
static void hci_ti_rx(const uint8_t *pkt, uint16_t len)
{
	struct net_buf *buf;
	bool discardable;

	if (len < 2U) {
		return;
	}

	LOG_HEXDUMP_DBG(pkt, MIN(len, 16U), "rx");

	switch (pkt[0]) {
	case BT_HCI_H4_EVT:
		discardable = hci_ti_is_evt_discardable(&pkt[1], len - 1U);
		buf = bt_buf_get_evt(pkt[1], discardable, discardable ? K_NO_WAIT : K_FOREVER);
		break;
	case BT_HCI_H4_ACL:
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
		break;
	default:
		LOG_ERR("Unknown HCI packet type %u", pkt[0]);
		return;
	}

	if (buf == NULL) {
		LOG_DBG("Discarding event 0x%02x, no buffer", pkt[1]);
		return;
	}

	net_buf_add_mem(buf, &pkt[1], len - 1U);
	k_fifo_put(&hci_ti_rx_fifo, buf);
}

static void hci_ti_assert(uint8_t cause, uint8_t subcause)
{
	LOG_ERR("BLE5-Stack assert, cause 0x%02x subcause 0x%02x", cause, subcause);
}

static void hci_ti_app_thread_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ti_ble_app_run();
}

static void hci_ti_rx_thread_func(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct net_buf *buf = k_fifo_get(&hci_ti_rx_fifo, K_FOREVER);

		bt_hci_recv(dev, buf);
	}
}

/* Buffers from the host are already H:4 encoded */
static int hci_ti_send(const struct device *dev, struct net_buf *buf)
{
	int err;

	ARG_UNUSED(dev);

	LOG_HEXDUMP_DBG(buf->data, MIN(buf->len, 16U), "tx");

	err = ti_ble_send(buf->data, buf->len);
	if (err != 0) {
		LOG_ERR("Failed to send HCI packet (%d)", err);
		return -EIO;
	}

	net_buf_unref(buf);

	return 0;
}

static int hci_ti_open(const struct device *dev)
{
	k_tid_t tid;
	int err;

	tid = k_thread_create(&hci_ti_rx_thread, hci_ti_rx_stack,
			      K_KERNEL_STACK_SIZEOF(hci_ti_rx_stack), hci_ti_rx_thread_func,
			      (void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "ti_ble_rx");

	err = ti_ble_init(hci_ti_rx, hci_ti_assert);
	if (err != 0) {
		LOG_ERR("BLE5-Stack init failed (%d)", err);
		k_thread_abort(&hci_ti_rx_thread);
		return -EIO;
	}

	tid = k_thread_create(&hci_ti_app_thread, hci_ti_app_stack,
			      K_KERNEL_STACK_SIZEOF(hci_ti_app_stack), hci_ti_app_thread_func, NULL,
			      NULL, NULL, K_PRIO_COOP(CONFIG_BT_TI_CC13XX_CC26XX_APP_THREAD_PRIO),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "ti_ble_hci");

	return 0;
}

static DEVICE_API(bt_hci, hci_ti_api) = {
	.open = hci_ti_open,
	.send = hci_ti_send,
};

#define HCI_TI_DEVICE_INIT(inst)                                                                   \
	static struct bt_hci_driver_data hci_ti_data_##inst;                                       \
	static const struct bt_hci_driver_config hci_ti_config_##inst =                            \
		BT_DT_HCI_DRIVER_CONFIG_INST_GET(inst);                                            \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &hci_ti_data_##inst, &hci_ti_config_##inst,        \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &hci_ti_api)

/* The SoC has a single radio */
HCI_TI_DEVICE_INIT(0)
