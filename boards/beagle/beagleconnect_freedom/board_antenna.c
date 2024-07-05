/*
 * Copyright (c) 2021 Florin Stancu
 * Copyright (c) 2021 Jason Kridner, BeagleBoard.org Foundation
 * Copyright (c) 2024 Ayush Singh <ayush@beagleboard.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Implements the RF driver callback to configure the on-board antenna
 * switch.
 */

#define DT_DRV_COMPAT skyworks_sky13317

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>

#include <ti/drivers/rf/RF.h>
#include <driverlib/rom.h>
#include <driverlib/interrupt.h>

/* custom pinctrl states for the antenna mux */
#define PINCTRL_STATE_ANT_SUBG    1
#define PINCTRL_STATE_ANT_SUBG_PA 2

static int board_antenna_init(const struct device *dev);
static void board_cc13xx_rf_callback(RF_Handle client, RF_GlobalEvent events, void *arg);

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
	.hwiPriority = INT_PRI_LEVEL7,
	.swiPriority = 0,
	.xoscHfAlwaysNeeded = true,
	/* RF driver callback for custom antenna switching */
	.globalCallback = board_cc13xx_rf_callback,
	/* Subscribe to events */
	.globalEventMask = (RF_GlobalEventRadioSetup | RF_GlobalEventRadioPowerDown),
};

PINCTRL_DT_INST_DEFINE(0);
DEVICE_DT_INST_DEFINE(0, board_antenna_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_BOARD_ANTENNA_INIT_PRIO, NULL);

static const struct pinctrl_dev_config *ant_pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0);

/**
 * Antenna switch GPIO init routine.
 */
static int board_antenna_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* default pinctrl configuration: set all antenna mux control pins as GPIOs */
	pinctrl_apply_state(ant_pcfg, PINCTRL_STATE_DEFAULT);
	return 0;
}

/**
 * Custom TI RFCC26XX callback for switching the on-board antenna mux on radio setup.
 */
static void board_cc13xx_rf_callback(RF_Handle client, RF_GlobalEvent events, void *arg)
{
	bool sub1GHz = false;
	uint8_t loDivider = 0;

	if (events & RF_GlobalEventRadioSetup) {
		/* Decode the current PA configuration. */
		RF_TxPowerTable_PAType paType =
			(RF_TxPowerTable_PAType)RF_getTxPower(client).paType;
		/* Decode the generic argument as a setup command. */
		RF_RadioSetup *setupCommand = (RF_RadioSetup *)arg;

		switch (setupCommand->common.commandNo) {
		case (CMD_RADIO_SETUP):
		case (CMD_BLE5_RADIO_SETUP):
			loDivider = RF_LODIVIDER_MASK & setupCommand->common.loDivider;
			/* Sub-1GHz front-end. */
			if (loDivider != 0) {
				sub1GHz = true;
			}
			break;
		case (CMD_PROP_RADIO_DIV_SETUP):
			loDivider = RF_LODIVIDER_MASK & setupCommand->prop_div.loDivider;
			/* Sub-1GHz front-end. */
			if (loDivider != 0) {
				sub1GHz = true;
			}
			break;
		default:
			break;
		}

		if (sub1GHz) {
			if (paType == RF_TxPowerTable_HighPA) {
				/* Note: RFC_GPO3 is a work-around because the RFC_GPO1 */
				/* is sometimes not de-asserted on CC1352 Rev A. */
				pinctrl_apply_state(ant_pcfg, PINCTRL_STATE_ANT_SUBG_PA);
			} else {
				pinctrl_apply_state(ant_pcfg, PINCTRL_STATE_ANT_SUBG);
			}
		}
	} else {
		pinctrl_apply_state(ant_pcfg, PINCTRL_STATE_DEFAULT);
	}
}
