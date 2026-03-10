/*
 * Copyright (c) 2026 BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief API for fast gpio on controllers that have set/clr registers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SET_CLR_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SET_CLR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Raw GPIO register mapping for ports with dedicated set/clear registers.
 *
 * This structure represents a GPIO port that provides separate write-only
 * registers for setting and clearing output pins. Each bit in the registers
 * corresponds to a GPIO pin.
 *
 * The registers follow write-1-to-modify semantics:
 * - Writing a bit value of @c 1 to the @c set register drives the corresponding
 *   pin to an active/high state.
 * - Writing a bit value of @c 1 to the @c clr register drives the corresponding
 *   pin to an inactive/low state.
 * - Writing @c 0 to any bit has no effect.
 */
typedef struct {
	/** Pointer to the GPIO port bit set register (write-1-to-set). */
	volatile uint32_t *set;
	/** Pointer to the GPIO port bit clear register (write-1-to-clear). */
	volatile uint32_t *clr;
} gpio_raw_regs_t;

/**
 * @brief Set one or more GPIO pins to a high state.
 *
 * Writes to the hardware "set" register to drive the specified pins high
 * without affecting the state of other pins.
 *
 * @param regs  GPIO register block, typically obtained via
 *              gpio_port_get_regs().
 * @param bits  Bitmask of pins to set high. Each bit corresponds to a GPIO pin;
 *              a value of 1 sets the respective pin high.
 */
static inline void gpio_raw_set(const gpio_raw_regs_t regs, uint32_t bits)
{
	*regs.set = bits;
}

/**
 * @brief Clear one or more GPIO pins to a low state.
 *
 * Writes to the hardware "clear" register to drive the specified pins low
 * without affecting the state of other pins.
 *
 * @param regs  GPIO register block, typically obtained via
 *              gpio_port_get_regs().
 * @param bits  Bitmask of pins to clear. Each bit corresponds to a GPIO pin;
 *              a value of 1 clears the respective pin (sets it low).
 */
static inline void gpio_raw_clear(const gpio_raw_regs_t regs, uint32_t bits)
{
	*regs.clr = bits;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SET_CLR_H_ */
