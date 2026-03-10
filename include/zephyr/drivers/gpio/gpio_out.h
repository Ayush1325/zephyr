/*
 * Copyright (c) 2026 BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief API for fast gpio on controllers that have set/clr registers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_OUT_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_OUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Raw GPIO register mapping for ports with a single output register.
 *
 * This structure represents a GPIO port that exposes only a single output
 * register controlling all pins. Each bit in the register corresponds to
 * a GPIO pin.
 *
 * Writing to the register directly affects the output state of the pins:
 * - Writing a bit value of @c 1 drives the corresponding pin to an active/high state.
 * - Writing a bit value of @c 0 drives the corresponding pin to an inactive/low state.
 */
typedef struct {
	/** Pointer to the GPIO port output register. */
	volatile uint32_t *out;
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
	*regs.out |= bits;
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
	*regs.out &= ~bits;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_SET_CLR_H_ */
