/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <zephyr.h>
#include <stdio.h>
#include <logging/log.h>

#include "hal_gpio.h"
#include "sensor.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_led.h"

LOG_MODULE_REGISTER(plat_led);

void led_io_expander_init()
{
	I2C_MSG msg = { 0 };

	msg.bus = LED_IO_EXPANDER_BUS;
	msg.target_addr = LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = LED_IO_EXPANDER_OUTPUT_PORT1_REG;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read LED I/O expander output port1 register failed");
		return;
	}

	msg.data[1] = msg.data[0] |=
		(BIT(LED_IO_EXPANDER_FAULT_BIT) | BIT(LED_IO_EXPANDER_PWR_BIT));
	msg.data[0] = LED_IO_EXPANDER_OUTPUT_PORT1_REG;
	msg.tx_len = 2;
	msg.rx_len = 0;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write LED I/O expander output port1 register failed");
		return;
	}
}

static bool sys_led_control(uint8_t reg, uint8_t offset, bool status)
{
	if ((reg != LED_IO_EXPANDER_CONFIG_PORT0_REG) &&
	    (reg != LED_IO_EXPANDER_CONFIG_PORT1_REG)) {
		LOG_ERR("Unsupport register(0x%x) set", reg);
		return false;
	}

	if (offset & 0xF8) {
		LOG_ERR("Unsupport offset(%d) set", offset);
		return false;
	}

	I2C_MSG msg = { 0 };
	msg.bus = LED_IO_EXPANDER_BUS;
	msg.target_addr = LED_IO_EXPANDER_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = reg;

	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Read register(0x%x) failed", reg);
		return false;
	}

	/* The LED pin is low active */
	WRITE_BIT(msg.data[0], offset, !status);
	msg.data[1] = msg.data[0];
	msg.data[0] = reg;
	msg.tx_len = 2;
	msg.rx_len = 0;

	if (i2c_master_write(&msg, 5)) {
		LOG_ERR("Write register(0x%x) failed", reg);
		return false;
	}

	return true;
}

bool pwr_led_control(uint8_t src, uint8_t status)
{
	if ((src >= LED_CTRL_SRC_MAX) || (status >= LED_CTRL_MAX)) {
		LOG_ERR("Unsupport source(%d) or status(%d)", src, status);
		return false;
	}

	static uint8_t ctrl_src = 0;
	WRITE_BIT(ctrl_src, src, status);

	return sys_led_control(LED_IO_EXPANDER_CONFIG_PORT1_REG, LED_IO_EXPANDER_PWR_BIT, ctrl_src);
}

bool fault_led_control(uint8_t src, uint8_t status)
{
	if ((src >= LED_CTRL_SRC_MAX) || (status >= LED_CTRL_MAX)) {
		LOG_ERR("Unsupport source(%d) or status(%d)", src, status);
		return false;
	}

	static uint8_t ctrl_src = 0;
	WRITE_BIT(ctrl_src, src, status);

	return sys_led_control(LED_IO_EXPANDER_CONFIG_PORT1_REG, LED_IO_EXPANDER_FAULT_BIT,
			       ctrl_src);
}

void light_fault_led_check()
{
	bool is_alert = 0;
	const uint8_t sys_alert_pin[] = {
		NIC_ADC_ALERT_N, SSD_0_7_ADC_ALERT_N, SSD_8_15_ADC_ALERT_N,
		PEX_ADC_ALERT_N, SMB_ALERT_PMBUS_R_N, SMB_ALERT_HSC_R_N,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(sys_alert_pin); i++)
		is_alert |= !gpio_get(sys_alert_pin[i]);

	if (!fault_led_control(LED_CTRL_SRC_BIC, is_alert ? LED_CTRL_ON : LED_CTRL_OFF))
		LOG_ERR("Control fault LED failed");
}

void pwr_led_check()
{
	if (!pwr_led_control(LED_CTRL_SRC_BIC, is_mb_dc_on() ? LED_CTRL_ON : LED_CTRL_OFF))
		LOG_ERR("Control power LED failed");
}
