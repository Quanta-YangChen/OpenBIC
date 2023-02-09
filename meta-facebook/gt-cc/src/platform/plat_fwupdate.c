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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>

#include "libutil.h"
#include "sensor.h"
#include "pex89000.h"
#include "isl69259.h"
#include "xdpe12284c.h"
#include "mp2971.h"
#include "hal_jtag.h"
#include "plat_version.h"
#include "plat_pldm_monitor.h"
#include "plat_class.h"
#include "plat_fwupdate.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "pldm_firmware_update.h"
#include "sensor.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "i2c-mux-tca9548.h"
#include "util_spi.h"

LOG_MODULE_REGISTER(plat_fwupdate);

static uint8_t pldm_pre_vr_update(void *fw_update_param);
static uint8_t pldm_post_vr_update(void *fw_update_param);
static uint8_t pldm_pre_cpld_update(void *fw_update_param);
static uint8_t pldm_pre_pex_update(void *fw_update_param);
static uint8_t pldm_pex_update(void *fw_update_param);
static uint8_t pldm_post_pex_update(void *fw_update_param);

/* PLDM FW update table */
/* clang-format off */
pldm_fw_update_info_t PLDMUPDATE_FW_CONFIG_TABLE[] = {
	[COMP_ID_BIC] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_BIC, 0x00, NULL, pldm_bic_update, NULL, COMP_UPDATE_VIA_SPI, COMP_ACT_SELF, pldm_bic_activate },
	[COMP_ID_VR0] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR0, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_VR1] =  { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_VR1, 0x00, pldm_pre_vr_update, pldm_vr_update, pldm_post_vr_update, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX0] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX0, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX1] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX1, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX2] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX2, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_PEX3] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_PEX3, 0x00, pldm_pre_pex_update, pldm_pex_update, pldm_post_pex_update, COMP_UPDATE_VIA_SPI, COMP_ACT_AC_PWR_CYCLE, NULL },
	[COMP_ID_CPLD] = { ENABLE, COMP_CLASS_TYPE_DOWNSTREAM, COMP_ID_CPLD, 0x00, pldm_pre_cpld_update, pldm_cpld_update, NULL, COMP_UPDATE_VIA_I2C, COMP_ACT_AC_PWR_CYCLE, NULL },
};
/* clang-format on */

void load_pldmupdate_comp_config(void)
{
	if (comp_config) {
		LOG_WRN("PLDM update component table has already been load");
		return;
	}

	comp_config_count = ARRAY_SIZE(PLDMUPDATE_FW_CONFIG_TABLE);
	comp_config = malloc(sizeof(pldm_fw_update_info_t) * comp_config_count);
	if (!comp_config) {
		LOG_ERR("comp_config malloc failed");
		return;
	}

	memcpy(comp_config, PLDMUPDATE_FW_CONFIG_TABLE, sizeof(PLDMUPDATE_FW_CONFIG_TABLE));
}

/* pldm pre-update func */
static uint8_t pldm_pre_vr_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	/* Stop sensor polling */
	disable_sensor_poll();

	/* Assign VR 0/1 related sensor number to get information for accessing VR */
	uint8_t sensor_num =
		(p->comp_id == COMP_ID_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP : SENSOR_NUM_PEX_2_VR_TEMP;

	if (!tca9548_select_chan(sensor_num, &mux_conf_addr_0xe0[6])) {
		LOG_ERR("Component %d: mux switched failed!", p->comp_id);
		return 1;
	}

	/* Get bus and target address by sensor number in sensor configuration */
	p->bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	p->addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;

	return 0;
}

static uint8_t pldm_pre_cpld_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	if (p->inf == COMP_UPDATE_VIA_I2C) {
		p->bus = I2C_BUS8;
		p->addr = 0x40;
	}

	return 0;
}

uint8_t flash_sel_pin[4] = { BIC_SEL_FLASH_SW0, BIC_SEL_FLASH_SW1, BIC_SEL_FLASH_SW2,
			     BIC_SEL_FLASH_SW3 };
uint8_t current_sel_pin = 0xFF;

static uint8_t pldm_pre_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	uint8_t flash_sel_base = flash_sel_pin[0] - COMP_ID_PEX0;

	/* change mux to pex flash */
	for (int i = 0; i < ARRAY_SIZE(flash_sel_pin); i++) {
		if (flash_sel_base + p->comp_id == flash_sel_pin[i]) {
			current_sel_pin = flash_sel_pin[i];
			gpio_set(current_sel_pin, GPIO_HIGH);
		} else {
			gpio_set(flash_sel_pin[i], GPIO_LOW);
		}
	}

	return 0;
}

/* pldm fw-update func */
static uint8_t pldm_pex_update(void *fw_update_param)
{
	CHECK_NULL_ARG_WITH_RETURN(fw_update_param, 1);

	pldm_fw_update_param_t *p = (pldm_fw_update_param_t *)fw_update_param;

	CHECK_NULL_ARG_WITH_RETURN(p->data, 1);

	uint8_t update_flag = 0;

	/* prepare next data offset and length */
	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	if (p->next_ofs < fw_update_cfg.image_size) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;

		if (((p->next_ofs % SECTOR_SZ_64K) + p->next_len) > SECTOR_SZ_64K)
			p->next_len = SECTOR_SZ_64K - (p->next_ofs % SECTOR_SZ_64K);
	} else {
		/* current data is the last packet
		 * set the next data length to 0 to inform the update completely
		 */
		p->next_len = 0;
		update_flag = (SECTOR_END_FLAG | NO_RESET_FLAG);
	}

	uint8_t ret = fw_update(p->data_ofs, p->data_len, p->data, update_flag, DEVSPI_SPI1_CS0);

	if (ret) {
		LOG_ERR("Firmware update failed, offset(0x%x), length(0x%x), status(%d)",
			p->data_ofs, p->data_len, ret);
		return 1;
	}

	return 0;
}

/* pldm post-update func */
static uint8_t pldm_post_vr_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* Start sensor polling */
	enable_sensor_poll();

	return 0;
}

static uint8_t pldm_post_pex_update(void *fw_update_param)
{
	ARG_UNUSED(fw_update_param);

	/* change mux to pex flash */
	if (current_sel_pin != 0xFF) {
		/* change mux to default */
		gpio_set(current_sel_pin, GPIO_LOW);
	}

	return 0;
}

static bool get_fpga_user_code(uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);

	uint8_t buffer[4] = { 0 };
	uint8_t ir_value = 0xc0;
	uint8_t dr_value = 0x00;
	const struct device *jtag_dev;

	jtag_dev = device_get_binding("JTAG0");

	if (!jtag_dev) {
		LOG_ERR("JTAG device not found");
		return false;
	}

	gpio_set(JTAG_BIC_EN, GPIO_HIGH);

	if (jtag_tap_set(jtag_dev, TAP_RESET))
		return false;

	k_msleep(10);

	if (jtag_ir_scan(jtag_dev, 8, &ir_value, buffer, TAP_IDLE) ||
	    jtag_dr_scan(jtag_dev, 32, &dr_value, buffer, TAP_IDLE))
		return false;

	gpio_set(JTAG_BIC_EN, GPIO_LOW);

	memcpy(buf, buffer, sizeof(buffer));
	*len = sizeof(buffer);
	return true;
}

static bool get_pex_fw_version(uint8_t component, uint8_t *buf, uint8_t *len,
			       uint8_t *completion_code)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	CHECK_NULL_ARG_WITH_RETURN(completion_code, false);
	CHECK_ARG_WITH_RETURN((component < COMP_ID_PEX0) || (component > COMP_ID_PEX3), false);

	/* Only can be read when DC is on */
	if (!is_mb_dc_on()) {
		*completion_code = PLDM_FW_UPDATE_GET_VERSION_CC_NOT_POWER_ON;
		return false;
	}

	/* Physical Layer User Test Patterns, Byte 0 Register */
	int reading = 0x6080020c;
	uint8_t idx = component - COMP_ID_PEX0;
	uint8_t pex_sensor_num = pex_sensor_num_table[idx];
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[pex_sensor_num]];
	pex89000_unit *p = (pex89000_unit *)cfg->priv_data;

	if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_register, &reading)) {
		*completion_code = PLDM_FW_UPDATE_GET_VERSION_CC_ACCESS_FAIL;
		return false;
	}

	/* Change version register to SBR because the old PEX firmware did not fill in version information at register 0x6080020c yet */
	if (((reading & 0xFF) == idx) && ((reading >> 8) & 0xFF) == 0xCC) {
		if (pex_access_engine(cfg->port, cfg->target_addr, p->idx, pex_access_sbr_ver,
				      &reading)) {
			*completion_code = PLDM_FW_UPDATE_GET_VERSION_CC_ACCESS_FAIL;
			return false;
		}
	}

	memcpy(buf, &reading, sizeof(reading));
	*len = sizeof(reading);
	return true;
}

static bool get_vr_fw_version(uint8_t sensor_num, uint8_t *buf, uint8_t *len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_NULL_ARG_WITH_RETURN(len, false);
	CHECK_ARG_WITH_RETURN((sensor_num != SENSOR_NUM_PEX_0_VR_TEMP) &&
				      (sensor_num != SENSOR_NUM_PEX_2_VR_TEMP),
			      false);

	bool ret = false;
	uint8_t type = get_vr_type();
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	if (!cfg) {
		LOG_ERR("The sensor config pointer is NULL");
		return ret;
	}

	if (cfg->pre_sensor_read_hook) {
		if (!cfg->pre_sensor_read_hook(cfg->num, cfg->pre_sensor_read_args)) {
			LOG_ERR("The VR%d pre-reading hook function failed",
				(sensor_num == SENSOR_NUM_PEX_0_VR_TEMP) ? 0 : 1);
			goto exit;
		}
	}

	uint32_t version;
	switch (type) {
	case VR_RNS_ISL69259: {
		uint8_t mode;

		if (!get_raa_hex_mode(cfg->port, cfg->target_addr, &mode)) {
			LOG_ERR("Get VR ISL69259 raa hex mode failed");
			goto exit;
		}

		if (!get_raa_crc(cfg->port, cfg->target_addr, mode, &version)) {
			LOG_ERR("The VR ISL69259 version reading failed");
			goto exit;
		}
		break;
	}
	case VR_INF_XDPE12284:
		if (!xdpe12284c_get_checksum(cfg->port, cfg->target_addr, (uint8_t *)&version)) {
			LOG_ERR("The VR XDPE12284 version reading failed");
			goto exit;
		}
		break;
	case VR_MPS_MPS2971:
		if (!mps2856_get_checksum(cfg->port, cfg->target_addr, &version)) {
			LOG_ERR("The VR MPS2971 version reading failed");
			goto exit;
		}
		break;
	default:
		LOG_ERR("Unsupport VR type(%d)", type);
		goto exit;
	}

	memcpy(buf, &version, sizeof(version));
	*len = sizeof(version);
	ret = true;

exit:
	if (cfg->post_sensor_read_hook) {
		if (!cfg->post_sensor_read_hook(sensor_num, cfg->post_sensor_read_args, NULL)) {
			LOG_ERR("The VR%d post-reading hook function failed",
				(sensor_num == SENSOR_NUM_PEX_0_VR_TEMP) ? 0 : 1);
		}
	}

	return ret;
}

uint8_t plat_get_component_fw_version(void *mctp_inst, uint8_t *buf, uint16_t len,
				      uint8_t instance_id, uint8_t *resp, uint16_t *resp_len,
				      void *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(resp_len, PLDM_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, PLDM_ERROR);

	struct oem_pldm_get_comp_fw_version_resp *resp_p =
		(struct oem_pldm_get_comp_fw_version_resp *)resp;

	*resp_len = 1;

	if (len != 1) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	uint8_t component = buf[0];

	if (component >= GT_COMPNT_MAX) {
		LOG_ERR("Unsupport component id(%d)", component);
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	switch (component) {
	case COMP_ID_BIC:
		resp_p->length = 7;
		resp_p->fw_version[0] = BIC_FW_YEAR_MSB;
		resp_p->fw_version[1] = BIC_FW_YEAR_LSB;
		resp_p->fw_version[2] = BIC_FW_WEEK;
		resp_p->fw_version[3] = BIC_FW_VER;
		resp_p->fw_version[4] = BIC_FW_platform_0;
		resp_p->fw_version[5] = BIC_FW_platform_1;
		resp_p->fw_version[6] = BIC_FW_platform_2;
		break;
	case COMP_ID_VR0:
	case COMP_ID_VR1: {
		uint8_t vr_sensor_num = (component == COMP_ID_VR0) ? SENSOR_NUM_PEX_0_VR_TEMP :
								     SENSOR_NUM_PEX_2_VR_TEMP;
		if (!get_vr_fw_version(vr_sensor_num, resp_p->fw_version, &resp_p->length)) {
			resp_p->completion_code = PLDM_FW_UPDATE_GET_VERSION_CC_ACCESS_FAIL;
			return PLDM_SUCCESS;
		}
		break;
	}
	case COMP_ID_PEX0:
	case COMP_ID_PEX1:
	case COMP_ID_PEX2:
	case COMP_ID_PEX3:
		if (!get_pex_fw_version(component, resp_p->fw_version, &resp_p->length,
					&resp_p->completion_code))
			return PLDM_SUCCESS;
		break;
	case COMP_ID_CPLD:
		if (!get_fpga_user_code(resp_p->fw_version, &resp_p->length)) {
			resp_p->completion_code = PLDM_FW_UPDATE_GET_VERSION_CC_ACCESS_FAIL;
			return PLDM_SUCCESS;
		}
		break;
	default:
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	};

	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->component_id = component;
	*resp_len = sizeof(struct oem_pldm_get_comp_fw_version_resp) + resp_p->length - 1;

	return PLDM_SUCCESS;
}
