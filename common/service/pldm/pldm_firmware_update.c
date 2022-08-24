#include <logging/log.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "util_spi.h"
#include "libutil.h"
#include "pldm_firmware_update.h"
#include "util_sys.h"

#define FW_UPDATE_BUF_SIZE 256
#define FW_UPDATE_SIZE 224
#define PLDM_FW_UPDATE_STACK_SIZE 2048
#define UPDATE_THREAD_DELAY_SECOND 1

k_tid_t fw_update_tid;
struct k_thread pldm_fw_update_thread;
K_KERNEL_STACK_MEMBER(pldm_fw_update_stack, PLDM_FW_UPDATE_STACK_SIZE);

static enum pldm_firmware_update_state current_state = STATE_IDLE;
static enum pldm_firmware_update_state pre_state = STATE_IDLE;
static uint32_t comp_image_size;

uint16_t pldm_fw_update_read(void *mctp_p, enum pldm_firmware_update_commands cmd, uint8_t *req,
			     uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len, void *ext_params)
{
	if (!mctp_p || !req || !rbuf || !ext_params) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		/* return read length zero means fail */
		return 0;
	}

	pldm_msg msg = { 0 };
	mctp_ext_params *extra_data = (mctp_ext_params *)ext_params;

	msg.ext_params = *extra_data;

	msg.hdr.pldm_type = PLDM_TYPE_FW_UPDATE;
	msg.hdr.cmd = cmd;
	msg.hdr.rq = 1;

	msg.buf = req;
	msg.len = req_len;

	return mctp_pldm_read(mctp_p, &msg, rbuf, rbuf_len);
}

static void update_fail_handler(void *mctp_p, void *ext_params)
{
	if (!mctp_p || !ext_params) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		return;
	}

	uint16_t read_len;
	uint16_t rbuf_len = 10;

	uint8_t rbuf[10] = { 0 };

	switch (current_state) {
	case STATE_DOWNLOAD: {
		struct pldm_transfer_complete_req tran_comp_req = { 0 };
		tran_comp_req.transferResult = PLDM_FW_UPDATE_GENERIC_ERROR;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE,
					       (uint8_t *)&tran_comp_req,
					       sizeof(struct pldm_transfer_complete_req), rbuf,
					       rbuf_len, ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			printf("[%s] Send transfer complete error fail \n", __func__);
		}
		break;
	}
	case STATE_VERIFY: {
		struct pldm_verify_complete_req verify_comp_req = { 0 };
		verify_comp_req.verifyResult = PLDM_FW_UPDATE_GENERIC_ERROR;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE,
					       (uint8_t *)&verify_comp_req,
					       sizeof(struct pldm_verify_complete_req), rbuf,
					       rbuf_len, ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			printf("[%s] Send verify complete error fail \n", __func__);
		}
		break;
	}
	case STATE_APPLY: {
		struct pldm_apply_complete_req apply_comp_req = { 0 };
		apply_comp_req.applyResult = PLDM_FW_UPDATE_GENERIC_ERROR;
		apply_comp_req.compActivationMethodsModification = 0x0000;

		read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE,
					       (uint8_t *)&apply_comp_req,
					       sizeof(struct pldm_apply_complete_req), rbuf,
					       rbuf_len, ext_params);

		if ((read_len != 1) || (rbuf[0] != PLDM_SUCCESS)) {
			printf("[%s] Send apply complete error fail \n", __func__);
		}
		break;
	}
	default:
		break;
	}

	if (fw_update_tid) {
		fw_update_tid = NULL;
	}
	SAFE_FREE(ext_params);
	return;
}

void req_fw_update_handler(void *mctp_p, void *ext_params, void *arg2)
{
	ARG_UNUSED(arg2);

	if (!mctp_p || !ext_params) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		SAFE_FREE(ext_params);
		return;
	}

	uint16_t read_len;
	uint8_t update_flag = 0;
	uint8_t resp_buf[FW_UPDATE_BUF_SIZE] = { 0 };

	struct pldm_request_firmware_data_req req = { 0 };

	while (req.offset < comp_image_size) {
		uint8_t ret;

		if (req.offset + FW_UPDATE_SIZE < comp_image_size) {
			/**
       * Check block size write flash shouldn't exceed over 64 KB to align the 
       * allocates space use in fw_update function.
       */
			if ((req.offset % SECTOR_SZ_64K) + FW_UPDATE_SIZE > SECTOR_SZ_64K) {
				req.length = SECTOR_SZ_64K - (req.offset % SECTOR_SZ_64K);
			} else {
				req.length = FW_UPDATE_SIZE;
			}
		} else {
			/* The remaining packets of the image */
			req.length = comp_image_size - req.offset;
			/**
       * Check block size write flash shouldn't exceed over 64 KB to align the 
       * allocates space use in fw_update function.
       */
			if ((req.offset % SECTOR_SZ_64K) + req.length > SECTOR_SZ_64K) {
				req.length = SECTOR_SZ_64K - (req.offset % SECTOR_SZ_64K);
			} else {
				/* The last packet, set the flag. */
				update_flag = (SECTOR_END_FLAG | NO_RESET_FLAG);
			}
		}

		read_len =
			pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_REQUEST_FIRMWARE_DATA,
					    (uint8_t *)&req,
					    sizeof(struct pldm_request_firmware_data_req), resp_buf,
					    FW_UPDATE_BUF_SIZE, ext_params);

		/* read_len = request length + completion code */
		if (read_len != req.length + 1) {
			printf("[%s] Request firmware update fail, offset 0x%x, length 0x%x, return length 0x%x",
			       __func__, req.offset, req.length, read_len);
			goto error;
		}
		ret = fw_update(req.offset, req.length, &resp_buf[1], update_flag, DEVSPI_FMC_CS0);

		if (ret) {
			printf("[%s] fw_update fail, offset 0x%x, length 0x%x, status %d", __func__,
			       req.offset, req.length, ret);
			goto error;
		}

		req.offset += req.length;
	}

	printf("[%s] Transfer complete\n", __func__);

	struct pldm_transfer_complete_req tran_comp_req = { 0 };
	/* Transfer has completed without error */
	tran_comp_req.transferResult = PLDM_FW_UPDATE_TRANSFER_SUCCESS;

	read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE,
				       (uint8_t *)&tran_comp_req,
				       sizeof(struct pldm_transfer_complete_req), resp_buf,
				       FW_UPDATE_BUF_SIZE, ext_params);

	if ((read_len != 1) || (resp_buf[0] != PLDM_SUCCESS)) {
		printf("[%s] Transfer complete fail comp code (0x%x)", __func__, resp_buf[0]);
		goto error;
	}
	pre_state = current_state;
	current_state = STATE_VERIFY;

	printf("[%s] Verify complete\n", __func__);

	struct pldm_verify_complete_req verify_comp_req = { 0 };
	/* Verify has completed without error */
	verify_comp_req.verifyResult = PLDM_FW_UPDATE_VERIFY_SUCCESS;

	read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_VERIFY_COMPLETE,
				       (uint8_t *)&verify_comp_req,
				       sizeof(struct pldm_verify_complete_req), resp_buf,
				       FW_UPDATE_BUF_SIZE, ext_params);

	if ((read_len != 1) || (resp_buf[0] != PLDM_SUCCESS)) {
		printf("[%s] Verify complete fail comp code (0x%x)", __func__, resp_buf[0]);
		goto error;
	}
	pre_state = current_state;
	current_state = STATE_APPLY;

	printf("[%s] Apply complete\n", __func__);

	struct pldm_apply_complete_req apply_comp_req = { 0 };

	/* Apply has completed without error */
	apply_comp_req.applyResult = PLDM_FW_UPDATE_APPLY_SUCCESS;
	apply_comp_req.compActivationMethodsModification = 0x0000;
	read_len = pldm_fw_update_read(mctp_p, PLDM_FW_UPDATE_CMD_CODE_APPLY_COMPLETE,
				       (uint8_t *)&apply_comp_req,
				       sizeof(struct pldm_apply_complete_req), resp_buf,
				       FW_UPDATE_BUF_SIZE, ext_params);

	if ((read_len != 1) || (resp_buf[0] != PLDM_SUCCESS)) {
		printf("[%s] Apply complete fail comp code (0x%x)", __func__, resp_buf[0]);
		goto error;
	}
	pre_state = current_state;
	current_state = STATE_RDY_XFER;
	comp_image_size = 0;

	if (fw_update_tid) {
		fw_update_tid = NULL;
	}

	SAFE_FREE(ext_params);
	return;

error:
	update_fail_handler(mctp_p, ext_params);
	comp_image_size = 0;
	SAFE_FREE(ext_params);
	return;
}

static uint8_t request_update(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
			      uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		return PLDM_ERROR;
	}

	struct pldm_request_update_req *req_p = (struct pldm_request_update_req *)buf;
	struct pldm_request_update_resp *resp_p = (struct pldm_request_update_resp *)resp;

	/* minimum return only completion code */
	*resp_len = 1;

	if (len != (sizeof(struct pldm_request_update_req) + req_p->comp_image_set_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	if (current_state != STATE_IDLE) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_ALREADY_IN_UPDATE_MODE;
		return PLDM_SUCCESS;
	}

	/* TBD: currently not support meta data */
	resp_p->fd_meta_data_len = 0x0000;

	if (req_p->pkg_data_len) {
		resp_p->fd_will_send_pkg_data = 0x01;
	} else {
		resp_p->fd_will_send_pkg_data = 0x00;
	}

	printf("[%s] max transfer size = %u, number of component = %d, max_outstanding_transfer_req = %d \n",
	       __func__, req_p->max_transfer_size, req_p->num_of_comp,
	       req_p->max_outstanding_transfer_req);
	printf("[%s] packet data length = %d, component sting type = %d len = %d\n", __func__,
	       req_p->pkg_data_len, req_p->comp_image_set_ver_str_type,
	       req_p->comp_image_set_ver_str_len);

	printf("[%s] Component Image Version String = ", __func__);
	for (int i = 0; i < req_p->comp_image_set_ver_str_len; i++) {
		printf("%c", *(buf + sizeof(struct pldm_request_update_req) + i));
	}
	printf("\n");

	*resp_len = sizeof(struct pldm_request_update_resp);
	resp_p->completion_code = PLDM_SUCCESS;

	/* go to next state machine */
	pre_state = current_state;
	current_state = STATE_LEARN_COMP;

	return PLDM_SUCCESS;
}

static uint8_t pass_component_table(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				    uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		return PLDM_ERROR;
	}

	struct pldm_pass_component_table_req *req_p = (struct pldm_pass_component_table_req *)buf;
	struct pldm_pass_component_table_resp *resp_p =
		(struct pldm_pass_component_table_resp *)resp;

	/* minimum return only completion code */
	*resp_len = 1;

	if (len != (sizeof(struct pldm_pass_component_table_req) + req_p->comp_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	/* if not in update mode currently */
	if (current_state == STATE_IDLE) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE;
		return PLDM_SUCCESS;
	}

	/* only support one currently */
	if (req_p->transfer_flag != PLDM_START_AND_END) {
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	/* The classification should be downstream device */
	if (req_p->comp_classification == PLDM_COMP_DOWNSTREAM_DEVICE) {
		/**
     * The identifier should be range 0x0000-0x0FFF when classification is the
     * downstream device
     */
		if (req_p->comp_identifier > 0x0FFF) {
			printf("[%s] Invalid component identifier id\n", __func__);
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			return PLDM_SUCCESS;
		}
	} else {
		printf("[%s] Invalid component classification \n", __func__);
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	if (req_p->comp_classification_index != 0x00) {
		printf("[%s] Only support update one device \n", __func__);
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	printf("[%s] Component version string = ", __func__);
	for (int i = 0; i < req_p->comp_ver_str_len; i++) {
		printf("%c", *(buf + sizeof(struct pldm_pass_component_table_req) + i));
	}
	printf("\n");

	/* Currently not support error condition, so only response zero */
	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->comp_resp = 0;
	resp_p->comp_resp_code = 0;
	*resp_len = sizeof(struct pldm_pass_component_table_resp);

	pre_state = current_state;
	current_state = STATE_RDY_XFER;

	return PLDM_SUCCESS;
}

static uint8_t update_component(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		return PLDM_ERROR;
	}

	struct pldm_update_component_req *req_p = (struct pldm_update_component_req *)buf;
	struct pldm_update_component_resp *resp_p = (struct pldm_update_component_resp *)resp;

	/* minimum return only completion code */
	*resp_len = 1;

	if (len != (sizeof(struct pldm_update_component_req) + req_p->comp_ver_str_len)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	/* if not in update mode currently */
	if (current_state == STATE_IDLE) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE;
		return PLDM_SUCCESS;
	}
	/* The classification should be downstream device */
	if (req_p->comp_classification == PLDM_COMP_DOWNSTREAM_DEVICE) {
		/**
     * The identifier should be range 0x0000-0x0FFF when classification is the
     * downstream device
     */
		if (req_p->comp_identifier > 0x0FFF) {
			printf("[%s] Invalid component identifier id\n", __func__);
			resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
			return PLDM_SUCCESS;
		}
	} else {
		printf("[%s] Invalid component classification \n", __func__);
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	if (req_p->comp_classification_index != 0x00) {
		printf("[%s] Only support update one device \n", __func__);
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}

	comp_image_size = req_p->comp_image_size;

	printf("[%s] image size = 0x%x \n", __func__, comp_image_size);

	printf("[%s] Component version string = ", __func__);
	for (int i = 0; i < req_p->comp_ver_str_len; i++) {
		printf("%c", *(buf + sizeof(struct pldm_update_component_req) + i));
	}
	printf("\n");

	resp_p->completion_code = PLDM_SUCCESS;
	/* Currently not support error condition, so only response zero */
	resp_p->comp_compatability_resp = 0x00;
	resp_p->comp_compatability_resp_code = 0x00;
	/* return update option flags by request message */
	resp_p->update_option_flags_enabled = req_p->update_option_flags;
	/* delay 1 second to start update thread */
	resp_p->time_before_req_fw_data = UPDATE_THREAD_DELAY_SECOND;
	*resp_len = sizeof(struct pldm_update_component_resp);

	mctp_ext_params *extra_data = (mctp_ext_params *)malloc(sizeof(mctp_ext_params));

	if (!extra_data) {
		printf("[%s] malloc fail\n", __func__);
		resp_p->completion_code = PLDM_ERROR;
		return PLDM_SUCCESS;
	}

	memcpy(extra_data, ext_params, sizeof(mctp_ext_params));

	fw_update_tid =
		k_thread_create(&pldm_fw_update_thread, pldm_fw_update_stack,
				K_THREAD_STACK_SIZEOF(pldm_fw_update_stack), req_fw_update_handler,
				mctp_inst, extra_data, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0,
				K_SECONDS(UPDATE_THREAD_DELAY_SECOND));
	k_thread_name_set(&pldm_fw_update_thread, "pldm_fw_update_thread");

	pre_state = current_state;
	current_state = STATE_DOWNLOAD;
	return PLDM_SUCCESS;
}

K_WORK_DELAYABLE_DEFINE(submit_warm_reset_work, submit_bic_warm_reset);

static uint8_t activate_firmware(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t *resp,
				 uint16_t *resp_len, void *ext_params)
{
	if (!mctp_inst || !buf || !resp || !resp_len) {
		printf("[%s] Pass argument is NULL, \n", __func__);
		pre_state = current_state;
		current_state = STATE_IDLE;
		return PLDM_ERROR;
	}

	struct pldm_activate_firmware_req *req_p = (struct pldm_activate_firmware_req *)buf;
	struct pldm_activate_firmware_resp *resp_p = (struct pldm_activate_firmware_resp *)resp;

	/* minimum return only completion code */
	*resp_len = 1;

	if (len != sizeof(struct pldm_activate_firmware_req)) {
		resp_p->completion_code = PLDM_ERROR_INVALID_LENGTH;
		return PLDM_SUCCESS;
	}

	/* Not in the update mode */
	if (current_state == STATE_IDLE) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE;
		return PLDM_SUCCESS;
	}

	/* Only expect this command in READY XFER state */
	if (current_state != STATE_RDY_XFER) {
		resp_p->completion_code = PLDM_FW_UPDATE_CC_INVALID_STATE_FOR_COMMAND;
		return PLDM_SUCCESS;
	}

	if (req_p->selfContainedActivationRequest != PLDM_NOT_ACTIVATE_SELF_CONTAINED_COMPONENTS) {
		resp_p->completion_code = PLDM_ERROR_INVALID_DATA;
		return PLDM_SUCCESS;
	}
	printf("[%s] Activate firmware \n", __func__);
	resp_p->completion_code = PLDM_SUCCESS;
	resp_p->estimated = 0;
	*resp_len = sizeof(struct pldm_activate_firmware_resp);

	k_work_schedule(&submit_warm_reset_work, K_SECONDS(1));

	pre_state = current_state;
	current_state = STATE_ACTIVATE;
	return PLDM_SUCCESS;
}

static pldm_cmd_handler pldm_fw_update_cmd_tbl[] = {
	{ PLDM_FW_UPDATE_CMD_CODE_REQUEST_UPDATE, request_update },
	{ PLDM_FW_UPDATE_CMD_CODE_PASS_COMPONENT_TABLE, pass_component_table },
	{ PLDM_FW_UPDATE_CMD_CODE_UPDATE_COMPONENT, update_component },
	{ PLDM_FW_UPDATE_CMD_CODE_ACTIVE_FIRMWARE, activate_firmware },
};

uint8_t pldm_fw_update_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn) {
		return PLDM_ERROR;
	}

	pldm_cmd_proc_fn fn = NULL;

	for (int i = 0; i < ARRAY_SIZE(pldm_fw_update_cmd_tbl); i++) {
		if (pldm_fw_update_cmd_tbl[i].cmd_code == code) {
			fn = pldm_fw_update_cmd_tbl[i].fn;
			break;
		}
	}

	*ret_fn = (void *)fn;
	return fn ? PLDM_SUCCESS : PLDM_ERROR;
}