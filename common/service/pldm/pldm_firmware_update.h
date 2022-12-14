#ifndef PLDM_FIRMWARE_UPDATE_H
#define PLDM_FIRMWARE_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pldm.h"

/** 
 * PLDM Firmware update commands
 */
enum pldm_firmware_update_commands {
	/* inventory commands */
	PLDM_FW_UPDATE_CMD_CODE_QUERY_DEVICE_IDENTIFIERS = 0x01,
	PLDM_FW_UPDATE_CMD_CODE_GET_FIRMWARE_PARAMETERS = 0x02,

	/* update commands */
	PLDM_FW_UPDATE_CMD_CODE_REQUEST_UPDATE = 0x10,
	PLDM_FW_UPDATE_CMD_CODE_PASS_COMPONENT_TABLE = 0x13,
	PLDM_FW_UPDATE_CMD_CODE_UPDATE_COMPONENT = 0x14,
	PLDM_FW_UPDATE_CMD_CODE_REQUEST_FIRMWARE_DATA = 0x15,
	PLDM_FW_UPDATE_CMD_CODE_TRANSFER_COMPLETE = 0x16,
	PLDM_FW_UPDATE_CMD_CODE_VERIFY_COMPLETE = 0x17,
	PLDM_FW_UPDATE_CMD_CODE_APPLY_COMPLETE = 0x18,
	PLDM_FW_UPDATE_CMD_CODE_ACTIVE_FIRMWARE = 0x1A,
	PLDM_FW_UPDATE_CMD_CODE_GET_STATUS = 0x1B,
	PLDM_FW_UPDATE_CMD_CODE_CANCEL_UPDATE_COMPONENT = 0x1C,
	PLDM_FW_UPDATE_CMD_CODE_CANCEL_UPDATE = 0x1D,
};

/**
 * PLDM Firmware update completion codes
 */
enum pldm_firmware_update_completion_codes {
	PLDM_FW_UPDATE_CC_NOT_IN_UPDATE_MODE = 0x80,
	PLDM_FW_UPDATE_CC_ALREADY_IN_UPDATE_MODE = 0x81,
	PLDM_FW_UPDATE_CC_DATA_OUT_OF_RANGE = 0x82,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_LENGTH = 0x83,
	PLDM_FW_UPDATE_CC_INVALID_STATE_FOR_COMMAND = 0x84,
	PLDM_FW_UPDATE_CC_INCOMPLETE_UPDATE = 0x85,
	PLDM_FW_UPDATE_CC_BUSY_IN_BACKGROUND = 0x86,
	PLDM_FW_UPDATE_CC_CANCEL_PENDING = 0x87,
	PLDM_FW_UPDATE_CC_COMMAND_NOT_EXPECTED = 0x88,
	PLDM_FW_UPDATE_CC_RETRY_REQUEST_FW_DATA = 0x89,
	PLDM_FW_UPDATE_CC_UNABLE_TO_INITIATE_UPDATE = 0x8A,
	PLDM_FW_UPDATE_CC_ACTIVATION_NOT_REQUIRED = 0x8B,
	PLDM_FW_UPDATE_CC_SELF_CONTAINED_ACTIVATION_NOT_PERMITTED = 0x8C,
	PLDM_FW_UPDATE_CC_NO_DEVICE_METADATA = 0x8D,
	PLDM_FW_UPDATE_CC_RETRY_REQUEST_UPDATE = 0x8E,
	PLDM_FW_UPDATE_CC_NO_PACKAGE_DATA = 0x8F,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_HANDLE = 0x90,
	PLDM_FW_UPDATE_CC_INVALID_TRANSFER_OPERATION_FLAG = 0x91,
	PLDM_FW_UPDATE_CC_ACTIVATE_PENDING_IMAGE_NOT_PERMITTED = 0x92,
	PLDM_FW_UPDATE_CC_PACKAGE_DATA_ERROR = 0x93
};

/**
 * PLDM Frimware update state 
 */
enum pldm_firmware_update_state {
	STATE_IDLE,
	STATE_LEARN_COMP,
	STATE_RDY_XFER,
	STATE_DOWNLOAD,
	STATE_VERIFY,
	STATE_APPLY,
	STATE_ACTIVATE,
};

/** 
 * Common error codes in TransferComplete, VerifyComplete and ApplyComplete request
 */
enum pldm_firmware_update_common_error_codes {
	PLDM_FW_UPDATE_TIME_OUT = 0x09,
	PLDM_FW_UPDATE_GENERIC_ERROR = 0x0A
};

/** 
 * TransferResult values in the request of TransferComplete
 */
enum pldm_firmware_update_transfer_result_values {
	PLDM_FW_UPDATE_TRANSFER_SUCCESS = 0x00,
	/* Other values that are not currently used, and will be defined if they are 
  used in the future. */
};

/**
 * VerifyResult values in the request of VerifyComplete
 */
enum pldm_firmware_update_verify_result_values {
	PLDM_FW_UPDATE_VERIFY_SUCCESS = 0x00,
	/* Other values that are not currently used, and will be defined if they are 
  used in the future. */
};

/**
 * ApplyResult values in the request of ApplyComplete
 */
enum pldm_firmware_update_apply_result_values {
	PLDM_FW_UPDATE_APPLY_SUCCESS = 0x00,
	/* Other values that are not currently used, and will be defined if they are 
  used in the future. */
};

/**
 * component classification values define in PLDM firmware update specification 
 * Table 27
 */
enum pldm_component_classification_values {
	PLDM_COMP_UNKNOWN = 0x0000,
	PLDM_COMP_OTHER = 0x0001,
	PLDM_COMP_DRIVER = 0x0002,
	PLDM_COMP_CONFIGURATION_SOFTWARE = 0x0003,
	PLDM_COMP_APPLICATION_SOFTWARE = 0x0004,
	PLDM_COMP_INSTRUMENTATION = 0x0005,
	PLDM_COMP_FIRMWARE_OR_BIOS = 0x0006,
	PLDM_COMP_DIAGNOSTIC_SOFTWARE = 0x0007,
	PLDM_COMP_OPERATING_SYSTEM = 0x0008,
	PLDM_COMP_MIDDLEWARE = 0x0009,
	PLDM_COMP_FIRMWARE = 0x000A,
	PLDM_COMP_BIOS_OR_FCODE = 0x000B,
	PLDM_COMP_SUPPORT_OR_SERVICEPACK = 0x000C,
	PLDM_COMP_SOFTWARE_BUNDLE = 0x000D,
	PLDM_COMP_DOWNSTREAM_DEVICE = 0xFFFF
};

/** 
 * Structure representing fixed part of Request Update request
 */
struct pldm_request_update_req {
	uint32_t max_transfer_size;
	uint16_t num_of_comp;
	uint8_t max_outstanding_transfer_req;
	uint16_t pkg_data_len;
	uint8_t comp_image_set_ver_str_type;
	uint8_t comp_image_set_ver_str_len;
} __attribute__((packed));

/**
 * Structure representing Request Update response
 */
struct pldm_request_update_resp {
	uint8_t completion_code;
	uint16_t fd_meta_data_len;
	uint8_t fd_will_send_pkg_data;
} __attribute__((packed));

/**
 * Structure representing PassComponentTable request
 */
struct pldm_pass_component_table_req {
	uint8_t transfer_flag;
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	uint32_t comp_comparison_stamp;
	uint8_t comp_ver_str_type;
	uint8_t comp_ver_str_len;
} __attribute__((packed));

/**
 * Structure representing PassComponentTable response
 */
struct pldm_pass_component_table_resp {
	uint8_t completion_code;
	uint8_t comp_resp;
	uint8_t comp_resp_code;
} __attribute__((packed));

/** 
 * Structure representing UpdateComponent request
 */
struct pldm_update_component_req {
	uint16_t comp_classification;
	uint16_t comp_identifier;
	uint8_t comp_classification_index;
	uint32_t comp_comparison_stamp;
	uint32_t comp_image_size;
	uint32_t update_option_flags;
	uint8_t comp_ver_str_type;
	uint8_t comp_ver_str_len;
} __attribute__((packed));

/** 
 * Structure representing UpdateComponent response
 */
struct pldm_update_component_resp {
	uint8_t completion_code;
	uint8_t comp_compatability_resp;
	uint8_t comp_compatability_resp_code;
	uint32_t update_option_flags_enabled;
	uint16_t time_before_req_fw_data;
} __attribute__((packed));

/**
 * Structure representing RequestFirmwareData request
 */
struct pldm_request_firmware_data_req {
	uint32_t offset;
	uint32_t length;
} __attribute__((packed));

/**
 * Structure representing ActivateFirmware request
 */
struct pldm_activate_firmware_req {
	uint8_t selfContainedActivationRequest;
} __attribute__((packed));

/**
 * Structure representing ActivateFirmware response
 */
struct pldm_activate_firmware_resp {
	uint8_t completion_code;
	uint16_t estimated;
} __attribute__((packed));

/**
 * SelfContainedActivationRequest in the request of ActivateFirmware
 */
enum pldm_self_contained_activation_req {
	PLDM_NOT_ACTIVATE_SELF_CONTAINED_COMPONENTS = false,
	PLDM_ACTIVATE_SELF_CONTAINED_COMPONENTS = true
};

/**
 * Structure representing TransferComplete request
 */
struct pldm_transfer_complete_req {
	uint8_t transferResult;
} __attribute__((packed));

struct pldm_verify_complete_req {
	uint8_t verifyResult;
} __attribute__((packed));

struct pldm_apply_complete_req {
	uint8_t applyResult;
	uint16_t compActivationMethodsModification;
} __attribute__((packed));

uint8_t pldm_fw_update_handler_query(uint8_t code, void **ret_fn);
uint16_t pldm_fw_update_read(void *mctp_p, enum pldm_firmware_update_commands cmd, uint8_t *req,
			     uint16_t req_len, uint8_t *rbuf, uint16_t rbuf_len, void *ext_params);

#ifdef __cplusplus
}
#endif

#endif // End of PLDM_FIRMWARE_UPDATE_H