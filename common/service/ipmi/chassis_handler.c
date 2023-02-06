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

#include "chassis_handler.h"

#include "power_status.h"
#include <logging/log.h>
#include "libutil.h"

LOG_MODULE_DECLARE(ipmi);

#ifdef CONFIG_ESPI
__weak void CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	CHASSIS_STATUS chassis_status = { 0 };

	chassis_status.currPwState.pwOn = get_DC_status();
	chassis_status.currPwState.pwFault = get_DC_status();
	chassis_status.currPwState.pwRestorePolicy = 0x2; // Always On

	memcpy(msg->data, &chassis_status, sizeof(CHASSIS_STATUS));
	msg->data_len = sizeof(CHASSIS_STATUS);
	msg->completion_code = CC_SUCCESS;
	return;
}
#endif

void IPMI_CHASSIS_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	switch (msg->cmd) {
#ifdef CONFIG_ESPI
	case CMD_CHASSIS_GET_CHASSIS_STATUS:
		CHASSIS_GET_CHASSIS_STATUS(msg);
		break;
#endif
	default:
		LOG_ERR("Invalid chassis msg netfn: %x, cmd: %x", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}
	return;
}
