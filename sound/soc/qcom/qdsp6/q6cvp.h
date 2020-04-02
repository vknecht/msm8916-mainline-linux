/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _Q6_CVP_H
#define _Q6_CVP_H
#include <linux/soc/qcom/apr.h>

struct q6cvp {
	struct apr_device *adev;

	struct aprv2_ibasic_rsp_result_t result;
	wait_queue_head_t wait;

	uint16_t handle;
};

struct q6cvp *q6cvp_create_session(uint16_t tx_port, uint16_t rx_port);

int q6cvp_enable(struct q6cvp *cvp);

#endif /*_Q6_CVP_H */
