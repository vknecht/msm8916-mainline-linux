/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _Q6_MVM_H
#define _Q6_MVM_H

extern struct apr_device *mvm_dev;

struct q6mvm_session {
	struct apr_device *adev;
	uint16_t handle;
};

int q6mvm_create_session(struct apr_device *adev, struct q6mvm_session *session);

#endif /*_Q6_MVM_H */
