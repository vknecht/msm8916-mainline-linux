/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _Q6_MVM_H
#define _Q6_MVM_H

struct q6mvm;

struct q6mvm *q6mvm_create_session(void);

int q6mvm_set_dual_control(struct q6mvm *mvm);
int q6mvm_attach(struct q6mvm *mvm, struct q6cvp *cvp);
int q6mvm_start(struct q6mvm *mvm);

#endif /*_Q6_MVM_H */
