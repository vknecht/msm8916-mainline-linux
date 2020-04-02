#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvp.h"
#include "q6mvm.h"
#include "q6voice-downstream.h"

struct q6mvm {
	struct apr_device *adev;

	struct aprv2_ibasic_rsp_result_t result;
	wait_queue_head_t wait;

	uint16_t handle;
};

static struct device *q6mvm_dev = NULL; /* FIXME */

static int q6mvm_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;
	struct q6mvm *mvm = dev_get_drvdata(dev);
	struct aprv2_ibasic_rsp_result_t *result = data->payload;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	if (data->hdr.opcode != APR_BASIC_RSP_RESULT) {
		return 0; /* Unhandled for now */
	}

	dev_info(dev, "basic result: opcode %#x, status: %#x\n",
		 result->opcode, result->status);

	switch (result->opcode) {
	case VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION:
		mvm->handle = data->hdr.src_port;
		/* fallthrough */
	case VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL:
	case VSS_IMVM_CMD_ATTACH_VOCPROC:
	case VSS_IMVM_CMD_START_VOICE:
		break;
	default:
		return 0; /* Unhandled command */
	}

	mvm->result = *result;
	wake_up(&mvm->wait);
	return 0;
}

static int q6mvm_send(struct q6mvm *mvm, struct apr_hdr *hdr)
{
	int ret;

	/* TODO: Lock? */

	mvm->result.opcode = 0;
	mvm->result.status = 0;

	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				       APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr->src_port = VOC_PATH_PASSIVE;
	hdr->dest_port = mvm->handle;
	hdr->token = 0;

	ret = apr_send_pkt(mvm->adev, (struct apr_pkt*)hdr);
	if (ret < 0)
		return ret;

	ret = wait_event_timeout(mvm->wait, (mvm->result.opcode == hdr->opcode),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		return -ETIMEDOUT;

	if (mvm->result.status > 0) {
		dev_err(&mvm->adev->dev, "command %#x failed with error %d\n",
			hdr->opcode, mvm->result.status);
		return -EIO;
	}

	return 0;
}

static int q6mvm_do_create_session(struct q6mvm *mvm)
{
	struct device *dev = &mvm->adev->dev;
	struct mvm_create_ctl_session_cmd cmd;
	int ret;

	dev_err(dev, "create session\n");

	if (mvm->handle)
		return -EBUSY;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION;

	strlcpy(cmd.mvm_session.name, "default modem voice",
		strlen("default modem voice")+1);

	ret = q6mvm_send(mvm, &cmd.hdr);
	if (ret)
		return ret;

	dev_err(dev, "handle: %d\n", mvm->handle);
	return 0;
}

struct q6mvm *q6mvm_create_session(void)
{
	struct q6mvm *mvm;
	int ret;

	if (!q6mvm_dev)
		return ERR_PTR(-ENODEV);

	mvm = dev_get_drvdata(q6mvm_dev);
	ret = q6mvm_do_create_session(mvm);
	if (ret)
		return ERR_PTR(ret);

	return mvm;
}

int q6mvm_set_dual_control(struct q6mvm *mvm)
{
	struct device *dev = &mvm->adev->dev;
	struct mvm_modem_dual_control_session_cmd cmd;

	dev_err(dev, "set dual control\n");

	if (!mvm->handle)
		return -EINVAL;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL;

	cmd.voice_ctl.enable_flag = true;
	return q6mvm_send(mvm, &cmd.hdr);
}

int q6mvm_attach(struct q6mvm *mvm, struct q6cvp *cvp)
{
	struct device *dev = &mvm->adev->dev;
	struct mvm_attach_vocproc_cmd cmd;

	dev_err(dev, "attach vocproc: %d\n", cvp->handle);

	if (!mvm->handle || !cvp->handle)
		return -EINVAL;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IMVM_CMD_ATTACH_VOCPROC;

	cmd.mvm_attach_cvp_handle.handle = cvp->handle;

	return q6mvm_send(mvm, &cmd.hdr);
}

int q6mvm_start(struct q6mvm *mvm)
{
	struct device *dev = &mvm->adev->dev;
	struct apr_pkt cmd;

	dev_err(dev, "start\n");

	if (!mvm->handle)
		return -EINVAL;

	cmd.hdr.pkt_size = APR_HDR_SIZE;
	cmd.hdr.opcode = VSS_IMVM_CMD_START_VOICE;

	return q6mvm_send(mvm, &cmd.hdr);
}

static int q6mvm_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;
	struct q6mvm *mvm;

	dev_info(dev, "Hello World!\n");

	mvm = devm_kzalloc(dev, sizeof(*mvm), GFP_KERNEL);
	if (!mvm)
		return -ENOMEM;

	mvm->adev = adev;
	init_waitqueue_head(&mvm->wait);

	dev_set_drvdata(dev, mvm);

	q6mvm_dev = dev;
	return of_platform_populate(dev->of_node, NULL, NULL, dev);
}

static int q6mvm_remove(struct apr_device *adev)
{
	of_platform_depopulate(&adev->dev);
	return 0;
}

static const struct of_device_id q6mvm_device_id[]  = {
	{ .compatible = "qcom,q6mvm" },
	{},
};
MODULE_DEVICE_TABLE(of, q6mvm_device_id);

static struct apr_driver qcom_q6mvm_driver = {
	.probe = q6mvm_probe,
	.remove = q6mvm_remove,
	.callback = q6mvm_callback,
	.driver = {
		.name = "qcom-q6mvm",
		.of_match_table = of_match_ptr(q6mvm_device_id),
	},
};

module_apr_driver(qcom_q6mvm_driver);
MODULE_DESCRIPTION("Q6 Multimode Voice Manager");
MODULE_LICENSE("GPL v2");
