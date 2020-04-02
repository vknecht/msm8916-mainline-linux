#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvs.h"
#include "q6voice-downstream.h"

struct q6cvs {
	struct apr_device *adev;

	struct aprv2_ibasic_rsp_result_t result;
	wait_queue_head_t wait;

	uint16_t handle;
};

static struct device *q6cvs_dev = NULL; /* FIXME */

static int q6cvs_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;
	struct q6cvs *cvs = dev_get_drvdata(dev);
	struct aprv2_ibasic_rsp_result_t *result = data->payload;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	if (data->hdr.opcode != APR_BASIC_RSP_RESULT) {
		return 0; /* Unhandled for now */
	}

	dev_info(dev, "basic result: opcode %#x, status: %#x\n",
		 result->opcode, result->status);

	switch (result->opcode) {
	case VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION:
		cvs->handle = data->hdr.src_port;
		/* fallthrough */
		break;
	default:
		return 0; /* Unhandled command */
	}

	cvs->result = *result;
	wake_up(&cvs->wait);
	return 0;
}

static int q6cvs_send(struct q6cvs *cvs, struct apr_hdr *hdr)
{
	int ret;

	/* TODO: Lock? */

	cvs->result.opcode = 0;
	cvs->result.status = 0;

	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				       APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr->src_port = VOC_PATH_PASSIVE;
	hdr->dest_port = cvs->handle;
	hdr->token = 0;

	ret = apr_send_pkt(cvs->adev, (struct apr_pkt*)hdr);
	if (ret < 0)
		return ret;

	ret = wait_event_timeout(cvs->wait, (cvs->result.opcode == hdr->opcode),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		return -ETIMEDOUT;

	if (cvs->result.status > 0) {
		dev_err(&cvs->adev->dev, "command %#x failed with error %d\n",
			hdr->opcode, cvs->result.status);
		return -EIO;
	}

	return 0;
}

static int q6cvs_do_create_session(struct q6cvs *cvs)
{
	struct device *dev = &cvs->adev->dev;
	struct cvs_create_passive_ctl_session_cmd cmd;
	int ret;

	dev_err(dev, "create session\n");

	if (cvs->handle)
		return -EBUSY;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION;

	strlcpy(cmd.cvs_session.name, "default modem voice",
		strlen("default modem voice")+1);

	ret = q6cvs_send(cvs, &cmd.hdr);
	if (ret)
		return ret;

	dev_err(dev, "handle: %d\n", cvs->handle);
	return 0;
}

struct q6cvs *q6cvs_create_session(void)
{
	struct q6cvs *cvs;
	int ret;

	if (!q6cvs_dev)
		return ERR_PTR(-ENODEV);

	cvs = dev_get_drvdata(q6cvs_dev);
	ret = q6cvs_do_create_session(cvs);
	if (ret)
		return ERR_PTR(ret);

	return cvs;
}

static int q6cvs_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;
	struct q6cvs *cvs;

	dev_info(dev, "Hello World!\n");

	cvs = devm_kzalloc(dev, sizeof(*cvs), GFP_KERNEL);
	if (!cvs)
		return -ENOMEM;

	cvs->adev = adev;
	init_waitqueue_head(&cvs->wait);

	dev_set_drvdata(dev, cvs);

	q6cvs_dev = dev;
	return 0;
}

static int q6cvs_remove(struct apr_device *adev)
{
	return 0;
}

static const struct of_device_id q6cvs_device_id[]  = {
	{ .compatible = "qcom,q6cvs" },
	{},
};
MODULE_DEVICE_TABLE(of, q6cvs_device_id);

static struct apr_driver qcom_q6cvs_driver = {
	.probe = q6cvs_probe,
	.remove = q6cvs_remove,
	.callback = q6cvs_callback,
	.driver = {
		.name = "qcom-q6cvs",
		.of_match_table = of_match_ptr(q6cvs_device_id),
	},
};

module_apr_driver(qcom_q6cvs_driver);
MODULE_DESCRIPTION("Q6 Core Voice Stream");
MODULE_LICENSE("GPL v2");
