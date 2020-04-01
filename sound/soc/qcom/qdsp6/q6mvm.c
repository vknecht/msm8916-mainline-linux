#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/apr.h>
#include "q6mvm.h"
#include "q6voice-downstream.h"

struct apr_device *mvm_dev = NULL;

static int q6mvm_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	switch (data->hdr.opcode) {
	case APR_BASIC_RSP_RESULT: {
		struct aprv2_ibasic_rsp_result_t *result = data->payload;

		dev_info(dev, "basic result: opcode %#x, status: %#x\n",
			 result->opcode, result->status);

		switch (result->opcode) {
		case VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION:
			break;
		default:
			break;
		}

		break;
	}
	case VSS_IVERSION_RSP_GET: {
		struct vss_iversion_rsp_get_t *result = data->payload;

		dev_info(dev, "CVD version: %s\n", result->version);
		break;
	}
	default:
		break;
	}

	return 0;
}

int q6mvm_create_session(struct apr_device *adev, struct q6mvm_session *session)
{
	struct device *dev = &adev->dev;
	struct mvm_create_ctl_session_cmd mvm_session_cmd;
	int ret;

	dev_err(dev, "create session\n");

	mvm_session_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_session_cmd.hdr.pkt_size =  sizeof(mvm_session_cmd);
	mvm_session_cmd.hdr.src_port = VOC_PATH_PASSIVE;
	mvm_session_cmd.hdr.dest_port = VOC_PATH_PASSIVE;
	mvm_session_cmd.hdr.token = 0;
	mvm_session_cmd.hdr.opcode = VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION;

	strlcpy(mvm_session_cmd.mvm_session.name, "default modem voice",
		strlen("default modem voice")+1);

	ret = apr_send_pkt(adev, (struct apr_pkt*)&mvm_session_cmd);
	if (ret < 0)
		return ret;

	session->adev = adev;
	return 0;
}

static int q6mvm_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;
	struct apr_pkt pkt;
	int ret;

	dev_info(dev, "Hello World!\n");
	mvm_dev = adev;

	/* Random command for testing communication */
	pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt.hdr.pkt_size = APR_HDR_SIZE;
	pkt.hdr.src_port = VOC_PATH_PASSIVE;
	pkt.hdr.dest_port = 0;
	pkt.hdr.token = 0;
	pkt.hdr.opcode = VSS_IVERSION_CMD_GET;

	ret = apr_send_pkt(adev, &pkt);
	if (ret < 0) {
		dev_err(dev, "Failed to send packet: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Packet sent.\n");

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
