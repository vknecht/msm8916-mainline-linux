#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvp.h"
#include "q6voice-downstream.h"

static struct device *q6cvp_dev = NULL; /* FIXME */

static int q6cvp_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;
	struct q6cvp *cvp = dev_get_drvdata(dev);
	struct aprv2_ibasic_rsp_result_t *result = data->payload;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	if (data->hdr.opcode != APR_BASIC_RSP_RESULT) {
		return 0; /* Unhandled for now */
	}

	dev_info(dev, "basic result: opcode %#x, status: %#x\n",
		 result->opcode, result->status);

	switch (result->opcode) {
	case VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2:
		cvp->handle = data->hdr.src_port;
		/* fallthrough */
	case VSS_IVOCPROC_CMD_ENABLE:
		break;
	default:
		return 0; /* Unhandled command */
	}

	cvp->result = *result;
	wake_up(&cvp->wait);
	return 0;
}

static int q6cvp_send(struct q6cvp *cvp, struct apr_hdr *hdr)
{
	int ret;

	/* TODO: Lock? */

	cvp->result.opcode = 0;
	cvp->result.status = 0;

	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				       APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr->src_port = VOC_PATH_PASSIVE;
	hdr->dest_port = cvp->handle;
	hdr->token = 0;

	ret = apr_send_pkt(cvp->adev, (struct apr_pkt*)hdr);
	if (ret < 0)
		return ret;

	ret = wait_event_timeout(cvp->wait, (cvp->result.opcode == hdr->opcode),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		return -ETIMEDOUT;

	if (cvp->result.status > 0) {
		dev_err(&cvp->adev->dev, "command %#x failed with error %d\n",
			hdr->opcode, cvp->result.status);
		return -EIO;
	}

	return 0;
}

static int q6cvp_do_create_session(struct q6cvp *cvp,
				   uint16_t tx_port, uint16_t rx_port)
{
	struct device *dev = &cvp->adev->dev;
	struct cvp_create_full_ctl_session_cmd cmd;
	int ret;

	dev_err(dev, "create session with tx: %#x, rx: %#x\n", tx_port, rx_port);

	if (cvp->handle)
		return -EBUSY;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2;

	/* TODO: Implement calibration */
	cmd.cvp_session.tx_topology_id = VSS_IVOCPROC_TOPOLOGY_ID_TX_SM_ECNS;
	cmd.cvp_session.rx_topology_id = VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;

	cmd.cvp_session.direction = 2; /* rx and tx */
	cmd.cvp_session.tx_port_id = tx_port;
	cmd.cvp_session.rx_port_id = rx_port;
	cmd.cvp_session.profile_id = VSS_ICOMMON_CAL_NETWORK_ID_NONE;
	cmd.cvp_session.vocproc_mode = VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING;
	cmd.cvp_session.ec_ref_port_id = VSS_IVOCPROC_PORT_ID_NONE;

	ret = q6cvp_send(cvp, &cmd.hdr);
	if (ret)
		return ret;

	dev_err(dev, "handle: %d\n", cvp->handle);
	return 0;
}

struct q6cvp *q6cvp_create_session(uint16_t tx_port, uint16_t rx_port)
{
	struct q6cvp *cvp;
	int ret;

	if (!q6cvp_dev)
		return ERR_PTR(-ENODEV);

	cvp = dev_get_drvdata(q6cvp_dev);
	ret = q6cvp_do_create_session(cvp, tx_port, rx_port);
	if (ret)
		return ERR_PTR(ret);

	return cvp;
}

int q6cvp_enable(struct q6cvp *cvp)
{
	struct device *dev = &cvp->adev->dev;
	struct apr_pkt cmd;

	dev_err(dev, "enable\n");

	if (!cvp->handle)
		return -EINVAL;

	cmd.hdr.pkt_size = APR_HDR_SIZE;
	cmd.hdr.opcode = VSS_IVOCPROC_CMD_ENABLE;

	return q6cvp_send(cvp, &cmd.hdr);
}

static int q6cvp_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;
	struct q6cvp *cvp;

	dev_info(dev, "Hello World!\n");

	cvp = devm_kzalloc(dev, sizeof(*cvp), GFP_KERNEL);
	if (!cvp)
		return -ENOMEM;

	cvp->adev = adev;
	init_waitqueue_head(&cvp->wait);

	dev_set_drvdata(dev, cvp);

	q6cvp_dev = dev;
	return 0;
}

static int q6cvp_remove(struct apr_device *adev)
{
	return 0;
}

static const struct of_device_id q6cvp_device_id[]  = {
	{ .compatible = "qcom,q6cvp" },
	{},
};
MODULE_DEVICE_TABLE(of, q6cvp_device_id);

static struct apr_driver qcom_q6cvp_driver = {
	.probe = q6cvp_probe,
	.remove = q6cvp_remove,
	.callback = q6cvp_callback,
	.driver = {
		.name = "qcom-q6cvp",
		.of_match_table = of_match_ptr(q6cvp_device_id),
	},
};

module_apr_driver(qcom_q6cvp_driver);
MODULE_DESCRIPTION("Q6 Core Voice Processor");
MODULE_LICENSE("GPL v2");
