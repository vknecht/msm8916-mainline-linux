#include <linux/module.h>
#include <linux/of.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvp.h"

struct apr_device *cvp_dev = NULL;

static int q6cvp_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	return 0;
}

static int q6cvp_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;

	dev_info(dev, "Hello World!\n");
	cvp_dev = adev;

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
