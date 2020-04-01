#include <linux/module.h>
#include <linux/of.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvs.h"

struct apr_device *cvs_dev = NULL;

static int q6cvs_callback(struct apr_device *adev, struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;

	dev_info(dev, "callback: %#x\n", data->hdr.opcode);

	return 0;
}

static int q6cvs_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;

	dev_info(dev, "Hello World!\n");
	cvs_dev = adev;

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
