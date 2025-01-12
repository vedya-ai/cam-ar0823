/*
 * ar0823_main.c - ar0823 sensor driver
 *
 * Copyright (c) 2023-2024, Define Design Deploy Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

#include <d3/common.h>
#include <d3/reg_tbl.h>
#include <d3/reg_ctl.h>
#include <d3/ub960.h>

#include "ar0823.h"
#include "ar0823_ctrl.h"
#include "ar0823_tables.h"


static int ar0823_power_on(struct camera_common_data *s_data)
{
	struct ar0823 *priv = (struct ar0823*)s_data->priv;
	struct device *dev = priv->tc_dev->dev;

	dev_dbg(dev, "power on.");

	s_data->power->state = SWITCH_ON;

	return 0;
}

static int ar0823_power_off(struct camera_common_data *s_data)
{
	struct ar0823 *priv = (struct ar0823*)s_data->priv;
	struct device *dev = priv->tc_dev->dev;

	dev_dbg(dev, "power off.");

	s_data->power->state = SWITCH_OFF;

	return 0;
}

static int ar0823_reset(struct gpio_desc *reset_gpio)
{

	if (!IS_ERR_OR_NULL(reset_gpio)) {
		gpiod_set_value_cansleep(reset_gpio, 0);
		usleep_range(500, 1000);
		gpiod_set_value_cansleep(reset_gpio, 1);
		/*Needs to sleep for quite a while before register writes*/
		msleep_range(200);

		return 0;
	}

	return -EINVAL;
}

static int ar0823_start_stream(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ar0823 *self = (struct ar0823*)tegracam_get_privdata(tc_dev);
	int err = 0;

	dev_info(dev, "start stream.");

	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, 1);
	}

	/* set the state register to start streaming */
	TRY(err, reg_tbl_write(tc_dev->s_data->regmap, ar0823_start));

	return 0;
}

static int ar0823_stop_stream(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ar0823 *self = (struct ar0823*)tegracam_get_privdata(tc_dev);
	int err = 0;

	dev_info(dev, "stop stream.");

	/* Notify the deserializer a stream is ending */
	if (self->deserializer) {
		ub960_s_stream(self->deserializer, self->client, 0);
	}

	/* set the state register to stop streaming */
	TRY(err, reg_tbl_write(tc_dev->s_data->regmap, ar0823_stop));

	return 0;
}

static int ar0823_deserializer_parse(struct ar0823 *self,
				      struct i2c_client **out)
{
	struct device_node *node = self->client->dev.of_node;
	struct device_node *deserializer_node;
	struct i2c_client *deserializer_client;
	int ret;

	deserializer_node = of_parse_phandle(node, "deserializer", 0);
	if (!deserializer_node) {
		dev_dbg(self->dev, "could not find deserializer node");
		return -ENOENT;
	}

	ret = of_device_is_compatible(deserializer_node, "d3,ub960");
	if (!ret)
		return -ENOENT;

	dev_dbg(self->dev, "ar0823 found compatible ub960 %d", ret);

	deserializer_client = of_find_i2c_device_by_node(deserializer_node);
	of_node_put(deserializer_node);
	deserializer_node = NULL;

	if (!deserializer_client) {
		dev_info(self->dev, "missing deserializer client");
		return -ENOENT;
	}

	*out = deserializer_client;
	return 0;
}

static struct camera_common_pdata *ar0823_parse_dt(
		struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	int err;

	if (!np)
		return NULL;

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT");

	return board_priv_pdata;
}

static int ar0823_power_get(struct tegracam_device *tc_dev)
{
	return 0;
}

static int ar0823_power_put(struct tegracam_device *tc_dev)
{
	return 0;
}

static const struct reg_tbl_t *ar0823_mode_table[] = {
	[ISX031_MODE_30FPS_4CH] = ar0823_30fps_4ch,
	[ISX031_MODE_30FPS_4CH_CROPPED] = ar0823_30fps_4ch_cropped
};

static int ar0823_set_mode(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0823 *priv = (struct ar0823 *)s_data->priv;
	const struct reg_tbl_t *table;
	int err = 0;

	dev_info(dev, "Setting mode %d", s_data->sensor_mode_id);

	// Use --sensor_mode=x to set mode x
	if (s_data->sensor_mode_id < ARRAY_SIZE(ar0823_mode_table)
		&& ar0823_mode_table[s_data->sensor_mode_id]) {
		table = ar0823_mode_table[s_data->sensor_mode_id];
	} else {
		dev_err(dev, "Invalid mode");
		return -EINVAL;
	}

	dev_dbg(dev, "Resetting...");
	TRY(err, ar0823_reset(priv->reset_gpio));

	/* Put the imager in remap mode 0
	 * See The App Note Section "3.5 Communication Protocols" for more info
	 */
	TRY(err, regmap_write(priv->ctrl_map, 0xffff, 0x0));

	switch (priv->frame_sync_mode) {
		case ISX031_FSYNC_OFF:
			break;
		case ISX031_FSYNC_EXTERNAL_PULSE:
			dev_info(dev, "Using external pulse-based frame synchronization");
			TRY(err, reg_tbl_write(priv->ctrl_map, ar0823_external_pulse_sync));
			break;
		case ISX031_FSYNC_SHUTTER_TRIGGER:
			dev_info(dev, "Using shutter trigger-based frame synchronization");
			TRY(err, reg_tbl_write(priv->ctrl_map, ar0823_shutter_trigger_sync));
			break;
		default:
			dev_warn(dev, "Unknown frame sync mode %d, not enabling", priv->frame_sync_mode);
			break;
	}

	dev_dbg(dev, "Writing mode table...");
	TRY(err, reg_tbl_write(priv->ctrl_map, table));

	dev_dbg(dev, "Disabling embedded data...");
	TRY(err, reg_tbl_write(priv->ctrl_map, ar0823_disable_metadata));

	return err;
}

static const int ar0823_framerates_30fps[] = {
	30,
};

static const struct camera_common_frmfmt ar0823_frmfmt[] = {
	{
		.size = {1920, 1536},
		.framerates = ar0823_framerates_30fps,
		.num_framerates = ARRAY_SIZE(ar0823_framerates_30fps),
		.hdr_en = false,
		.mode = ISX031_MODE_30FPS_4CH, 
	},
	{
		.size = {1600, 1280},
		.framerates = ar0823_framerates_30fps,
		.num_framerates = ARRAY_SIZE(ar0823_framerates_30fps),
		.hdr_en = false,
		.mode = ISX031_MODE_30FPS_4CH_CROPPED,
	},
};

static struct camera_common_sensor_ops ar0823_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ar0823_frmfmt),
	.frmfmt_table = ar0823_frmfmt,
	.power_on = ar0823_power_on,
	.power_off = ar0823_power_off,
	.start_streaming = ar0823_start_stream,
	.stop_streaming = ar0823_stop_stream,

	.parse_dt = ar0823_parse_dt,
	.power_get = ar0823_power_get,
	.power_put = ar0823_power_put,
	.set_mode = ar0823_set_mode,
};

static int ar0823_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0823_subdev_internal_ops = {
	// No ops needed
	.open = ar0823_open,
	// Hook to allow us to add custom controls
	.registered = ar0823_ctrls_init,
};

static const struct regmap_config sensor_regmap_reg_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};

// TODO: remove one of these?
static const struct regmap_config sensor_regmap_ctrl_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};

static int ar0823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ar0823 *priv;
	int ret = 0;

	dev_info(dev, "probing ar0823");

	/* Validate if should probe */
	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	/* Allocate memory */
	dev_dbg(dev, "allocating mem for ar0823");
	TRY_MEM(priv, devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL));
	TRY_MEM(tc_dev, devm_kzalloc(dev, sizeof(*tc_dev), GFP_KERNEL));

	/* Get sensor out of reset 
	 * NOTE: This requires firmware that starts with MODE_SET_F=0 
	 * Failure to do so will not work w/ GMSL */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(priv->reset_gpio)) {
		dev_warn(dev, "Reset GPIO not found, deferring");

		return -EPROBE_DEFER;
	} else {
		dev_dbg(dev, "Found reset GPIO");
		ar0823_reset(priv->reset_gpio);
	}

	/* Prepare tc_dev and ar0823 structs */
	priv->client = client;
	priv->dev = dev;
	tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ar0823", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_reg_config;
	tc_dev->sensor_ops = &ar0823_common_ops;
	tc_dev->v4l2sd_internal_ops = &ar0823_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ar0823_ctrl_ops;
	priv->frame_sync_mode = ISX031_FSYNC_OFF;
	priv->custom_ctrl_handler = &priv->_custom_ctrl_handler;

	/* Register tc_dev with tegracam framework */
	ret = tegracam_device_register(tc_dev);
	if (ret) {
		dev_err(dev, "tegra camera driver registration failed");
		return ret;
	}

	/* Prepare ar0823 and tc_dev structs further */
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void*)priv);

	/* Register tc_dev with v4l2 through tegracam function */
	/* This will also add our custom controls through the .registered hook */
	ret = tegracam_v4l2subdev_register(tc_dev, true);
	if (ret) {
		dev_err(dev, "tegra camera subdev registration failed");
		return ret;
	}

	/* Two regmaps needed for this driver
	 * - reg_bits=16 and val_bits=16 for register access
	 * - reg_bits=16 and val_bits=8 for control
	 * create the 2nd one here
	 * TODO: this doesn't seem to be true. val_bits should be 8 for both regmaps,
	 * otherwise we get weird behavior when trying to read or write either.
	 */
	TRY_MEM(priv->ctrl_map, devm_regmap_init_i2c(tc_dev->client,
					&sensor_regmap_ctrl_config));

	/* Register with reg_ctl for sysfs register access */
	priv->reg_ctl = reg_ctl_init(priv->s_data->regmap, &client->dev);

	if (ar0823_deserializer_parse(priv, &priv->deserializer) == 0) {
		dev_info(priv->dev, "deserializer present");
	} else {
		priv->deserializer = NULL;
	}

	dev_info(dev, "Probed ar0823 imager");
	return 0;
}

static int ar0823_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0823 *priv = (struct ar0823 *)s_data->priv;

	reg_ctl_free(priv->reg_ctl);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct of_device_id ar0823_of_match[] = {
	{.compatible = "d3,ar0823",},
	{ },
};
MODULE_DEVICE_TABLE(of, ar0823_of_match);

static const struct i2c_device_id ar0823_id[] = {
	{"ar0823", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ar0823_id);

static struct i2c_driver ar0823_i2c_driver = {
	.driver = {
		.name = "ar0823",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ar0823_of_match),
	},
	.probe = ar0823_probe,
	.remove = ar0823_remove,
	.id_table = ar0823_id,
};

module_i2c_driver(ar0823_i2c_driver);

MODULE_DESCRIPTION("Driver for ISX031 camera on NVIDIA Jetson");
MODULE_VERSION(D3_MODULE_VERSION);
MODULE_AUTHOR("Daniel Breslawski <dbreslawski@d3engineering.com>");
MODULE_AUTHOR("Jacob Kiggins <jkiggins@d3engineering.com>");
MODULE_LICENSE("GPL v2");
