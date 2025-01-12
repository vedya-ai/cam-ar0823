/*
 * ar0823_ctrl.c - ar0823 sensor controls
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
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>

#include "ar0823.h"
#include "ar0823_ctrl.h"

enum {
	ISX031_CID_BASE = (TEGRA_CAMERA_CID_BASE + 200),
	ISX031_CID_FRAME_SYNC,
};

// Tegracam exposed settings
static int ar0823_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ar0823_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int ar0823_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ar0823_set_group_hold(struct tegracam_device *tc_dev, bool val);


// V4L2 custom settings
static int ar0823_s_ctrl(struct v4l2_ctrl *ctrl);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct tegracam_ctrl_ops ar0823_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ar0823_set_gain,
	.set_exposure = ar0823_set_exposure,
	.set_frame_rate = ar0823_set_frame_rate,
	.set_group_hold = ar0823_set_group_hold,
};

// Custom V4L2 settings
static const struct v4l2_ctrl_ops ar0823_v4l2_ctrl_ops = {
	.s_ctrl = ar0823_s_ctrl,
};

struct v4l2_ctrl_config ar0823_custom_controls[] = {
	{
		.ops = &ar0823_v4l2_ctrl_ops,
		.id = ISX031_CID_FRAME_SYNC,
		.name = "Frame Sync",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 0,
		.max = (ISX031_NUM_FSYNC_MODES - 1),
		.def = ISX031_FSYNC_OFF,
		.step = 1,
	}
};
unsigned int ar0823_num_custom_controls = ARRAY_SIZE(ar0823_custom_controls);


/**
 * Tegracam doesn't officially support custom controls, so this method initializes
 * its own v4l2_ctrl_handler, adds the custom controls defined in
 * ar0823_custom_controls, and then merges it with Tegracam's internal
 * v4l2_ctrl_handler. This should not be called directly - it should be declared
 * as the .registered hook in tc_dev->v4l2sd_internal_ops.
 *
 * @see https://stackoverflow.com/a/68002497
 * @param sd V4L2 Sub-device (automagically passed)
 * @return 0 on success, anything else on failure
 */
int ar0823_ctrls_init(struct v4l2_subdev *sd) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);

	struct ar0823 *priv = (struct ar0823 *)s_data->priv;
	struct v4l2_ctrl *ctrl;
	int i, err;

	// Pre-flight checks
	if (priv == NULL || priv->custom_ctrl_handler == NULL
			|| sd == NULL || sd->ctrl_handler == NULL) {
		dev_err(dev, "Failed to obtain a control handler");
		return -EINVAL;
	}

	// Initialize a control handler
	v4l2_ctrl_handler_init(priv->custom_ctrl_handler, ar0823_num_custom_controls);

	// Add standard controls
	ctrl = v4l2_ctrl_new_std(priv->custom_ctrl_handler, &ar0823_v4l2_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(priv->dev, "Error initializing standard control");
		return -EINVAL;
	}

	ctrl = v4l2_ctrl_new_std(priv->custom_ctrl_handler, &ar0823_v4l2_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ctrl == NULL) {
		dev_err(priv->dev, "Error initializing standard control");
		return -EINVAL;
	}

	// Add each custom control in ar0823_custom_controls
	for (i = 0; i < ar0823_num_custom_controls; i++) {
		dev_dbg(dev, "Registering custom control %d with v4l2", i);
		ctrl = v4l2_ctrl_new_custom(priv->custom_ctrl_handler, &ar0823_custom_controls[i], NULL);

		if (ctrl == NULL) {
			// Note: ctrl_handler->error is set only by the first failed addition
			dev_err(dev, "Failed to add custom control at index %d, error %d", i, priv->custom_ctrl_handler->error);
			return -EINVAL;
		}
	}

	// Add control handler with our custom controls to Tegracam's
	if ((err = v4l2_ctrl_add_handler(sd->ctrl_handler, priv->custom_ctrl_handler, NULL, false))) {
		dev_err(dev, "Failed to add control handler, error %d", err);
		return err;
	}

	return 0;
}

static int ar0823_set_vflip(struct ar0823 *priv, s64 val)
{
	int err = 0;

	dev_dbg(priv->dev, "%d", (int)val);

	err = regmap_update_bits(
		priv->s_data->regmap,
		ISX031_REG_VREVERSE,
		ISX031_REVERSE_MASK,
		FIELD_PREP(ISX031_REVERSE_MASK, val ? 1 : 0)
	);

	err |= regmap_update_bits(
		priv->s_data->regmap,
		ISX031_REG_VREVERSE_APL,
		ISX031_REVERSE_MASK,
		FIELD_PREP(ISX031_REVERSE_MASK, val ? 1 : 0)
	);

	return err;
}

static int ar0823_set_hflip(struct ar0823 *priv, s64 val)
{
	int err = 0;

	dev_dbg(priv->dev, "%d", (int)val);

	err = regmap_update_bits(
		priv->s_data->regmap,
		ISX031_REG_HREVERSE,
		ISX031_REVERSE_MASK,
		FIELD_PREP(ISX031_REVERSE_MASK, val ? 1 : 0)
	);

	err |= regmap_update_bits(
		priv->s_data->regmap,
		ISX031_REG_HREVERSE_APL,
		ISX031_REVERSE_MASK,
		FIELD_PREP(ISX031_REVERSE_MASK, val ? 1 : 0)
	);

	return err;
}

static int ar0823_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;

	dev_dbg(dev, "Set gain: %lli", val);

	return 0;
}

static int ar0823_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;

	dev_dbg(dev, "Set exposure: %lli", val);

	return 0;
}

/**
 * Set the frame rate of the sensor. In frame sync mode however, the frame
 * rate should not be manually set, otherwise the frame rate will be less
 * then what was set. For example, manually setting a frame rate of 60 fps
 * with a 60 fps frame sync signal may result in frame rates as low as 30 fps.
 * To combat this issue, when frame sync is enabled, the frame rate is fixed
 * at 120 fps.
 */
static int ar0823_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;

	dev_dbg(dev, "Set frame rate: %lli", val);

	return 0;
}

// This control is required by 2.0 framework
static int ar0823_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct device *dev = tc_dev->dev;

	dev_dbg(dev, "Group hold %d", val);

	return 0;
}

static int ar0823_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// The user of container_of is OK here since this driver owns the struct, of
	// which the ctrl_handler is a member
	struct ar0823 *priv =
		container_of(ctrl->handler, struct ar0823,
			     _custom_ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
		case V4L2_CID_HFLIP:
			err = ar0823_set_hflip(priv, ctrl->val);
			break;
		case V4L2_CID_VFLIP:
			err = ar0823_set_vflip(priv, ctrl->val);
			break;
		case ISX031_CID_FRAME_SYNC:
			dev_dbg(priv->dev, "Configuring frame sync mode %d", ctrl->val);
			priv->frame_sync_mode = ctrl->val;
			break;
		default:
			dev_err(priv->dev, "%s: unknown ctrl id=%d", __func__, ctrl->id);
			return -EINVAL;
	}

	return err;
}
