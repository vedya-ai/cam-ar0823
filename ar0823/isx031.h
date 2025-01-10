/*
 * isx031.h - isx031 constants
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

#ifndef _ISX031_H
#define _ISX031_H

#include <linux/bitops.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

enum isx031_fsync_modes {
	ISX031_FSYNC_OFF = 0,
	ISX031_FSYNC_EXTERNAL_PULSE,
	ISX031_FSYNC_SHUTTER_TRIGGER,
	ISX031_NUM_FSYNC_MODES
};

struct isx031 {
	struct i2c_client *client;
	struct device *dev;
	struct v4l2_subdev *subdev;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
	struct v4l2_ctrl_handler *custom_ctrl_handler;
	struct v4l2_ctrl_handler _custom_ctrl_handler;
	struct regmap *ctrl_map;

	struct i2c_client *deserializer;
	struct gpio_desc *reset_gpio;

	struct reg_ctl *reg_ctl;

	/*
	 * 0 = disabled (free running)
	 * 1 = External Pulse-Based
	 * 2 = Shutter Trigger-Based
	 */
	enum isx031_fsync_modes frame_sync_mode;
};

enum isx031_regs {
	/* Camera Control Registers */
	ISX031_REG_HREVERSE = 0x8A68,
	ISX031_REG_HREVERSE_APL = 0xBEFE,
	ISX031_REG_VREVERSE = 0x8A69,
	ISX031_REG_VREVERSE_APL = 0xBEFF,
	/* Image Cropping Registers */
	ISX031_REG_DCROP_ON_APL = 0xBF04,
	ISX031_REG_DCROP_HSIZE_L_APL = 0xBF06,
	ISX031_REG_DCROP_HSIZE_H_APL = 0xBF07,
	ISX031_REG_DCROP_HOFFSET_L_APL= 0xBF08,
	ISX031_REG_DCROP_HOFFSET_H_APL= 0xBF09,
	ISX031_REG_DCROP_VSIZE_L_APL = 0xBF0A,
	ISX031_REG_DCROP_VSIZE_H_APL = 0xBF0B,
	ISX031_REG_DCROP_VOFFSET_L_APL= 0xBF0C,
	ISX031_REG_DCROP_VOFFSET_H_APL= 0xBF0D,
	ISX031_REG_DCROP_ON_ = 0x8AA8,
	ISX031_REG_DCROP_HSIZE_L = 0x8AAA,
	ISX031_REG_DCROP_HSIZE_H = 0x8AAB,
	ISX031_REG_DCROP_HOFFSET_L = 0x8AAC,
	ISX031_REG_DCROP_HOFFSET_H = 0x8AAD,
	ISX031_REG_DCROP_VSIZE_L = 0x8AAE,
	ISX031_REG_DCROP_VSIZE_H = 0x8AAF,
	ISX031_REG_DCROP_VOFFSET_L = 0x8AB0,
	ISX031_REG_DCROP_VOFFSET_H = 0x8AB1,
	ISX031_REG_DCROP_DATA_SEL = 0x8ADA,
	/* Embedded Metadata Control Registers */
	ISX031_REG_IR_DR_I2I_FEBD_EN = 0x0171,
	ISX031_REG_IR_DR_I2I_REBD_EN = 0x0172,
	/* Modesetting Registers */
	ISX031_REG_MODE_L = 0x8A00,
	ISX031_REG_MODE_H = 0x8A01,
	ISX031_REG_MODE_SET_F_LOCK = 0xBEF0,
	/* Fsync registers */
	ISX031_REG_FSYNC_FUNCSEL = 0x8AFE,    // [3:2]
	ISX031_REG_DRVABTY = 0x8AFF,          // [3:2]
	ISX031_REG_SG_MODE_ = 0x8AF0,         // [1:0]
	ISX031_REG_SG_MODE_APL = 0xBF14,      // [1:0]
	ISX031_REG_SG_TRIG_AUTOMODE = 0x8AF1, // [1:0]
	ISX031_REG_IR_DR_SG_FSYNCIN_SEL = 0x0153,
};

/* Some handy values and masks */
#define ISX031_REG_MODE_SET_F_LOCK_EN (0x53)
#define ISX031_REVERSE_MASK BIT(0)

#endif
