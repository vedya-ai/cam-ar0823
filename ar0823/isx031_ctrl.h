/*
 * isx031_ctrl.c - isx031 sensor controls
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


#ifndef ISX031_CTRL_H
#define ISX031_CTRL_H

extern struct tegracam_ctrl_ops isx031_ctrl_ops;
extern struct v4l2_ctrl_config isx031_custom_controls[];
extern unsigned int isx031_num_custom_controls;

extern int isx031_ctrls_init(struct v4l2_subdev *sd);

struct isx031_analog_gain_settings_t {
	u16 coarse;
	u16 fine;
	u64 total;
	u16 reg_30ba_90mhz;
	u16 reg_30ba_45mhz;
	u16 reg_30ba_22p5mhz;
};

#endif /* !ISX031_CTRL_H */
