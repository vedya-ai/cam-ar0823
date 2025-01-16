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


#ifndef AR0823_CTRL_H
#define AR0823_CTRL_H

extern struct tegracam_ctrl_ops ar0823_ctrl_ops;
extern struct v4l2_ctrl_config ar0823_custom_controls[];
extern unsigned int ar0823_num_custom_controls;

extern int ar0823_ctrls_init(struct v4l2_subdev *sd);

struct ar0823_analog_gain_settings_t {
	u16 coarse;
	u16 fine;
	u64 total;
	u16 reg_30ba_90mhz;
	u16 reg_30ba_45mhz;
	u16 reg_30ba_22p5mhz;
};

#endif /* !AR0823_CTRL_H */
