/*
 * ar0823_tables.h - ar0823 sequence tables
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

#ifndef ISX031AT_H_
#define ISX031AT_H_

#include "ar0823.h"
#include <d3/reg_tbl.h>

extern const struct reg_tbl_t ar0823_start[];
extern const struct reg_tbl_t ar0823_stop[];
extern const struct reg_tbl_t ar0823_disable_metadata[];
extern const struct reg_tbl_t ar0823_external_pulse_sync[];
extern const struct reg_tbl_t ar0823_shutter_trigger_sync[];

extern const struct reg_tbl_t ar0823_30fps_4ch[];
extern const struct reg_tbl_t ar0823_30fps_4ch_cropped[];

enum ar0823_modes {
	ISX031_MODE_30FPS_4CH = 0,		// Sensor Mode 23
	ISX031_MODE_30FPS_4CH_CROPPED		// Sensor Mode 23 (cropped)
};

#endif // ISX031AT_H_
