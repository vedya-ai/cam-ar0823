#include "ar0823_tables.h"
#include "ar0823.h"

/* OPERATIONS */
const struct reg_tbl_t ar0823_start[] = {
	{ .action = WRITE_REG, .addr = AR0823_REG_MODE_SET_F_LOCK, .val = AR0823_REG_MODE_SET_F_LOCK_EN },
	{ .action = SET_FIELD, .addr = AR0823_REG_MODE_H, .mask = BIT(7), .val = BIT(7) },
	{ .action = DELAY_MS, .val = 128 },
	{ .action = END_TABLE }
};

const struct reg_tbl_t ar0823_stop[] = {
	{ .action = WRITE_REG, .addr = AR0823_REG_MODE_SET_F_LOCK, .val = AR0823_REG_MODE_SET_F_LOCK_EN },
	{ .action = SET_FIELD, .addr = AR0823_REG_MODE_H, .mask = BIT(7), .val = 0x0 },
	{ .action = DELAY_MS, .val = 10 },
	{ .action = END_TABLE }
};

const struct reg_tbl_t ar0823_disable_metadata[] = {
	{ .action = WRITE_REG, .addr = AR0823_REG_IR_DR_I2I_FEBD_EN, .val = 0x0 },
	{ .action = WRITE_REG, .addr = AR0823_REG_IR_DR_I2I_REBD_EN, .val = 0x0 },
	{ .action = END_TABLE }
};

/* FRAME SYNC */
const struct reg_tbl_t ar0823_external_pulse_sync[] = {
	{ .action = SET_FIELD, .addr = AR0823_REG_DRVABTY, .mask = GENMASK(3,2), .val = 0xc },
	{ .action = SET_FIELD, .addr = AR0823_REG_FSYNC_FUNCSEL, .mask = GENMASK(3,2), .val = 0x0 },
	{ .action = SET_FIELD, .addr = AR0823_REG_IR_DR_SG_FSYNCIN_SEL, .mask = BIT(0), .val = 0x0 },
	{ .action = SET_FIELD, .addr = AR0823_REG_SG_MODE_APL, .mask = GENMASK(1,0), .val = 0x1 },
	{ .action = SET_FIELD, .addr = AR0823_REG_SG_MODE_, .mask = GENMASK(1,0), .val = 0x1 },
	{ .action = DELAY_MS, .val = 128 }
};

const struct reg_tbl_t ar0823_shutter_trigger_sync[] = {
	{ .action = SET_FIELD, .addr = AR0823_REG_DRVABTY, .mask = GENMASK(3,2), .val = 0xc },
	{ .action = SET_FIELD, .addr = AR0823_REG_FSYNC_FUNCSEL, .mask = GENMASK(3,2), .val = 0x0 },
	{ .action = SET_FIELD, .addr = AR0823_REG_IR_DR_SG_FSYNCIN_SEL, .mask = BIT(0), .val = 0x0 },
	{ .action = SET_FIELD, .addr = AR0823_REG_SG_MODE_APL, .mask = GENMASK(1,0), .val = 0x2 },
	{ .action = SET_FIELD, .addr = AR0823_REG_SG_MODE_, .mask = GENMASK(1,0), .val = 0x2 },
	{ .action = DELAY_MS, .val = 128 }
};

/* MODES */
// 1920x1536 @ 30fps
const struct reg_tbl_t ar0823_30fps_4ch[] = {
	{ .action = WRITE_REG, .addr = AR0823_REG_MODE_L, .val = 0x17 },
	{ .action = DELAY_MS, .val = 128 },
	{ .action = END_TABLE }
};
// 1600x1280 @ 30fps
const struct reg_tbl_t ar0823_30fps_4ch_cropped[] = {
	// crop image width to 1600 = 0x0640
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HSIZE_H_APL, .val = 0x06},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HSIZE_L_APL, .val = 0x40},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HSIZE_H, .val = 0x06},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HSIZE_L, .val = 0x40},
	// crop image height to 1280 = 0x0500
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VSIZE_H_APL, .val = 0x05},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VSIZE_L_APL, .val = 0x00},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VSIZE_H, .val = 0x05},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VSIZE_L, .val = 0x00},
	// use hoffset and voffset to recenter window
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HOFFSET_L_APL, .val = 0xa0},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_HOFFSET_L, .val = 0xa0},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VOFFSET_L_APL, .val = 0x80},
	{ .action = WRITE_REG, .addr = AR0823_REG_DCROP_VOFFSET_L, .val = 0x80},
	// turn cropping on
	{ .action = SET_FIELD, .addr = AR0823_REG_DCROP_ON_APL, .mask = BIT(0), .val = BIT(0) },
	{ .action = SET_FIELD, .addr = AR0823_REG_DCROP_ON_, .mask = BIT(0), .val = BIT(0) },
	// set the crop value source to the updated registers
	{ .action = SET_FIELD, .addr = AR0823_REG_DCROP_DATA_SEL, .mask = BIT(0), .val = BIT(0) },

	/* ar0823_30fps_4ch[] */
	{ .action = WRITE_REG, .addr = AR0823_REG_MODE_L, .val = 0x17 },

	{ .action = DELAY_MS, .val = 128 },
	{ .action = END_TABLE }
};
