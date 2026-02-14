// SPDX-License-Identifier: GPL-2.0
/*
 * imx385 driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 * support mipi 2-lane 37.125MHz 1920x1080@30fps only
 */

#include "linux/media-bus-format.h"
#include "linux/mod_devicetable.h"
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <uapi/linux/rk-preisp.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION KERNEL_VERSION(0, 0x01, 0x00)

#define IMX385_NAME "imx385"

#define IMX385_LINK_FREQ_371M 371250000
#define IMX385_2LANES 2
#define IMX385_BITS_PER_SAMPLE 10
#define IMX385_PIXEL_RATE                                                      \
	(IMX385_LINK_FREQ_371M * 2 * IMX385_2LANES / IMX385_BITS_PER_SAMPLE)

#define IMX385_XVCLK_FREQ 37125000

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP "rockchip,camera_sleep"

#define CHIP_ID 0xF0
#define IMX385_REG_CHIP_ID 0x3012

#define IMX385_REG_CTRL_MODE 0x3000
#define IMX385_MODE_STANDBY 0x1
#define IMX385_MODE_OPERATING 0x0

#define IMX385_REG_SHS1_H 0x3022
#define IMX385_REG_SHS1_M 0x3021
#define IMX385_REG_SHS1_L 0x3020

#define IMX385_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 16) & 0x0F)
#define IMX385_FETCH_MID_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)
#define IMX385_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)

#define IMX385_REG_VTS_H 0x301A
#define IMX385_REG_VTS_M 0x3019
#define IMX385_REG_VTS_L 0x3018
#define IMX385_FETCH_HIGH_BYTE_VTS(VAL) (((VAL) >> 16) & 0x03)
#define IMX385_FETCH_MID_BYTE_VTS(VAL) (((VAL) >> 8) & 0xFF)
#define IMX385_FETCH_LOW_BYTE_VTS(VAL) ((VAL) & 0xFF)

#define IMX385_FLIP_REG 0x3007
#define MIRROR_BIT_MASK BIT(1)
#define FLIP_BIT_MASK BIT(0)

#define IMX385_EXPOSURE_MIN 0x0002

#define IMX385_EXPOSURE_STEP 0x01
#define IMX385_VTS_MAX 0x7FFF

#define IMX385_GAIN_SWITCH_REG 0x3009
#define IMX385_REG_LF_GAIN_LOW 0x3014
#define IMX385_REG_LF_GAIN_HIGH 0x3015

#define IMX385_GAIN_MIN 0x0000
#define IMX385_GAIN_MAX 0x02D0
#define IMX385_GAIN_STEP 1
#define IMX385_GAIN_DEFAULT 0x00

#define IMX385_GROUP_HOLD_REG 0x3001
#define IMX385_GROUP_HOLD_START 0x01
#define IMX385_GROUP_HOLD_END 0x00

#define USED_TEST_PATTERN
#ifdef USED_TEST_PATTERN
#define IMX327_REG_TEST_PATTERN		0x308c
#define	IMX327_TEST_PATTERN_ENABLE	BIT(0)
#endif


#define REG_NULL 0xFFFF
#define REG_DELAY 0xFFFE

#define IMX385_REG_VALUE_08BIT 1

static bool g_isHCG;

struct regval {
	u16 addr;
	u8 val;
};

static const char *const imx385_supply_names[] = {
	"dvdd", /* Digital core power */
	"dovdd", /* Digital I/O power */
	"avdd", /* Analog power */
};

#define IMX385_NUM_SUPPLIES ARRAY_SIZE(imx385_supply_names)

struct imx385_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	struct rkmodule_lvds_cfg lvds_cfg;
};

struct imx385 {
	struct i2c_client *client;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[IMX385_NUM_SUPPLIES];

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *anal_gain;
	struct v4l2_ctrl *digi_gain;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *h_flip;
	struct v4l2_ctrl *v_flip;
#ifdef USED_TEST_PATTERN
	struct v4l2_ctrl *test_pattern;
#endif
	struct mutex mutex;
	bool streaming;
	bool power_on;
	const struct imx385_mode *support_modes;
	u32 support_modes_num;
	const struct imx385_mode *cur_mode;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	u32 cur_vts;
	bool has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	struct v4l2_fwnode_endpoint bus_cfg;
	u8 flip;
};

#define to_imx385(sd) container_of(sd, struct imx385, subdev)

/*
 * Xclk 37.125Mhz
 */
static const struct regval imx385_global_regs[] = {
	{ REG_NULL, 0x00 },
};

/* Xclk 37.125Mhz, mipi 2-lane, 1920x1080@30fps */
static const struct regval imx385_mipi_2lane_1080p_30fps_regs[] = {
	{ 0x3003, 0x01 },
	{ REG_DELAY, 0x10 },
	{ 0x3000, 0x01 },
	{ 0x3001, 0x00 },
	{ 0x3002, 0x00 },
	{ 0x3005, 0x00 },
	{ 0x3007, 0x00 },
	{ 0x3009, 0x02 },
	{ 0x300A, 0x3C },
	{ 0x300B, 0x00 },
	{ 0x3012, 0x2C },
	{ 0x3013, 0x01 },
	{ 0x3016, 0x08 },
	{ 0x3018, 0x65 },
	{ 0x3019, 0x04 },
	{ 0x301A, 0x00 },
	{ 0x301B, 0x30 },
	{ 0x301C, 0x11 },
	{ 0x3020, 0x7D },
	{ 0x3021, 0x00 },
	{ 0x3022, 0x00 },
	{ 0x3044, 0x00 },
	{ 0x3046, 0x00 },
	{ 0x3047, 0x00 },
	{ 0x3054, 0x00 },
	// INCLK
	{ 0x305C, 0x28 },
	{ 0x305D, 0x00 },
	{ 0x305E, 0x20 },
	{ 0x305F, 0x00 },

	{ 0x310B, 0x07 },
	{ 0x3110, 0x12 },
	{ 0x31ED, 0x38 },

	{ 0x3338, 0xD4 },
	{ 0x3339, 0x40 },
	{ 0x333A, 0x10 },
	{ 0x333B, 0x00 },
	{ 0x333C, 0xD4 },
	{ 0x333D, 0x40 },
	{ 0x333E, 0x10 },
	{ 0x333F, 0x00 },

	{ 0x3344, 0x02 },
	{ 0x3346, 0x01 },
	{ 0x3353, 0x0E },
	{ 0x3357, 0x49 },
	{ 0x3358, 0x04 },
	{ 0x336B, 0x27 },
	{ 0x336C, 0x1F },
	{ 0x337D, 0x0A },
	{ 0x337E, 0x0A },
	{ 0x337F, 0x01 },
	{ 0x3380, 0x20 },
	{ 0x3381, 0x25 },
	{ 0x3382, 0x57 },
	{ 0x3383, 0x0F },
	{ 0x3384, 0x2F },
	{ 0x3385, 0x17 },
	{ 0x3386, 0x0F },
	{ 0x3387, 0x0F },
	{ 0x3388, 0x37 },
	{ 0x3389, 0x1F },
	{ 0x338D, 0xB4 },
	{ 0x338E, 0x01 },
	{ 0x3000, 0x00 },
	{ REG_NULL, 0x00 },
};

static const struct imx385_mode mipi_supported_modes[] = { {
	.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
	.width = 1920,
	.height = 1080,
	.max_fps = {
		.numerator = 10000,
		.denominator = 300000,
	},
	.exp_def = 0x400,
	.hts_def = 0x1130,
	.vts_def = 0x000465,
	.reg_list = imx385_mipi_2lane_1080p_30fps_regs,
	.hdr_mode = NO_HDR,
} };

static const s64 link_freq_menu_items[] = {
	IMX385_LINK_FREQ_371M,
};

#ifdef USED_TEST_PATTERN
static const char * const imx385_test_pattern_menu[] = {
	"Disabled",
	"Bar Type 1",
	"Bar Type 2",
	"Bar Type 3",
	"Bar Type 4",
	"Bar Type 5",
	"Bar Type 6",
	"Bar Type 7",
	"Bar Type 8",
	"Bar Type 9",
	"Bar Type 10",
	"Bar Type 11",
	"Bar Type 12",
	"Bar Type 13",
	"Bar Type 14",
	"Bar Type 15"
};
#endif

/* Write registers up to 4 at a time */
static int imx385_write_reg(struct i2c_client *client, u16 reg, u32 len,
			    u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx385_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val * 1000, regs[i].val * 2000);
		else
			ret = imx385_write_reg(client, regs[i].addr,
					       IMX385_REG_VALUE_08BIT,
					       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int imx385_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);
	return 0;
}

static int imx385_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx385 *imx385 = to_imx385(sd);
	const struct imx385_mode *mode;
	s64 h_blank, vblank_def;
	s32 dst_link_freq = 0;
	s64 dst_pixel_rate = 0;

	mutex_lock(&imx385->mutex);

	mode = v4l2_find_nearest_size(imx385->support_modes,
				      imx385->support_modes_num, width, height,
				      fmt->format.width, fmt->format.height);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx385->mutex);
		return -ENOTTY;
#endif
	} else {
		imx385->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx385->hblank, h_blank, h_blank, 1,
					 h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx385->vblank, vblank_def,
					 IMX385_VTS_MAX - mode->height, 1,
					 vblank_def);
		dst_link_freq = 0;
		dst_pixel_rate = IMX385_LINK_FREQ_371M;
		__v4l2_ctrl_s_ctrl_int64(imx385->pixel_rate, dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(imx385->link_freq, dst_link_freq);
		imx385->cur_vts = mode->vts_def;
	}

	mutex_unlock(&imx385->mutex);

	return 0;
}

static int imx385_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx385 *imx385 = to_imx385(sd);
	const struct imx385_mode *mode = imx385->cur_mode;

	mutex_lock(&imx385->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx385->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx385->mutex);
	return 0;
}

static int imx385_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx385 *imx385 = to_imx385(sd);
	const struct imx385_mode *mode = imx385->cur_mode;

	if (code->index != 0)
		return -EINVAL;
	code->code = mode->bus_fmt;

	return 0;
}

static int imx385_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx385 *imx385 = to_imx385(sd);

	if (fse->index >= imx385->support_modes_num)
		return -EINVAL;

	if (fse->code != imx385->support_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = imx385->support_modes[fse->index].width;
	fse->max_width = imx385->support_modes[fse->index].width;
	fse->max_height = imx385->support_modes[fse->index].height;
	fse->min_height = imx385->support_modes[fse->index].height;

	return 0;
}

static int imx385_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx385 *imx385 = to_imx385(sd);
	const struct imx385_mode *mode = imx385->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int imx385_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct imx385 *imx385 = to_imx385(sd);
	u32 val = 0;

	val = 1 << (IMX385_2LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = imx385->bus_cfg.bus_type;
	config->flags = val;

	return 0;
}

static void imx385_get_module_inf(struct imx385 *imx385,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX385_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx385->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx385->len_name, sizeof(inf->base.lens));
}

static int imx385_set_conversion_gain(struct imx385 *imx385, u32 *cg)
{
	int ret = 0;
	struct i2c_client *client = imx385->client;
	int cur_cg = *cg;
	u32 gain_switch = 0;

	ret = imx385_read_reg(client, IMX385_GAIN_SWITCH_REG,
			      IMX385_REG_VALUE_08BIT, &gain_switch);
	if (g_isHCG && cur_cg == GAIN_MODE_LCG) {
		gain_switch &= 0xef;
		gain_switch |= 0x0100;
		g_isHCG = false;
	} else if (!g_isHCG && cur_cg == GAIN_MODE_HCG) {
		gain_switch |= 0x0110;
		g_isHCG = true;
	}

	if (gain_switch & 0x100) {
		ret |= imx385_write_reg(client, IMX385_GROUP_HOLD_REG,
					IMX385_REG_VALUE_08BIT,
					IMX385_GROUP_HOLD_START);
		ret |= imx385_write_reg(client, IMX385_GAIN_SWITCH_REG,
					IMX385_REG_VALUE_08BIT,
					gain_switch & 0xff);
		ret |= imx385_write_reg(client, IMX385_GROUP_HOLD_REG,
					IMX385_REG_VALUE_08BIT,
					IMX385_GROUP_HOLD_END);
	}

	return ret;
}

#define USED_SYS_DEBUG
#ifdef USED_SYS_DEBUG
//ag: echo 0 >  /sys/devices/platform/ff510000.i2c/i2c-1/1-0037/cam_s_cg
static ssize_t set_conversion_gain_status(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx385 *imx385 = to_imx385(sd);
	int status = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &status);
	if (!ret && status >= 0 && status < 2)
		imx385_set_conversion_gain(imx385, &status);
	else
		dev_err(dev, "input 0 for LCG, 1 for HCG, cur %d\n", status);
	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(cam_s_cg, S_IWUSR, NULL, set_conversion_gain_status),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}
#endif

static long imx385_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx385 *imx385 = to_imx385(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_lvds_cfg *lvds_cfg;
	u32 i, h, w;
	long ret = 0;
	s64 dst_pixel_rate = 0;
	s32 dst_link_freq = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx385_get_module_inf(imx385, (struct rkmodule_inf *)arg);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		ret = 0;
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = NO_HDR;
		break;
	case RKMODULE_SET_HDR_CFG:
		for (i = 0; i < imx385->support_modes_num; i++) {
			if (imx385->support_modes[i].hdr_mode == NO_HDR) {
				imx385->cur_mode = &imx385->support_modes[i];
				break;
			}
		}

		if (i == imx385->support_modes_num) {
			dev_err(&imx385->client->dev,
				"Critical Error: NO_HDR mode not found in support_modes\n");
			ret = -EINVAL;
		} else {
			// 重新计算并应用针对单帧模式的消隐参数 (Blanking)
			w = imx385->cur_mode->hts_def - imx385->cur_mode->width;
			h = imx385->cur_mode->vts_def -
			    imx385->cur_mode->height;

			__v4l2_ctrl_modify_range(imx385->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(
				imx385->vblank, h,
				IMX385_VTS_MAX - imx385->cur_mode->height, 1,
				h);

			// 强制固定为单帧时钟频率和链路速率
			// 忽略上层可能请求的高频率需求
			// 对应 link_freqs 数组中 371M (或你定义的单帧频率) 的索引
			dst_link_freq = 0;
			dst_pixel_rate = IMX385_PIXEL_RATE;

			__v4l2_ctrl_s_ctrl_int64(imx385->pixel_rate,
						 dst_pixel_rate);
			__v4l2_ctrl_s_ctrl(imx385->link_freq, dst_link_freq);

			// 更新当前 VTS 记录，确保单帧曝光计算正确
			imx385->cur_vts = imx385->cur_mode->vts_def;

			dev_info(&imx385->client->dev,
				 "HDR mode forced to NO_HDR, fixed at %dx%d\n",
				 imx385->cur_mode->width,
				 imx385->cur_mode->height);
		}
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = imx385_set_conversion_gain(imx385, (u32 *)arg);
		break;
	case RKMODULE_GET_LVDS_CFG:
		lvds_cfg = (struct rkmodule_lvds_cfg *)arg;
		if (imx385->bus_cfg.bus_type == V4L2_MBUS_CCP2)
			memcpy(lvds_cfg, &imx385->cur_mode->lvds_cfg,
			       sizeof(struct rkmodule_lvds_cfg));
		else
			ret = -ENOIOCTLCMD;
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx385_write_reg(imx385->client,
					       IMX385_REG_CTRL_MODE,
					       IMX385_REG_VALUE_08BIT, 0);
		else
			ret = imx385_write_reg(imx385->client,
					       IMX385_REG_CTRL_MODE,
					       IMX385_REG_VALUE_08BIT, 1);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long imx385_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				  unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx385_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}
		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx385_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx385_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = imx385_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = imx385_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = imx385_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx385_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx385_init_conversion_gain(struct imx385 *imx385)
{
	int ret = 0;
	struct i2c_client *client = imx385->client;
	u32 val = 0;

	ret = imx385_read_reg(client, IMX385_GAIN_SWITCH_REG,
			      IMX385_REG_VALUE_08BIT, &val);
	val &= 0xef;
	ret = imx385_write_reg(client, IMX385_GAIN_SWITCH_REG,
			       IMX385_REG_VALUE_08BIT, val);
	if (!ret)
		g_isHCG = false;
	return ret;
}

static int __imx385_start_stream(struct imx385 *imx385)
{
	int ret;

	ret = imx385_write_array(imx385->client, imx385->cur_mode->reg_list);
	if (ret)
		return ret;
	ret = imx385_init_conversion_gain(imx385);
	if (ret)
		return ret;
	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&imx385->ctrl_handler);
	if (imx385->has_init_exp && imx385->cur_mode->hdr_mode != NO_HDR) {
		ret = imx385_ioctl(&imx385->subdev, PREISP_CMD_SET_HDRAE_EXP,
				   &imx385->init_hdrae_exp);
		if (ret) {
			dev_err(&imx385->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	ret = imx385_write_reg(imx385->client, IMX385_REG_CTRL_MODE,
			       IMX385_REG_VALUE_08BIT, 0);
	return ret;
}

static int __imx385_stop_stream(struct imx385 *imx385)
{
	return imx385_write_reg(imx385->client, IMX385_REG_CTRL_MODE,
				IMX385_REG_VALUE_08BIT, 1);
}

static int imx385_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx385 *imx385 = to_imx385(sd);
	struct i2c_client *client = imx385->client;
	int ret = 0;

	mutex_lock(&imx385->mutex);
	on = !!on;
	if (on == imx385->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx385_start_stream(imx385);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx385_stop_stream(imx385);
		pm_runtime_put(&client->dev);
	}

	imx385->streaming = on;

unlock_and_return:
	mutex_unlock(&imx385->mutex);

	return ret;
}

static int imx385_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx385 *imx385 = to_imx385(sd);
	struct i2c_client *client = imx385->client;
	int ret = 0;

	mutex_lock(&imx385->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx385->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = imx385_write_array(imx385->client, imx385_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx385->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx385->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx385->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx385_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX385_XVCLK_FREQ / 1000 / 1000);
}

static int __imx385_power_on(struct imx385 *imx385)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx385->client->dev;

	if (!IS_ERR_OR_NULL(imx385->pins_default)) {
		ret = pinctrl_select_state(imx385->pinctrl,
					   imx385->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(imx385->xvclk, IMX385_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (37.125M Hz)\n");

	if (clk_get_rate(imx385->xvclk) != IMX385_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched,based on 24M Hz\n");

	ret = clk_prepare_enable(imx385->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(IMX385_NUM_SUPPLIES, imx385->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx385->reset_gpio))
		gpiod_set_value_cansleep(imx385->reset_gpio, 0);
	usleep_range(500, 1000);
	if (!IS_ERR(imx385->reset_gpio))
		gpiod_set_value_cansleep(imx385->reset_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR(imx385->pwdn_gpio))
		gpiod_set_value_cansleep(imx385->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx385_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	usleep_range(5000, 10000);
	return 0;

disable_clk:
	clk_disable_unprepare(imx385->xvclk);

	return ret;
}

static void __imx385_power_off(struct imx385 *imx385)
{
	int ret;
	struct device *dev = &imx385->client->dev;

	if (!IS_ERR(imx385->pwdn_gpio))
		gpiod_set_value_cansleep(imx385->pwdn_gpio, 0);
	clk_disable_unprepare(imx385->xvclk);
	if (!IS_ERR(imx385->reset_gpio))
		gpiod_set_value_cansleep(imx385->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(imx385->pins_sleep)) {
		ret = pinctrl_select_state(imx385->pinctrl, imx385->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(IMX385_NUM_SUPPLIES, imx385->supplies);
}

static int imx385_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx385 *imx385 = to_imx385(sd);

	return __imx385_power_on(imx385);
}

static int imx385_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx385 *imx385 = to_imx385(sd);

	__imx385_power_off(imx385);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx385_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx385 *imx385 = to_imx385(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx385_mode *def_mode = &imx385->support_modes[0];

	mutex_lock(&imx385->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx385->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int
imx385_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx385 *imx385 = to_imx385(sd);

	if (fie->index >= imx385->support_modes_num)
		return -EINVAL;

	fie->code = imx385->support_modes[fie->index].bus_fmt;
	fie->width = imx385->support_modes[fie->index].width;
	fie->height = imx385->support_modes[fie->index].height;
	fie->interval = imx385->support_modes[fie->index].max_fps;
	fie->reserved[0] = imx385->support_modes[fie->index].hdr_mode;
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH 1920
#define DST_HEIGHT 1080

/*
 * The resolution of the driver configuration needs to be exactly
 * the same as the current output resolution of the sensor,
 * the input width of the isp needs to be 16 aligned,
 * the input height of the isp needs to be 8 aligned.
 * Can be cropped to standard resolution by this function,
 * otherwise it will crop out strange resolution according
 * to the alignment rules.
 */
static int imx385_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct imx385 *imx385 = to_imx385(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = CROP_START(imx385->cur_mode->width, DST_WIDTH);
		sel->r.width = DST_WIDTH;
		if (imx385->bus_cfg.bus_type == V4L2_MBUS_CCP2) {
			if (imx385->cur_mode->hdr_mode == NO_HDR)
				sel->r.top = 21;
			else
				sel->r.top = 13;
		} else {
			sel->r.top = CROP_START(imx385->cur_mode->height,
						DST_HEIGHT);
		}
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static const struct dev_pm_ops imx385_pm_ops = { SET_RUNTIME_PM_OPS(
	imx385_runtime_suspend, imx385_runtime_resume, NULL) };

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx385_internal_ops = {
	.open = imx385_open,
};
#endif

static const struct v4l2_subdev_core_ops imx385_core_ops = {
	.s_power = imx385_s_power,
	.ioctl = imx385_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx385_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx385_video_ops = {
	.s_stream = imx385_s_stream,
	.g_frame_interval = imx385_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx385_pad_ops = {
	.enum_mbus_code = imx385_enum_mbus_code,
	.enum_frame_size = imx385_enum_frame_sizes,
	.enum_frame_interval = imx385_enum_frame_interval,
	.get_fmt = imx385_get_fmt,
	.set_fmt = imx385_set_fmt,
	.get_selection = imx385_get_selection,
	.get_mbus_config = imx385_g_mbus_config,
};

static const struct v4l2_subdev_ops imx385_subdev_ops = {
	.core = &imx385_core_ops,
	.video = &imx385_video_ops,
	.pad = &imx385_pad_ops,
};

static int imx385_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx385 *imx385 =
		container_of(ctrl->handler, struct imx385, ctrl_handler);
	struct i2c_client *client = imx385->client;
	s64 max;
	int ret = 0;
	u32 shs1 = 0;
	u32 vts = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx385->cur_mode->height + ctrl->val - 2;
		__v4l2_ctrl_modify_range(imx385->exposure,
					 imx385->exposure->minimum, max,
					 imx385->exposure->step,
					 imx385->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (imx385->cur_mode->hdr_mode == NO_HDR) {
			shs1 = imx385->cur_vts - ctrl->val - 1;
			ret = imx385_write_reg(
				imx385->client, IMX385_REG_SHS1_H,
				IMX385_REG_VALUE_08BIT,
				IMX385_FETCH_HIGH_BYTE_EXP(shs1));
			ret |= imx385_write_reg(
				imx385->client, IMX385_REG_SHS1_M,
				IMX385_REG_VALUE_08BIT,
				IMX385_FETCH_MID_BYTE_EXP(shs1));
			ret |= imx385_write_reg(
				imx385->client, IMX385_REG_SHS1_L,
				IMX385_REG_VALUE_08BIT,
				IMX385_FETCH_LOW_BYTE_EXP(shs1));
			dev_dbg(&client->dev,
				"set exposure 0x%x, cur_vts 0x%x,shs1 0x%x\n",
				ctrl->val, imx385->cur_vts, shs1);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (imx385->cur_mode->hdr_mode == NO_HDR) {
			ret = imx385_write_reg(imx385->client,
					       IMX385_REG_LF_GAIN_LOW,
					       IMX385_REG_VALUE_08BIT,
					       ctrl->val & 0xFF);

			/* 写入高 2 位 (GAIN [9:8]) 到 3015h */
			ret |= imx385_write_reg(imx385->client,
						IMX385_REG_LF_GAIN_HIGH,
						IMX385_REG_VALUE_08BIT,
						(ctrl->val >> 8) & 0x03);

			dev_dbg(&client->dev,
				"set analog gain 0x%x (%.1f dB)\n", ctrl->val,
				(float)ctrl->val / 10.0);
		}
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + imx385->cur_mode->height;
		imx385->cur_vts = vts;
		if (imx385->cur_mode->hdr_mode == HDR_X2)
			vts /= 2;
		ret = imx385_write_reg(imx385->client, IMX385_REG_VTS_H,
				       IMX385_REG_VALUE_08BIT,
				       IMX385_FETCH_HIGH_BYTE_VTS(vts));
		ret |= imx385_write_reg(imx385->client, IMX385_REG_VTS_M,
					IMX385_REG_VALUE_08BIT,
					IMX385_FETCH_MID_BYTE_VTS(vts));
		ret |= imx385_write_reg(imx385->client, IMX385_REG_VTS_L,
					IMX385_REG_VALUE_08BIT,
					IMX385_FETCH_LOW_BYTE_VTS(vts));
		dev_dbg(&client->dev, "set vts 0x%x\n", vts);
		break;
	case V4L2_CID_TEST_PATTERN:
#ifdef USED_TEST_PATTERN
		ret = 0;
#endif
		break;
	case V4L2_CID_HFLIP:
		ret = imx385_read_reg(client, IMX385_FLIP_REG,
				      IMX385_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= MIRROR_BIT_MASK;
		else
			val &= ~MIRROR_BIT_MASK;
		ret |= imx385_write_reg(client, IMX385_FLIP_REG,
					IMX385_REG_VALUE_08BIT, val);
		if (ret == 0)
			imx385->flip = val;
		break;
	case V4L2_CID_VFLIP:
		ret = imx385_read_reg(client, IMX385_FLIP_REG,
				      IMX385_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		ret |= imx385_write_reg(client, IMX385_FLIP_REG,
					IMX385_REG_VALUE_08BIT, val);
		if (ret == 0)
			imx385->flip = val;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx385_ctrl_ops = {
	.s_ctrl = imx385_set_ctrl,
};

static int imx385_initialize_controls(struct imx385 *imx385)
{
	const struct imx385_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	s32 dst_link_freq = 0;
	s64 dst_pixel_rate = 0;

	handler = &imx385->ctrl_handler;
	mode = imx385->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &imx385->mutex;

	imx385->link_freq = v4l2_ctrl_new_int_menu(
		handler, NULL, V4L2_CID_LINK_FREQ, 1, 0, link_freq_menu_items);

	dst_link_freq = 0;
	dst_pixel_rate = IMX385_PIXEL_RATE;
	__v4l2_ctrl_s_ctrl(imx385->link_freq, dst_link_freq);
	imx385->pixel_rate =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
				  IMX385_PIXEL_RATE, 1, dst_pixel_rate);

	h_blank = mode->hts_def - mode->width;

	imx385->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (imx385->hblank)
		imx385->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx385->cur_vts = mode->vts_def;
	imx385->vblank =
		v4l2_ctrl_new_std(handler, &imx385_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, IMX385_VTS_MAX - mode->height, 1,
				  vblank_def);

	exposure_max = mode->vts_def - 2;

	imx385->exposure =
		v4l2_ctrl_new_std(handler, &imx385_ctrl_ops, V4L2_CID_EXPOSURE,
				  IMX385_EXPOSURE_MIN, exposure_max,
				  IMX385_EXPOSURE_STEP, mode->exp_def);

	imx385->anal_gain =
		v4l2_ctrl_new_std(handler, &imx385_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, IMX385_GAIN_MIN,
				  IMX385_GAIN_MAX, IMX385_GAIN_STEP,
				  IMX385_GAIN_DEFAULT);

#ifdef USED_TEST_PATTERN
	imx385->test_pattern = v4l2_ctrl_new_std_menu_items(
		handler, &imx385_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(imx385_test_pattern_menu) - 1, 0, 0,
		imx385_test_pattern_menu);
#endif
	imx385->h_flip = v4l2_ctrl_new_std(handler, &imx385_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx385->v_flip = v4l2_ctrl_new_std(handler, &imx385_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx385->flip = 0;
	if (handler->error) {
		ret = handler->error;
		dev_err(&imx385->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	imx385->subdev.ctrl_handler = handler;
	imx385->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}


static int imx385_check_sensor_id(struct imx385 *imx385,
				  struct i2c_client *client)
{
	struct device *dev = &imx385->client->dev;
	u32 id = 0;
	int ret;

	ret = imx385_read_reg(client, IMX385_REG_CHIP_ID,
			      IMX385_REG_VALUE_08BIT, &id);

	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -EINVAL;
	}
	return ret;
}

static int imx385_configure_regulators(struct imx385 *imx385)
{
	unsigned int i;

	for (i = 0; i < IMX385_NUM_SUPPLIES; i++)
		imx385->supplies[i].supply = imx385_supply_names[i];

	return devm_regulator_bulk_get(&imx385->client->dev,
				       IMX385_NUM_SUPPLIES,
				       imx385->supplies);
}


static int imx385_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx385 *imx385;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	struct device_node *endpoint;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	imx385 = devm_kzalloc(dev, sizeof(*imx385), GFP_KERNEL);
	if (!imx385)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx385->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx385->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx385->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx385->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
		&imx385->bus_cfg);
	if (imx385->bus_cfg.bus_type == V4L2_MBUS_CCP2)
		dev_warn(dev, "CCP2 is not supported, fallback to MIPI mode\n");
	imx385->support_modes = mipi_supported_modes;
	imx385->support_modes_num = ARRAY_SIZE(mipi_supported_modes);
	imx385->client = client;
	imx385->cur_mode = &imx385->support_modes[0];

	imx385->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx385->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx385->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx385->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx385->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx385->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = imx385_configure_regulators(imx385);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	imx385->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx385->pinctrl)) {
		imx385->pins_default =
			pinctrl_lookup_state(imx385->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx385->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		imx385->pins_sleep =
			pinctrl_lookup_state(imx385->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx385->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&imx385->mutex);

	sd = &imx385->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx385_subdev_ops);
	ret = imx385_initialize_controls(imx385);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx385_power_on(imx385);
	if (ret)
		goto err_free_handler;

	ret = imx385_check_sensor_id(imx385, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	dev_err(dev, "set the video v4l2 subdev api\n");
	sd->internal_ops = &imx385_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	dev_err(dev, "set the media controller\n");
	imx385->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx385->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx385->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx385->module_index, facing,
		 IMX385_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	g_isHCG = false;
#ifdef USED_SYS_DEBUG
	add_sysfs_interfaces(dev);
#endif
	dev_err(dev, "v4l2 async register subdev success\n");
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx385_power_off(imx385);
err_free_handler:
	v4l2_ctrl_handler_free(&imx385->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx385->mutex);

	return ret;
}


static int imx385_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx385 *imx385 = to_imx385(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx385->ctrl_handler);
	mutex_destroy(&imx385->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx385_power_off(imx385);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx385_of_match[] = {
	{ .compatible = "sony,imx385" },
	{}
};
MODULE_DEVICE_TABLE(of, imx385_of_match);
#endif

static const struct i2c_device_id imx385_match_id[] = { { "sony,imx385", 0 },
							{} };

static struct i2c_driver imx385_i2c_driver = {
	.driver = {
		.name = IMX385_NAME,
		.of_match_table = of_match_ptr(imx385_of_match),
	},
	.probe = &imx385_probe,
	.remove = &imx385_remove,
	.id_table = imx385_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx385_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx385_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx385 sensor driver (mipi 2lane 1080p30 only)");
MODULE_LICENSE("GPL v2");
