/*
 * A V4L2 driver for rn6854m cameras and AHD Coax protocol.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Chen Weihong <chenweihong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include "camera.h"
#include "sensor_helper.h"
#include "ov5640_dvp_firmware.h"


MODULE_AUTHOR("cwh");
MODULE_DESCRIPTION("A low-level driver for ov5640 mipi chip for cvbs sensor");
MODULE_LICENSE("GPL");

/*define module timing*/
#define MCLK				(24*1000*1000)
#define VREF_POL			V4L2_MBUS_VSYNC_ACTIVE_LOW
#define HREF_POL			V4L2_MBUS_HSYNC_ACTIVE_HIGH

//	V4L2_MBUS_PCLK_SAMPLE_RISING or V4L2_MBUS_PCLK_SAMPLE_FALLING
#define CLK_POL				V4L2_MBUS_PCLK_SAMPLE_RISING

#define DRIVER_NAME "ov5640_dvp"

/*
    * OV5640 register definitions
	 */
#define REG_SYSTEM_CTROL0			0x3008
#define REG_CHIP_ID_HIGH		0x300a
#define REG_CHIP_ID_LOW			0x300b

#define OV5640_SOFT_RESET			(1 << 7)

#define OV564X_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))
#define OV5640_ID			0x5640

#define SENSOR_FRAME_RATE	30

#define I2C_ADDR  0x78

#define SENSOR_NAME "ov5640_dvp"


static struct regval_list ov5640_default_regs[] = {

	//缺省色彩饱和度
	{0x3212, 0x03}, 
	{0x5381, 0x1c},
	{0x5382, 0x5a}, {0x5383, 0x06}, {0x5384, 0x1a},
	{0x5385, 0x66}, {0x5386, 0x80}, {0x5387, 0x82},
	{0x5388, 0x80}, {0x5389, 0x02}, {0x538a, 0x01},
	{0x538b, 0x98}, 
	{0x3212, 0x13}, { 0x3212, 0xa3},


	//亮度 +1
	{ 0x3212, 0x03 },
	{ 0x5587, 0x10 }, { 0x5588, 0x01}, {0x3212, 0x13 }, {0x3212, 0xa3 },

	
	//曝光补偿	
	{0x3a0f, 0x30}, {0x3a10, 0x28}, {0x3a1b, 0x30},
	{0x3a1e, 0x26}, {0x3a11, 0x60}, {0x3a1f, 0x14},

	{0x3a0f, 0x40}, {0x3a10, 0x38}, {0x3a11, 0x71},
	{0x3a1b, 0x40}, {0x3a1e, 0x38}, {0x3a1f, 0x10},

	
	//特效
//	{0x5580, 0x02}, {0x5583, 0x40}, {0x5584, 0x10},
	{0x5580, 0x06}, {0x5583, 0x40}, {0x5584, 0x10}, { 0x5003, 0x08 },

	//环境光模式 日光
//	{0x3406, 0x01}, {0x3400, 0x06}, {0x3401, 0x1c}, {0x3402, 0x04},
//	{0x3403, 0x00}, {0x3404, 0x04}, {0x3405, 0xf3},

	// 环境光模式  自动
	{0x3212, 0x03},
	{0x3406, 0x00}, {0x3400, 0x04}, {0x3401, 0x00},
	{0x3402, 0x04}, {0x3403, 0x00}, {0x3404, 0x04},
	{0x3405, 0x00}, {0x3212, 0x13}, {0x3212, 0xa3},

	//	去除灯光条纹
	{0x3a00, 0x34}, // 0x78 0x34
};

static struct regval_list ov5640_init_regs[] = {
	{0x3008, 0x42},
	{0x3103, 0x03}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3034, 0x1a}, {0x3037, 0x13}, {0x3108, 0x01},
	{0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2},
	{0x3633, 0x12}, {0x3621, 0xe0}, {0x3704, 0xa0},
	{0x3703, 0x5a}, {0x3715, 0x78}, {0x3717, 0x01},
	{0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12},
	{0x3600, 0x08}, {0x3601, 0x33}, {0x302d, 0x60},
	{0x3620, 0x52}, {0x371b, 0x20}, {0x471c, 0x50},
	{0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0x7c},
	{0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40},
	{0x3622, 0x01}, {0x3c04, 0x28},
	{0x3c05, 0x98}, {0x3c06, 0x00}, {0x3c07, 0x07},
	{0x3c08, 0x00}, {0x3c09, 0x1c}, {0x3c0a, 0x9c},
	{0x3c0b, 0x40}, {0x3810, 0x00}, {0x3811, 0x10},
	{0x3812, 0x00}, {0x3708, 0x64}, {0x4001, 0x02},
	{0x4005, 0x1a}, {0x3000, 0x00}, {0x3004, 0xff},

	{0x300e, 0x58}, {0x302e, 0x00}, {0x4300, 0x31},
	{0x501f, 0x00}, {0x440e, 0x00}, {0x5000, 0xa7},
	{0x3008, 0x02},
};


/* 2592x1944 QSXGA */
static struct regval_list ov5640_qsxga_regs[] = {//caichsh
	{0x3503, 0x07},
	{0x3a00, 0x78},
	{0x350c, 0x00}, {0x350d, 0x00}, {0x3c07, 0x07},
	{0x3820, 0x46}, {0x3821, 0x00}, {0x3814, 0x11},
	{0x3815, 0x11}, {0x3803, 0x00}, {0x3807, 0x9f},
	{0x3808, 0x0a}, {0x3809, 0x20}, {0x380a, 0x07},
	{0x380b, 0x98}, {0x380c, 0x0b}, {0x380d, 0x1c},
	{0x380e, 0x07}, {0x380f, 0xb0}, {0x3813, 0x04},
	{0x3618, 0x04}, {0x3612, 0x2b}, {0x3709, 0x12},
	{0x370c, 0x00}, {0x3a02, 0x07}, {0x3a03, 0xb0},
	{0x3a0e, 0x06}, {0x3a0d, 0x08}, {0x3a14, 0x07},
	{0x3a15, 0xb0}, {0x4004, 0x06}, {0x3035, 0x21},
	{0x3036, 0x46}, {0x4837, 0x2c}, {0x5001, 0x83},
	{0x3503, 0x00},

	{0x3212, 0x03}, {0x5580, 0x06}, {0x5583, 0x40},
	{0x5584, 0x10}, {0x5003, 0x08}, {0x3212, 0x13},
	{0x3212, 0xa3},

	{0x3212, 0x03},
	{0x3406, 0x00}, {0x3400, 0x04}, {0x3401, 0x00},
	{0x3402, 0x04}, {0x3403, 0x00}, {0x3404, 0x04},
	{0x3405, 0x00}, {0x3212, 0x13}, {0x3212, 0xa3},
};

static struct regval_list ov5640_vga_regs[] = {
	{0x3008, 0x42},
	{0x3017, 0x00}, {0x3018, 0x00},
	{0x3034, 0x1a}, {0x3035, 0x11}, {0x3036, 0x46},
	{0x3037, 0x13}, {0x3108, 0x01}, {0x3630, 0x36},
	{0x3631, 0x0e}, {0x3632, 0xe2}, {0x3633, 0x12},
	{0x3621, 0xe0}, {0x3704, 0xa0}, {0x3703, 0x5a},
	{0x3715, 0x78}, {0x3717, 0x01}, {0x370b, 0x60},
	{0x3705, 0x1a}, {0x3905, 0x02}, {0x3906, 0x10},
	{0x3901, 0x0a}, {0x3731, 0x12}, {0x3600, 0x08},
	{0x3601, 0x33}, {0x302d, 0x60}, {0x3620, 0x52},
	{0x371b, 0x20}, {0x471c, 0x50}, {0x3a13, 0x43},
	{0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3635, 0x13},
	{0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01},
	{0x3c04, 0x28}, {0x3c05, 0x98},
	{0x3c06, 0x00}, {0x3c07, 0x08}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x46}, {0x3821, 0x00}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x04}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9b},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01},
	{0x380b, 0xe0}, {0x380c, 0x07}, {0x380d, 0x68},
	{0x380e, 0x03}, {0x380f, 0xd8}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x06},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3708, 0x64},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x02}, {0x3000, 0x00},
	{0x3002, 0x1c}, {0x3004, 0xff}, {0x3006, 0xc3},
	{0x300e, 0x58}, {0x302e, 0x00}, {0x4300, 0x30},
	{0x501f, 0x00}, {0x4713, 0x02}, {0x4407, 0x04},
	{0x440e, 0x00}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x4837, 0x22}, {0x3824, 0x02}, {0x5000, 0xa7},
	{0x5001, 0xa3}, {0x5180, 0xff}, {0x5181, 0xf2},
	{0x5182, 0x00}, {0x5183, 0x14}, {0x5184, 0x25},
	{0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
	{0x5188, 0x09}, {0x5189, 0x88}, {0x518a, 0x54},
	{0x518b, 0xee}, {0x518c, 0xb2}, {0x518d, 0x50},
	{0x518e, 0x34}, {0x518f, 0x6b}, {0x5190, 0x46},
	{0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
	{0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03},
	{0x5197, 0x01}, {0x5198, 0x04}, {0x5199, 0x6c},
	{0x519a, 0x04}, {0x519b, 0x00}, {0x519c, 0x09},
	{0x519d, 0x2b}, {0x519e, 0x38}, 
	
	{0x5300, 0x08}, {0x5301, 0x30},
	{0x5302, 0x10}, {0x5303, 0x00}, {0x5304, 0x08},
	{0x5305, 0x30}, {0x5306, 0x08}, {0x5307, 0x16},
	{0x5309, 0x08}, {0x530a, 0x30}, {0x530b, 0x04},
	{0x530c, 0x06}, {0x5480, 0x01}, {0x5481, 0x08},
	{0x5482, 0x14}, {0x5483, 0x28}, {0x5484, 0x51},
	{0x5485, 0x65}, {0x5486, 0x71}, {0x5487, 0x7d},
	{0x5488, 0x87}, {0x5489, 0x91}, {0x548a, 0x9a},
	{0x548b, 0xaa}, {0x548c, 0xb8}, {0x548d, 0xcd},
	{0x548e, 0xdd}, {0x548f, 0xea}, {0x5490, 0x1d},

	{0x5589, 0x10}, {0x558a, 0x00}, {0x558b, 0xf8},
	{0x5800, 0x23}, {0x5801, 0x14}, {0x5802, 0x0f},
	{0x5803, 0x0f}, {0x5804, 0x12}, {0x5805, 0x26},
	{0x5806, 0x0c}, {0x5807, 0x08}, {0x5808, 0x05},
	{0x5809, 0x05}, {0x580a, 0x08}, {0x580b, 0x0d},
	{0x580c, 0x08}, {0x580d, 0x03}, {0x580e, 0x00},
	{0x580f, 0x00}, {0x5810, 0x03}, {0x5811, 0x09},
	{0x5812, 0x07}, {0x5813, 0x03}, {0x5814, 0x00},
	{0x5815, 0x01}, {0x5816, 0x03}, {0x5817, 0x08},
	{0x5818, 0x0d}, {0x5819, 0x08}, {0x581a, 0x05},
	{0x581b, 0x06}, {0x581c, 0x08}, {0x581d, 0x0e},
	{0x581e, 0x29}, {0x581f, 0x17}, {0x5820, 0x11},
	{0x5821, 0x11}, {0x5822, 0x15}, {0x5823, 0x28},
	{0x5824, 0x46}, {0x5825, 0x26}, {0x5826, 0x08},
	{0x5827, 0x26}, {0x5828, 0x64}, {0x5829, 0x26},
	{0x582a, 0x24}, {0x582b, 0x22}, {0x582c, 0x24},
	{0x582d, 0x24}, {0x582e, 0x06}, {0x582f, 0x22},
	{0x5830, 0x40}, {0x5831, 0x42}, {0x5832, 0x24},
	{0x5833, 0x26}, {0x5834, 0x24}, {0x5835, 0x22},
	{0x5836, 0x22}, {0x5837, 0x26}, {0x5838, 0x44},
	{0x5839, 0x24}, {0x583a, 0x26}, {0x583b, 0x28},
	{0x583c, 0x42}, {0x583d, 0xce}, {0x5025, 0x00},


	{0x3008, 0x02}, {0x3034, 0x1a}, {0x3035, 0x11},
	{0x3036, 0x46}, {0x3037, 0x13},
};


/* 720p 15fps @ 1280x720 */
static struct regval_list ov5640_720p_regs[] = {
	{0x3035, 0x21}, {0x3036, 0x69}, {0x3c07, 0x07},
	{0x3820, 0x46}, {0x3821, 0x00}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0xfa}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x06}, {0x3807, 0xa9},
	{0x3808, 0x05}, {0x3809, 0x00}, {0x380a, 0x02},
	{0x380b, 0xd0}, {0x380c, 0x07}, {0x380d, 0x64},
	{0x380e, 0x02}, {0x380f, 0xe4}, {0x3813, 0x04},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3709, 0x52},
	{0x370c, 0x03}, {0x3a02, 0x02}, {0x3a03, 0xe0},
	{0x3a14, 0x02}, {0x3a15, 0xe0}, {0x4004, 0x02},
	{0x3002, 0x1c}, {0x3006, 0xc3}, {0x4713, 0x02},
	{0x4407, 0x0C}, {0x460b, 0x37}, {0x460c, 0x20},
	{0x4837, 0x16}, {0x3824, 0x04}, {0x5001, 0x83},
	{0x3503, 0x00},
};

static struct regval_list ov5640_1080p_regs[] = {
	{0x3008, 0x42},
	{0x3035, 0x21}, {0x3036, 0x54}, {0x3c07, 0x08},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x46}, {0x3821, 0x00}, {0x3814, 0x11},
	{0x3815, 0x11}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x00}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9f},
	{0x3808, 0x0a}, {0x3809, 0x20}, {0x380a, 0x07},
	{0x380b, 0x98}, {0x380c, 0x0b}, {0x380d, 0x1c},
	{0x380e, 0x07}, {0x380f, 0xb0}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x04},
	{0x3618, 0x04}, {0x3612, 0x29}, {0x3708, 0x21},
	{0x3709, 0x12}, {0x370c, 0x00}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x06}, {0x4713, 0x02},
	{0x4407, 0x04}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x3824, 0x02}, {0x5001, 0x83}, {0x3035, 0x21},
	{0x3036, 0x54}, {0x3c07, 0x07}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3800, 0x01}, {0x3801, 0x50}, {0x3802, 0x01},
	{0x3803, 0xb2}, {0x3804, 0x08}, {0x3805, 0xef},
	{0x3806, 0x05}, {0x3807, 0xf1}, {0x3808, 0x07},
	{0x3809, 0x80}, {0x380a, 0x04}, {0x380b, 0x38},
	{0x380c, 0x09}, {0x380d, 0xc4}, {0x380e, 0x04},
	{0x380f, 0x60}, {0x3612, 0x2b}, {0x3708, 0x64},
	{0x3a02, 0x04}, {0x3a03, 0x60}, {0x3a08, 0x01},
	{0x3a09, 0x50}, {0x3a0a, 0x01}, {0x3a0b, 0x18},
	{0x3a0e, 0x03}, {0x3a0d, 0x04}, {0x3a14, 0x04},
	{0x3a15, 0x60}, {0x4713, 0x02}, {0x4407, 0x04},
	{0x460b, 0x37}, {0x460c, 0x20}, {0x3824, 0x04},
	{0x4005, 0x1a}, {0x3008, 0x02}, {0x3503, 0},
};

static int ov5640_af_setting(struct v4l2_subdev *client);

/*
 * window size list
 */
#define VGA_WIDTH           640
#define VGA_HEIGHT          480

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
		case STBY_ON:
			break;
		case STBY_OFF:
			break;
		case PWR_ON:
			cci_lock(sd);
			//	vin_gpio_set_status(sd, RESET, CSI_GPIO_HIGH);
			//	vin_gpio_set_status(sd, PWDN, CSI_GPIO_HIGH);
			//			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			//			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);

			vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			vin_set_mclk_freq(sd, MCLK);
			vin_set_mclk(sd, ON);
			vin_set_pmu_channel(sd, CAMERAVDD, ON);
			vin_set_pmu_channel(sd, IOVDD, ON);
			vin_set_pmu_channel(sd, DVDD, ON);
			vin_set_pmu_channel(sd, AVDD, ON);
			usleep_range(5000, 6000);
			vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
			usleep_range(20000, 22000);
			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			usleep_range(20000, 22000);
			cci_unlock(sd);

			break;
		case PWR_OFF:
			cci_lock(sd);
			vin_set_mclk(sd, OFF);
			//vin_gpio_set_status(sd, RESET, CSI_GPIO_HIGH);
			//vin_gpio_set_status(sd, PWDN, CSI_GPIO_HIGH);
			//			vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
			//			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);

			vin_set_pmu_channel(sd, CAMERAVDD, OFF);
			vin_set_pmu_channel(sd, IOVDD, OFF);
			vin_set_pmu_channel(sd, DVDD, OFF);
			vin_set_pmu_channel(sd, AVDD, OFF);
			vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
			vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
			usleep_range(1000, 1200);
			vin_set_mclk(sd, OFF);
			//vin_gpio_set_status(sd, RESET, CSI_GPIO_LOW);
			cci_unlock(sd);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
	usleep_range(5000, 6000);
	vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
	usleep_range(5000, 6000);
	return 0;
}


static int ov5640_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int ret = 0;
	int i = 0;
	for (i = 0; i < array_size; i++) {
		if (regs[i].addr == REG_DLY) {
			msleep(regs[i].data);
		} else {
			ret = sensor_write(sd, regs[i].addr, regs[i].data);
		}
	}

	return ret;
}

static int ov5640_read(struct v4l2_subdev *sd, data_type reg, data_type *val)
{
	return cci_read(sd, reg, val);
}

static int ov5640_write(struct v4l2_subdev *sd, data_type reg, data_type *val)
{
	return cci_write(sd, reg, val);
}

static struct regval_list ov5640_reset_regs[] = {
	{0x3103,0x11},
	{0x3008,0x82},
	{0x3f00,0x01},
	{0x3000,0x60},
	{0x3001,0x40},
};

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	data_type rdval = 0;
	data_type pid, ver;
	printk("!!!! %s\n", __func__);
	ret = sensor_write(sd, 0x3103, 0x11);
	if (ret < 0)
		return -ENODEV;
	printk("ov5640 sensor write cmd !\n");
	ret = ov5640_write_array(sd, ov5640_reset_regs, sizeof(ov5640_reset_regs) / sizeof(ov5640_reset_regs[0]));
	if (ret < 0)
		return -ENODEV;
	printk("ov5640 sensor write reg !\n");
	msleep(100);
	ret = ov5640_write_array(sd, ov5640_init_regs, sizeof(ov5640_init_regs) / sizeof(ov5640_init_regs[0]));
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to ov5640_write_array init regs\n", __func__);
		return -EIO;
	}

	/* Check sensor revision */
	ret = ov5640_read(sd, REG_CHIP_ID_HIGH, &pid);
	if (!ret)
		ret = ov5640_read(sd, REG_CHIP_ID_LOW, &ver);

	if (!ret) {
		unsigned short id;

		id = OV564X_ID(pid, ver);
		printk("ov5640 id:%x\n", id); 
		if (id != OV5640_ID) {
			ret = -ENODEV;
		}
	} else
		return -ENODEV;


	return 0;
}
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0; 
	info->low_speed = 0; 
	info->width = 1280; 
	info->height = 720; 
	info->brightness = 0; 
	info->contrast = 0; 
	info->saturation = 0; 
	info->hue = 0; 
	info->hflip = 0; 
	info->vflip = 0; 
	info->gain = 0; 
	info->autogain = 1; 
	info->exp_bias = 0; 
	info->autoexp = 1; 
	info->autowb = 1; 

	info->wb = V4L2_WHITE_BALANCE_AUTO;
	info->clrfx = V4L2_COLORFX_NONE;
	info->band_filter = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

	info->tpf.numerator = 1; 
	info->tpf.denominator = 30;
	info->preview_first_flag = 1; 

	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);
	return 0;
}

static struct regval_list ov5640_fmt_yuv422_yuyv[] =
{
	{0x4300, 0x30},
};

static struct regval_list ov5640_fmt_yuv422_yvyu[] =
{
	{0x4300, 0x31},
};

static struct regval_list ov5640_fmt_yuv422_vyuy[] =
{
	{0x4300, 0x33},
};

static struct regval_list ov5640_fmt_yuv422_uyvy[] =
{
	{0x4300, 0x32},
};

static struct regval_list ov5640_fmt_raw[] __attribute__((unused)) =
{
	{0x4300, 0x00},
};


static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "YUYV 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.regs = ov5640_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(ov5640_fmt_yuv422_yuyv),
		.bpp = 2,
	}, {
		.desc = "YVYU 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.regs = ov5640_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(ov5640_fmt_yuv422_yvyu),
		.bpp = 2,
	}, {
		.desc = "UYVY 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.regs = ov5640_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(ov5640_fmt_yuv422_uyvy),
		.bpp = 2,
	}, {
		.desc = "VYUY 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_VYUY8_2X8,
		.regs = ov5640_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(ov5640_fmt_yuv422_vyuy),
		.bpp = 2,
	}, 
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
static struct sensor_win_size sensor_win_sizes[] = {
	// 640X480
	{
		.width = VGA_WIDTH,
		.height = VGA_HEIGHT,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30, 
		.regs = ov5640_vga_regs,
		.regs_size = ARRAY_SIZE(ov5640_vga_regs),
		.set_size = NULL,
	},
	// 1280x 720
	{
		.width = 1280,
		.height = 720,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30, 
		.regs = ov5640_720p_regs,
		.regs_size = ARRAY_SIZE(ov5640_720p_regs),
		.set_size = NULL,
	},
	{
		.width = 1920,
		.height = 1080,	//1088
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30,
		.regs = ov5640_1080p_regs,
		.regs_size = ARRAY_SIZE(ov5640_1080p_regs),
		.set_size = NULL,
	},

	// 2592x1944	1952
	{
		.width = 2592,
		.height = 1944,
		.hoffset = 0,
		.voffset = 0,
		.fps_fixed = 30, 
		.regs = ov5640_qsxga_regs,
		.regs_size = ARRAY_SIZE(ov5640_qsxga_regs),
		.set_size = NULL,
	}
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *cfg)
{

	/*
	   V4L2_MBUS_PARALLEL
	   parallel interface with hsync and vsync
	   V4L2_MBUS_BT656
	   parallel interface with embedded synchronisation, can also be used for BT.1120
	   V4L2_MBUS_CSI2
	   MIPI CSI-2 serial interface
	 */

	//ITU-BT656
	//	cfg->type = V4L2_MBUS_BT656;
	//	cfg->flags = CLK_POL | CSI_CH_0;

	// DVP
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags =  V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL;

	//	cfg->type = V4L2_MBUS_CSI2;
	// 现在是2个通道， 如果需要更改多个通道时 请在这里添加通道数量
	//	cfg->flags = V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CHANNEL_1 | V4L2_MBUS_CSI2_2_LANE;

	return 0;
}

static struct regval_list sensor_oe_disable_regs[] = {
	{0x3017, 0x00},
	{0x3018, 0x00},
};

static struct regval_list sensor_oe_enable_regs[] = {
	{0x3017, 0xFF},
	{0x3018, 0xFF},
};

/* OV5640 register set for AF command */
static struct regval_list ov5640_af_cmd_reg[] = {
	{ 0x3022, 0x00 },
	{ 0x3023, 0x00 },
	{ 0x3024, 0x00 },
	{ 0x3025, 0x00 },
	{ 0x3026, 0x00 },
	{ 0x3027, 0x00 },
	{ 0x3028, 0x00 },
	{ 0x3029, 0xFF },
	{ 0x3000, 0x00 },
};


static struct v4l2_subdev *gclient = NULL;
static void ov5640_auto_focus(struct work_struct *work);
static DECLARE_DELAYED_WORK(focus_work, ov5640_auto_focus);

static int ov5640_af_setting(struct v4l2_subdev *client)
{
	int ret = 0;
	unsigned short val = 0x70;
	gclient = client;
	ret = sensor_write(client, 0x3000, 0x20);
	if (ret < 0)
		return ret;

	cci_write_a16_d8_continuous_helper(client, 0x8000, ov5640_af_firmware, ARRAY_SIZE(ov5640_af_firmware));

	ret = sensor_write_array(client, ov5640_af_cmd_reg, 
			sizeof(ov5640_af_cmd_reg) / sizeof(ov5640_af_cmd_reg[0]));
	if (ret) {
		printk("Failed to load AF command\n");
	} else {
		int index = 20;
		while (val == 0x70) {
			sensor_read(client, 0x3029, &val);
			if (val == 0x7F)
				break;
			else if (val == 0x00 || val == 0x10)
				break;
			if (index -- <= 0)
				break;
			mdelay(10);
		}
	}
	return 0;
}

static void ov5640_auto_focus(struct work_struct *work)
{
	unsigned short val = 1;
	int index = 20;
	sensor_write(gclient, 0x3022, 0x06);
	val = 1;
	while(val)
	{
		sensor_read(gclient, 0x3023, &val);
		mdelay(10);
		if (index -- <= 0)
			break;
	}

	sensor_write(gclient, 0x3022, 0x04);
	val = 1;
	index = 20;
	while(val) {
		sensor_read(gclient, 0x3023, &val);
		mdelay(10);
		if (index-- <= 0)
			break;
	}
}


static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	unsigned short value = 0;
	struct v4l2_subdev *ov5640_sd = sd;
	struct sensor_info *info = to_state(ov5640_sd);
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;
	data_type val;

	if (!enable) {
		cancel_delayed_work_sync(&focus_work);
		ov5640_write_array(ov5640_sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
		return 0;
	}
	ov5640_write_array(ov5640_sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
	
	if (wsize->regs) {
		ov5640_write_array(ov5640_sd, wsize->regs, wsize->regs_size);
	}

	if (wsize->set_size) {
		wsize->set_size(ov5640_sd);
	}

	if (sensor_fmt->regs)  {
		ov5640_write_array(ov5640_sd, sensor_fmt->regs, sensor_fmt->regs_size);
	}

	sensor_write_array(ov5640_sd, ov5640_default_regs, 
			sizeof(ov5640_default_regs) / sizeof(ov5640_default_regs[0]));

	sensor_write(ov5640_sd, 0x3a00, 0x04);
	sensor_write(ov5640_sd, 0x3a01, 0x80);
	sensor_read(ov5640_sd, 0x3a00, &value);
	sensor_write(ov5640_sd, 0x3a00, value | 0x20);


	//	sensor_write(ov5640_sd, 0x503D, 0x82); // 彩色四方块
	//	sensor_write(ov5640_sd, 0x4741, 0x00);

	// brightness
//	sensor_write(ov5640_sd, 0x5001, 0xff),
//	sensor_write(ov5640_sd, 0x5587, 0x00),
//	sensor_write(ov5640_sd, 0x5580, 0x04),
//	sensor_write(ov5640_sd, 0x5588, 0x01),

	ov5640_af_setting(ov5640_sd);
	INIT_DELAYED_WORK(&focus_work, ov5640_auto_focus);
	schedule_delayed_work(&focus_work, msecs_to_jiffies(2000));

	ov5640_write_array(ov5640_sd, sensor_oe_enable_regs, ARRAY_SIZE(sensor_oe_enable_regs));
	
	// 测试图案
//	sensor_write(ov5640_sd, 0x503d, 0x80);
//	sensor_write(ov5640_sd, 0x4741, 0x00);

	
//	sensor_write(ov5640_sd, 0x4202, 0x00);

	return 0;
}

static const struct regval_list ov5640_brightness_p3[] = {
	{0x5001, 0xff},
	{0x5587, 0x30},
	{0x5580, 0x04},
	{0x5588, 0x01},
};
static const struct regval_list ov5640_brightness_p2[] = {
	{0x5001, 0xff},
	{0x5587, 0x20},
	{0x5580, 0x04},
	{0x5588, 0x01},
};
static const struct regval_list ov5640_brightness_p1[] = {
	{0x5001, 0xff},
	{0x5587, 0x10},
	{0x5580, 0x04},
	{0x5588, 0x01},
};
static const struct regval_list ov5640_brightness_p0[] = {
	{0x5001, 0xff},
	{0x5587, 0x00},
	{0x5580, 0x04},
	{0x5588, 0x01},
};
static const struct regval_list ov5640_brightness_n1[] = {
	{0x5001, 0xff},
	{0x5587, 0x10},
	{0x5580, 0x04},
	{0x5588, 0x09},
};
static const struct regval_list ov5640_brightness_n2[] = {
	{0x5001, 0xff},
	{0x5587, 0x20},
	{0x5580, 0x04},
	{0x5588, 0x09},
};
static const struct regval_list ov5640_brightness_n3[] = {
	{0x5001, 0xff},
	{0x5587, 0x30},
	{0x5580, 0x04},
	{0x5588, 0x09},
};


static int sensor_set_brightness(struct v4l2_subdev *sd, int val)
{
	int ret;
	switch(val){
		case -3:
			ret = ov5640_write_array(sd, ov5640_brightness_p3, ARRAY_SIZE(ov5640_brightness_p3));
			break;
		case -2:
			ret = ov5640_write_array(sd, ov5640_brightness_p2, ARRAY_SIZE(ov5640_brightness_p2));
			break;
		case -1:
			ret = ov5640_write_array(sd, ov5640_brightness_p1, ARRAY_SIZE(ov5640_brightness_p1));
			break;
		case 0:
			ret = ov5640_write_array(sd, ov5640_brightness_p0, ARRAY_SIZE(ov5640_brightness_p0));
			break;
		case 1:
			ret = ov5640_write_array(sd, ov5640_brightness_n1, ARRAY_SIZE(ov5640_brightness_n1));
			break;
		case 2:
			ret = ov5640_write_array(sd, ov5640_brightness_n2, ARRAY_SIZE(ov5640_brightness_n2)) ;
			break;
		case 3:
			ret = ov5640_write_array(sd, ov5640_brightness_n3, ARRAY_SIZE(ov5640_brightness_n3));
			break;
		default:
			ret = -1;
			break;
	}
	return ret;
}

static int ov5640_set_exposure_auto(struct v4l2_subdev *sd, int val)
{
	struct sensor_info *info = to_state(sd);

	int exposure_auto = val;
	if (exposure_auto < 0 || exposure_auto > 1) {
		return -ERANGE;
	}
	info->exp = exposure_auto;
	return 0;
}

/*
 *AWB
 */
static const struct regval_list ov5640_awb_regs_enable[] =
{
};

static const struct regval_list ov5640_awb_regs_diable[] =
{
};

static int ov5640_set_auto_white_balance(struct v4l2_subdev *sd, int val)
{
	int auto_white_balance = val;
	int ret = -1;

	if (auto_white_balance < 0 || auto_white_balance > 1) {
		return -ERANGE;
	}

	switch(auto_white_balance) {
		case 0:
			ret = ov5640_write_array(sd, ov5640_awb_regs_diable, ARRAY_SIZE(ov5640_awb_regs_diable));
			break;
		case 1:
			ret = ov5640_write_array(sd, ov5640_awb_regs_enable, ARRAY_SIZE(ov5640_awb_regs_enable));
			break;
	}
	return ret;
}

static const struct regval_list ov5640_ev_p3[] = {
	{0x3a0f, 0x60},
	{0x3a10, 0x58},
	{0x3a11, 0xa0},
	{0x3a1b, 0x60},
	{0x3a1e, 0x58},
	{0x3a1f, 0x20},
};
static const struct regval_list ov5640_ev_p2[] = {
	{0x3a0f, 0x50},
	{0x3a10, 0x48},
	{0x3a11, 0x90},
	{0x3a1b, 0x50},
	{0x3a1e, 0x48},
	{0x3a1f, 0x20},
};
static const struct regval_list ov5640_ev_p1[] = {
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a11, 0x71},
	{0x3a1b, 0x40},
	{0x3a1e, 0x38},
	{0x3a1f, 0x10},
};
static const struct regval_list ov5640_ev_p0[] = {
	{0x3a0f, 0x38},
	{0x3a10, 0x30},
	{0x3a11, 0x61},
	{0x3a1b, 0x38},
	{0x3a1e, 0x30},
	{0x3a1f, 0x10},
};
static const struct regval_list ov5640_ev_n1[] = {
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a11, 0x61},
	{0x3a1b, 0x30},
	{0x3a1e, 0x28},
	{0x3a1f, 0x10},
};
static const struct regval_list ov5640_ev_n2[] = {
	{0x3a0f, 0x20},
	{0x3a10, 0x18},
	{0x3a11, 0x41},
	{0x3a1b, 0x20},
	{0x3a1e, 0x18},
	{0x3a1f, 0x10},
};
static const struct regval_list ov5640_ev_n3[] = {
	{0x3a0f, 0x10},
	{0x3a10, 0x08},
	{0x3a11, 0x10},
	{0x3a1b, 0x08},
	{0x3a1e, 0x20},
	{0x3a1f, 0x10},
};


static int ov5640_set_exposure(struct v4l2_subdev *sd, int exp_val)
{
	struct sensor_info *info = to_state(sd);
	int ret = 0;
	switch(exp_val){
		case -3:
			ret = ov5640_write_array(sd, ov5640_ev_n3, ARRAY_SIZE(ov5640_ev_n3));
			break;
		case -2:
			ret = ov5640_write_array(sd, ov5640_ev_n2, ARRAY_SIZE(ov5640_ev_n2));
			break;
		case -1:
			ret = ov5640_write_array(sd, ov5640_ev_n1, ARRAY_SIZE(ov5640_ev_n1));
			break;
		case 0:
			ret = ov5640_write_array(sd, ov5640_ev_p0, ARRAY_SIZE(ov5640_ev_p0));
			break;
		case 1:
			ret = ov5640_write_array(sd, ov5640_ev_p1, ARRAY_SIZE(ov5640_ev_p1));
			break;
		case 2:
			ret = ov5640_write_array(sd, ov5640_ev_p2, ARRAY_SIZE(ov5640_ev_p2));
			break;
		case 3:
			ret = ov5640_write_array(sd, ov5640_ev_p3, ARRAY_SIZE(ov5640_ev_p3));
			break;
		default:
			ret = -1;
			break;
	}
	return ret;
	

	info->exp = exp_val;
	return 0;
}


static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;
	data_type rdval;
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return sensor_set_brightness(sd, ctrl->val);
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			return ov5640_set_auto_white_balance(sd, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			return ov5640_set_exposure(sd, ctrl->val);
			break;
		default:
			break;
	}
	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;
	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			break;
		case V4L2_CID_EXPOSURE:
			break;
			break;
		case V4L2_CID_DO_WHITE_BALANCE:
			break;
		default:
			break;
	}

	return -EINVAL;
}


static const struct v4l2_ctrl_ops sensor_ctrl_ops = { 
	.s_ctrl = sensor_s_ctrl,
	.g_volatile_ctrl = sensor_g_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,	//override
	.init = sensor_init,	//override
	.s_power = sensor_power,	// override
	.ioctl = sensor_ioctl,		//override
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.s_stream = sensor_s_stream,	//override
	.g_mbus_config = sensor_g_mbus_config,	// override
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,		//芯片寄存器地址位数
	.data_width = CCI_BITS_8,		// csi->csi_fmt->data_width 芯片视频数据位宽
};


static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 3);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);	
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_BRIGHTNESS, 0, 6, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0, 0xFFFF, 1, 500);
	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}  
	sd->ctrl_handler = handler;
	return ret;
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	sd = &info->sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);
	sensor_init_controls(sd, &sensor_ctrl_ops);

	mutex_init(&info->lock);

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	/* info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE; */
//	info->combo_mode = CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE;
//	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->af_first_flag = 0;
	info->exp = 0;
	info->gain = 0;

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   },
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

#ifdef CONFIG_VIDEO_SUNXI_VIN_SPECIAL
subsys_initcall_sync(init_sensor);
#else
module_init(init_sensor);
#endif
module_exit(exit_sensor);
