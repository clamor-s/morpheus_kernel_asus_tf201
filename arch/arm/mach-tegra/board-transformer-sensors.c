/*
 * arch/arm/mach-tegra/board-transformer-sensors.c
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/nct1008.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/therm_est.h>
#include <linux/slab.h>

#include <mach/edp.h>
#include <mach/board-transformer-misc.h>
#include <mach/thermal.h>

#include "gpio-names.h"
#include "board-transformer.h"
#include "cpu-tegra.h"

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int nct_get_itemp(void *dev_data, long *temp)
{
	struct nct1008_data *data = dev_data;
	return nct1008_thermal_get_temps(data, NULL, temp);
}
#endif

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *ext_nct;

	ext_nct = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!ext_nct) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	ext_nct->name = "nct_ext";
	ext_nct->id = THERMAL_DEVICE_ID_NCT_EXT;
	ext_nct->data = data;
	ext_nct->offset = TDIODE_OFFSET;
	ext_nct->get_temp = nct_get_temp;
	ext_nct->get_temp_low = nct_get_temp_low;
	ext_nct->set_limits = nct_set_limits;
	ext_nct->set_alert = nct_set_alert;
	ext_nct->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(ext_nct);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		struct tegra_thermal_device *int_nct;
		int_nct = kzalloc(sizeof(struct tegra_thermal_device),
						GFP_KERNEL);
		if (!int_nct) {
			kfree(int_nct);
			pr_err("unable to allocate thermal device\n");
			return;
		}

		int_nct->name = "nct_int";
		int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
		int_nct->data = data;
		int_nct->get_temp = nct_get_itemp;

		tegra_thermal_device_register(int_nct);
	}
#endif
}

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info cardhu_i2c4_pad_bat_board_info[] = {
	{
		I2C_BOARD_INFO("pad-battery", 0xb),
	},
};

static struct i2c_board_info cardhu_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

static int cardhu_nct1008_init(void)
{
	int ret = 0;

	if (gpio_get_value(TEGRA_GPIO_PCC2) >= 0) {
		/* FIXME: enable irq when throttling is supported */
		cardhu_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PCC2);

		ret = gpio_request(TEGRA_GPIO_PCC2, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(TEGRA_GPIO_PCC2);
		if (ret < 0)
			gpio_free(TEGRA_GPIO_PCC2);
	}

	return ret;
}

static struct mpu_platform_data mpu3050_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu3050_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU3050_GYRO_NAME, MPU3050_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if	MPU_ACCEL_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
#endif
		.platform_data = &mpu3050_accel_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if	MPU_COMPASS_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
#endif
		.platform_data = &mpu3050_compass_data,
	},
};

/*Sensors orientation definition*/
struct mpu_orientation_def{
	__s8 gyro_orientation[9];
	__s8 accel_orientation[9];
	__s8 compass_orientation[9];
};

static void mpuirq_init(void)
{
	u32 project_info;
	int ret = 0;
	pr_info("*** MPU START *** cardhu_mpuirq_init...\n");

	project_info = tegra3_get_project_id();

	if (project_info == TEGRA3_PROJECT_TF300T)
	{
		/* Use "TF300T" to check the project name */
		struct mpu_orientation_def TF300T = {
			TF300T_GYRO_ORIENTATION,
			TF300T_ACCEL_ORIENTATION,
			TF300T_COMPASS_ORIENTATION,
			};

		pr_info("initial mpu with TF300T config...\n");
		memcpy( mpu3050_data.orientation, TF300T.gyro_orientation, sizeof(mpu3050_data.orientation));
		memcpy( mpu3050_accel_data.orientation, TF300T.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy( mpu3050_compass_data.orientation, TF300T.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (project_info == TEGRA3_PROJECT_TF300TG)
	{
		/* Use "TF300TG" to check the project name */
		struct mpu_orientation_def TF300TG = {
			TF300TG_GYRO_ORIENTATION,
			TF300TG_ACCEL_ORIENTATION,
			TF300TG_COMPASS_ORIENTATION,
			};

		pr_info("initial mpu with TF300TG config...\n");
		memcpy( mpu3050_data.orientation, TF300TG.gyro_orientation, sizeof(mpu3050_data.orientation));
		memcpy( mpu3050_accel_data.orientation, TF300TG.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy( mpu3050_compass_data.orientation, TF300TG.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (project_info == TEGRA3_PROJECT_TF700T)
	{
		/* Use "TF700T" to check the project name */
		struct mpu_orientation_def TF700T = {
			TF700T_GYRO_ORIENTATION,
			TF700T_ACCEL_ORIENTATION,
			TF700T_COMPASS_ORIENTATION,
			};

		pr_info("initial mpu with TF700T config...\n");
		memcpy( mpu3050_data.orientation, TF700T.gyro_orientation, sizeof(mpu3050_data.orientation));
		memcpy( mpu3050_accel_data.orientation, TF700T.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy( mpu3050_compass_data.orientation, TF700T.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}
	else if (project_info == TEGRA3_PROJECT_TF300TL)
	{
		/* Use "TF300TL" to check the project name */
		struct mpu_orientation_def TF300TL = {
			TF300TL_GYRO_ORIENTATION,
			TF300TL_ACCEL_ORIENTATION,
			TF300TL_COMPASS_ORIENTATION,
			};

		pr_info("initial mpu with TF300TL config...\n");
		memcpy( mpu3050_data.orientation, TF300TL.gyro_orientation, sizeof(mpu3050_data.orientation));
		memcpy( mpu3050_accel_data.orientation, TF300TL.accel_orientation, sizeof(mpu3050_accel_data.orientation));
		memcpy( mpu3050_compass_data.orientation, TF300TL.compass_orientation, sizeof(mpu3050_compass_data.orientation));
	}

#if	MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	// tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif

	/* MPU-IRQ assignment */
    /*	tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}*/
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
}

static const struct i2c_board_info cardhu_i2c1_board_info_al3010[] = {
    {
        I2C_BOARD_INFO("al3010",0x1C),
        .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
    },
};

int __init cardhu_sensors_init(void)
{
	int err;

	err = cardhu_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(4, cardhu_i2c4_nct1008_board_info,
			ARRAY_SIZE(cardhu_i2c4_nct1008_board_info));

	i2c_register_board_info(2, cardhu_i2c1_board_info_al3010,
		ARRAY_SIZE(cardhu_i2c1_board_info_al3010));

	i2c_register_board_info(4, cardhu_i2c4_pad_bat_board_info,
		ARRAY_SIZE(cardhu_i2c4_pad_bat_board_info));

	mpuirq_init();

	return 0;
}
