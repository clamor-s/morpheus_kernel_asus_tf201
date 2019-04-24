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

#ifdef CONFIG_VIDEO_YUV
#include <media/yuv_sensor.h>
#endif /* CONFIG_VIDEO_YUV */

#include <mach/edp.h>
#include <mach/board-transformer-misc.h>
#include <mach/thermal.h>

#include "gpio-names.h"
#include "board-transformer.h"
#include "cpu-tegra.h"

#ifdef CONFIG_VIDEO_YUV
static struct regulator *reg_cardhu_cam;	/* LDO6 */
static struct regulator *reg_cardhu_1v8_cam;	/* VDDIO_CAM 1.8V PBB4 */
static struct regulator *reg_cardhu_2v85_cam;	/* Front Camera 2.85V power */
static struct regulator *reg_cardhu_1v2_cam;	/* VDDIO_CAM 1.2V PS0 */
static struct regulator *reg_cardhu_af_pwr_en;	/* ICATCH7002A_AF_PWR_EN_GPIO PS0 */
static struct regulator *reg_cardhu_vdda_en;	/* ICATCH7002A_VDDA_EN_GPIO GPIO_PR6*/
static struct regulator *reg_cardhu_vddio_cam;	/* LDO5 It's only for ME301T */
static bool camera_busy = false;
#endif /* CONFIG_VIDEO_YUV */

static int IsTF300(void)
{
    u32 project_info = tegra3_get_project_id();

    if (project_info == TEGRA3_PROJECT_TF300T)
        return 1;
    else if (project_info == TEGRA3_PROJECT_TF300TG)
        return 1;
    else if (project_info == TEGRA3_PROJECT_TF300TL)
        return 1;
    else
        return 0;
}

static int cardhu_camera_init(void)
{
    pr_info("cardhu_camera_init");
    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
        gpio_request(TF700T_ISP_POWER_1V2_EN_GPIO, "isp_power_1v2_en");
        gpio_request(TF700T_ISP_POWER_1V8_EN_GPIO, "isp_power_1v8_en");
        gpio_request(ISP_POWER_RESET_GPIO, "isp_power_rstx");
    }
    else if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201){
        gpio_request(ISP_POWER_1V2_EN_GPIO, "isp_power_1v2_en");
        gpio_request(ISP_POWER_RESET_GPIO, "isp_power_rstx");
        gpio_request(CAM3_POWER_DWN_GPIO, "cam3_power_dwn");
        gpio_request(FRONT_YUV_SENSOR_RST_GPIO, "yuv_sensor_rst_lo");
    }
	else if(IsTF300() ||(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) ||
		(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)) {
		if(IsTF300() || (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T))
			gpio_request(ICATCH7002A_VDDIO_EN_GPIO, "cam_vddio_ldo_en");
		if(tegra3_get_project_id() == TEGRA3_PROJECT_TF500T) {
			gpio_request(ICATCH7002A_ISP_1V2_EN, "icatch_cam_vddio_ldo_en");
		}
		gpio_request(ICATCH7002A_VDDC_EN_GPIO, "cam_vddc_ldo_en");
		gpio_request(ICATCH7002A_PWR_DN_GPIO, "cam_power_dwn");
		gpio_request(ICATCH7002A_RST_GPIO, "cam_sensor_rst_lo");
	}
	return 0;
}

#ifdef CONFIG_VIDEO_YUV
static int yuv_sensor_power_on_TF700T(void)
{
    if (!reg_cardhu_1v2_cam) {
        reg_cardhu_1v2_cam = regulator_get(NULL, "vdd_cam3");
        if (IS_ERR_OR_NULL(reg_cardhu_1v2_cam)) {
            pr_err("TF700T_m6mo_power_on PS0: vdd_cam3 failed\n");
            reg_cardhu_1v2_cam = NULL;
            return PTR_ERR(reg_cardhu_1v2_cam);
        }
        regulator_set_voltage(reg_cardhu_1v2_cam, 1200000, 1200000);
        regulator_enable(reg_cardhu_1v2_cam);
    }

    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V2_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V2_EN_GPIO));
    gpio_direction_output(TF700T_ISP_POWER_1V2_EN_GPIO, 1);
    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V2_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V2_EN_GPIO));

    if (!reg_cardhu_1v8_cam) {
        reg_cardhu_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1");
        if (IS_ERR_OR_NULL(reg_cardhu_1v8_cam)) {
            pr_err("TF700T_m6mo_power_on PBB4: vdd_1v8_cam1 failed\n");
            reg_cardhu_1v8_cam = NULL;
            return PTR_ERR(reg_cardhu_1v8_cam);
        }
        regulator_set_voltage(reg_cardhu_1v8_cam, 1800000, 1800000);
        regulator_enable(reg_cardhu_1v8_cam);
    }

    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V8_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V8_EN_GPIO));
    gpio_direction_output(TF700T_ISP_POWER_1V8_EN_GPIO, 1);
    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V8_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V8_EN_GPIO));

    msleep(1);
    tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_NORMAL);

    return 0;
}

static int yuv_sensor_power_off_TF700T(void)
{
    tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_TRISTATE);

    gpio_direction_output(TF700T_ISP_POWER_1V8_EN_GPIO, 0);
    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V8_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V8_EN_GPIO));

    gpio_direction_output(TF700T_ISP_POWER_1V2_EN_GPIO, 0);
    pr_info("gpio %d set to %d\n",TF700T_ISP_POWER_1V2_EN_GPIO, gpio_get_value(TF700T_ISP_POWER_1V2_EN_GPIO));

    return 0;
}

static int yuv_sensor_power_on(void)
{
    printk("yuv_sensor_power_on+\n");

    if(camera_busy){
        printk("yuv_sensor busy\n");
        return -EBUSY;
    }
    camera_busy = true;
    if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
    {
        yuv_sensor_power_on_TF700T();
    }
    else{
        //For i2c bus
        gpio_request(143, "gpio_pr7");
        gpio_direction_output(143, 1);
        pr_info("gpio 2.85V %d set to %d\n",143, gpio_get_value(143));

        pr_info("gpio %d set to %d\n",ISP_POWER_1V2_EN_GPIO, gpio_get_value(ISP_POWER_1V2_EN_GPIO));
        gpio_direction_output(ISP_POWER_1V2_EN_GPIO, 1);
        pr_info("gpio %d set to %d\n",ISP_POWER_1V2_EN_GPIO, gpio_get_value(ISP_POWER_1V2_EN_GPIO));

        msleep(5);

        if (!reg_cardhu_1v8_cam) {
            reg_cardhu_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1");
            if (IS_ERR_OR_NULL(reg_cardhu_1v8_cam)) {
                pr_err("TF201_m6mo_power_on PBB4: vdd_1v8_cam1 failed\n");
                reg_cardhu_1v8_cam = NULL;
                return PTR_ERR(reg_cardhu_1v8_cam);
            }
            regulator_set_voltage(reg_cardhu_1v8_cam, 1800000, 1800000);
            regulator_enable(reg_cardhu_1v8_cam);
        }
    }
    if (!reg_cardhu_cam) {
        reg_cardhu_cam = regulator_get(NULL, "avdd_dsi_csi");
        if (IS_ERR_OR_NULL(reg_cardhu_cam)) {
            pr_err("TF201_m6mo_power_on LDO6: p_tegra_cam failed\n");
            reg_cardhu_cam = NULL;
            return PTR_ERR(reg_cardhu_cam);
        }
        regulator_set_voltage(reg_cardhu_cam, 1200000, 1200000);
        regulator_enable(reg_cardhu_cam);
    }

    return 0;
}
int yuv_sensor_power_on_reset_pin(void)
{
    pr_info("gpio %d set to %d\n",ISP_POWER_RESET_GPIO, gpio_get_value(ISP_POWER_RESET_GPIO));
    gpio_direction_output(ISP_POWER_RESET_GPIO, 1);
    pr_info("gpio %d set to %d\n",ISP_POWER_RESET_GPIO, gpio_get_value(ISP_POWER_RESET_GPIO));

    printk("yuv_sensor_power_on -\n");
    return 0;
}

static int yuv_sensor_power_off(void)
{
    if(reg_cardhu_cam){
        regulator_disable(reg_cardhu_cam);
        regulator_put(reg_cardhu_cam);
        reg_cardhu_cam = NULL;
    }

    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
        yuv_sensor_power_off_TF700T();
    }
    else{
        if(reg_cardhu_1v8_cam){
            regulator_disable(reg_cardhu_1v8_cam);
            regulator_put(reg_cardhu_1v8_cam);
            reg_cardhu_1v8_cam = NULL;
        }
        gpio_direction_output(ISP_POWER_1V2_EN_GPIO, 0);
        pr_info("gpio %d set to %d\n",ISP_POWER_1V2_EN_GPIO, gpio_get_value(ISP_POWER_1V2_EN_GPIO));
    }

    printk("yuv_sensor_power_off-\n");
    return 0;
}

int yuv_sensor_power_off_reset_pin(void)
{
    printk("yuv_sensor_power_off+\n");
    camera_busy = false;
    gpio_direction_output(ISP_POWER_RESET_GPIO, 0);
    pr_info("gpio %d set to %d\n",ISP_POWER_RESET_GPIO, gpio_get_value(ISP_POWER_RESET_GPIO));
    return 0;
}

struct yuv_sensor_platform_data yuv_rear_sensor_data = {
    .power_on = yuv_sensor_power_on,
    .power_off = yuv_sensor_power_off,
};

static int yuv_front_sensor_power_on(void)
{
	printk("yuv_front_sensor_power_on+\n");

	if(camera_busy){
		printk("yuv_sensor busy\n");
		return -EBUSY;
	}
	camera_busy = true;
	/* 1.8V VDDIO_CAM controlled by "EN_1V8_CAM(GPIO_PBB4)" */
	if (!reg_cardhu_1v8_cam) {
		reg_cardhu_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1"); /*cam2/3?*/
		if (IS_ERR_OR_NULL(reg_cardhu_1v8_cam)) {
			reg_cardhu_1v8_cam = NULL;
			pr_err("Can't get reg_cardhu_1v8_cam.\n");
			goto fail_to_get_reg;
		}
		regulator_set_voltage(reg_cardhu_1v8_cam, 1800000, 1800000);
		regulator_enable(reg_cardhu_1v8_cam);
	}

  	/* 2.85V VDD_CAM2 controlled by CAM2/3_LDO_EN(GPIO_PS0)*/
  	if (!reg_cardhu_2v85_cam) {
  		reg_cardhu_2v85_cam = regulator_get(NULL, "vdd_cam3");
  		if (IS_ERR_OR_NULL(reg_cardhu_2v85_cam)) {
  			reg_cardhu_2v85_cam = NULL;
  			pr_err("Can't get reg_cardhu_2v85_cam.\n");
  			goto fail_to_get_reg;
  		}
  		regulator_set_voltage(reg_cardhu_2v85_cam, 2850000, 2850000);
  		regulator_enable(reg_cardhu_2v85_cam);
  	}

	/* cam_power_en, powdn*/
	pr_info("gpio %d: %d",CAM3_POWER_DWN_GPIO, gpio_get_value(CAM3_POWER_DWN_GPIO));
	gpio_set_value(CAM3_POWER_DWN_GPIO, 0);
	gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);
	pr_info("--> %d\n", gpio_get_value(CAM3_POWER_DWN_GPIO));

	/* yuv_sensor_rst_lo*/
	pr_info("gpio %d: %d", FRONT_YUV_SENSOR_RST_GPIO, gpio_get_value(FRONT_YUV_SENSOR_RST_GPIO));
	gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 1);
	gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 1);
	pr_info("--> %d\n", gpio_get_value(FRONT_YUV_SENSOR_RST_GPIO));

	printk("yuv_front_sensor_power_on-\n");
	return 0;

fail_to_get_reg:
	if (reg_cardhu_2v85_cam) {
		regulator_put(reg_cardhu_2v85_cam);
		reg_cardhu_2v85_cam = NULL;
	}
	if (reg_cardhu_1v8_cam) {
		regulator_put(reg_cardhu_1v8_cam);
		reg_cardhu_1v8_cam = NULL;
	}

	camera_busy = false;
	printk("yuv_front_sensor_power_on- : -ENODEV\n");
	return -ENODEV;
}

static int yuv_front_sensor_power_off(void)
{
	printk("yuv_front_sensor_power_off+\n");

	gpio_set_value(FRONT_YUV_SENSOR_RST_GPIO, 0);
	gpio_direction_output(FRONT_YUV_SENSOR_RST_GPIO, 0);

	gpio_set_value(CAM3_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);

	if (reg_cardhu_2v85_cam) {
		regulator_disable(reg_cardhu_2v85_cam);
		regulator_put(reg_cardhu_2v85_cam);
		reg_cardhu_2v85_cam = NULL;
	}
	if (reg_cardhu_1v8_cam) {
		regulator_disable(reg_cardhu_1v8_cam);
		regulator_put(reg_cardhu_1v8_cam);
		reg_cardhu_1v8_cam = NULL;
	}

	camera_busy = false;
	printk("yuv_front_sensor_power_off-\n");
	return 0;
}
struct yuv_sensor_platform_data yuv_front_sensor_data = {
	.power_on = yuv_front_sensor_power_on,
	.power_off = yuv_front_sensor_power_off,
};
/*==============++iCatch++================================*/
static int iCatch7002a_power_on(void)
{
    printk("%s+\n", __FUNCTION__);

	if(IsTF300() ||(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) ||
		(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)) {
		Asus_camera_enable_set_emc_rate(667000000);
	}

    if((tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) || (tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
        if (!reg_cardhu_vddio_cam) {
            reg_cardhu_vddio_cam = regulator_get(NULL, "avdd_vdac");
            if (IS_ERR_OR_NULL(reg_cardhu_vddio_cam)) {
                pr_err("ME301T_vddio_power_on LDO5: avdd_vdac failed\n");
                reg_cardhu_vddio_cam = NULL;
                goto fail_to_get_reg;
            }
            regulator_set_voltage(reg_cardhu_vddio_cam, 1800000, 1800000);
            regulator_enable(reg_cardhu_vddio_cam);
        }
    }
    else{
        pr_info("gpio %d read as %d\n",ICATCH7002A_VDDIO_EN_GPIO, gpio_get_value(ICATCH7002A_VDDIO_EN_GPIO));
        gpio_direction_output(ICATCH7002A_VDDIO_EN_GPIO, 1);
        pr_info("gpio %d set to %d\n",ICATCH7002A_VDDIO_EN_GPIO, gpio_get_value(ICATCH7002A_VDDIO_EN_GPIO));
    }

    msleep(1);

    if (!reg_cardhu_vdda_en) {
        reg_cardhu_vdda_en = regulator_get(NULL, "vdd_2v8_cam1");
        if (IS_ERR_OR_NULL(reg_cardhu_vdda_en)) {
            pr_err("vdda_power_on gpio_pr6: vdd_2v8_cam1 failed\n");
            reg_cardhu_vdda_en = NULL;
            goto fail_to_get_reg;
        }
        regulator_enable(reg_cardhu_vdda_en);
        mdelay(5);
    }



    if (!reg_cardhu_af_pwr_en) {
        reg_cardhu_af_pwr_en = regulator_get(NULL, "vdd_cam3");
        if (IS_ERR_OR_NULL(reg_cardhu_af_pwr_en)) {
            pr_err("af_power_on gpio_ps0: vdd_cam3 failed\n");
            reg_cardhu_af_pwr_en = NULL;
            goto fail_to_get_reg;
        }
        regulator_enable(reg_cardhu_af_pwr_en);
        mdelay(5);
    }
   	if(IsTF300() ||(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) ||
		(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)) {
        pr_info("gpio %d read as %d\n",ICATCH7002A_VDDC_EN_GPIO, gpio_get_value(ICATCH7002A_VDDC_EN_GPIO));
        gpio_direction_output(ICATCH7002A_VDDC_EN_GPIO, 1);
        pr_info("gpio %d set to %d\n",ICATCH7002A_VDDC_EN_GPIO, gpio_get_value(ICATCH7002A_VDDC_EN_GPIO));
    }
    msleep(1);

    tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_NORMAL);

	/* cam_power_en, powdn*/
	pr_info("gpio %d: %d",ICATCH7002A_PWR_DN_GPIO, gpio_get_value(ICATCH7002A_PWR_DN_GPIO));
	gpio_set_value(ICATCH7002A_PWR_DN_GPIO, 1);
	gpio_direction_output(ICATCH7002A_PWR_DN_GPIO, 1);
	pr_info("--> %d\n", gpio_get_value(ICATCH7002A_PWR_DN_GPIO));

    /* yuv_sensor_rst_lo*/
    if (IsTF300()){
        pr_info("gpio %d: %d", ICATCH7002A_RST_GPIO, gpio_get_value(ICATCH7002A_RST_GPIO));
        gpio_set_value(ICATCH7002A_RST_GPIO, 1);//high
        gpio_direction_output(ICATCH7002A_RST_GPIO, 1);
        pr_info("gpio %d--> %d\n", ICATCH7002A_RST_GPIO, gpio_get_value(ICATCH7002A_RST_GPIO));
        msleep(5);
    }
    gpio_set_value(ICATCH7002A_RST_GPIO, 0);//low
	gpio_direction_output(ICATCH7002A_RST_GPIO, 0);
	pr_info("gpio %d--> %d\n", ICATCH7002A_RST_GPIO, gpio_get_value(ICATCH7002A_RST_GPIO));
	msleep(25);
	gpio_set_value(ICATCH7002A_RST_GPIO, 1);//high
	gpio_direction_output(ICATCH7002A_RST_GPIO, 1);
	pr_info("gpio %d--> %d\n", ICATCH7002A_RST_GPIO, gpio_get_value(ICATCH7002A_RST_GPIO));
	msleep(6);
	gpio_set_value(ICATCH7002A_PWR_DN_GPIO, 0);//low
	gpio_direction_output(ICATCH7002A_PWR_DN_GPIO, 0);
	pr_info("gpio %d--> %d\n", ICATCH7002A_PWR_DN_GPIO, gpio_get_value(ICATCH7002A_PWR_DN_GPIO));

    return 0;

fail_to_get_reg:
	if (reg_cardhu_vddio_cam) {
		regulator_disable(reg_cardhu_vddio_cam);
		regulator_put(reg_cardhu_vddio_cam);
		reg_cardhu_vddio_cam = NULL;
	}
	if (reg_cardhu_vdda_en) {
		regulator_disable(reg_cardhu_vdda_en);
		regulator_put(reg_cardhu_vdda_en);
		reg_cardhu_vdda_en = NULL;
	}
	if (reg_cardhu_af_pwr_en) {
		regulator_disable(reg_cardhu_af_pwr_en);
		regulator_put(reg_cardhu_af_pwr_en);
		reg_cardhu_af_pwr_en = NULL;
	}
	if (reg_cardhu_1v8_cam) {
		regulator_disable(reg_cardhu_1v8_cam);
		regulator_put(reg_cardhu_1v8_cam);
		reg_cardhu_1v8_cam = NULL;
	}

    printk("%s- : -ENODEV\n", __FUNCTION__);
    return -ENODEV;
}
static int iCatch7002a_power_off(void)
{
	printk("%s+\n", __FUNCTION__);
	gpio_set_value(ICATCH7002A_RST_GPIO, 0);
	gpio_direction_output(ICATCH7002A_RST_GPIO, 0);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CAM_MCLK, TEGRA_TRI_TRISTATE);

	if(IsTF300() ||(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) ||
		(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
        gpio_set_value(ICATCH7002A_VDDC_EN_GPIO, 0);
        gpio_direction_output(ICATCH7002A_VDDC_EN_GPIO, 0);
    }

    if (reg_cardhu_af_pwr_en) {
        regulator_disable(reg_cardhu_af_pwr_en);
        regulator_put(reg_cardhu_af_pwr_en);
        reg_cardhu_af_pwr_en = NULL;
    }

    if (reg_cardhu_vdda_en) {
        regulator_disable(reg_cardhu_vdda_en);
        regulator_put(reg_cardhu_vdda_en);
        reg_cardhu_vdda_en = NULL;
    }
    if((tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) || (tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
        if (reg_cardhu_vddio_cam) {
            regulator_disable(reg_cardhu_vddio_cam);
            regulator_put(reg_cardhu_vddio_cam);
            reg_cardhu_vddio_cam = NULL;
        }
    }
    else{
        gpio_set_value(ICATCH7002A_VDDIO_EN_GPIO, 0);
        gpio_direction_output(ICATCH7002A_VDDIO_EN_GPIO, 0);
    }

	if(IsTF300() ||(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T) ||
		(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
		Asus_camera_disable_set_emc_rate();
	}

	printk("%s-\n", __FUNCTION__);
  return 0;
}
struct yuv_sensor_platform_data iCatch7002a_data = {
	.power_on = iCatch7002a_power_on,
	.power_off = iCatch7002a_power_off,
};
/*==============--iCatch--================================*/
#endif  /* CONFIG_VIDEO_YUV */

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

#ifdef CONFIG_VIDEO_YUV
static struct i2c_board_info rear_sensor_i2c3_board_info[] = {  //ddebug
    {
        I2C_BOARD_INFO("fjm6mo", 0x1F),
        .platform_data = &yuv_rear_sensor_data,
    },
};

static struct i2c_board_info front_sensor_i2c2_board_info[] = {  //ddebug
	{
		I2C_BOARD_INFO("mi1040", 0x48),
		.platform_data = &yuv_front_sensor_data,
	},
};

static struct i2c_board_info iCatch7002a_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("i7002a", 0x3C),
		.platform_data = &iCatch7002a_data,
	},
};
#endif /* CONFIG_VIDEO_YUV */

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

	cardhu_camera_init();

#ifdef CONFIG_VIDEO_YUV
//+ m6mo rear camera
    pr_info("fjm6mo i2c_register_board_info");
    i2c_register_board_info(2, rear_sensor_i2c3_board_info,
        ARRAY_SIZE(rear_sensor_i2c3_board_info));

/* Front Camera mi1040 + */
    pr_info("mi1040 i2c_register_board_info");
	i2c_register_board_info(2, front_sensor_i2c2_board_info,
		ARRAY_SIZE(front_sensor_i2c2_board_info));
/* Front Camera mi1040 - */

/* iCatch7002a + */
	pr_info("iCatch7002a i2c_register_board_info");
	i2c_register_board_info(2, iCatch7002a_i2c2_board_info,
		ARRAY_SIZE(iCatch7002a_i2c2_board_info));
/* iCatch7002a - */
#endif /* CONFIG_VIDEO_YUV */

	i2c_register_board_info(2, cardhu_i2c1_board_info_al3010,
		ARRAY_SIZE(cardhu_i2c1_board_info_al3010));

	i2c_register_board_info(4, cardhu_i2c4_pad_bat_board_info,
		ARRAY_SIZE(cardhu_i2c4_pad_bat_board_info));

	mpuirq_init();

	return 0;
}
