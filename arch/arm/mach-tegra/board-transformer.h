/*
 * arch/arm/mach-tegra/board-cardhu.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_CARDHU_H
#define _MACH_TEGRA_BOARD_CARDHU_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE			TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8				(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END			(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

/* PMU_TCA6416 GPIO assignment */
#define EN_HSIC_GPIO				TEGRA_GPIO_PR7  /* PMU_GPIO25 */

/* WM8903 GPIOs */
#define CARDHU_GPIO_WM8903(_x_)			(TPS6591X_GPIO_END + 32 + (_x_))
#define CARDHU_GPIO_WM8903_END			CARDHU_GPIO_WM8903(4)

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ			TEGRA_GPIO_PW3
#define TEGRA_GPIO_SPKR_EN			CARDHU_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET			TEGRA_GPIO_PW2

/* CAMERA RELATED GPIOs on CARDHU */
#define OV5650_RESETN_GPIO			TEGRA_GPIO_PBB0
#define CAM1_POWER_DWN_GPIO			TEGRA_GPIO_PBB5
#define CAM2_POWER_DWN_GPIO			TEGRA_GPIO_PBB6
#define CAM3_POWER_DWN_GPIO			TEGRA_GPIO_PBB7
#define CAMERA_CSI_CAM_SEL_GPIO		TEGRA_GPIO_PBB4
#define CAMERA_CSI_MUX_SEL_GPIO		TEGRA_GPIO_PCC1
#define CAM1_LDO_EN_GPIO			TEGRA_GPIO_PR6
#define CAM2_LDO_EN_GPIO			TEGRA_GPIO_PR7
#define CAM3_LDO_EN_GPIO			TEGRA_GPIO_PS0
#define OV14810_RESETN_GPIO			TEGRA_GPIO_PBB0

#define CAMERA_FLASH_SYNC_GPIO		TEGRA_GPIO_PBB3
#define CAMERA_FLASH_MAX_TORCH_AMP	7
#define CAMERA_FLASH_MAX_FLASH_AMP	7

/* CAMERA RELATED GPIOs on TF201*/
#define ISP_POWER_1V2_EN_GPIO       TEGRA_GPIO_PS3      //ISP_1V2_EN VDD_ISP_1V2
#define ISP_POWER_RESET_GPIO        TEGRA_GPIO_PBB0     //CAM_RST_5M, RSTX
#define FRONT_YUV_SENSOR_RST_GPIO	TEGRA_GPIO_PO0      //1.2M CAM_RST

//TF700T
#define TF700T_ISP_POWER_1V2_EN_GPIO       TEGRA_GPIO_PR7      //ISP_1V2_EN VDD_ISP_1V2
#define TF700T_ISP_POWER_1V8_EN_GPIO       TEGRA_GPIO_PBB7     //ISP_1V8_EN VDD_ISP_1V8

//TF300T, TF500T
#define ICATCH7002A_RST_GPIO TEGRA_GPIO_PBB0
#define ICATCH7002A_VDDIO_EN_GPIO TEGRA_GPIO_PBB4
#define ICATCH7002A_PWR_DN_GPIO TEGRA_GPIO_PBB5
#define ICATCH7002A_VDDC_EN_GPIO TEGRA_GPIO_PBB7
#define ICATCH7002A_ISP_1V2_EN TEGRA_GPIO_PS3   //For TF500T; PBB7 in other porjects
#define ICATCH7002A_CAM_2V85_EN TEGRA_GPIO_PR7

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE			TEGRA_NR_IRQS
#define TPS6591X_IRQ_END			(TPS6591X_IRQ_BASE + 18)
#define DOCK_DETECT_GPIO			TEGRA_GPIO_PU4

int cardhu_regulator_init(void);
int cardhu_suspend_init(void);
int cardhu_sdhci_init(void);
int cardhu_pinmux_init(void);
int cardhu_panel_init(void);
int cardhu_sensors_init(void);
int cardhu_keys_init(void);
int cardhu_pins_state_init(void);
int cardhu_emc_init(void);
int cardhu_edp_init(void);
struct platform_device *tegra_cardhu_usb_utmip_host_register(void);
void tegra_cardhu_usb_utmip_host_unregister(struct platform_device *pdev);
struct platform_device *tegra_usb3_utmip_host_register(void);
void tegra_usb3_utmip_host_unregister(struct platform_device *pdev);

#define MPU_TYPE_MPU3050	1
#define MPU_TYPE_MPU6050	2
#define MPU_GYRO_TYPE		MPU_TYPE_MPU3050
/* Invensense MPU Definitions */
#define MPU3050_GYRO_NAME		"mpu3050"
#define MPU6050_GYRO_NAME		"mpu6050"
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PX1
#define MPU3050_GYRO_ADDR		0x68
#define MPU6050_GYRO_ADDR		0x69
#define MPU6050_TF500T_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	2
#define MPU_GYRO_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	TEGRA_GPIO_PO5
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	2
#define MPU_ACCEL_ORIENTATION	{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define MPU_COMPASS_NAME	"ami306"
#define MPU_COMPASS_IRQ_GPIO	TEGRA_GPIO_PW0
#define MPU_COMPASS_ADDR	0x0E
#define MPU_COMPASS_BUS_NUM	2
#define MPU_COMPASS_ORIENTATION	{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }

//Sensors orientation matrix for TF300T
#define TF300T_GYRO_ORIENTATION		{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define TF300T_ACCEL_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define TF300T_COMPASS_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }

//Sensors orientation matrix for TF300TG
#define TF300TG_GYRO_ORIENTATION		{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define TF300TG_ACCEL_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define TF300TG_COMPASS_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }

//Sensors orientation matrix for TF700T
#define TF700T_GYRO_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define TF700T_ACCEL_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define TF700T_COMPASS_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }

//Sensors orientation matrix for TF300TL
#define TF300TL_GYRO_ORIENTATION		{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define TF300TL_ACCEL_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define TF300TL_COMPASS_ORIENTATION	{ -1, 0, 0, 0, -1, 0, 0, 0, 1 }

//Sensors orientation matrix for TF500T
#define TF500T_GYRO_ORIENTATION		{ 0, -1, 0, 1, 0, 0, 0, 0, 1 }
#define TF500T_COMPASS_ORIENTATION	{ 0, -1, 0, 1, 0, 0, 0, 0, 1 }

//Sensors orientation matrix for ME301T and ME301TL
#define ME301T_GYRO_ORIENTATION		{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define ME301T_COMPASS_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }

/* Kionix Accel sensor Definitions*/
#define KIONIX_ACCEL_NAME	"KXT_9"
#define KIONIX_ACCEL_IRQ_GPIO	TEGRA_GPIO_PO5
#define KIONIX_ACCEL_ADDR		0x0F
#define KIONIX_ACCEL_BUS_NUM	2

//Sensors orientation matrix for P1801
#define P1801_ACCEL_ORIENTATION		{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }

/* Baseband GPIO addresses */
#define BB_GPIO_BB_EN			TEGRA_GPIO_PX7 //MODEM_ON
#define BB_GPIO_BB_RST			TEGRA_GPIO_PU3 //MOD_nRST_PWRDWN
#define BB_GPIO_SPI_INT			TEGRA_GPIO_PX0 //AP_Active
#define BB_GPIO_SPI_SS			TEGRA_GPIO_PY3 //MOD_suspend_req
#define BB_GPIO_AWR				TEGRA_GPIO_PY2 //AP_WAKE_MOD
#define BB_GPIO_CWR				TEGRA_GPIO_PU5 //MOD_WAKE_AP

#define XMM_GPIO_BB_ON			BB_GPIO_BB_EN
#define XMM_GPIO_BB_RST			BB_GPIO_BB_RST
#define XMM_GPIO_IPC_HSIC_ACTIVE	BB_GPIO_SPI_INT
#define XMM_GPIO_IPC_HSIC_SUS_REQ	BB_GPIO_SPI_SS
#define XMM_GPIO_IPC_BB_WAKE		BB_GPIO_AWR
#define XMM_GPIO_IPC_AP_WAKE		BB_GPIO_CWR
#define XMM_GPIO_IPC_BB_FORCE_CRASH            TEGRA_GPIO_PN1

/* Asus baseband GPIO addresses */
#define BB_GPIO_VBAT_ON			TEGRA_GPIO_PC6  //MOD_VBAT_ON
#define BB_GPIO_VBUS_ON			TEGRA_GPIO_PD2  //MOD_VBUS_ON
#define BB_GPIO_SW_SEL			TEGRA_GPIO_PP1  //USB_SW_SEL
#define BB_GPIO_RESET_IND		TEGRA_GPIO_PEE1 //n_MOD_RST_IND
#define BB_GPIO_SAR_DET			TEGRA_GPIO_PR3  //SAR_DET#_3G
#define BB_GPIO_SIM_DET			TEGRA_GPIO_PW3  //n_SIM_CD

#define TDIODE_OFFSET	(10000)	/* in millicelsius */

enum tegra_bb_type {
	TEGRA_BB_TANGO = 1,
};

#endif
