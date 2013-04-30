/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/leds.h>
#include <linux/leds-pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/restart.h>
#include "devices.h"
#include "board-m7wl.h"
#include <asm/setup.h>

void m7wl_pm8xxx_adc_device_register(void);

struct pm8xxx_gpio_init {
	unsigned			gpio;
	struct pm_gpio			config;
};

struct pm8xxx_mpp_init {
	unsigned			mpp;
	struct pm8xxx_mpp_config_data	config;
};

#define PM8921_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= PM8921_GPIO_PM_TO_SYS(_gpio), \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8921_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8921_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8821_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8821_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8921_GPIO_DISABLE(_gpio) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, 0, PM_GPIO_VIN_S4, \
			 0, 0, 0, 1)

#define PM8921_GPIO_OUTPUT(_gpio, _val, _strength) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_##_strength, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8921_GPIO_OUTPUT_BUFCONF(_gpio, _val, _strength, _bufconf) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT,\
			PM_GPIO_OUT_BUF_##_bufconf, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_##_strength, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8921_GPIO_INPUT(_gpio, _pull) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8921_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)

#define PM8921_GPIO_OUTPUT_VIN(_gpio, _val, _vin) \
	PM8921_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

static struct pm8xxx_gpio_init pm8921_gpios[] __initdata = {
	PM8921_GPIO_OUTPUT_FUNC(26, 0, PM_GPIO_FUNC_2),
	
	PM8921_GPIO_OUTPUT(34, 1, MED),
};

static struct pm8xxx_gpio_init pm8921_cdp_kp_gpios[] __initdata = {
	
};

static struct pm8xxx_mpp_init pm8xxx_mpps[] __initdata = {
	PM8921_MPP_INIT(3, D_OUTPUT, PM8921_MPP_DIG_LEVEL_VPH, DOUT_CTRL_LOW),
	
	PM8921_MPP_INIT(7, D_OUTPUT, PM8921_MPP_DIG_LEVEL_VPH, DOUT_CTRL_LOW),
	PM8921_MPP_INIT(PM8XXX_AMUX_MPP_8, A_INPUT, PM8XXX_MPP_AIN_AMUX_CH5, AOUT_CTRL_DISABLE),
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	PM8921_MPP_INIT(PM8XXX_AMUX_MPP_1, D_BI_DIR, PM8921_MPP_DIG_LEVEL_S4, BI_PULLUP_10KOHM),
	PM8921_MPP_INIT(PM8XXX_AMUX_MPP_2, D_BI_DIR, PM8921_MPP_DIG_LEVEL_L17, BI_PULLUP_10KOHM),
	PM8921_MPP_INIT(PM8XXX_AMUX_MPP_3, D_BI_DIR, PM8921_MPP_DIG_LEVEL_S4, BI_PULLUP_10KOHM),
	PM8921_MPP_INIT(PM8XXX_AMUX_MPP_4, D_BI_DIR, PM8921_MPP_DIG_LEVEL_L17, BI_PULLUP_10KOHM),
#endif
};

void __init m7wl_pm8xxx_gpio_mpp_init(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(pm8921_gpios); i++) {
		rc = pm8xxx_gpio_config(pm8921_gpios[i].gpio,
					&pm8921_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n", __func__, rc);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(pm8921_cdp_kp_gpios); i++) {
		rc = pm8xxx_gpio_config(pm8921_cdp_kp_gpios[i].gpio,
					&pm8921_cdp_kp_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n",
				__func__, rc);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(pm8xxx_mpps); i++) {
		rc = pm8xxx_mpp_config(pm8xxx_mpps[i].mpp,
					&pm8xxx_mpps[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_mpp_config: rc=%d\n", __func__, rc);
			break;
		}
	}
}

static struct pm8xxx_pwrkey_platform_data m7wl_pm8921_pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us	= 15625,
	.wakeup			= 1,
};

static struct pm8xxx_misc_platform_data m7wl_pm8921_misc_pdata = {
	.priority		= 0,
};

#define PM8921_LC_LED_MAX_CURRENT	4	
#define PM8921_LC_LED_LOW_CURRENT	1	
#define PM8XXX_LED_PWM_PERIOD		1000
#define PM8XXX_LED_PWM_DUTY_MS		20
#define PM8XXX_PWM_CHANNEL_NONE		-1
static DEFINE_MUTEX(led_lock);
static struct regulator *led_reg_l29;
static int led_power_LPM(int on)
{
	int rc = 0;

	mutex_lock(&led_lock);
	pr_info("[LED] %s: enter:%d\n", __func__, on);

	if (led_reg_l29 == NULL) {
	led_reg_l29 = regulator_get(NULL, "8921_l29");
	if (IS_ERR(led_reg_l29)) {
			pr_err("[LED] %s: Unable to get '8921_l29' \n", __func__);
			mutex_unlock(&led_lock);
			return -ENODEV;
		}
	}
	if (on == 1) {
		rc = regulator_set_optimum_mode(led_reg_l29, 100);
	   if (rc < 0)
		pr_err("[LED] %s: enter LMP,set_optimum_mode l29 failed, rc=%d\n", __func__, rc);

	rc = regulator_enable(led_reg_l29);
		if (rc) {
			pr_err("'%s' regulator enable failed rc=%d\n", "led_reg_l29", rc);
			mutex_unlock(&led_lock);
			return rc;
		}
		pr_info("[LED] %s: enter LMP mode\n", __func__);
	} else {
		rc = regulator_set_optimum_mode(led_reg_l29, 100000);
	   if (rc < 0)
		pr_err("[LED] %s: leave LMP,set_optimum_mode l29 failed, rc=%d\n", __func__, rc);

		rc = regulator_enable(led_reg_l29);
		if (rc) {
			 pr_err("'%s' regulator enable failed, rc=%d\n", "led_reg_l29", rc);
		mutex_unlock(&led_lock);
		   return rc;
		}
		pr_info("[LED] %s: leave LMP mode\n", __func__);
		msleep(10);
	}
	mutex_unlock(&led_lock);
	return rc;
}
static struct pm8xxx_led_configure pm8921_led_info[] = {
	[0] = {
		.name		= "button-backlight",
		.flags		= PM8XXX_ID_LED_0,
		.function_flags = LED_PWM_FUNCTION | LED_BRETH_FUNCTION,
		.period_us 	= USEC_PER_SEC / 1000,
		.start_index 	= 0,
		.duites_size 	= 8,
		.duty_time_ms 	= 64,
		.lut_flag 	= PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current    = 20,
		.duties		= {0, 15, 30, 45, 60, 75, 90, 100,
				100, 90, 75, 60, 45, 30, 15, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0},
		.lpm_power = led_power_LPM,
	},
        [1] = {
                .name           = "green",
                .flags          = PM8XXX_ID_LED_1,
                .function_flags = LED_PWM_FUNCTION | LED_BLINK_FUNCTION,
                .out_current    = 2,
                .pwm_coefficient = 5,
                .blink_duty_per_2sec = 10000,
        },
        [2] = {
                .name           = "amber",
                .flags          = PM8XXX_ID_LED_2,
                .function_flags = LED_PWM_FUNCTION | LED_BLINK_FUNCTION,
		.out_current    = 3,
                .pwm_coefficient = 5,
        },
};

static struct pm8xxx_led_platform_data m7wl_pm8921_leds_pdata = {
	.num_leds = ARRAY_SIZE(pm8921_led_info),
	.leds = pm8921_led_info,
};


static struct pm8xxx_adc_amux m7wl_pm8921_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
	{"mpp_amux6", ADC_MPP_1_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"amux_in", ADC_MPP_1_AMUX4, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
};

static struct pm8xxx_adc_properties m7wl_pm8921_adc_data = {
	.adc_vdd_reference	= 1800, 
	.bitresolution		= 15,
	.bipolar                = 0,
};
static const struct pm8xxx_adc_map_pt m7wls_adcmap_btm_table[] = {
	{-200,	1670},
	{-190,	1662},
	{-180,	1654},
	{-170,	1645},
	{-160,	1636},
	{-150,	1626},
	{-140,	1616},
	{-130,	1606},
	{-120,	1595},
	{-110,	1583},
	{-100,	1571},
	{-90,	1559},
	{-80,	1546},
	{-70,	1533},
	{-60,	1519},
	{-50,	1505},
	{-40,	1491},
	{-30,	1476},
	{-20,	1460},
	{-10,	1444},
	{-0,	1428},
	{10,	1411},
	{20,	1393},
	{30,	1376},
	{40,	1358},
	{50,	1339},
	{60,	1321},
	{70,	1301},
	{80,	1282},
	{90,	1262},
	{100,	1242},
	{110,	1222},
	{120,	1202},
	{130,	1181},
	{140,	1161},
	{150,	1140},
	{160,	1119},
	{170,	1097},
	{180,	1077},
	{190,	1055},
	{200,	1034},
	{210,	1013},
	{220,	992},
	{230,	971},
	{240,	950},
	{250,	930},
	{260,	909},
	{270,	889},
	{280,	869},
	{290,	849},
	{300,	829},
	{310,	810},
	{320,	790},
	{330,	772},
	{340,	753},
	{350,	735},
	{360,	717},
	{370,	700},
	{380,	683},
	{390,	666},
	{400,	649},
	{410,	633},
	{420,	618},
	{430,	602},
	{440,	587},
	{450,	573},
	{460,	559},
	{470,	545},
	{480,	531},
	{490,	518},
	{500,	506},
	{510,	493},
	{520,	481},
	{530,	470},
	{540,	458},
	{550,	447},
	{560,	437},
	{570,	426},
	{580,	416},
	{590,	406},
	{600,	397},
	{610,	388},
	{620,	379},
	{630,	371},
	{640,	362},
	{650,	354},
	{660,	347},
	{670,	339},
	{680,	332},
	{690,	325},
	{700,	318},
	{710,	312},
	{720,	306},
	{730,	299},
	{740,	294},
	{750,	288},
	{760,	282},
	{770,	277},
	{780,	272},
	{790,	267}
};

static struct pm8xxx_adc_map_table pm8xxx_adcmap_btm_table = {
	.table = m7wls_adcmap_btm_table,
	.size = ARRAY_SIZE(m7wls_adcmap_btm_table),
};

static struct pm8xxx_adc_platform_data m7wl_pm8921_adc_pdata = {
	.adc_channel		= m7wl_pm8921_adc_channels_data,
	.adc_num_board_channel	= ARRAY_SIZE(m7wl_pm8921_adc_channels_data),
	.adc_prop		= &m7wl_pm8921_adc_data,
	.adc_mpp_base		= PM8921_MPP_PM_TO_SYS(1),
	.adc_map_btm_table	= &pm8xxx_adcmap_btm_table,
	.pm8xxx_adc_device_register	= m7wl_pm8xxx_adc_device_register,
};

static struct pm8xxx_mpp_platform_data
m7wl_pm8921_mpp_pdata __devinitdata = {
	.mpp_base	= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_gpio_platform_data
m7wl_pm8921_gpio_pdata __devinitdata = {
	.gpio_base	= PM8921_GPIO_PM_TO_SYS(1),
};

static struct pm8xxx_irq_platform_data
m7wl_pm8921_irq_pdata __devinitdata = {
	.irq_base		= PM8921_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PM8921_APC_USR_IRQ_N),
	.irq_trigger_flag	= IRQF_TRIGGER_LOW,
	.dev_id			= 0,
};

static struct pm8xxx_rtc_platform_data
m7wl_pm8921_rtc_pdata = {
	.rtc_write_enable       = true,
	.rtc_alarm_powerup      = false,
};

static int m7wl_pm8921_therm_mitigation[] = {
	1100,
	700,
	600,
	225,
};

#define MAX_VOLTAGE_MV          4200
static struct pm8921_charger_platform_data
pm8921_chg_pdata __devinitdata = {
	.safety_time		= 960,
	.update_time		= 60000,
	.max_voltage		= MAX_VOLTAGE_MV,
	.min_voltage		= 3200,
	.resume_voltage_delta	= 50,
	.term_current		= 50,
	.cool_temp		= 0,
	.warm_temp		= 48,
	.temp_check_period	= 1,
	.max_bat_chg_current	= 1025,
	.cool_bat_chg_current	= 1025,
	.warm_bat_chg_current	= 1025,
	.cool_bat_voltage	= 4200,
	.warm_bat_voltage	= 4000,
	.mbat_in_gpio		= 0, 
	.cable_in_irq		= PM8921_GPIO_IRQ(PM8921_IRQ_BASE, CABLE_IN_N),
	.cable_in_gpio		= PM8921_GPIO_PM_TO_SYS(CABLE_IN_N),
	.is_embeded_batt	= 1,
	.thermal_mitigation	= m7wl_pm8921_therm_mitigation,
	.thermal_levels		= ARRAY_SIZE(m7wl_pm8921_therm_mitigation),
	.cold_thr = PM_SMBC_BATT_TEMP_COLD_THR__HIGH,
	.hot_thr = PM_SMBC_BATT_TEMP_HOT_THR__LOW,
};

static struct pm8xxx_ccadc_platform_data
m7wl_pm8xxx_ccadc_pdata = {
	.r_sense		= 10,
	.calib_delay_ms		= 600000,
};

static struct pm8921_bms_platform_data
pm8921_bms_pdata __devinitdata = {
	.r_sense		= 10,
	.i_test			= 2000,
	.v_failure		= 3000,
	.max_voltage_uv		= MAX_VOLTAGE_MV * 1000,
	.rconn_mohm		= 0,
	.level_ocv_update_stop_begin	= 10,
	.level_ocv_update_stop_end		= 20,
};

static int __init check_dq_setup(char *str)
{
	if (!strcmp(str, "PASS")) {
		pr_info("[BATT] overwrite HV battery config\n");
		pm8921_chg_pdata.max_voltage = 4340;
		pm8921_chg_pdata.cool_bat_voltage = 4340;
		pm8921_bms_pdata.max_voltage_uv = 4340 * 1000;
	} else {
		pr_info("[BATT] use default battery config\n");
		pm8921_chg_pdata.max_voltage = 4200;
		pm8921_chg_pdata.cool_bat_voltage = 4200;
		pm8921_bms_pdata.max_voltage_uv = 4200 * 1000;
	}
	return 1;
}
__setup("androidboot.dq=", check_dq_setup);

static struct pm8xxx_vibrator_platform_data pm8xxx_vib_pdata = {
	.initial_vibrate_ms = 0,
	.max_timeout_ms = 15000,
	.level_mV = 3000,
	};

static struct pm8921_platform_data
m7wl_pm8921_platform_data __devinitdata = {
	.regulator_pdatas	= m7wl_pm8921_regulator_pdata,
	.irq_pdata		= &m7wl_pm8921_irq_pdata,
	.gpio_pdata		= &m7wl_pm8921_gpio_pdata,
	.mpp_pdata		= &m7wl_pm8921_mpp_pdata,
	.rtc_pdata		= &m7wl_pm8921_rtc_pdata,
	.pwrkey_pdata	= &m7wl_pm8921_pwrkey_pdata,
	.misc_pdata		= &m7wl_pm8921_misc_pdata,
	.leds_pdata		= &m7wl_pm8921_leds_pdata,
	.adc_pdata		= &m7wl_pm8921_adc_pdata,
	.charger_pdata		= &pm8921_chg_pdata,
	.bms_pdata		= &pm8921_bms_pdata,
	.ccadc_pdata		= &m7wl_pm8xxx_ccadc_pdata,
	.vibrator_pdata         = &pm8xxx_vib_pdata,
};

static struct pm8xxx_irq_platform_data
m7wl_pm8821_irq_pdata __devinitdata = {
	.irq_base		= PM8821_IRQ_BASE,
	.devirq			= PM8821_SEC_IRQ_N,
	.irq_trigger_flag	= IRQF_TRIGGER_HIGH,
	.dev_id			= 1,
};

static struct pm8xxx_mpp_platform_data
m7wl_pm8821_mpp_pdata __devinitdata = {
	.mpp_base	= PM8821_MPP_PM_TO_SYS(1),
};

static struct pm8821_platform_data
m7wl_pm8821_platform_data __devinitdata = {
	.irq_pdata	= &m7wl_pm8821_irq_pdata,
	.mpp_pdata	= &m7wl_pm8821_mpp_pdata,
};

static struct msm_ssbi_platform_data m7wl_ssbi_pm8921_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name		= "pm8921-core",
		.platform_data	= &m7wl_pm8921_platform_data,
	},
};

static struct msm_ssbi_platform_data m7wl_ssbi_pm8821_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name		= "pm8821-core",
		.platform_data	= &m7wl_pm8821_platform_data,
	},
};

void __init m7wl_init_pmic(void)
{
	if (system_rev <= XD) {
		pm8921_chg_pdata.cable_in_irq = 0;
		pm8921_chg_pdata.cable_in_gpio = 0;
	}

	pmic_reset_irq = PM8921_IRQ_BASE + PM8921_RESOUT_IRQ;
	apq8064_device_ssbi_pmic1.dev.platform_data =
						&m7wl_ssbi_pm8921_pdata;
	apq8064_device_ssbi_pmic2.dev.platform_data =
				&m7wl_ssbi_pm8821_pdata;
	m7wl_pm8921_platform_data.num_regulators =
					m7wl_pm8921_regulator_pdata_len;

}

void __init m7wl_init_pmic_register_cam_cb(void *cam_vcm_on_cb, void *cam_vcm_off_cb)
{
	if (cam_vcm_on_cb)
		pm8xxx_vib_pdata.camera_cb = cam_vcm_on_cb;
	if (cam_vcm_off_cb)
		pm8xxx_vib_pdata.camera_off_cb = cam_vcm_off_cb;
}
