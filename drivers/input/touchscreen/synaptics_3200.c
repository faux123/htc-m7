/* drivers/input/touchscreen/synaptics_3200.c - Synaptics 3200 serious touch panel driver
 *
 * Copyright (C) 2011 HTC Corporation.
 * Copyright (C) 2013 CyanogenMod
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * Synaptics sweep2wake mods based off of showp1984's endeavoru mod
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/slab.h>
#include <linux/rmi.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_hsusb.h>
#include <asm/gpio.h>
#include <linux/input/mt.h>
#include <linux/pl_sensor.h>
#include <linux/async.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
#include <linux/ctype.h>
#include <linux/mfd/pm8xxx/vibrator.h>
#include <linux/pl_sensor.h>
#endif

#define SYN_I2C_RETRY_TIMES 10
#define SHIFT_BITS 10
#define SYN_WIRELESS_DEBUG
#define SYN_CALIBRATION_CONTROL

#define SYN_FW_NAME "tp_SYN.img"
#define SYN_FW_TIMEOUT (30000)
static DEFINE_MUTEX(syn_fw_mutex);

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *sr_input_dev;
	struct workqueue_struct *syn_wq;
	struct function_t *address_table;
	int use_irq;
	int gpio_irq;
	int gpio_reset;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	uint32_t flags;
	uint8_t num_function;
	uint8_t finger_support;
	uint16_t finger_pressed;
	int (*power)(int on);
	int (*lpm_power)(int on);
	struct early_suspend early_suspend;
	int pre_finger_data[11][4];
	uint32_t debug_log_level;
	uint32_t enable_noise_log;
	uint32_t raw_base;
	uint32_t raw_ref;
	uint64_t timestamp;
	uint16_t *filter_level;
	uint8_t *reduce_report_level;
	unsigned long tap_timeout[10];
	int16_t *report_data;
	uint8_t *temp_report_data;
	uint8_t grip_suppression;
	uint8_t grip_b_suppression;
	uint16_t tap_suppression;
	uint8_t ambiguous_state;
	uint8_t diag_command;
	uint8_t disable_CBC;
	uint8_t cable_support;
	uint8_t cable_config;
	uint8_t key_number;
	uint16_t key_postion_x[4];
	uint16_t key_postion_y;
	uint8_t intr_bit;
	uint8_t finger_count;
	uint8_t page_select;
	uint8_t config_table[SYN_CONFIG_SIZE];
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t *config;
	uint32_t config_version;
	uint16_t package_id;
	uint32_t packrat_number;
	int layout[4];
	uint8_t htc_event;
	atomic_t data_ready;
	uint8_t relaxation;
	uint8_t irq_enabled;
	uint8_t large_obj_check;
	uint8_t default_large_obj;
	uint16_t tw_vendor;
	uint16_t tw_pin_mask;
	uint8_t support_htc_event;
	uint8_t mfg_flag;
	uint8_t first_pressed;
	uint8_t segmentation_bef_unlock;
	uint8_t segmentation_aft_unlock;
	uint8_t psensor_status;
	uint8_t i2c_err_handler_en;
	uint8_t threshold_bef_unlock;
	uint8_t threshold_aft_unlock;
	uint16_t saturation_bef_unlock;
	uint16_t saturation_aft_unlock;
	uint8_t energy_ratio_relaxation;
	uint8_t multitouch_calibration;
	uint32_t width_factor;
	uint32_t height_factor;
	uint8_t finger_func_idx;
	uint8_t psensor_detection;
	uint8_t psensor_resume_enable;
	uint8_t psensor_phone_enable;
	uint8_t PixelTouchThreshold_bef_unlock;
	uint8_t PixelTouchThreshold_aft_unlock;
	struct work_struct  psensor_work;
	struct workqueue_struct *syn_psensor_wq;
	struct synaptics_virtual_key *button;
	wait_queue_head_t syn_fw_wait;
	atomic_t syn_fw_condition;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static DECLARE_WAIT_QUEUE_HEAD(syn_data_ready_wq);
static DEFINE_MUTEX(syn_mutex);

static struct synaptics_ts_data *gl_ts;
static uint16_t syn_panel_version;
static uint8_t vk_press;

static int i2c_syn_write_byte_data(struct i2c_client *client, uint16_t addr, uint8_t value);
static int syn_pdt_scan(struct synaptics_ts_data *ts, int num_page);
static int synaptics_init_panel(struct synaptics_ts_data *ts);

static irqreturn_t synaptics_irq_thread(int irq, void *ptr);

extern unsigned int get_tamper_sf(void);

bool scr_suspended = false;

extern uint8_t touchscreen_is_on(void)
{
	if (scr_suspended == false)
	{
		return 1;
	}
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
#define BACK_BUTTON		250
#define HOME_BUTTON		818
#define MENU_BUTTON		1335

int pwp_switch = 1; // 1 -> pocket wake protection on, 0 - off

int l2m_2_phase = 0; // 0 -> logo used as power off on long tap, and short tap syncs input on/off at same time,  1 -> logo used as full menu button, sync on/off events separately

int s2w_switch = 0;
int s2w_temp = 0;

int h2w_switch = 0; // 0 -> No wake 1 -> Home2Wake, 2 -> Home2Wake/Logo2Sleep if l2m_switch = 0,  3 -> Logo2wake/Logo2sleep if l2m_switch = 0
int h2w_temp = 0;

int l2m_switch = 1; // 0 -> No menu map, 1 Menu map
int l2m_temp = 1;

int logo_delay_switch = 1; // if 1 -> Logo2Sleep/Logo2Wake will wait for long tap, if 0, it won't wait for no jiffies.

int sleep_wake_vibration_time = 6; // length of vibration in msec/5 - set 0 to deactivate it, 1 -> 5, 2 -> 10, 6 -> 30, 9 -> 45 (max)

bool exec_count = true, h2w_switch_changed = false, s2w_switch_changed = false;
bool scr_on_touch = false, led_exec_count = false, barrier[2] = {false, false};
static struct input_dev * sweep2wake_pwrdev;
static struct led_classdev * sweep2wake_leddev;
int barrier1 = 0, barrier2 = 0, barrier3 = 0, barrier4 = 0;
static DEFINE_MUTEX(pwrlock);
static DEFINE_MUTEX(longtap_count_lock);

static unsigned long BETWEEN_SLEEP_OR_WAKE_PRESS_MIN_DIFF = 15;
static unsigned long BETWEEN_LONG_PRESS_MIN_DIFF = 50;


typedef struct {
	int	x;
	char	name[10];
} button;

static button buttons[] = {
			{BACK_BUTTON, "BACK"},
			{HOME_BUTTON, "HOME"},
			{MENU_BUTTON, "MENU"},
};

int s2w_startbutton = -1;
int s2w_endbutton = -1;

int sweep2wake_buttonset(const char * button_name) {
	int i = 0;
	int future_button = -1;
	char temp_button_name[2] = "";
	char temp_button_name_from_array[2] = "";

	strncpy(temp_button_name,button_name,1);
	temp_button_name[1] = '\0';

	for (i = 0; i < sizeof(buttons)/sizeof(button); i++)
	{
		strncpy(temp_button_name_from_array,buttons[i].name,1);
		temp_button_name_from_array[1] = '\0';

		if (strcmp(temp_button_name,temp_button_name_from_array) == 0)
			future_button = buttons[i].x;

		temp_button_name_from_array[0] = tolower(temp_button_name_from_array[0]);

		if (strcmp(temp_button_name,temp_button_name_from_array) == 0)
			future_button = buttons[i].x;
	}

	return future_button;
}

extern void sweep2wake_setdev(struct input_dev * input_device) {
        sweep2wake_pwrdev = input_device;
        return;
}
EXPORT_SYMBOL(sweep2wake_setdev);

extern void sweep2wake_setleddev(struct led_classdev * led_dev) {
        sweep2wake_leddev = led_dev;
        return;
}
EXPORT_SYMBOL(sweep2wake_setleddev);

// set this to 1 to stop longtap count work
static int break_longtap_count = 0;

static void sweep2wake_presspwr(struct work_struct * sweep2wake_presspwr_work) {
	if (scr_suspended == true && pwp_switch == 1 && power_key_check_in_pocket()) return; // don't wake if in pocket

	if (!mutex_trylock(&pwrlock))
	    return;

	break_longtap_count = 1;
	printk("sending event KEY_POWER 1\n");
	if (sleep_wake_vibration_time)
	{
		vibrate(sleep_wake_vibration_time * 5);
	}
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_sync(sweep2wake_pwrdev);
	msleep(100);
	input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_sync(sweep2wake_pwrdev);
	msleep(100);

	mutex_unlock(&pwrlock);
	return;
}
static DECLARE_WORK(sweep2wake_presspwr_work, sweep2wake_presspwr);

static int menu_pressed = 0;

static void sweep2wake_pressmenu(struct work_struct * sweep2wake_pressmenu_work) {
	struct synaptics_ts_data *ts = gl_ts;
	int do_0 = 1;
	int wake_switch = (h2w_switch > 0 || s2w_switch > 0) ? 1:0;

	// break long tap count if 2 phase input sync is not active
	if (l2m_2_phase == 0 && wake_switch) break_longtap_count = 1;

	if ((l2m_2_phase == 1 || !wake_switch) && menu_pressed == 0)
	{
		do_0 = 0;
	}
	if ((l2m_2_phase == 0 && wake_switch) || menu_pressed == 0)
	{
		printk("sending event KEY_MENU 1\n");
		input_event(ts->input_dev, EV_KEY, KEY_MENU, 1);
		input_sync(ts->input_dev);
		if (l2m_2_phase == 0 && wake_switch) msleep(100);
		menu_pressed = 1;
        }
        if ((l2m_2_phase == 0 && wake_switch) || (do_0 == 1 && menu_pressed == 1))
        {
		printk("sending event KEY_MENU 0\n");
		input_event(ts->input_dev, EV_KEY, KEY_MENU, 0);
		input_sync(ts->input_dev);
		if (l2m_2_phase == 0 && wake_switch) msleep(100);
		menu_pressed = 0;
        }
        return;
}
static DECLARE_WORK(sweep2wake_pressmenu_work, sweep2wake_pressmenu);



void sweep2wake_pwrtrigger(void) {
	schedule_work(&sweep2wake_presspwr_work);
	return;
}
void sweep2wake_menutrigger(void) {
	schedule_work(&sweep2wake_pressmenu_work);
	return;
}

// if finger is released, set this to 1, means longtap count can begin. If user keeps finger on screen, dont allow it after one count went down...
static int allow_longtap_count = 1;

static void sweep2wake_longtap_count(struct work_struct * sweep2wake_longtap_count_work) {
	struct synaptics_ts_data *ts = gl_ts;
	int time_count = 0;
	if (!mutex_trylock(&longtap_count_lock))
	    return;
	time_count = jiffies;
	while (1)
	{
		if (break_longtap_count)
		{
			printk("[LTC] breaking longtap count work...\n");
			break_longtap_count = 0;
			mutex_unlock(&longtap_count_lock);
			return;
		}
		if (jiffies - time_count > BETWEEN_LONG_PRESS_MIN_DIFF)
		{
			printk("[LTC] counted to longtap time!\n");
			break;
		}
		msleep(3);
	}
	if (!break_longtap_count)
	{
		if (scr_suspended == false && l2m_switch == 1 && ( l2m_2_phase == 1 || (s2w_switch == 0 && h2w_switch == 0) ))
		{
			if (menu_pressed == 1)
			{
				printk("sending event KEY_MENU 0\n");
				input_event(ts->input_dev, EV_KEY, KEY_MENU, 0);
				input_sync(ts->input_dev);
				menu_pressed = 0;
			}
		}
		else
		if ( scr_suspended == true || ( (s2w_switch > 0 || h2w_switch > 0) && ( l2m_switch == 0 || (l2m_switch == 1 && l2m_2_phase == 0) ) ) ) // screen is off, or wake option is set with logo2menu is not used, or logo2menu is set but not 2_phase
		{
			if (scr_suspended == false || pwp_switch == 0 || (pwp_switch == 1 && !power_key_check_in_pocket())) {
				if (sleep_wake_vibration_time)
				{
					vibrate(sleep_wake_vibration_time * 5);
				}
				printk("LONGTAP sending event KEY_POWER 1\n");
				input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 1);
				input_sync(sweep2wake_pwrdev);
				msleep(100);
				input_event(sweep2wake_pwrdev, EV_KEY, KEY_POWER, 0);
				input_sync(sweep2wake_pwrdev);
				msleep(100);
			}
		}
	}
	mutex_unlock(&longtap_count_lock);
	return;
}

static DECLARE_WORK(sweep2wake_longtap_count_work, sweep2wake_longtap_count);

void sweep2wake_longtap_count_trigger(void) {
	break_longtap_count = 0;
	if (allow_longtap_count)
	{
		// dont allow another longtap count work, till finger is released!
		allow_longtap_count = 0;
		printk("[LTC] starting longtap count work\n");
		schedule_work(&sweep2wake_longtap_count_work);
	} else
	{
		printk("[LTC] not longtap count work - not allowed (waiting for finger release)\n");
	}
        return;
}

#endif
static void syn_page_select(struct i2c_client *client, uint8_t page)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	if (page ^ ts->page_select) {
		i2c_smbus_write_byte_data(client, 0xFF, page);
		ts->page_select = page;
	}

}

static int i2c_syn_read(struct i2c_client *client, uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry, buf;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	buf = addr & 0xFF;

	mutex_lock(&syn_mutex);
	syn_page_select(client, addr >> 8);
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		hr_msleep(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk(KERN_INFO "[TP] i2c_read retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_syn_write(struct i2c_client *client, uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&syn_mutex);
	syn_page_select(client, addr >> 8);

	buf[0] = addr & 0xFF;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk(KERN_INFO "[TP] i2c_write retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

int i2c_rmi_read(uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry, buf;
	struct synaptics_ts_data *ts = gl_ts;
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	buf = addr & 0xFF;

	mutex_lock(&syn_mutex);
	syn_page_select(ts->client, addr >> 8);
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 2) == 2)
			break;
		hr_msleep(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk(KERN_INFO "[TP] i2c_read retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(i2c_rmi_read);

int i2c_rmi_write(uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry;
	uint8_t buf[length + 1];
	struct synaptics_ts_data *ts = gl_ts;
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&syn_mutex);
	syn_page_select(ts->client, addr >> 8);

	buf[0] = addr & 0xFF;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		printk(KERN_INFO "[TP] i2c_write retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(i2c_rmi_write);

static int i2c_syn_write_byte_data(struct i2c_client *client, uint16_t addr, uint8_t value)
{
	return i2c_syn_write(client, addr, &value, 1);
}

static int i2c_syn_error_handler(struct synaptics_ts_data *ts, uint8_t reset, char *reason, const char *fun_name)
{
	int ret;

	if (reason && fun_name)
		printk(KERN_ERR "[TP] TOUCH_ERR: I2C Error: %s:%s, reset = %d\n", fun_name, reason, reset);
	else
		printk(KERN_INFO "[TP] %s: rason and fun_name can't be null\n", __func__);

	if (reset) {
		if (ts->power) {
			ret = ts->power(0);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler power off failed\n");
			hr_msleep(10);
			ret = ts->power(1);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler power on failed\n");
			ret = synaptics_init_panel(ts);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler init panel failed\n");
		} else if (ts->gpio_reset) {
			gpio_direction_output(ts->gpio_reset, 0);
			hr_msleep(1);
			gpio_direction_output(ts->gpio_reset, 1);
			printk(KERN_INFO "[TP] %s: synaptics touch chip reseted.\n", __func__);
		}

		if (!ts->use_irq) {
			hrtimer_cancel(&ts->timer);
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}

	return -EIO;
}

static int get_address_base(struct synaptics_ts_data *ts, uint8_t command, uint8_t type)
{
	uint8_t i;
	for (i = 0; i < ts->num_function; i++) {
		if (ts->address_table[i].function_type == command) {
			switch (type) {
			case QUERY_BASE:
				return ts->address_table[i].query_base;
			case COMMAND_BASE:
				return ts->address_table[i].command_base;
			case CONTROL_BASE:
				return ts->address_table[i].control_base;
			case DATA_BASE:
				return ts->address_table[i].data_base;
			case INTR_SOURCE:
				return ts->address_table[i].interrupt_source;
			case FUNCTION:
				return 1;
			}
		}
	}
	if (type == FUNCTION)
		return 0;
	else
		return -1;
}

static int wait_flash_interrupt(struct synaptics_ts_data *ts, int attr, int fw)
{
	uint8_t data = 0;
	int i, ret;

	for (i = 0; i < 5000; i++) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
		printk(KERN_INFO "[TP] ATT: %d\n", gpio_get_value(attr));
#endif
		if (!gpio_get_value(attr)) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, DATA_BASE) + 1, &data, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
			if ((data & 0x01) == 0x01) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
				printk(KERN_INFO "[TP] ATT: %d, status: %x\n", gpio_get_value(attr), data);
#endif
				break;
			}
		}
		if (fw)
			mdelay(2);
		else
			hr_msleep(2);
	}

	if (i == 5000 && syn_panel_version == 0) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
	} else if (i == 5000) {
		printk(KERN_INFO "[TP] wait_flash_interrupt: interrupt over time!\n");
		return SYN_PROCESS_ERR;
	}

	if (ts->package_id < 3400)
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 0x12, &data, 1);
	else
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 0x3, &data, 1);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
	
	if (!(data & 0x80)) {
		printk(KERN_INFO "[TP] Not in program enable mode, F34_data = %x\n", data);
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		printk(KERN_INFO "[TP] Not in program enable mode, 01_data = %x\n", data);
		return SYN_PROCESS_ERR;
	} else if (data != 0x80) {
		printk(KERN_INFO "[TP] Not in program enable mode, data = %x\n", data);
	}
	return 0;
}

static int enable_flash_programming(struct synaptics_ts_data *ts, int attr)
{
	int ret;
	uint8_t data[2] = {0};

	if (attr) {
		if (!gpio_get_value(attr)) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, DATA_BASE) + 1, data, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
			printk(KERN_INFO "[TP] %s: clear gpio_irq before enter bootloader mode, data = %x\n", __func__, data[0]);
		}
	}
	
	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x34, QUERY_BASE), data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);

	if (ts->package_id < 3400)
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, data, 2);
	else
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

	if (ts->package_id < 3400)
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 18, 0x0F);
	else
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, 0x0F);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);

	ret = wait_flash_interrupt(ts, attr, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int disable_flash_programming(struct synaptics_ts_data *ts, int status)
{
	int ret;
	uint8_t data = 0, i;

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x01, COMMAND_BASE), 0x01);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

	for (i = 0; i < 25; i++) {
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);

		if ((data & 0x40) == 0)
			break;
		else
			hr_msleep(20);
	}

	if (i == 25) {
		printk(KERN_INFO "[TP] Disable flash programming fail! F01_data: %X\n", data);
		return SYN_PROCESS_ERR;
	} else {
		printk(KERN_INFO "[TP] Disable flash programming success! F01_data: %X\n", data);
		return status;
	}
}


static int program_fw_img(struct synaptics_ts_data *ts,
			  const uint8_t *imgbuf, const uint16_t filelength)
{
	uint8_t data[20] = {0};
	uint16_t blocksize, fwcnt, cfgcnt ,i;
	int ret;

	if (syn_pdt_scan(ts, SYN_BL_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR:%s: PDT scan fail\n", __func__);
		return -1;
	}
	
	i2c_syn_read(ts->client, get_address_base(ts, 0x34, QUERY_BASE), data, 2);

	
	if (ts->package_id < 3400)
		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE) + 0x2, data, 2);
	else
		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE) + 0x1, data, 2);

	
	printk(KERN_INFO "[TP][FW] Erase firmware and config\n");
	data[0] = 0x03;
	if (ts->package_id < 3400)
		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE) + 0x12, data, 1);
	else
		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE) + 0x2, data, 1);
	ret = wait_flash_interrupt(ts, ts->gpio_irq, 1);
	if (ret < 0)
		return ret;

	
	memset(data, 0x0, sizeof(data));
	if (ts->package_id < 3400)
		i2c_syn_read(ts->client, get_address_base(ts, 0x34, QUERY_BASE) + 0x3, data, 6);
	else
		i2c_syn_read(ts->client, get_address_base(ts, 0x34, QUERY_BASE) + 0x2, data, 6);
	blocksize = data[0] | data[1] << 8;
	fwcnt     = data[2] | data[3] << 8;
	cfgcnt    = data[4] | data[5] << 8;
#ifdef SYN_FLASH_PROGRAMMING_LOG
	printk(KERN_INFO "[TP][FW]Block size: %x, FW Block count: %x, Configure count: %x\n",
		blocksize, fwcnt, cfgcnt);
#endif

	
	printk(KERN_INFO "[TP][FW] Program firmware\n");
	for (i = 0; i < fwcnt; i++)
	{
		data[0] = i & 0xFF;
		data[1] = (i & 0xFF00) >> 8;
		memcpy(&data[2], &imgbuf[0x100 + (i * blocksize)], blocksize);
		data[2 + blocksize] = 0x02;

		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE), data, 3 + blocksize);

		
		ret = wait_flash_interrupt(ts, ts->gpio_irq, 1);
		if (ret < 0)
			return ret;
	}
	
	printk(KERN_INFO "[TP][FW] Program config\n");
	for (i = 0; i < cfgcnt; i++) {
		data[0] = i & 0xFF;
		data[1] = (i & 0xFF00) >> 8;
		memcpy(&data[2],
			&imgbuf[0x100 + (fwcnt* blocksize) + (i * blocksize)], blocksize);
		data[2 + blocksize] = 0x06;

		i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE), data, 3 + blocksize);

		
		ret = wait_flash_interrupt(ts, ts->gpio_irq, 1);
		if (ret < 0)
			return ret;
	}

	
	printk(KERN_INFO "[TP][FW] Disable flash programming\n");
	disable_flash_programming(ts, 1);
	return 0;
}

static int syn_firmware_update(struct synaptics_ts_data *ts)
{
	int ret = 0, i = 0;
	uint prnum = 0;
	uint8_t  start_addr = 0;
	u8 *pos, *pos_end;
	char prstr[10] = {0};
	const struct firmware *fw;

	ret = request_firmware(&fw, SYN_FW_NAME, &ts->client->dev);
	if (fw == NULL) {
		printk(KERN_INFO"[TP][FW] No firmware file, ignored firmware update\n");
		goto SYN_FW_REQUEST_FAILUE;
	}
	else if (ret) {
		printk(KERN_INFO"[TP][FW] Request_firmware failed, ret = %d\n", ret);
		goto SYN_FW_REQUEST_FAILUE;
	}
	if (fw->data[0] == 'T' && fw->data[1] == 'P') {
		pos = strchr(fw->data, 'R');				

		pos_end    = strchr(fw->data, '\n');                    
		start_addr = pos_end - fw->data + 1;
		memcpy(prstr, &fw->data[(pos - fw->data + 1)], 7);
		ret        = strict_strtoul(prstr, 10, (unsigned long*)&prnum);
		if (ret < 0) {
			printk(KERN_INFO "[TP][FW] TP tag parse failed %s, %d\n", prstr, prnum);
			goto SYN_FW_CHECK_FAILURE;
		}

		printk(KERN_INFO "[TP][FW] size=%d, pr=%u\n",
			(fw->size-(pos_end - fw->data + 1)), prnum);

		if (prnum != ts->packrat_number) {
			printk(KERN_INFO "[TP][FW] PR number is not the same, firmware update...\n");

			mutex_lock(&syn_fw_mutex);	
			for (i = 0; i < 5; i++) {
				if (!enable_flash_programming(ts, ts->gpio_irq)) {
					printk(KERN_INFO "[TP][FW] Enable flash ok\n");
					break;
				}
				hr_msleep(20);
			}
			if (i == 5) {
				printk(KERN_ERR "[TP][FW] TOUCH_ERR: syn_eanbl_flash retry 5 time Faile\n");
				ret = -1;
				goto SYN_FW_FAILUE;
			}

			if (!program_fw_img(ts, &fw->data[start_addr],
					    fw->size - start_addr )) {
				printk(KERN_INFO "[TP][FW] update successfully\n");
				release_firmware(fw);
			} else {
				printk(KERN_INFO "[TP][FW] update failed\n");
				ret = -2;
				goto SYN_FW_FAILUE;
			}
			mutex_unlock(&syn_fw_mutex);
		} else {
			printk(KERN_INFO "[TP][FW] PR number is the same. Ignored\n");
			ret = 1;
			goto SYN_FW_PRCHECK_NOT_ALLOW;
		}
	} else {
		printk(KERN_INFO "[TP][FW]Miss TP firmware tag. Ignored\n");
		ret = 2;
		goto SYN_FW_CHECK_FAILURE;
	}

	return 0;

SYN_FW_FAILUE:
SYN_FW_PRCHECK_NOT_ALLOW:
SYN_FW_CHECK_FAILURE:
SYN_FW_REQUEST_FAILUE:
	release_firmware(fw);

	return ret;
}

static int syn_fw_update_init(void *arg)
{
	int ret;
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)arg;
	printk(KERN_INFO "[TP][FW] %s\n", __func__);
	ret = syn_firmware_update(ts);
	if (ret)
		printk(KERN_INFO "[TP][FW] Bypassed firmware update, ret = %d\n", ret);
	atomic_set(&ts->syn_fw_condition, 1);
	wake_up(&ts->syn_fw_wait);

	return ret;
}

static int get_int_mask(uint8_t number, uint8_t offset)
{
	uint8_t i, mask = 0;
	for (i = 0; i < number; i++)
		mask |= BIT(i);
	return mask << offset;
}

static uint32_t syn_crc(uint16_t *data, uint16_t len)
{
	uint32_t sum1, sum2;
	sum1 = sum2 = 0xFFFF;
	while (len--) {
		sum1 += *data++;
		sum2 += sum1;
		sum1 = (sum1 & 0xFFFF) + (sum1 >> 16);
		sum2 = (sum2 & 0xFFFF) + (sum2 >> 16);
	}
	return sum1 | (sum2 << 16);
}

static int crc_comparison(struct synaptics_ts_data *ts, uint32_t config_crc, int attr)
{
	int ret;
	uint8_t data[17];
	uint32_t flash_crc;
#ifdef SYN_FLASH_PROGRAMMING_LOG
	uint8_t i, j;

	for (i = 0; i < 0x20; i++) {
		data[0] = i;
		data[1] = 0x00;
#else
		data[0] = 0x1F;
		data[1] = 0x00;
#endif
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE), data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

		if (ts->package_id < 3400)
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x34, DATA_BASE) + 18, 0x05);
		else
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x34, DATA_BASE) + 2, 0x05);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

		ret = wait_flash_interrupt(ts, attr, 0);
		if (ret < 0)
			return ret;

		if (ts->package_id < 3400)
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x34, DATA_BASE) + 2, data, 17);
		else
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x34, DATA_BASE) + 1, data, 17);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);

		memcpy(&flash_crc, &data[12], 4);

#ifdef SYN_FLASH_PROGRAMMING_LOG
		printk(KERN_INFO "[TP] config_crc = %X, flash_crc = %X\n", config_crc, flash_crc);
		for (j = 0; j < 0x11; j++)
			printk(KERN_INFO " %d:%X ", j, data[j]);
		printk(KERN_INFO "\n");
	}
#endif

	if (flash_crc == config_crc)
		return 0;
	else
		return 1;
}

static int program_config(struct synaptics_ts_data *ts, uint8_t *config, int attr)
{
	int ret;
	uint8_t data[19];
	uint16_t i;

	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x34, QUERY_BASE), data, 2);
	
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);

	if (ts->package_id < 3400)
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, data, 2);
	else
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

	
	if (ts->package_id < 3400)
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 18, 0x07);
	else
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, 0x07);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);

	ret = wait_flash_interrupt(ts, attr, 0);
	if (ret < 0)
		return ret;

	for (i = 0; i < 0x20; i++) {
		data[0] = i & 0xFF;
		data[1] = (i & 0xFF00) >> 8;
		memcpy(&data[2], &config[16 * i], 16);
		data[18] = 0x06;
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE), data, 19);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);

		ret = wait_flash_interrupt(ts, attr, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int syn_config_update(struct synaptics_ts_data *ts, int attr)
{
	uint8_t retry;
	uint32_t crc_checksum;
	int ret;

	crc_checksum =
		syn_crc((uint16_t *)ts->config, SYN_CONFIG_SIZE / 2 - 2);
	memcpy(&ts->config[SYN_CONFIG_SIZE - 4], &crc_checksum, 4);
	printk(KERN_INFO "[TP] CRC = %X\n" , syn_crc((uint16_t *)ts->config, SYN_CONFIG_SIZE / 2 - 2));

	if (ts->tw_pin_mask == 0) {
		ret = enable_flash_programming(ts, attr);
		if (ret < 0) {
			printk(KERN_INFO "[TP] syn_config_update: Enable flash programming fail!\n");
			return disable_flash_programming(ts, ret);
		}

		ret = syn_pdt_scan(ts, SYN_BL_PAGE);
		if (ret < 0) {
			printk(KERN_INFO "[TP] syn_config_update: pdt scan failed\n");
			return disable_flash_programming(ts, ret);
		}
	}

	if ((ts->config != NULL && (ts->config[0] << 24 | ts->config[1] << 16 |
		ts->config[2] << 8 | ts->config[3]) == ts->config_version)) {
		ret = crc_comparison(ts, crc_checksum, attr);
		if (ret < 0) {
			printk(KERN_INFO "[TP] syn_config_update: CRC comparison fail!\n");
			return disable_flash_programming(ts, ret);
		} else if (ret == 0)
			return disable_flash_programming(ts, 1);
	}

	for (retry = 0; retry < 3; retry++) {
		ret = program_config(ts, ts->config, attr);
		if (ret < 0) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
			printk(KERN_INFO "[TP] syn_config_update: Program config fail %d!\n", retry + 1);
#endif
			continue;
		}

		ret = disable_flash_programming(ts, 0);
		if (ret == 0)
			break;
		else
			printk(KERN_INFO "[TP] syn_config_update: Disable flash programming fail %d\n", retry + 1);
	}

	if (retry == 3) {
		printk(KERN_INFO "[TP] syn_config_update: Program config fail 3 times\n");
		return ret;
	}
	return 0;
}

static int syn_get_tw_vendor(struct synaptics_ts_data *ts, int attr)
{
	uint8_t data[6] = {0};
	int ret;

	ret = enable_flash_programming(ts, attr);
	if (ret < 0) {
		printk(KERN_INFO "[TP] Enable flash programming fail!\n");
		return disable_flash_programming(ts, -1);
	}

	ret = syn_pdt_scan(ts, SYN_BL_PAGE);
	if (ret < 0) {
		printk(KERN_INFO "[TP] syn_config_update: pdt scan failed\n");
		return disable_flash_programming(ts, ret);
	}

	memcpy(&data, &ts->tw_pin_mask, sizeof(ts->tw_pin_mask));
	printk(KERN_INFO "[TP] tw mask = %X %X , %X\n", data[0], data[1], ts->tw_pin_mask);
	data[2] = data[0];
	data[3] = data[1];
	if (ts->package_id < 3400) {
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, data, 2);
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 4, data, 2);
		i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 18, 0x08);
	} else {
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 4);
		i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, 0x08);
	}

	if (wait_flash_interrupt(ts, attr, 0) < 0)
		return disable_flash_programming(ts, -1);

	if (ts->package_id < 3400) {
		i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 6, data, 2);
		ts->tw_vendor = (data[1] << 8) | data[0];
		printk(KERN_INFO "[TP] tw vendor= %x %x\n", data[1], data[0]);
	} else {
		i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 6);
		ts->tw_vendor = (data[5] << 8) | data[4];
		printk(KERN_INFO "[TP] tw vendor= %x %x\n", data[5], data[4]);
	}

	return 0;
}

static int synaptics_input_register(struct synaptics_ts_data *ts)
{
	int ret;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "[TP] TOUCH_ERR: %s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	printk(KERN_INFO "[TP] input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);

	if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
		input_mt_init_slots(ts->input_dev, ts->finger_support);
	} else {
		ts->input_dev->mtsize = ts->finger_support;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			ts->finger_support - 1, 0, 0);
	}
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		ts->layout[0], ts->layout[1], 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		ts->layout[2], ts->layout[3], 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 30, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
			0, ((255 << 16) | 15), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, ((1 << 31) | (ts->layout[1] << 16) | ts->layout[3]), 0, 0);

	return input_register_device(ts->input_dev);
}

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	char fw_version[2];
	struct synaptics_ts_data *ts;

	ts = gl_ts;
	memcpy(fw_version, &syn_panel_version, 2);
	ret = sprintf(buf, "synaptics-%d_%c.%c", ts->package_id, fw_version[1], fw_version[0]);
	if (ts->tw_pin_mask != 0)
		ret += sprintf(buf+ret, "_twID-%x", ts->tw_vendor);
	else
		ret += sprintf(buf+ret, "\n");
	ret += sprintf(buf+ret, "_PR: %d\n", ts->packrat_number);

	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, touch_vendor_show, NULL);

static ssize_t gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct synaptics_ts_data *ts;

	ts = gl_ts;

	ret = gpio_get_value(ts->gpio_irq);
	printk(KERN_DEBUG "[TP] GPIO_TP_INT_N=%d\n", ret);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, gpio_show, NULL);

static uint16_t syn_reg_addr;

static ssize_t register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data = 0;
	struct synaptics_ts_data *ts;
	ts = gl_ts;

	ret = i2c_syn_read(ts->client, syn_reg_addr, &data, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
		ret += sprintf(buf, "addr: 0x , data: 0x \n");
	} else {
		ret += sprintf(buf, "addr: 0x%X, data: 0x%X\n", syn_reg_addr, data);
	}
	return ret;
}

static ssize_t register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct synaptics_ts_data *ts;
	char buf_tmp[4];
	uint8_t write_da;
	unsigned long addr;

	ts = gl_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[5] == ':' || buf[5] == '\n')) {
		memcpy(buf_tmp, buf + 2, 3);
		ret = strict_strtoul(buf_tmp, 16, &addr);
		syn_reg_addr = addr;
		printk(KERN_DEBUG "[TP] %s: set syn_reg_addr is: 0x%X\n",
						__func__, syn_reg_addr);
		if (buf[0] == 'w' && buf[5] == ':' && buf[9] == '\n') {
			memcpy(buf_tmp, buf + 6, 3);
			ret = strict_strtoul(buf_tmp, 16, &addr);
			write_da = addr;
			printk(KERN_DEBUG "[TP] write addr: 0x%X, data: 0x%X\n",
						syn_reg_addr, write_da);
			ret = i2c_syn_write_byte_data(ts->client,
					syn_reg_addr, write_da);
			if (ret < 0) {
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w", __func__);
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
	register_show, register_store);

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->debug_log_level);
}

static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	int i;

	ts->debug_log_level = 0;
	for(i=0; i<count-1; i++)
	{
		if( buf[i]>='0' && buf[i]<='9' )
			ts->debug_log_level |= (buf[i]-'0');
		else if( buf[i]>='A' && buf[i]<='F' )
			ts->debug_log_level |= (buf[i]-'A'+10);
		else if( buf[i]>='a' && buf[i]<='f' )
			ts->debug_log_level |= (buf[i]-'a'+10);

		if(i!=count-2)
			ts->debug_log_level <<= 4;
	}
	if (ts->debug_log_level & BIT(17))
		ts->enable_noise_log = 1;
	else
		ts->enable_noise_log = 0;

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	debug_level_show, debug_level_store);

static ssize_t syn_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;
	uint16_t i, j;
	int ret;

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x54, DATA_BASE), ts->diag_command);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
		count += sprintf(buf, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
		return count;
	}

	atomic_set(&ts->data_ready, 0);

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x54, COMMAND_BASE), 0x01);
	if (ret < 0) {
		atomic_set(&ts->data_ready, 1);
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
		count += sprintf(buf, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
		return count;
	}

	wait_event_interruptible_timeout(syn_data_ready_wq,
					atomic_read(&ts->data_ready), 50);

	for (i = 0; i < ts->y_channel; i++) {
		for (j = 0; j < ts->x_channel; j++) {
			if(ts->package_id == 3201)
				count += sprintf(buf + count, "%5d", ts->report_data[i*ts->x_channel + j]);
			else
				count += sprintf(buf + count, "%5d", ts->report_data[i + j*ts->y_channel]);
		}
		count += sprintf(buf + count, "\n");
	}
	return count;
}

static ssize_t syn_diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts;
	ts = gl_ts;
	if (buf[0] == '1')
		ts->diag_command = 2;
	else if (buf[0] == '2')
		ts->diag_command = 3;

	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	syn_diag_show, syn_diag_store);

static ssize_t syn_cbc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts;
	int ret;
	uint8_t data = 0, update = 0;
	ts = gl_ts;
	if (buf[0] == '0') {
		if (ts->package_id < 3400) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, &data, 1);
			if ((data & 0x10) == 0) {
				data |= 0x10;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, data);
				update = 1;
			}
		} else {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, &data, 1);
			if ((data & 0x20) == 0) {
				data |= 0x20;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, data);
				update = 1;
			}
		}
		if (ts->button) {
			if (ts->package_id < 3400) {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, &data, 1);
				if ((data & 0x10) == 0) {
					data |= 0x10;
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, data);
					update = 1;
				}
			} else {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, &data, 1);
				if ((data & 0x10) == 0) {
					data |= 0x10;
					ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, data);
					update = 1;
				}
			}
		}
		if (update) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
		}
		ts->disable_CBC = 0;
	} else if (buf[0] == '1') {
		if (ts->package_id < 3400) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, &data, 1);
			if (data & 0x10) {
				data &= 0xEF;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, data);
				update = 1;
			}
		} else {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, &data, 1);
			if (data & 0x20) {
				data &= 0xDF;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, data);
				update = 1;
			}
		}
		if (ts->button) {
			if (ts->package_id < 3400) {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, &data, 1);
				if (data & 0x10) {
					data &= 0xEF;
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, data);
					update = 1;
				}
			} else {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, &data, 1);
				if (data & 0x10) {
					data &= 0xEF;
					ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, data);
					update = 1;
				}
			}
		}
		if (update) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
		}
		ts->disable_CBC = 1;
	}

	return count;
}

static DEVICE_ATTR(disable_cbc, (S_IWUSR|S_IRUGO),
	NULL, syn_cbc_store);

static ssize_t syn_unlock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts;
	int unlock = -1;
	int ret = 0;

	ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		unlock = buf[0] - '0';

	printk(KERN_INFO "[TP] Touch: unlock change to %d\n", unlock);

	if (unlock == 2 && ts->first_pressed && ts->pre_finger_data[0][0] < 2) {
			ts->pre_finger_data[0][0] = 2;
		if(ts->psensor_detection) {
			if(ts->psensor_resume_enable == 1) {
				printk(KERN_INFO "[TP] %s: Disable P-sensor by Touch\n", __func__);
				psensor_enable_by_touch_driver(0);
				ts->psensor_resume_enable = 0;
			}
			else if(ts->psensor_resume_enable == 2) {
				ts->psensor_resume_enable = 0;
			}
		}
		if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
#ifdef SYN_CALIBRATION_CONTROL
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, 0x0);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

			if (ts->energy_ratio_relaxation) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE), 0x0);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
			}

			if (ts->saturation_bef_unlock) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_aft_unlock & 0xFF);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_aft_unlock & 0xFF00) >> 8);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
				printk(KERN_INFO "[TP] %s: unlock confirmed. set saturation: %x\n"
					, __func__, ts->saturation_aft_unlock);
			}

			if ( ts->PixelTouchThreshold_bef_unlock ) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_aft_unlock);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
				printk(KERN_INFO "[TP] %s: set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", __func__, ts->PixelTouchThreshold_aft_unlock);
			}


			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);

			if (ts->multitouch_calibration) {
				if (ts->package_id < 3400) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
					printk(KERN_INFO "[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
				}
			}
#endif
			if (ts->large_obj_check) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x26, ts->default_large_obj);

				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x29, ts->default_large_obj);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:7", __func__);
				printk(KERN_INFO "[TP] %s: unlock confirmed. set large obj suppression: %x\n"
					, __func__, ts->default_large_obj);
			}

			if (ts->segmentation_bef_unlock) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x25, ts->segmentation_aft_unlock);

				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x22, ts->segmentation_aft_unlock);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:8", __func__);
				printk(KERN_INFO "[TP] %s: unlock confirmed. set segmentation aggressiveness: %x\n"
					, __func__, ts->segmentation_aft_unlock);
			}

			if (ts->threshold_bef_unlock) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0A, ts->threshold_aft_unlock);
				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0C, ts->threshold_aft_unlock);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:9", __func__);
				printk(KERN_INFO "[TP] %s: unlock confirmed. set Z Touch threshold: %x\n"
					, __func__, ts->threshold_aft_unlock);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(unlock, (S_IWUSR|S_IRUGO),
	NULL, syn_unlock_store);

static ssize_t syn_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint16_t i, length = 0;
	uint8_t j, temp_func_cmd = 0, temp_func_query = 0, size = 0;
	size_t count = 0;
	int ret;

	printk(KERN_INFO "[TP] ts->num_function: %d\n", ts->num_function);
	for (i = 0; i < SYN_MAX_PAGE; i++) {
		for (j = 0; j < ts->num_function; j++) {
			if (((ts->address_table[j].control_base >> 8) & 0xFF) == i) {
				temp_func_query = 0;
				for (temp_func_cmd = j; temp_func_cmd < ts->num_function; temp_func_cmd++) {
					uint16_t max_addr = (i << 8) | 0xFF;
					uint16_t min_addr = (i << 8) | 0;
					if ((ts->address_table[temp_func_cmd].command_base > min_addr) &&
						(ts->address_table[temp_func_cmd].command_base <= max_addr))
						break;
					if ((ts->address_table[temp_func_cmd].query_base > min_addr) &&
						(ts->address_table[temp_func_cmd].query_base <= max_addr)
						&& temp_func_query == 0)
						temp_func_query = temp_func_cmd;
				}

				if (temp_func_cmd != ts->num_function) {
					size = ts->address_table[temp_func_cmd].command_base -
						ts->address_table[j].control_base;
					printk(KERN_INFO "[TP] page%d has command function, function: %X\n"
						, i, ts->address_table[temp_func_cmd].function_type);
				} else {
					size = ts->address_table[temp_func_query].query_base -
						ts->address_table[j].control_base;
					printk(KERN_INFO "[TP] page%d has no command function, use query function, function: %X\n"
						, i, ts->address_table[temp_func_query].function_type);
				}

				ret = i2c_syn_read(ts->client, ts->address_table[j].control_base,
					&ts->config_table[length], size);
				if (ret < 0) {
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w", __func__);
					count += sprintf(buf, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
					return count;
				}

				length += size;
				printk(KERN_INFO "[TP] Size: %x, Length: %x\n", size, length);
				break;
			}
		}
	}
	if(length > SYN_CONFIG_SIZE)
		length = SYN_CONFIG_SIZE;

	printk(KERN_INFO "");
	for (i = 0; i < length; i++) {
		printk(KERN_INFO "%2.2X ", ts->config_table[i]);
		if ((i % 16) == 15)
			printk(KERN_INFO "\n");
	}

	for (i = 0; i < length; i++) {
		count += sprintf(buf + count, "%2.2X ", ts->config_table[i]);
		if ((i % 16) == (16 - 1))
			count += sprintf(buf + count, "\n");
	}
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t syn_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i, j, k = 0, length = 0;
	printk(KERN_INFO "[TP] ts->num_function: %d\n", ts->num_function);
	for (i = 0; i < SYN_MAX_PAGE; i++) {
		for (j = 0; j < ts->num_function; j++) {
			if (((ts->address_table[j].control_base >> 8) & 0xFF) == i) {
				for (k = j; k < ts->num_function; k++)
					if (ts->address_table[k].command_base != 0)
						break;
				length += ts->address_table[k].command_base -
					ts->address_table[j].control_base;
				printk(KERN_INFO "[%d]Length: %x\n", i, length);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(config, (S_IWUSR|S_IRUGO),
	syn_config_show, syn_config_store);


static ssize_t syn_layout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i;
	size_t count = 0;
	for (i = 0; i < 4; i++)
		count += sprintf(buf + count, "%d ", ts->layout[i]);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t syn_layout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};

	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				printk(KERN_INFO "[TP] buffer size is over 5 char\n");
				return count;
			}
			j = i + 1;
			if (k < 4) {
				ret = strict_strtol(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}
	if (k == 4) {
		memcpy(ts->layout, layout, sizeof(layout));
		printk(KERN_INFO "[TP] %d, %d, %d, %d\n",
			ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);
		input_unregister_device(ts->input_dev);
		synaptics_input_register(ts);
	} else
		printk(KERN_INFO "[TP] ERR@%d, %d, %d, %d\n",
			ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);
	return count;

}

static DEVICE_ATTR(layout, (S_IWUSR|S_IRUGO),
	syn_layout_show, syn_layout_store);


static ssize_t syn_pdt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i;
	size_t count = 0;
	for (i = 0; i < ts->num_function; i++) {
		count += sprintf(buf + count,
			"Funtion: %2X, Query: %3X, Command: %3X, "
			"Control: %3X, Data: %3X, INTR: %2X\n",
			ts->address_table[i].function_type, ts->address_table[i].query_base ,
			ts->address_table[i].command_base, ts->address_table[i].control_base,
			ts->address_table[i].data_base, ts->address_table[i].interrupt_source);
	}
	return count;
}

static DEVICE_ATTR(pdt, S_IRUGO, syn_pdt_show, NULL);

static ssize_t syn_htc_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->htc_event);
}

static ssize_t syn_htc_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->htc_event = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(htc_event, (S_IWUSR|S_IRUGO),
	syn_htc_event_show, syn_htc_event_store);

#ifdef SYN_WIRELESS_DEBUG
static ssize_t syn_int_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;

	count += sprintf(buf + count, "%d ", ts->irq_enabled);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t syn_int_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	int value, ret=0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
		ret = request_threaded_irq(ts->client->irq, NULL, synaptics_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, CONTROL_BASE) + 1, &ts->intr_bit, 1);
			printk(KERN_INFO "[TP] %s: interrupt enable: %x\n", __func__, ts->intr_bit);
			if (ret)
				free_irq(ts->client->irq, ts);
		}
	} else {
		disable_irq(ts->client->irq);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return count;
}

static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO),
	syn_int_status_show, syn_int_status_store);

static ssize_t syn_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] == '1' && ts->gpio_reset) {
		gpio_direction_output(ts->gpio_reset, 0);
		hr_msleep(1);
		gpio_direction_output(ts->gpio_reset, 1);
		printk(KERN_INFO "[TP] %s: synaptics touch chip reseted.\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(reset, (S_IWUSR),
	0, syn_reset);

#endif

enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};

static char *vk_name = "virtualkeys.sr_touchscreen";
static struct kobj_attribute vk_dev;

static int register_sr_touch_device(void)
{
	struct synaptics_ts_data *ts = gl_ts;
	struct synaptics_i2c_rmi_platform_data *pdata = ts->client->dev.platform_data;
	int ret = 0;

	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		printk(KERN_ERR "[TP][TOUCH_ERR]%s: Failed to allocate SR input device\n", __func__);
		return ALLOCATE_DEV_FAIL;
	}

	if (pdata->vk_obj) {
		memcpy(&vk_dev, pdata->vk2Use, sizeof(struct kobj_attribute));
		vk_dev.attr.name = vk_name;
		ret = sysfs_create_file(pdata->vk_obj, &(vk_dev.attr));
	}

	ts->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, ts->sr_input_dev->evbit);
	set_bit(EV_ABS, ts->sr_input_dev->evbit);
	set_bit(EV_KEY, ts->sr_input_dev->evbit);

	set_bit(KEY_BACK, ts->sr_input_dev->keybit);
	set_bit(KEY_HOME, ts->sr_input_dev->keybit);
	set_bit(KEY_MENU, ts->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, ts->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, ts->sr_input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->sr_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->sr_input_dev->propbit);
	ts->sr_input_dev->mtsize = ts->finger_support;
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
		0, ts->finger_support - 1, 0, 0);
	printk(KERN_INFO "[TP][SR]input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d\n", ts->layout[0],
		 ts->layout[1], ts->layout[2], ts->layout[3]);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X,
		ts->layout[0], ts->layout[1], 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y,
		ts->layout[2], ts->layout[3], 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE,
		0, 30, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR,
		0, 30, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		printk(KERN_ERR "[TP][SR][TOUCH_ERR]%s: Unable to register %s input device\n",
			__func__, ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t set_en_sr(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	if (buf[0]) {
		if (ts->sr_input_dev)
			printk(KERN_INFO "[TP]%s: SR device already exist!\n", __func__);
		else
			printk(KERN_INFO "[TP]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}

static DEVICE_ATTR(sr_en, S_IWUSR, 0, set_en_sr);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
static ssize_t synaptics_sweep2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	if (s2w_switch == s2w_temp )
		count += sprintf(buf, "%d\n", s2w_switch);
	else
		count += sprintf(buf, "%d->%d\n", s2w_switch, s2w_temp);

	return count;
}

static ssize_t synaptics_sweep2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (s2w_switch != buf[0] - '0') {
			s2w_temp = buf[0] - '0';
			if (scr_suspended == false)
				s2w_switch = s2w_temp;
			else 
//				h2w_switch = s2w_temp;
				s2w_switch_changed = true;
		}

	if (s2w_temp == 0) 
		printk(KERN_INFO "[SWEEP2WAKE]: Disabled.\n");
	else if (s2w_temp == 1)
		printk(KERN_INFO "[SWEEP2WAKE]: Enabled.\n");

	return count;
}

static DEVICE_ATTR(sweep2wake, (S_IWUSR|S_IRUGO),
	synaptics_sweep2wake_show, synaptics_sweep2wake_dump);


static ssize_t synaptics_home2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	if (h2w_switch == h2w_temp )
		count += sprintf(buf, "%d\n", h2w_switch);
	else
		count += sprintf(buf, "%d->%d\n", h2w_switch, h2w_temp);

	return count;
}

static ssize_t synaptics_home2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '3' && buf[1] == '\n')
		if (h2w_switch != buf[0] - '0') {
			h2w_temp = buf[0] - '0';
			if (scr_suspended == false)
				h2w_switch = h2w_temp;
			else 
//				h2w_switch = h2w_temp;
				h2w_switch_changed = true;
		}

	if (h2w_temp == 0) 
		printk(KERN_INFO "[HOME2WAKE]: Disabled.\n");
	else if (h2w_temp == 1)
		printk(KERN_INFO "[HOME2WAKE]: Enabled.\n");
	else if (h2w_temp == 2)
		printk(KERN_INFO "[HOME2WAKE]: Enabled with LOGO2SLEEP!\n");
	else if (h2w_temp == 3)
		printk(KERN_INFO "[HOME2WAKE]: Enabled with LOGO2WAKE & LOGO2SLEEP!\n");

	return count;
}

static DEVICE_ATTR(home2wake, (S_IWUSR|S_IRUGO),
	synaptics_home2wake_show, synaptics_home2wake_dump);


static ssize_t synaptics_pocket_detect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", pwp_switch);

	return count;
}

static ssize_t synaptics_pocket_detect_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (pwp_switch != buf[0] - '0') {
			pwp_switch = buf[0] - '0';
		}

	if (pwp_switch == 0) 
		printk(KERN_INFO "[POCKETWAKEPROT]: Disabled.\n");
	else if (pwp_switch == 1)
		printk(KERN_INFO "[POCKETWAKEPROT]: Enabled.\n");

	return count;
}

static DEVICE_ATTR(pocket_detect, (S_IWUSR|S_IRUGO),
	synaptics_pocket_detect_show, synaptics_pocket_detect_dump);


static ssize_t synaptics_logo2menu_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	if (l2m_switch == l2m_temp )
		count += sprintf(buf, "%d\n", l2m_switch);
	else
		count += sprintf(buf, "%d->%d\n", l2m_switch, l2m_temp);

	return count;
}

static ssize_t synaptics_logo2menu_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (l2m_switch != buf[0] - '0') {
			l2m_temp = buf[0] - '0';
			if (scr_suspended == false)
				l2m_switch = l2m_temp;
			else 
				l2m_switch = l2m_temp;
		}

	if (l2m_temp == 0) 
		printk(KERN_INFO "[LOGO2MENU]: Disabled.\n");
	else if (l2m_temp == 1)
		printk(KERN_INFO "[LOGO2MENU]: Enabled.\n");

	return count;
}

static DEVICE_ATTR(logo2menu, (S_IWUSR|S_IRUGO),
	synaptics_logo2menu_show, synaptics_logo2menu_dump);


static ssize_t synaptics_logo_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", logo_delay_switch);

	return count;
}

static ssize_t synaptics_logo_delay_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (logo_delay_switch != buf[0] - '0') {
			logo_delay_switch = buf[0] - '0';
		}

	if (logo_delay_switch == 0) 
		printk(KERN_INFO "[LOGO_DELAY]: Disabled.\n");
	else if (logo_delay_switch == 1)
		printk(KERN_INFO "[LOGO_DELAY]: Enabled.\n");

	return count;
}

static DEVICE_ATTR(logo_delay, (S_IWUSR|S_IRUGO),
	synaptics_logo_delay_show, synaptics_logo_delay_dump);


static ssize_t synaptics_l2m_2_phase_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", l2m_2_phase);

	return count;
}

static ssize_t synaptics_l2m_2_phase_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (l2m_2_phase != buf[0] - '0') {
			l2m_2_phase = buf[0] - '0';
		}

	if (l2m_2_phase == 0) 
		printk(KERN_INFO "[L2M]: 2 Phase button Disabled.\n");
	else if (l2m_2_phase == 1)
		printk(KERN_INFO "[L2M]: 2 Phase button Enabled.\n");

	return count;
}

static DEVICE_ATTR(l2m_2_phase, (S_IWUSR|S_IRUGO),
	synaptics_l2m_2_phase_show, synaptics_l2m_2_phase_dump);

static ssize_t synaptics_sleep_wake_vibration_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", sleep_wake_vibration_time);

	return count;
}

static ssize_t synaptics_sleep_wake_vibration_time_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		if (sleep_wake_vibration_time != buf[0] - '0') {
			sleep_wake_vibration_time = buf[0] - '0';
		}

	printk(KERN_INFO "[sleep_wake_vibration_time]: %d.\n", sleep_wake_vibration_time);


	return count;
}

static DEVICE_ATTR(sleep_wake_vibration_time, (S_IWUSR|S_IRUGO),
	synaptics_sleep_wake_vibration_time_show, synaptics_sleep_wake_vibration_time_dump);

#endif

static struct kobject *android_touch_kobj;

static int synaptics_touch_sysfs_init(void)
{
	int ret;
#ifdef SYN_WIRELESS_DEBUG
	struct synaptics_ts_data *ts = gl_ts;
#endif

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[TP] TOUCH_ERR: %s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	syn_reg_addr = 0;
	if (sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_register.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_unlock.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_config.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_layout.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_pdt.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_htc_event.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr)
#ifdef SYN_WIRELESS_DEBUG
		|| sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr)
#endif
		)
		return -ENOMEM;
	if (get_address_base(gl_ts, 0x54, FUNCTION))
		if (sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr) ||
			sysfs_create_file(android_touch_kobj, &dev_attr_disable_cbc.attr))
			return -ENOMEM;

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_home2wake.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_logo2menu.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_logo_delay.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_l2m_2_phase.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_sleep_wake_vibration_time.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_pocket_detect.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

#endif
#ifdef SYN_WIRELESS_DEBUG
	ret= gpio_request(ts->gpio_irq, "synaptics_attn");
	if (ret) {
		printk(KERN_INFO "[TP]%s: Failed to obtain touchpad IRQ %d. Code: %d.", __func__, ts->gpio_irq, ret);
		return ret;
	}
	if (ts->gpio_reset) {
		ret = gpio_request(ts->gpio_reset, "synaptics_reset");
		if (ret)
			printk(KERN_INFO "[TP]%s: Failed to obtain reset pin: %d. Code: %d.", __func__, ts->gpio_reset, ret);
	}
	ret = gpio_export(ts->gpio_irq, true);
	if (ret) {
		printk(KERN_INFO "[TP]%s: Failed to "
			"export ATTN gpio!\n", __func__);
		ret = 0;
	} else {
		ret = gpio_export_link(&(ts->input_dev->dev), "attn",
			ts->gpio_irq);
		if (ret) {
			printk(KERN_INFO "[TP]%s: Failed to "
				"symlink ATTN gpio!\n", __func__);
			ret = 0;
		} else {
			printk(KERN_INFO "[TP]%s: Exported GPIO %d.", __func__, ts->gpio_irq);
		}
	}
#endif
	return 0;
}

static void synaptics_touch_sysfs_remove(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (get_address_base(gl_ts, 0x54, FUNCTION))
		sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_unlock.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_config.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_layout.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_pdt.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_htc_event.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	sysfs_remove_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_home2wake.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_logo2menu.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_logo_delay.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_l2m_2_phase.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_sleep_wake_vibration_time.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_pocket_detect.attr);
#endif
#ifdef SYN_WIRELESS_DEBUG
	sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
#endif
	kobject_del(android_touch_kobj);
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret = 0;

	
	ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x01, CONTROL_BASE), 0x80);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
	if (ts->pre_finger_data[0][0] < 2) {
		if (ts->large_obj_check) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x26, ts->default_large_obj & 0x7F);
				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x29, ts->default_large_obj & 0x7F);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
				printk(KERN_INFO "[TP] %s: set large obj suppression register to: %x\n", __func__, ts->default_large_obj & 0x7F);
			}
		}

		if (ts->segmentation_bef_unlock) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x25, ts->segmentation_bef_unlock);
				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x22, ts->segmentation_bef_unlock);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
				printk(KERN_INFO "[TP] %s: set segmentation aggressiveness to: %x\n", __func__, ts->segmentation_bef_unlock);
			}
		}

		if (ts->threshold_bef_unlock) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0A, ts->threshold_bef_unlock);
				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0C, ts->threshold_bef_unlock);
				}
			} else {
				ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x01, ts->threshold_bef_unlock);
			}
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
			printk(KERN_INFO "[TP] %s: set Z Touch threshold to: %x\n", __func__, ts->threshold_bef_unlock);
		}

		if (ts->saturation_bef_unlock) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_bef_unlock & 0xFF);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_bef_unlock & 0xFF00) >> 8);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
			printk(KERN_INFO "[TP] %s: set saturation to: %x\n", __func__, ts->saturation_bef_unlock);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);
		}

		if ( ts->PixelTouchThreshold_bef_unlock ) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_bef_unlock );
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
			printk(KERN_INFO "[TP] %s: set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", __func__, ts->PixelTouchThreshold_bef_unlock);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
		}
	}

#ifdef SYN_CALIBRATION_CONTROL
	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->pre_finger_data[0][0] >= 2 || ts->mfg_flag == 1) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, 0x0);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);

			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:7", __func__);

			printk(KERN_INFO "[TP] %s: Touch init: set fast relaxation to 0x0\n", __func__);
		}
	}
#endif

	return ret;
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
static int last_touch_position_x = 0;
static int last_touch_position_y = 0;

static int logo_press_state = 0;
static unsigned long logo_last_pressed_time;

static int report_htc_logo_area(int x, int y)
{
    if (s2w_switch > 0 && scr_suspended == true) return 0; // s2w should wake on logo

    if (h2w_switch < 2 && l2m_switch == 0) return 0; // logo2sleep and logo2menu is both off, so don't report logo area!

    if (last_touch_position_x>600 && last_touch_position_x<1200)
    {
		int below_y = 2835;
		if (scr_suspended == true)
		{
			// if screen is suspended we can use a bigger area for Logo tapping (easier wake)
			below_y = 2750;
		}

		if (last_touch_position_y > below_y)
		{
			if (logo_press_state == 0)
			{
				logo_press_state = 1;
				logo_last_pressed_time = jiffies;
				return 1;
			}
			printk("[L2W]\n");
			if (logo_press_state == 1)
			{
				if (jiffies - logo_last_pressed_time > BETWEEN_LONG_PRESS_MIN_DIFF)
				{
					printk("[L2W] - LONG PRESS ON LOGO POSSIBLE\n");
					return 3;
				} else
				if (jiffies - logo_last_pressed_time > BETWEEN_SLEEP_OR_WAKE_PRESS_MIN_DIFF)
				{
					printk("[L2W] - SLEEP OR WAKE PRESS ON LOGO POSSIBLE\n");
					return 2;
				}
			}
			return 1;
		}
    }
    logo_press_state = 0;
    break_longtap_count = 1; // not in area, stop counting
    return 0;
}

static unsigned int s2w_back_swept_time = 0;
static unsigned int s2w_home_swept_time = 0;
static unsigned int MIN_TIME_DIFF_FOR_SWEEP2WAKE = 5;
static unsigned int MAX_TIME_DIFF_FOR_SWEEP2WAKE = 60;

static void synaptics_ts_s2w_check_wake(void)
{
		if (scr_suspended == true)
		{
				printk("[S2W] checking s2w wake condition: %u - %u\n", s2w_back_swept_time, s2w_home_swept_time);
				if (s2w_back_swept_time && s2w_home_swept_time && s2w_home_swept_time > s2w_back_swept_time)
				{
					if (s2w_home_swept_time - s2w_back_swept_time > MIN_TIME_DIFF_FOR_SWEEP2WAKE
						&& jiffies - s2w_back_swept_time < MAX_TIME_DIFF_FOR_SWEEP2WAKE)
					{
						printk("[S2W] sweep2wake_pwrtrigger call\n");
						sweep2wake_pwrtrigger();
						s2w_home_swept_time = 0;
						s2w_back_swept_time = 0;
					}
				}
				if (s2w_back_swept_time && s2w_home_swept_time && s2w_home_swept_time < s2w_back_swept_time)
				{
					if (s2w_back_swept_time - s2w_home_swept_time > MIN_TIME_DIFF_FOR_SWEEP2WAKE
						&& jiffies - s2w_home_swept_time < MAX_TIME_DIFF_FOR_SWEEP2WAKE)
					{
						printk("[S2W] sweep2wake_pwrtrigger call\n");
						sweep2wake_pwrtrigger();
						s2w_home_swept_time = 0;
						s2w_back_swept_time = 0;
					}
				}
		} else
		{
						s2w_home_swept_time = 0;
						s2w_back_swept_time = 0;
		}
}

#endif
static void synaptics_ts_finger_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t buf[ts->finger_support * 8 ], noise_index[10], noise_state = 0;
	static int x_pos[10] = {0}, y_pos[10] = {0};
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	int report_ret = 0;
	int ts_is_on = 0;
	ts_is_on = touchscreen_is_on() ? 1:0;
#endif

	memset(buf, 0x0, sizeof(buf));
	memset(noise_index, 0x0, sizeof(noise_index));
	if (ts->package_id < 3400)
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE) + 2, buf, sizeof(buf));
	else {
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, ts->finger_func_idx, DATA_BASE), buf, sizeof(buf));
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x54, DATA_BASE) + 8, &noise_state, 1);
		if (noise_state == 2) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, DATA_BASE) + 4, noise_index, sizeof(noise_index));
			ts->debug_log_level |= BIT(17);
		} else {
			if (!ts->enable_noise_log)
				ts->debug_log_level &= ~BIT(17);
		}
	}
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	} else {
		int finger_data[ts->finger_support][4];
		int base = 0;
		uint8_t i, j;
		uint16_t finger_press_changed = 0, finger_release_changed = 0, finger_pressed = 0;

		if (ts->package_id < 3400)
			base = (ts->finger_support + 3) / 4;
		ts->finger_count = 0;
		if (ts->debug_log_level & BIT(0)) {
			printk(KERN_INFO "[TP] Touch:");
			for (i = 0; i < sizeof(buf); i++)
				printk(KERN_INFO " %2x", buf[i]);
			printk(KERN_INFO "\n");
		}
		for (i = 0; i < ts->finger_support; i++) {
			uint8_t finger_state;
			if (ts->package_id < 3400)
				finger_state = buf[(i / 4)] >> ((i * 2) % 8);
			else
				finger_state = buf[i * 8];

			if (finger_state & 0x03) {
				finger_pressed |= BIT(i);
				ts->finger_count++;
				if (finger_state == 0x02)
					printk(KERN_INFO "[TP] Finger state[%d] = 0x02\n", i);
				else if (finger_state == 0x03)
					printk(KERN_INFO "[TP] Finger state[%d] = 0x03\n", i);
			}

#ifdef SYN_FILTER_CONTROL
			else if ((ts->grip_suppression | ts->grip_b_suppression) & BIT(i)) {
				ts->grip_suppression &= ~BIT(i);
				ts->grip_b_suppression &= ~BIT(i);
			}
#endif
		}
		if (ts->finger_pressed != finger_pressed
			) {
			finger_press_changed = ts->finger_pressed ^ finger_pressed;
			finger_release_changed = finger_press_changed & ts->finger_pressed;
			finger_press_changed &= finger_pressed;
			ts->finger_pressed = finger_pressed;
		}

		if(ts->debug_log_level & BIT(3)) {
			for(i = 0; i < ts->finger_support; i++) {
				if (finger_release_changed & BIT(i) ) {
					if (ts->package_id < 3400) {
						uint32_t flip_flag = SYNAPTICS_FLIP_X;
						uint8_t pos_mask = 0x0f;
						for (j = 0; j < 2; j++) {
							finger_data[i][j]
								= (buf[base+2] & pos_mask) >> (j * 4) |
								(uint16_t)buf[base + j] << 4;
							if (ts->flags & flip_flag)
								finger_data[i][j] = ts->max[j] - finger_data[i][j];
							flip_flag <<= 1;
							pos_mask <<= 4;
						}
						finger_data[i][2] = (buf[base+3] >> 4 & 0x0F) + (buf[base+3] & 0x0F);
						finger_data[i][3] = buf[base+4];
					} else {
						finger_data[i][0] = (buf[base+2] << 8) + buf[base+1];
						finger_data[i][1] = (buf[base+4] << 8) + buf[base+3];
						finger_data[i][2] = buf[base+6] + buf[base+7];
						finger_data[i][3] = buf[base+5];
					}

					if (ts->flags & SYNAPTICS_SWAP_XY)
						swap(finger_data[i][0], finger_data[i][1]);

					if (ts->layout[1] < finger_data[i][0])
						finger_data[i][0] = ts->layout[1];
					if(ts->width_factor && ts->height_factor){
						printk(KERN_INFO
							"[TP] Screen:F[%02d]:Up, X=%d, Y=%d, W=%d, Z=%d\n",
							i+1, (x_pos[i]*ts->width_factor)>>SHIFT_BITS,
							(y_pos[i]*ts->height_factor)>>SHIFT_BITS,
							finger_data[i][2], finger_data[i][3]);
					} else {
						printk(KERN_INFO
							"[TP] Raw:F[%02d]:Up, X=%d, Y=%d, W=%d, Z=%d\n",
							i+1, x_pos[i], y_pos[i],
							finger_data[i][2], finger_data[i][3]);
					}
				}
				base += 5;
			}
		}

		if (ts->htc_event == SYN_AND_REPORT_TYPE_B && finger_release_changed) {
			for (i = 0; i < ts->finger_support; i++) {
				if (finger_release_changed & BIT(i)) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
					ts->tap_suppression &= ~BIT(i);
				}
			}
		}

		if (finger_pressed == 0 
) {
			if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
				
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
				}
				input_mt_sync(ts->input_dev);
			} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
				}
			}
			else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}

#ifdef SYN_FILTER_CONTROL
			if (ts->filter_level[0])
				ts->ambiguous_state = 0;
			ts->grip_b_suppression = 0;
#endif
			if (ts->reduce_report_level[0])
				ts->tap_suppression = 0;
			if (ts->debug_log_level & BIT(1))
				printk(KERN_INFO "[TP] Finger leave\n");
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
			/* if finger released, reset count & barriers */
			if ((((ts->finger_count > 0)?1:0) == 0)) { 
				exec_count = true;
				led_exec_count = true;
				barrier[0] = false;
				barrier[1] = false;
				scr_on_touch = false;
				// released (supposedly before long tap happened), break long tap count work...
				break_longtap_count = 1;
				// released, should allow logo long tap counting again...
				allow_longtap_count = 1;
				printk(KERN_INFO "[home2wake/S2W]: Finger released, reseting vars. Last post %d %d\n",last_touch_position_x, last_touch_position_y);
				{
				    if (s2w_switch > 0 && scr_suspended == true)
				    {
					if (last_touch_position_y > 2750)
					{
						synaptics_ts_s2w_check_wake();
						s2w_back_swept_time = 0;
						s2w_home_swept_time = 0;
					}
				    } else
				    {
				    report_ret = report_htc_logo_area(last_touch_position_x,last_touch_position_x);
				    // reseting logo_press_state, so long press count in report_htc_logo_area start again 
				    logo_press_state = 0;
				    if (report_ret)
				    {
						// printk(KERN_INFO "[sweep2wake]: POWER ON/OFF.\n");
						if (scr_suspended == false)
						{
							if (l2m_switch > 0) // logo2menu active
							{
								if (report_ret == 3)
								{
									// long press logo, power off
									// OFF
									//sweep2wake_pwrtrigger(); // commented - long tap time count worker used instead
								} else
								{
									if ((l2m_2_phase == 1 || (h2w_switch == 0 && s2w_switch == 0)) && menu_pressed == 1) // two phase menu input sync (or no longtap menu sleep == no wake set), and was pressed, trigger menu 0
									{
										// MENU
										sweep2wake_menutrigger();
									} else
									if (l2m_2_phase == 0) // no two phase menu input sync, just trigger menu
									{
										// MENU
										sweep2wake_menutrigger();
									}
								}
							} else
							{
								// no logo2menu branch...
								if (h2w_switch >= 2)
								{ //  logo to wake (h2w_switch 3) or at least logo to sleep (h2w_switch 2) enabled. screen is on, switch it off
									if (report_ret >= 2 && logo_delay_switch == 0) // 2 or 3 = enough time passed for non-accidental tap and logo delay is off
									{
										// OFF
										sweep2wake_pwrtrigger();
									}
								}
							}
						} else
						{
							if (h2w_switch == 3)
							{ // logo to wake enabled. screen is off, switch it on
								if (report_ret >= 2 && logo_delay_switch == 0) // 2 or 3 = enough time passed for non-accidental tap, power on
								{
									// ON
									sweep2wake_pwrtrigger();
								}
							}
						}
				    }
				    }
				}
			}
#endif
		}

		if (ts->pre_finger_data[0][0] < 2 || finger_pressed) {
			if (ts->package_id < 3400)
				base = (ts->finger_support + 3) / 4;
			else
				base = 0;
			for (i = 0; i < ts->finger_support; i++) {
				uint32_t flip_flag = SYNAPTICS_FLIP_X;
				if ((finger_pressed | finger_release_changed) & BIT(i)) {
					if (ts->package_id < 3400) {
						uint8_t pos_mask = 0x0f;
						for (j = 0; j < 2; j++) {
							finger_data[i][j]
								= (buf[base+2] & pos_mask) >> (j * 4) |
								(uint16_t)buf[base + j] << 4;
							if (ts->flags & flip_flag)
								finger_data[i][j] = ts->max[j] - finger_data[i][j];
							flip_flag <<= 1;
							pos_mask <<= 4;
						}
						finger_data[i][2] = (buf[base+3] >> 4 & 0x0F) + (buf[base+3] & 0x0F);
						finger_data[i][3] = buf[base+4];
					} else {
						finger_data[i][0] = (buf[base+2] << 8) + buf[base+1];
						finger_data[i][1] = (buf[base+4] << 8) + buf[base+3];
						finger_data[i][2] = buf[base+6] + buf[base+7];
						finger_data[i][3] = buf[base+5];
					}

					if (ts->flags & SYNAPTICS_SWAP_XY)
						swap(finger_data[i][0], finger_data[i][1]);

					if (ts->layout[1] < finger_data[i][0])
						finger_data[i][0] = ts->layout[1];

					if ((finger_release_changed & BIT(i)) && ts->pre_finger_data[0][0] < 2) {
						if (!ts->first_pressed) {
							if (ts->finger_count == 0)
								ts->first_pressed = 1;
							printk(KERN_INFO "[TP] E%d@%d, %d\n", i + 1,
							x_pos[i], y_pos[i]);
						}
					}
#ifdef SYN_FILTER_CONTROL
					if (abs((buf[base+3] >> 4 & 0x0F) - (buf[base+3] & 0x0F)) >= 10)
						ts->grip_b_suppression |= BIT(i);

					if (ts->filter_level[0] &&
						((finger_press_changed | ts->grip_suppression) & BIT(i))) {
						if ((finger_data[i][0] < (ts->filter_level[0] + ts->ambiguous_state * 20) ||
							finger_data[i][0] > (ts->filter_level[3] - ts->ambiguous_state * 20)) &&
							!(ts->grip_suppression & BIT(i))) {
							ts->grip_suppression |= BIT(i);
						} else if ((finger_data[i][0] < (ts->filter_level[1] + ts->ambiguous_state * 20) ||
							finger_data[i][0] > (ts->filter_level[2] - ts->ambiguous_state * 20)) &&
							(ts->grip_suppression & BIT(i)))
							ts->grip_suppression |= BIT(i);
						else if (finger_data[i][0] > (ts->filter_level[1] + ts->ambiguous_state * 20) &&
							finger_data[i][0] < (ts->filter_level[2] - ts->ambiguous_state * 20)) {
							ts->grip_suppression &= ~BIT(i);
						}
					}
					if ((ts->grip_suppression | ts->grip_b_suppression) & BIT(i)) {
						finger_pressed &= ~BIT(i);
					} else
#endif

					if (ts->htc_event == SYN_AND_REPORT_TYPE_B && ts->reduce_report_level[0]) {
						if (ts->tap_suppression & BIT(i) && finger_pressed & BIT(i)) {
							int dx, dy = 0;
							dx = abs(ts->pre_finger_data[i + 1][2] - finger_data[i][0]);
							dy = abs(ts->pre_finger_data[i + 1][3] - finger_data[i][1]);
							if (dx > ts->reduce_report_level[TAP_DX_OUTER] || dy > ts->reduce_report_level[TAP_DY_OUTER]) {
								ts->tap_suppression &= ~BIT(i);
							} else if (ts->reduce_report_level[TAP_TIMEOUT] && time_after(jiffies, ts->tap_timeout[i]) && (dx > ts->reduce_report_level[TAP_DX_INTER] || dy > ts->reduce_report_level[TAP_DY_INTER])) {
								ts->tap_suppression &= ~BIT(i);
							} else {
								finger_pressed &= ~BIT(i);
								if (ts->debug_log_level & BIT(1))
									printk(KERN_INFO
										"[TP] Filtered Finger %d=> X:%d, Y:%d w:%d, z:%d\n",
										i + 1, finger_data[i][0], finger_data[i][1],
										finger_data[i][2], finger_data[i][3]);
							}
						}
					}

					if ((finger_pressed & BIT(i)) == BIT(i)) {

						if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							last_touch_position_x = finger_data[i][0];
							last_touch_position_y = finger_data[i][1];
							report_ret = report_htc_logo_area(last_touch_position_x,last_touch_position_y);
							if (!report_ret)
							{
							if (ts_is_on == 0) {
								finger_data[i][0] = -10;
								finger_data[i][1] = -10;
							}
#endif
							if (ts->support_htc_event) {
								input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
									finger_data[i][3] << 16 | finger_data[i][2]);
								input_report_abs(ts->input_dev, ABS_MT_POSITION,
									(finger_pressed == 0) << 31 |
									finger_data[i][0] << 16 | finger_data[i][1]);
							}
							input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
								finger_data[i][3]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
								finger_data[i][0]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
								finger_data[i][1]);
							input_mt_sync(ts->input_dev);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							} else // !report_ret
							{
								printk("[L2M] till in area %d %d",last_touch_position_x,last_touch_position_y);
								if (scr_suspended == false)
								{
									if (h2w_switch >= 2)
									{ // logo to sleep or logo to wake enabled. screen is on, switch it off
										if (l2m_switch == 0 && logo_delay_switch == 0 && report_ret == 3) // 3 = enough time passed for long tap, or logo delay is off
										{
											// OFF 
											//sweep2wake_pwrtrigger(); // - don't sleep here, on short tap, only sleep when user finger left touchscreen
										} else
										if (l2m_2_phase == 1 || logo_delay_switch == 1)
										{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										}
									} else
									// logo2menu enabled and some wake option too - longtap count for power off can start
									if (l2m_switch > 0 && (s2w_switch > 0 || h2w_switch > 0) && logo_delay_switch == 1)
									{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											sweep2wake_longtap_count_trigger();
									} else
									{
										if (l2m_switch > 0 && (l2m_2_phase == 1 || (s2w_switch == 0 && h2w_switch == 0)) )
										{
											menu_pressed = 0;
											// MENU event -> 1
											sweep2wake_menutrigger();
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										} else
										if (l2m_switch > 0)
										{
											sweep2wake_longtap_count_trigger();
										}
									}
								} else
								{
									if (s2w_switch == 0 && h2w_switch == 3 && logo_delay_switch == 1)
									{ // logo to wake enabled with logo Delay. screen is off, start counting
										sweep2wake_longtap_count_trigger();
									}
								}
							}
#endif
						} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							last_touch_position_x = finger_data[i][0];
							last_touch_position_y = finger_data[i][1];
							report_ret = report_htc_logo_area(last_touch_position_x,last_touch_position_y);
							if (!report_ret)
							{
							if (ts_is_on == 0) {
								finger_data[i][0] = -10;
								finger_data[i][1] = -10;
							}
#endif
							if (ts->support_htc_event) {
								input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
									finger_data[i][3] << 16 | finger_data[i][2]);
								input_report_abs(ts->input_dev, ABS_MT_POSITION,
									(finger_pressed == 0) << 31 |
									finger_data[i][0] << 16 | finger_data[i][1]);
							}
							input_mt_slot(ts->input_dev, i);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
							1);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
								finger_data[i][3]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
								finger_data[i][0]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
								finger_data[i][1]);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							} else
							{
								printk("[L2M] till in area %d %d",last_touch_position_x,last_touch_position_y);
								if (scr_suspended == false)
								{
									if (h2w_switch >= 2)
									{ // logo to sleep or logo to wake enabled. screen is on, switch it off
										if (l2m_switch == 0 && logo_delay_switch == 0 && report_ret == 3) // 3 = enough time passed for long tap, or logo delay is off
										{
											// OFF 
											//sweep2wake_pwrtrigger(); // - don't sleep here, on short tap, only sleep when user finger left touchscreen
										} else
										if (l2m_2_phase == 1 || logo_delay_switch == 1)
										{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										}
									} else
									// logo2menu enabled and some wake option too - longtap count for power off can start
									if (l2m_switch > 0 && (s2w_switch > 0 || h2w_switch > 0) && logo_delay_switch == 1)
									{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											sweep2wake_longtap_count_trigger();
									} else
									{
										if (l2m_switch > 0 && (l2m_2_phase == 1 || (s2w_switch == 0 && h2w_switch == 0)) )
										{
											menu_pressed = 0;
											// MENU event -> 1
											sweep2wake_menutrigger();
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										} else
										if (l2m_switch > 0)
										{
											sweep2wake_longtap_count_trigger();
										}
									}
								} else
								{
									if (s2w_switch == 0 && h2w_switch == 3 && logo_delay_switch == 1)
									{ // logo to wake enabled with logo Delay. screen is off, start counting
										sweep2wake_longtap_count_trigger();
									}
								}
							}
#endif
						} else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							last_touch_position_x = finger_data[i][0];
							last_touch_position_y = finger_data[i][1];
							report_ret = report_htc_logo_area(last_touch_position_x,last_touch_position_y);
							if (!report_ret)
							{
							if (ts_is_on == 0) {
								finger_data[i][0] = -10;
								finger_data[i][1] = -10;
							}
#endif
							input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
							input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
								finger_data[i][3] << 16 | finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION,
								(finger_pressed == 0) << 31 |
								finger_data[i][0] << 16 | finger_data[i][1]);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
							} else
							{
								printk("[L2M] till in area %d %d",last_touch_position_x,last_touch_position_y);
								if (scr_suspended == false)
								{
									if (h2w_switch >= 2)
									{ // logo to sleep or logo to wake enabled. screen is on, switch it off
										if (l2m_switch == 0 && logo_delay_switch == 0 && report_ret == 3) // 3 = enough time passed for long tap, or logo delay is off
										{
											// OFF 
											//sweep2wake_pwrtrigger(); // - don't sleep here, on short tap, only sleep when user finger left touchscreen
										} else
										if (l2m_2_phase == 1 || logo_delay_switch == 1)
										{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										}
									} else
									// logo2menu enabled and some wake option too - longtap count for power off can start
									if (l2m_switch > 0 && (s2w_switch > 0 || h2w_switch > 0) && logo_delay_switch == 1)
									{
											if (l2m_2_phase == 1)
											{
												menu_pressed = 0;
												// MENU event -> 1
												sweep2wake_menutrigger();
											}
											sweep2wake_longtap_count_trigger();
									} else
									{
										if (l2m_switch > 0 && (l2m_2_phase == 1 || (s2w_switch == 0 && h2w_switch == 0)) )
										{
											menu_pressed = 0;
											// MENU event -> 1
											sweep2wake_menutrigger();
											// long tap needed, start counting
											sweep2wake_longtap_count_trigger();
										} else
										if (l2m_switch > 0)
										{
											sweep2wake_longtap_count_trigger();
										}
									}
								} else
								{
									if (s2w_switch == 0 && h2w_switch == 3 && logo_delay_switch == 1)
									{ // logo to wake enabled with logo Delay. screen is off, start counting
										sweep2wake_longtap_count_trigger();
									}
								}
							}
#endif
						}
						x_pos[i] = finger_data[i][0];
						y_pos[i] = finger_data[i][1];
						finger_pressed &= ~BIT(i);

						if ((finger_press_changed & BIT(i)) && ts->debug_log_level & BIT(3)) {
							if(ts->width_factor && ts->height_factor){
								printk(KERN_INFO
									"[TP] Screen:F[%02d]:Down, X=%d, Y=%d, W=%d, Z=%d\n",
									i+1, (finger_data[i][0]*ts->width_factor)>>SHIFT_BITS,
									(finger_data[i][1]*ts->height_factor)>>SHIFT_BITS,
									finger_data[i][2], finger_data[i][3]);
							} else {
								printk(KERN_INFO
									"[TP] Raw:F[%02d]:Down, X=%d, Y=%d, W=%d, Z=%d\n",
									i+1, finger_data[i][0], finger_data[i][1],
									finger_data[i][2], finger_data[i][3]);
							}
						}

						if (ts->pre_finger_data[0][0] < 2) {
							if (finger_press_changed & BIT(i)) {
								ts->pre_finger_data[i + 1][0] = finger_data[i][0];
								ts->pre_finger_data[i + 1][1] = finger_data[i][1];

								if (!ts->first_pressed)
									printk(KERN_INFO "[TP] S%d@%d, %d\n", i + 1,
										finger_data[i][0], finger_data[i][1]);
								if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
#ifdef SYN_CALIBRATION_CONTROL
									if (ts->multitouch_calibration) {
										if (ts->finger_count == ts->finger_support) {
											if (ts->package_id < 3400) {
												ret = i2c_syn_write_byte_data(ts->client,
													get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
												if (ret < 0)
													i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_1", __func__);
												printk(KERN_INFO "[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
											}
										} else if (!ts->pre_finger_data[0][0] && ts->finger_count > 1)
											ts->pre_finger_data[0][0] = 1;
									}
#endif
								}
							}
						}

						if (ts->htc_event == SYN_AND_REPORT_TYPE_B && ts->reduce_report_level[TAP_DX_OUTER]) {
							if (finger_press_changed & BIT(i)) {
								ts->tap_suppression &= ~BIT(i);
								ts->tap_suppression |= BIT(i);
								ts->pre_finger_data[i + 1][2] = finger_data[i][0];
								ts->pre_finger_data[i + 1][3] = finger_data[i][1];
								if (ts->reduce_report_level[TAP_TIMEOUT] && (ts->tap_suppression))
									ts->tap_timeout[i] = jiffies + msecs_to_jiffies(ts->reduce_report_level[TAP_TIMEOUT]);
							}
						}

						if (ts->debug_log_level & BIT(1))
							printk(KERN_INFO
								"[TP] Finger %d=> X:%d, Y:%d W:%d, Z:%d\n",
								i + 1, finger_data[i][0], finger_data[i][1],
								finger_data[i][2], finger_data[i][3]);
						if (ts->debug_log_level & BIT(17))
							printk(KERN_INFO
								"[TP] Finger %d=> X:%d, Y:%d W:%d, Z:%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
								i + 1, finger_data[i][0], finger_data[i][1], finger_data[i][2],
								finger_data[i][3], noise_index[1] | noise_index[0],
								noise_index[6] | noise_index[5], noise_index[9], noise_index[4]);
					}
					if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
#ifdef SYN_CALIBRATION_CONTROL
						if (ts->multitouch_calibration) {
							if ((finger_release_changed & BIT(i)) && ts->pre_finger_data[0][0] == 1) {
								if (ts->package_id < 3400) {
									ret = i2c_syn_write_byte_data(ts->client,
										get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
									if (ret < 0)
										i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_2", __func__);
									printk(KERN_INFO "[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
								}
							}
						}
#endif
					}
					if (!ts->finger_count)
						ts->pre_finger_data[0][0] = 0;
				}
				if (ts->package_id < 3400)
					base += 5;
				else
					base += 8;
			}
#ifdef SYN_FILTER_CONTROL
			if (ts->filter_level[0] && ts->grip_suppression) {
				ts->ambiguous_state = 0;
				for (i = 0; i < ts->finger_support; i++)
					if (ts->grip_suppression & BIT(i))
						ts->ambiguous_state++;
			}
			if (ts->debug_log_level & BIT(16))
				printk(KERN_INFO "[TP] ts->grip_suppression: %x, ts->ambiguous_state: %x\n",
					ts->grip_suppression, ts->ambiguous_state);
#endif
		}
	}
	input_sync(ts->input_dev);

}

static void synaptics_ts_report_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data[2] = {0};

	ret = i2c_syn_write(ts->client,
		get_address_base(ts, 0x54, DATA_BASE) + 1, &data[0], 2);

	if (ret < 0)
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
	else {
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x54, DATA_BASE) + 3, ts->temp_report_data,
			ts->x_channel * ts->y_channel * 2);
		if (ret >= 0)
			memcpy(&ts->report_data[0], &ts->temp_report_data[0], ts->x_channel * ts->y_channel * 2);
		else {
			memset(&ts->report_data[0], 0x0, sizeof(ts->report_data));
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		}
	}
	atomic_set(&ts->data_ready, 1);
	wake_up(&syn_data_ready_wq);

}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
static int home_key_pressed = 0;
static int back_key_pressed = 0;
static unsigned int home_key_first_touch_time = 0;
static unsigned int MIN_HOME_KEY_TOUCH_TIME_TO_WAKE = 22;



static void synaptics_ts_s2w_button_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data = 0;

	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x1A, DATA_BASE), &data, 1);
	if (data) {
		if (data & 0x01) {
			printk("[S2W] back key swept\n");
			s2w_back_swept_time = jiffies;
		}
		else if (data & 0x02) {
			printk("[S2W] home key swept\n");
			s2w_home_swept_time = jiffies;
		}
	} else
	{
		printk("[S2W] released\n");
	}
}

#endif
static void synaptics_ts_button_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data = 0;
	uint16_t x_position = 0, y_position = 0;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	int ts_is_on = touchscreen_is_on() ? 1:0;
#endif

	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x1A, DATA_BASE), &data, 1);
	if (data) {
		if (data & 0x01) {
			printk("[TP] back key pressed\n");
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
			home_key_pressed = 0;
			back_key_pressed = 1;
			s2w_back_swept_time = jiffies;
			last_touch_position_x = 0;
			last_touch_position_y = 0;
#endif
			vk_press = 1;
			if (ts->button) {
				if (ts->button[0].index) {
					x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
					y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
				}
			}
			if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				if (ts_is_on) {
#endif
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
				input_mt_sync(ts->input_dev);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				}
#endif
			} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				if (ts_is_on) {
#endif
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
				1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				}
#endif
			}
		}
		else if (data & 0x02) {
			printk("[TP] home key pressed\n");
			vk_press = 1;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
			home_key_pressed = 1;
			back_key_pressed = 0;
			s2w_home_swept_time = jiffies;
			home_key_first_touch_time = jiffies;
			last_touch_position_x = 0;
			last_touch_position_y = 0;
#endif
			if (ts->button) {
				if (ts->button[1].index) {
					x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
					y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
				}
			}
			if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				if (ts_is_on) {
#endif
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
				input_mt_sync(ts->input_dev);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				}
#endif
			} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				if (ts_is_on) {
#endif
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
				1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				}
#endif
			}
		}
	}else {
		printk("[TP] virtual key released\n");
		vk_press = 0;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
		if (s2w_switch > 0)
		{
		    if (scr_suspended == true)
		    {
			if (home_key_pressed)
			{
				printk("[S2W] home key swept in release\n");
				s2w_home_swept_time = jiffies;
			}
			if (back_key_pressed)
			{
				printk("[S2W] back key swept in release\n");
				s2w_back_swept_time = jiffies;
			}
		    }
		} else
		if (home_key_pressed && h2w_switch > 0 && h2w_switch != 3) // if H2w not off (0) , or not Logo2Wake (3), Home should trigger screen on
		{
			if (scr_suspended == true)
			{
				if (jiffies - home_key_first_touch_time > MIN_HOME_KEY_TOUCH_TIME_TO_WAKE)
				{
					printk("[H2W] sweep2wake_pwrtrigger call\n");
					sweep2wake_pwrtrigger();
				}
			}
		}
		home_key_pressed = 0;
		back_key_pressed = 0;

		if (ts_is_on) {
#endif
		if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
			if (ts->support_htc_event) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_mt_sync(ts->input_dev);
		}
		else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
			if (ts->support_htc_event) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
		}
#endif
	}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	if (ts_is_on) {
#endif
	input_sync(ts->input_dev);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	}
#endif
}

static void synaptics_ts_status_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data = 0;

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE), &data, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		data &= 0x0F;
		printk(KERN_INFO "[TP] Device Status = %x\n", data);
		if (data == 1) {
			mutex_lock(&syn_mutex);
			ts->page_select = 0;
			mutex_unlock(&syn_mutex);
			printk(KERN_INFO "[TP] TOUCH: Page Select: %s: %d\n", __func__, ts->page_select);
			ret = synaptics_init_panel(ts);
			if (ret < 0)
				printk(KERN_INFO "[TP]%s: synaptics_init_panel fail\n", __func__);
		}
	}

}

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int ret;
	uint8_t buf = 0;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &buf, 1);

	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		if (buf & get_address_base(ts, ts->finger_func_idx, INTR_SOURCE))
			synaptics_ts_finger_func(ts);
		if (buf & get_address_base(ts, 0x01, INTR_SOURCE))
			synaptics_ts_status_func(ts);
		if (buf & get_address_base(ts, 0x54, INTR_SOURCE))
			synaptics_ts_report_func(ts);
	}

}

static irqreturn_t synaptics_irq_thread(int irq, void *ptr)
{
	struct synaptics_ts_data *ts = ptr;
	int ret;
	uint8_t buf = 0;
	struct timespec timeStart, timeEnd, timeDelta;

	if (ts->debug_log_level & BIT(2)) {
			getnstimeofday(&timeStart);
	}

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &buf, 1);

	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		if (buf & get_address_base(ts, 0x1A, INTR_SOURCE)) {
			if (!ts->finger_count)
				synaptics_ts_button_func(ts);
			else
			{
				printk("[TP] Ignore VK interrupt due to 2d points did not leave\n");
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
				if (s2w_switch > 0)
				{
					synaptics_ts_s2w_button_func(ts);
				}
#endif
			}
		}
		if (buf & get_address_base(ts, ts->finger_func_idx, INTR_SOURCE)) {
			if (!vk_press) {
				synaptics_ts_finger_func(ts);
				if(ts->debug_log_level & BIT(2)) {
					getnstimeofday(&timeEnd);
					timeDelta.tv_nsec = (timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
						-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
					printk(KERN_INFO "[TP] Touch latency = %ld us\n", timeDelta.tv_nsec/1000);
				}
			}
		}
		if (buf & get_address_base(ts, 0x01, INTR_SOURCE))
			synaptics_ts_status_func(ts);
		if (buf & get_address_base(ts, 0x54, INTR_SOURCE))
			synaptics_ts_report_func(ts);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	queue_work(ts->syn_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#ifdef SYN_CABLE_CONTROL
static void cable_tp_status_handler_func(int connect_status)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t data;
	int ret;

	printk(KERN_INFO "[TP] Touch: cable change to %d\n", connect_status);

	if ((connect_status > 0) || (connect_status == CONNECT_TYPE_UNKNOWN))
		connect_status = 1;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, CONTROL_BASE), &data, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	} else {
		ts->cable_config = (data & 0xDF) | (connect_status << 5);
		printk(KERN_INFO "[TP] %s: ts->cable_config: %x\n", __func__, ts->cable_config);
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x01, CONTROL_BASE), ts->cable_config);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
		}
	}
}
static struct t_usb_status_notifier cable_status_handler = {
	.name = "usb_tp_connected",
	.func = cable_tp_status_handler_func,
};
#endif

static void synaptics_ts_close_psensor_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, psensor_work);
	if(ts->psensor_resume_enable == 1) {
		printk(KERN_INFO "[TP] %s: Disable P-sensor by Touch\n", __func__);
		psensor_enable_by_touch_driver(0);
		ts->psensor_resume_enable = 0;
	}
}

static int psensor_tp_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	struct synaptics_ts_data *ts = gl_ts;
	int ret;

	printk(KERN_INFO "[TP] psensor status %d -> %lu\n",
		ts->psensor_status, status);

	if(ts->psensor_detection) {
		if(status == 3 && ts->psensor_resume_enable >= 1) {
			if(!(ts->psensor_status==1 && ts->psensor_resume_enable==1)) {
				if (ts->package_id < 3400) {
					ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_1", __func__);
					printk(KERN_INFO "[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
				}
			}

			if(ts->psensor_resume_enable == 1)
				queue_work(ts->syn_psensor_wq, &ts->psensor_work);
			else
				ts->psensor_resume_enable = 0;
		}
	}

	if (ts->psensor_status == 0) {
		if (status == 1)
			ts->psensor_status = status;
		else
			ts->psensor_status = 0;
	} else
		ts->psensor_status = status;

	if(ts->psensor_detection) {
		if(ts->psensor_status == 0) {
			ts->psensor_resume_enable = 0;
			ts->psensor_phone_enable = 0;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block psensor_status_handler = {
	.notifier_call = psensor_tp_status_handler_func,
};

static int syn_pdt_scan(struct synaptics_ts_data *ts, int num_page)
{
	uint8_t intr_count = 0, data[6] = {0}, num_function[SYN_MAX_PAGE] = {0};
	uint16_t i, j, k = 0;
	int ret = 0;
	ts->num_function = 0;

	for (i = 0; i < num_page; i++) {
		for (j = (0xEE | (i << 8)); j >= (0xBE | (i << 8)); j -= 6) {
			ret = i2c_syn_read(ts->client, j, data, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r1", __func__);
			if (data[0] == 0)
				break;
			else
				num_function[i]++;
		}
		ts->num_function += num_function[i];
	}

	if (ts->address_table != NULL) {
		kfree(ts->address_table);
		ts->address_table = NULL;
	}

	if (ts->address_table == NULL) {
		ts->address_table = kzalloc(sizeof(struct function_t) * ts->num_function, GFP_KERNEL);
		if (ts->address_table == NULL) {
			printk(KERN_INFO "[TP] syn_pdt_scan: memory allocate fail\n");
			return -ENOMEM;
		}
		printk(KERN_INFO "[TP] syn_pdt_scan: memory allocate success. ptr: %p\n", ts->address_table);
	}

	printk(KERN_INFO "[TP] synaptics: %d function supported\n", ts->num_function);
	for (i = 0; i < num_page; i++) {
		for (j = 0; j < num_function[i]; j++) {
			ret = i2c_syn_read(ts->client, i << 8 | (0xE9 - 6*j), data, 6);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
			ts->address_table[j + k].query_base = i << 8 | data[0];
			ts->address_table[j + k].command_base = i << 8 | data[1];
			ts->address_table[j + k].control_base = i << 8 | data[2];
			ts->address_table[j + k].data_base = i << 8 | data[3];
			if (data[4] & 0x07) {
				ts->address_table[j + k].interrupt_source =
					get_int_mask(data[4] & 0x07, intr_count);
				intr_count += (data[4] & 0x07);
			}
			ts->address_table[j + k].function_type = data[5];
			printk(KERN_INFO
				"Query: %2.2X, Command: %4.4X, Control: %2X, Data: %2X, INTR: %2X, Funtion: %2X\n",
				ts->address_table[j + k].query_base , ts->address_table[j + k].command_base,
				ts->address_table[j + k].control_base, ts->address_table[j + k].data_base,
				ts->address_table[j + k].interrupt_source, ts->address_table[j + k].function_type);
		}
		k += num_function[i];
	}
	return ts->num_function;
}
static int syn_get_version(struct synaptics_ts_data *ts)
{
	uint8_t data[16] = {0};
	int ret = 0;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 17, data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	ts->package_id = data[1] << 8 | data[0];
	printk(KERN_INFO "[TP] %s: package_id: %d\n", __func__, ts->package_id);
	if (ts->package_id >= 3400)
		ts->finger_func_idx = 0x12;
	else
		ts->finger_func_idx = 0x11;

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 18, data, 3);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
	ts->packrat_number = data[2] << 16 | data[1] << 8 | data[0];
	printk(KERN_INFO "[TP] %s: packrat_number: %d\n", __func__, ts->packrat_number);

	if (ts->packrat_number < SYNAPTICS_FW_3_2_PACKRAT) {

		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 16, data, 3);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		syn_panel_version = data[0] << 8 | data[2];
		printk(KERN_INFO "[TP] %s: panel_version: %x\n", __func__, syn_panel_version);

	} else {

		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 28, data, 16);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		syn_panel_version = data[5] << 8 | data[7];
		printk(KERN_INFO "[TP] %s: panel_version: %x\n", __func__, syn_panel_version);
	}

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x34, CONTROL_BASE), data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
	ts->config_version = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	printk(KERN_INFO "[TP] %s: config version: %x\n", __func__, ts->config_version);

	return 0;
}

static int syn_get_information(struct synaptics_ts_data *ts)
{
	uint8_t data[4] = {0}, i, num_channel, *buf;
	int ret = 0;

	if (ts->package_id < 3400) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, QUERY_BASE) + 1, data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
		if ((data[0] & 0x07) == 5)
			ts->finger_support = 10;
		else if ((data[0] & 0x07) < 5)
			ts->finger_support = (data[0] & 0x07) + 1;
		else {
			printk(KERN_INFO "[TP] %s: number of fingers not define: %x\n",
				__func__, data[0] & 0x07);
			return SYN_PROCESS_ERR;
		}
	} else {
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 7, data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
		ts->finger_support = data[1];
	}

	printk(KERN_INFO "[TP] %s: finger_support: %d\n", __func__, ts->finger_support);

	if (ts->package_id < 3400)
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 6, data, 4);
	else
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE), data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);

	ts->max[0] = data[0] | data[1] << 8;
	ts->max[1] = data[2] | data[3] << 8;
	printk(KERN_INFO "[TP] %s: max_x: %d, max_y: %d\n", __func__, ts->max[0], ts->max[1]);

	if (get_address_base(ts, 0x54, FUNCTION)) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, QUERY_BASE), data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);

		ts->y_channel = data[0];
		ts->x_channel = data[1];

		num_channel = ts->y_channel + ts->x_channel;
		buf = kzalloc(num_channel + 1, GFP_KERNEL);
		if (buf == NULL) {
			printk(KERN_INFO "[TP] %s: memory allocate fail\n", __func__);
			return -ENOMEM;
		}
		if (ts->packrat_number < SYNAPTICS_FW_3_2_PACKRAT)
			ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 17,
				buf, num_channel + 1);
		else
			ret = i2c_syn_read(ts->client, get_address_base(ts, 0x55, CONTROL_BASE),
				buf, num_channel + 1);
		if (ret < 0) {
			kfree(buf);
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
		}

		for (i = 1; i < num_channel + 1; i++) {
			if (buf[i] == 0xFF) {
				if (i <= num_channel - ts->x_channel)
					ts->y_channel--;
				else
					ts->x_channel--;
			}
		}

		if (buf[0] & 0x01)
			swap(ts->y_channel, ts->x_channel);
		printk(KERN_INFO "[TP] %s: X: %d, Y: %d\n", __func__,
			ts->x_channel, ts->y_channel);
		kfree(buf);

		ts->temp_report_data = kzalloc(2 * ts->x_channel * ts->y_channel, GFP_KERNEL);
		ts->report_data = kzalloc(2 * ts->x_channel * ts->y_channel, GFP_KERNEL);
		if(ts->temp_report_data == NULL || ts->report_data == NULL)
			return -ENOMEM;

		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, &ts->relaxation, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:5", __func__);
		printk(KERN_INFO "[TP] %s: ts->relaxation: %d\n", __func__, ts->relaxation);

	}
	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->large_obj_check) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x26, &ts->default_large_obj, 1);
				} else {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x29, &ts->default_large_obj, 1);
				}
				if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:6", __func__);
				printk(KERN_INFO "[TP] %s: ts->default_large_obj: %x\n", __func__, ts->default_large_obj);
			}
		}

		if (ts->segmentation_bef_unlock) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x25, &ts->segmentation_aft_unlock, 1);

				} else {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x22, &ts->segmentation_aft_unlock, 1);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:7", __func__);
				printk(KERN_INFO "[TP] %s: ts->segmentation_aft_unlock: %x\n", __func__, ts->segmentation_aft_unlock);
			}
		}

		if (ts->threshold_bef_unlock) {
			if (ts->package_id < 3400) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0A, &ts->threshold_aft_unlock, 1);

				} else {
					ret = i2c_syn_read(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0C, &ts->threshold_aft_unlock, 1);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:8", __func__);
				printk(KERN_INFO "[TP] %s: ts->z_threshold_aft_unlock: %x\n", __func__, ts->threshold_aft_unlock);
			}
		}

		if (ts->saturation_bef_unlock) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, data, 2);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:8", __func__);
			ts->saturation_aft_unlock = (data[1] << 8) | data[0];
			printk(KERN_INFO "[TP] %s: ts->saturation_aft_unlock: %x\n", __func__, ts->saturation_aft_unlock);
		}

		if (ts->PixelTouchThreshold_bef_unlock) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, &ts->PixelTouchThreshold_aft_unlock, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:9", __func__);
			printk(KERN_INFO "[TP] %s: ts->PixelTouchThreshold_aft_unlock: %x\n", __func__, ts->PixelTouchThreshold_aft_unlock);
		}
	}

	return 0;
}

static int syn_probe_init(void *arg)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)arg;
	struct synaptics_i2c_rmi_platform_data *pdata;
	int ret = 0;
	uint8_t data = 0, i;
	uint16_t wait_time = SYN_FW_TIMEOUT;

	printk(KERN_INFO "[TP] %s: enter", __func__);
	pdata = ts->client->dev.platform_data;

	if (pdata == NULL) {
		printk(KERN_INFO "[TP] pdata is NULL\n");
		goto err_get_platform_data_fail;
	}

	if (board_build_flag() == MFG_BUILD) {
		wait_time = SYN_FW_TIMEOUT;
		wait_event_interruptible_timeout(ts->syn_fw_wait, atomic_read(&ts->syn_fw_condition),
							msecs_to_jiffies(wait_time));
	}

	for (i = 0; i < 10; i++) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "read device status failed!", __func__);
			goto err_init_failed;
		}
		if (data & 0x44) {
			hr_msleep(20);
#ifdef SYN_FLASH_PROGRAMMING_LOG
			printk(KERN_INFO "[TP] synaptics probe: F01_data: %x touch controller stay in bootloader mode!\n", data);
#endif
		} else if (data & 0x40) {
			printk(KERN_ERR "[TP] TOUCH_ERR: synaptics probe: F01_data: %x touch controller stay in bootloader mode!\n", data);
			goto err_init_failed;
		} else
			break;
	}

	if (i == 10) {
		uint8_t num = 0;
		printk(KERN_INFO "[TP] synaptics probe: touch controller doesn't enter UI mode! F01_data: %x\n", data);

		if (pdata) {
			while (pdata->default_config != 1) {
				if (pdata->default_config == 0) {
					printk(KERN_ERR "[TP] TOUCH_ERR: touch controller stays in bootloader mode "
						"and recovery method doesn't enable\n");
					goto err_init_failed;
				}
				pdata++;
				num++;
			}
			ts->config = pdata->config;

			ret = syn_config_update(ts, pdata->gpio_irq);
			if (ret < 0) {
				printk(KERN_ERR "[TP] TOUCH_ERR: syn_config_update fail\n");
				goto err_init_failed;
			} else if (ret == 0)
				printk(KERN_INFO "[TP] syn_config_update success\n");
			else
				printk(KERN_INFO "[TP] Warning: syn_config_update: the same "
					"config version and CRC but touch controller always stay in bootloader mode\n");
			pdata = pdata - num;
		}
	}

	if (syn_pdt_scan(ts, SYN_MAX_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_init_failed;
	}
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING) {
		printk(KERN_INFO "[TP] %s: offmode charging. Set touch chip to sleep mode and skip touch driver probe\n", __func__);
		ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
		if (ret < 0)
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
		ret = -ENODEV;
		goto err_check_offmode_charging;
	}

	ts->finger_func_idx = 0x11;
	if (syn_get_version(ts) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_version fail\n");
		goto err_init_failed;
	}

	if (pdata) {
		while (pdata->packrat_number && pdata->packrat_number > ts->packrat_number) {
			pdata++;
		}
		if (pdata->tw_pin_mask) {
			ts->tw_pin_mask = pdata->tw_pin_mask;
			ret = syn_get_tw_vendor(ts, pdata->gpio_irq);
			if (ret < 0) {
				printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_tw_vendor fail\n");
				goto err_init_failed;
			}
		}

		while (pdata->sensor_id > 0 && pdata->sensor_id != (SENSOR_ID_CHECKING_EN | ts->tw_vendor)) {
			pdata++;
		}
		printk(KERN_INFO "[TP] synaptics_ts_probe: pdata->version = %x, pdata->packrat_number = %d,"
				" pdata->sensor_id = %x\n", pdata->version, pdata->packrat_number, pdata->sensor_id);

		if (!pdata->packrat_number) {
			printk(KERN_ERR "[TP] TOUCH_ERR: get null platform data\n");
				goto err_init_failed;
		}

		ts->power = pdata->power;
		ts->lpm_power = pdata->lpm_power;
		ts->flags = pdata->flags;
		ts->htc_event = pdata->report_type;
		ts->filter_level = pdata->filter_level;
		ts->reduce_report_level = pdata->reduce_report_level;
		ts->gpio_irq = pdata->gpio_irq;
		ts->gpio_reset = pdata->gpio_reset;
		ts->large_obj_check = pdata->large_obj_check;
		ts->support_htc_event = pdata->support_htc_event;
		ts->mfg_flag = pdata->mfg_flag;
		ts->segmentation_bef_unlock = pdata->segmentation_bef_unlock;
		ts->i2c_err_handler_en = pdata->i2c_err_handler_en;
		ts->threshold_bef_unlock = pdata->threshold_bef_unlock;
		ts->saturation_bef_unlock = pdata->saturation_bef_unlock;
		ts->energy_ratio_relaxation = pdata->energy_ratio_relaxation;
		ts->multitouch_calibration = pdata->multitouch_calibration;
		ts->psensor_detection = pdata->psensor_detection;
		ts->PixelTouchThreshold_bef_unlock = pdata->PixelTouchThreshold_bef_unlock;
#ifdef SYN_CABLE_CONTROL
		ts->cable_support = pdata->cable_support; 
#endif
		ts->config = pdata->config;
		if (pdata->virtual_key)
			ts->button = pdata->virtual_key;
	}

#ifndef SYN_DISABLE_CONFIG_UPDATE
	ret = syn_config_update(ts, pdata->gpio_irq);
	if (ret < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: syn_config_update fail\n");
		goto err_init_failed;
	} else if (ret == 0)
		printk(KERN_INFO "[TP] syn_config_update success\n");
	else
		printk(KERN_INFO "[TP] syn_config_update: the same config version and CRC\n");
#else
	if (pdata->tw_pin_mask) {
		ret = disable_flash_programming(ts, 0);
		if (ret < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: disable_flash_programming fail\n");
			goto err_init_failed;
		}
	}
#endif
	if (syn_pdt_scan(ts, SYN_MAX_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_init_failed;
	}

#ifndef SYN_DISABLE_CONFIG_UPDATE
	if (pdata->customer_register[CUS_REG_BASE]) {
		ret = i2c_syn_write(ts->client, pdata->customer_register[CUS_REG_BASE],
			&pdata->customer_register[CUS_BALLISTICS_CTRL], CUS_REG_SIZE - 1);
		printk(KERN_INFO "[TP] Loads customer register\n");
	}
#endif

	if (syn_get_information(ts) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_information fail\n");
		goto err_syn_get_info_failed;
	}

	if (pdata->abs_x_max  == 0 && pdata->abs_y_max == 0) {
		ts->layout[0] = ts->layout[2] = 0;
		ts->layout[1] = ts->max[0];
		ts->layout[3] = ts->max[1];
	} else {
		ts->layout[0] = pdata->abs_x_min;
		ts->layout[1] = pdata->abs_x_max;
		ts->layout[2] = pdata->abs_y_min;
		ts->layout[3] = pdata->abs_y_max;
	}

	if(pdata->display_width && pdata->display_height){
		printk(KERN_INFO "[TP] Load display resolution: %dx%d\n", pdata->display_width, pdata->display_height);
		ts->width_factor = (pdata->display_width<<SHIFT_BITS)/(ts->layout[1]-ts->layout[0]);
		ts->height_factor = (pdata->display_height<<SHIFT_BITS)/(ts->layout[3]-ts->layout[2]);
	}

	if(get_tamper_sf()==0) {
		ts->debug_log_level |= BIT(3);
		printk(KERN_INFO "[TP] Debug log level=0x%02X\n", ts->debug_log_level);
	}

	if (get_address_base(ts, 0x19, FUNCTION)) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x19, QUERY_BASE) + 1,
			&ts->key_number, 1);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F19 Query fail", __func__);
			goto err_F19_query_failed;
		}
		for (i = 0; i < ts->key_number; i++) {
			ts->key_postion_x[i] =
				(ts->layout[1] - ts->layout[0]) * (i * 2 + 1) / (ts->key_number * 2)
				+ ts->layout[0];
			printk(KERN_INFO "[TP] ts->key_postion_x[%d]: %d\n",
				i, ts->key_postion_x[i]);
		}
		ts->key_postion_y = ts->layout[2] +
			(21 * (ts->layout[3] - ts->layout[2]) / 20);
		printk(KERN_INFO "[TP] ts->key_postion_y: %d\n", ts->key_postion_y);
	}

	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_init_panel fail\n");
		goto err_init_panel_failed;
	}

	init_waitqueue_head(&syn_data_ready_wq);

	if(ts->psensor_detection) {
		INIT_WORK(&ts->psensor_work, synaptics_ts_close_psensor_func);
		ts->syn_psensor_wq = create_singlethread_workqueue("synaptics_psensor_wq");
		if (!ts->syn_psensor_wq)
			goto err_create_wq_failed;
	}

	ret = synaptics_input_register(ts);
	if (ret) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_ts_probe: "
				"Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	gl_ts = ts;

	ts->irq_enabled = 0;
	if (ts->client->irq) {
		ts->use_irq = 1;
		ret = request_threaded_irq(ts->client->irq, NULL, synaptics_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, CONTROL_BASE) + 1, &ts->intr_bit, 1);
			if (ret < 0) {
				free_irq(ts->client->irq, ts);
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "get interrupt bit failed", __func__);
				goto err_get_intr_bit_failed;
			}
			printk(KERN_INFO "[TP] %s: interrupt enable: %x\n", __func__, ts->intr_bit);
		} else {
			dev_err(&ts->client->dev, "[TP] TOUCH_ERR: request_irq failed\n");
			ts->use_irq = 0;
		}
	}

	if (!ts->use_irq) {

		ts->syn_wq = create_singlethread_workqueue("synaptics_wq");
		if (!ts->syn_wq)
			goto err_create_wq_failed;

		INIT_WORK(&ts->work, synaptics_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef SYN_CABLE_CONTROL
	if (ts->cable_support) {
		usb_register_notifier(&cable_status_handler);
		
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, ts->finger_func_idx, CONTROL_BASE), &ts->cable_config, 1);
		if (ret < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: get cable config failed\n");
			goto err_get_cable_config_failed;
		}
		if (usb_get_connect_type())
			cable_tp_status_handler_func(1);
		printk(KERN_INFO "[TP] %s: ts->cable_config: %x\n", __func__, ts->cable_config);
	}
#endif
	register_notifier_by_psensor(&psensor_status_handler);
	synaptics_touch_sysfs_init();
#ifdef SYN_WIRELESS_DEBUG
	if (rmi_char_dev_register())
		printk(KERN_INFO "[TP] %s: error register char device", __func__);
#endif

	printk(KERN_INFO "[TP] synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

#ifdef SYN_CABLE_CONTROL
err_get_cable_config_failed:
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		destroy_workqueue(ts->syn_wq);
#endif

err_create_wq_failed:

err_get_intr_bit_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_init_panel_failed:
err_F19_query_failed:
err_syn_get_info_failed:
	if(ts->report_data != NULL)
		kfree(ts->report_data);
	if(ts->temp_report_data != NULL)
		kfree(ts->temp_report_data);
err_init_failed:
err_check_offmode_charging:
	if(ts->address_table != NULL)
		kfree(ts->address_table);
err_get_platform_data_fail:
	kfree(ts);

	return ret;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	struct synaptics_i2c_rmi_platform_data *pdata;
	int ret = 0;
	uint8_t data = 0;

	printk(KERN_INFO "[TP] %s: enter", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		printk(KERN_INFO "[TP] pdata is NULL\n");
		goto err_get_platform_data_fail;
	}

	ret = i2c_syn_read(ts->client, 0x00EE, &data, 1);
	if (ret < 0) {
		printk(KERN_INFO "[TP] No Synaptics chip\n");
		goto err_detect_failed;
	}
	if (syn_pdt_scan(ts, SYN_BL_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_detect_failed;
	}
	if (board_build_flag() == MFG_BUILD) {
		ts->gpio_irq = pdata->gpio_irq;
		if (syn_get_version(ts) < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_version fail\n");
			goto err_detect_failed;
		}

		init_waitqueue_head(&ts->syn_fw_wait);
		atomic_set(&ts->syn_fw_condition, 0);
		kthread_run(syn_fw_update_init, (void *)ts, "SYN_FW_UPDATE");
	}

	kthread_run(syn_probe_init, (void *)ts, "SYN_PROBE_INIT");
	return 0;

err_detect_failed:
err_get_platform_data_fail:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else {
		hrtimer_cancel(&ts->timer);
		if (ts->syn_wq)
			destroy_workqueue(ts->syn_wq);
	}
	if(ts->sr_input_dev != NULL)
		input_unregister_device(ts->sr_input_dev);
	input_unregister_device(ts->input_dev);

	synaptics_touch_sysfs_remove();

	if(ts->report_data != NULL)
		kfree(ts->report_data);
	if(ts->temp_report_data != NULL)
		kfree(ts->temp_report_data);
	if(ts->address_table != NULL)
		kfree(ts->address_table);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	uint8_t data = 0, update = 0;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	if (h2w_switch > 0 || s2w_switch > 0) {
		enable_irq_wake(client->irq);
		printk(KERN_INFO "[sweep2wake]: suspend but keep interupt wake going.\n");
		if (h2w_switch == 2) {
			//ensure backlight is turned off
		//	pm8xxx_led_current_set(sweep2wake_leddev, 0);
			printk(KERN_INFO "[sweep2wake]: deactivated button backlight.\n");
		}
	}
#endif
	printk(KERN_INFO "[TP] %s: enter\n", __func__);

	if (ts->use_irq) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
                if (h2w_switch == 0 && s2w_switch == 0) {
#endif
		disable_irq(client->irq);
		ts->irq_enabled = 0;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
		}
#endif
	} else {
		hrtimer_cancel(&ts->timer);
		ret = cancel_work_sync(&ts->work);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        if (h2w_switch == 0 && s2w_switch == 0) {
                if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
                        enable_irq(client->irq);
        }
#endif
	}

	if(ts->psensor_detection) {
		if(ts->psensor_resume_enable == 1){
			printk(KERN_INFO "[TP] %s: Disable P-sensor by Touch\n", __func__);
			psensor_enable_by_touch_driver(0);
			ts->psensor_resume_enable = 0;
		}
	}

	if (ts->psensor_status == 0) {
		ts->pre_finger_data[0][0] = 0;
		if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
			ts->first_pressed = 0;
#ifdef SYN_CALIBRATION_CONTROL
			if (ts->mfg_flag != 1) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, ts->relaxation);
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "fast relaxation", __func__);

				if (ts->energy_ratio_relaxation) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE), 0x20);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "energy ratio relaxation", __func__);
				}

				if (ts->saturation_bef_unlock) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_bef_unlock & 0xFF);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_bef_unlock & 0xFF00) >> 8);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
					printk(KERN_INFO "[TP] touch suspend, saturation capacitance: %x\n", ts->saturation_bef_unlock);
				}

				if ( ts->PixelTouchThreshold_bef_unlock ) {
					if (ts->package_id <= 3400 ) {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_bef_unlock );
						if (ret < 0)
							return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
						printk(KERN_INFO "[TP] touch suspend, set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", ts->PixelTouchThreshold_bef_unlock);
					}
				}

				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "force update", __func__);
				printk(KERN_INFO "[TP] touch suspend, fast relasxation: %x\n", ts->relaxation);
			}
#endif

			if (ts->large_obj_check) {
				if (ts->package_id < 3400) {
					if (ts->package_id == 2200) {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x26, ts->default_large_obj & 0x7F);

					} else {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x29, ts->default_large_obj & 0x7F);
					}
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "large obj suppression", __func__);
					printk(KERN_INFO "[TP] touch suspend, set large obj suppression: %x\n", ts->default_large_obj & 0x7F);
				}
			}

			if (ts->segmentation_bef_unlock) {
				if (ts->package_id < 3400) {
					if (ts->package_id == 2200) {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x25, ts->segmentation_bef_unlock);
					} else {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x22, ts->segmentation_bef_unlock);
					}
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "segmentation aggressiveness", __func__);
					printk(KERN_INFO "[TP] touch suspend, set segmentation aggressiveness: %x\n", ts->segmentation_bef_unlock);
				}
			}

			if (ts->threshold_bef_unlock) {
				if (ts->package_id < 3400) {
					if (ts->package_id == 2200) {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0A, ts->threshold_bef_unlock);

					} else {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0C, ts->threshold_bef_unlock);
					}
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "Z Touch threshold", __func__);
					printk(KERN_INFO "[TP] touch suspend, set Z Touch threshold: %x\n", ts->threshold_bef_unlock);
				}
			}
		}
	}
	else if(ts->psensor_detection)
		ts->psensor_phone_enable = 1;

#ifdef CONFIG_PWRKEY_STATUS_API
	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT)
		printk(KERN_INFO "[TP][PWR][STATE] get power key state = %d\n", getPowerKeyState());
#endif
	if (ts->disable_CBC) {
		if (ts->package_id < 3400) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, &data, 1);
			if ((data & 0x10) == 0) {
				printk(KERN_INFO "[TP] %s: CBC disabled : %d", __func__, ts->package_id);
				data |= 0x10;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x07, data);
				update = 1;
			}
		} else {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, &data, 1);
			if ((data & 0x20) == 0) {
				printk(KERN_INFO "[TP] %s: CBC disabled : %d", __func__, ts->package_id);
				data |= 0x20;
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x1E, data);
				update = 1;
			}
		}
		if (ts->button) {
			if (ts->package_id < 3400) {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, &data, 1);
				if ((data & 0x10) == 0) {
					printk(KERN_INFO "[TP] %s: 0D CBC disabled : %d", __func__, ts->package_id);
					data |= 0x10;
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x56, data);
					update = 1;
				}
			} else {
				ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, &data, 1);
				if ((data & 0x10) == 0) {
					printk(KERN_INFO "[TP] %s: 0D CBC disabled : %x", __func__, ts->package_id);
					data |= 0x10;
					ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x19, data);
					update = 1;
				}
			}
		}
		if (update) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
		}
		ts->disable_CBC = 0;
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        if (h2w_switch == 0 && s2w_switch == 0) {
#endif
	if (ts->power)
		ts->power(0);
	else {
		if (ts->packrat_number >= SYNAPTICS_FW_2IN1_PACKRAT) {
			ret = i2c_syn_write_byte_data(client,
				get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
			if (ret < 0)
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
		} else {
			if (ts->psensor_status > 0
#ifdef CONFIG_PWRKEY_STATUS_API
			&& getPowerKeyState() == 0
#endif
			 ) {
				ret = i2c_syn_write_byte_data(client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x02); 
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x02", __func__);
			} else {
				ret = i2c_syn_write_byte_data(client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
			}
		}
		if (ts->lpm_power)
			ts->lpm_power(1);
	}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
    }
#endif
	scr_suspended = true;
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        if (h2w_switch > 0 || s2w_switch > 0) {
                //screen on, disable_irq_wake
                disable_irq_wake(client->irq);
        }
#endif
	printk(KERN_INFO "[TP] %s: enter\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        if (h2w_switch == 0 && s2w_switch == 0) {
#endif
	if (ts->power) {
		ts->power(1);
		hr_msleep(100);
#ifdef SYN_CABLE_CONTROL
		if (ts->cable_support) {
			if (usb_get_connect_type())
				cable_tp_status_handler_func(1);
			printk(KERN_INFO "%s: ts->cable_config: %x\n", __func__, ts->cable_config);
		}
#endif
	} else {
		if (ts->lpm_power)
			ts->lpm_power(0);
		ret = i2c_syn_write_byte_data(client,
			get_address_base(ts, 0x01, CONTROL_BASE), 0x00); 
		if (ret < 0)
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "wake up", __func__);
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        }
#endif
	if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
		if (ts->support_htc_event) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			input_sync(ts->input_dev);
		}
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(ts->input_dev);
	} else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
	}
	if (ts->psensor_detection) {
		if(ts->psensor_status == 0) {
			ts->psensor_resume_enable = 1;
			printk(KERN_INFO "[TP] %s: Enable P-sensor by Touch\n", __func__);
			psensor_enable_by_touch_driver(1);
		}
		else if(ts->psensor_phone_enable == 0) {
			if(ts->psensor_status != 3)
				ts->psensor_resume_enable = 2;

			ts->psensor_phone_enable = 1;
		}
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
        if (h2w_switch == 0 && s2w_switch == 0) {
#endif
	if (ts->use_irq) {
		enable_irq(client->irq);
		ts->irq_enabled = 1;
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE
	}

	if (h2w_switch_changed)
	{
		h2w_switch = h2w_temp;
	}
	if (s2w_switch_changed)
	{
		s2w_switch = s2w_temp;
	}
#endif
        scr_suspended = false;
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_3200_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_3200_NAME,
	},
};

static void __devinit synaptics_ts_init_async(void *unused, async_cookie_t cookie)
{
	i2c_add_driver(&synaptics_ts_driver);
}

static int __devinit synaptics_ts_init(void)
{
	async_schedule(synaptics_ts_init_async, NULL);
	return 0;
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
