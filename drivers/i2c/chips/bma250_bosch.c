
/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 * (C) Copyright 2013 Illes Pal Zoltan - Gyrowake modifications
 */



#undef CONFIG_HAS_EARLYSUSPEND

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#ifdef CONFIG_BMA250_WAKE_OPTIONS
#include <linux/jiffies.h>
#include <linux/pl_sensor.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/mfd/pm8xxx/vibrator.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/bma250.h>
#define D(x...) printk(KERN_DEBUG "[GSNR][BMA250_BOSCH] " x)
#define I(x...) printk(KERN_INFO "[GSNR][BMA250_BOSCH] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA250_BOSCH] " x)

#define HTC_ATTR 1

struct bma250acc{
	s16	x,
		y,
		z;
} ;

struct bma250_data {
	struct i2c_client *bma250_client;
	atomic_t delay;
	atomic_t enable;
	atomic_t selftest_result;
	unsigned char mode;
	struct input_dev *input;
#ifdef CONFIG_CIR_ALWAYS_READY
	struct input_dev *input_cir;
#endif
	struct bma250acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int IRQ;
	int chip_layout;
#ifdef HTC_ATTR
	struct class *g_sensor_class;
	struct device *g_sensor_dev;
#endif 

	struct bma250_platform_data *pdata;
	short offset_buf[3];
};

struct bma250_data *gdata;

#ifdef CONFIG_CIR_ALWAYS_READY
#define BMA250_ENABLE_INT1 1
static int cir_flag = 0;
static int power_key_pressed = 0;
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h);
static void bma250_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_BMA250_WAKE_OPTIONS
// flick of the phone wakes/sleeps the phone
static int FLICK_WAKE_ENABLED = 1;
static int FLICK_SLEEP_ENABLED = 1;
static int FLICK_WAKE_SENSITIVITY = 1; // 0-1, 0 less sensitive, 1 more sensible
static int FLICK_WAKE_MIN_SLEEP_TIME = 0;
// if phone has been laying around on the table (horizontal still), and gyro turns to mostly vertical for a bit of time, wake phone
static int PICK_WAKE_ENABLED = 0;
static int suspended = 1;
static int screen_on = 1;

static int keep_sensor_on(void)
{
	return PICK_WAKE_ENABLED;
}
#endif

static int bma250_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma250_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1;

#ifdef CONFIG_CIR_ALWAYS_READY
	if(cir_flag && Mode == BMA250_MODE_SUSPEND) {
	    return 0;
	} else {
#endif
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	if (keep_sensor_on() && Mode != BMA250_MODE_NORMAL) {
	    return 0;
	}
#endif
	if (Mode < 3) {
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_LOW_POWER__REG, &data1);
		switch (Mode) {
		case BMA250_MODE_NORMAL:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_LOWPOWER:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 1);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_SUSPEND:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 1);
			break;
		default:
			break;
		}

		comres += bma250_smbus_write_byte(client,
				BMA250_EN_LOW_POWER__REG, &data1);
	} else{
		comres = -1;
	}
#ifdef CONFIG_CIR_ALWAYS_READY
	}
#endif


	return comres;
}
#ifdef BMA250_ENABLE_INT1
static int bma250_set_int1_pad_sel(struct i2c_client *client, unsigned char
		int1sel)
{
	int comres = 0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_LOWG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_LOWG,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_HIGHG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_HIGHG,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_SLOPE__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SLOPE,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_DB_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_DB_TAP,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_SNG_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SNG_TAP,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_ORIENT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_ORIENT,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT1_PAD_FLAT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_FLAT,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT1_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif 
#ifdef BMA250_ENABLE_INT2
static int bma250_set_int2_pad_sel(struct i2c_client *client, unsigned char
		int2sel)
{
	int comres = 0;
	unsigned char data;
	unsigned char state;
	state = 0x01;


	switch (int2sel) {
	case 0:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_LOWG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_LOWG,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_HIGHG__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_HIGHG,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_SLOPE__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_SLOPE,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_DB_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_DB_TAP,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_SNG_TAP__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_SNG_TAP,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_ORIENT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_ORIENT,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_INT2_PAD_FLAT__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT2_PAD_FLAT,
				state);
		comres = bma250_smbus_write_byte(client,
				BMA250_EN_INT2_PAD_FLAT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif 

static int bma250_set_Int_Enable(struct i2c_client *client, unsigned char
		InterruptType , unsigned char value)
{
	int comres = 0;
	unsigned char data1, data2;


	comres = bma250_smbus_read_byte(client, BMA250_INT_ENABLE1_REG, &data1);
	comres = bma250_smbus_read_byte(client, BMA250_INT_ENABLE2_REG, &data2);

	value = value & 1;
	switch (InterruptType) {
	case 0:
		
		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_LOWG_INT, value);
		break;
	case 1:
		

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_X_INT,
				value);
		break;
	case 2:
		

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Y_INT,
				value);
		break;
	case 3:
		

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Z_INT,
				value);
		break;
	case 4:
		

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_NEW_DATA_INT,
				value);
		break;
	case 5:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_X_INT,
				value);
		break;
	case 6:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Y_INT,
				value);
		break;
	case 7:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Z_INT,
				value);
		break;
	case 8:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SINGLE_TAP_INT,
				value);
		break;
	case 9:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_DOUBLE_TAP_INT,
				value);
		break;
	case 10:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_ORIENT_INT, value);
		break;
	case 11:
		

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_FLAT_INT, value);
		break;
	default:
		break;
	}
	comres = bma250_smbus_write_byte(client, BMA250_INT_ENABLE1_REG,
			&data1);
	comres = bma250_smbus_write_byte(client, BMA250_INT_ENABLE2_REG,
			&data2);

	return comres;
}


static int bma250_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;


	comres = bma250_smbus_read_byte(client,
			BMA250_EN_LOW_POWER__REG, Mode);
	*Mode  = (*Mode) >> 6;


	return comres;
}

static int bma250_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data1;


	if (Range < 4) {
		comres = bma250_smbus_read_byte(client,
				BMA250_RANGE_SEL_REG, &data1);
		switch (Range) {
		case 0:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_RANGE_SEL, 0);
			break;
		case 1:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_RANGE_SEL, 5);
			break;
		case 2:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_RANGE_SEL, 8);
			break;
		case 3:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_RANGE_SEL, 12);
			break;
		default:
			break;
		}
		comres += bma250_smbus_write_byte(client,
				BMA250_RANGE_SEL_REG, &data1);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma250_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client, BMA250_RANGE_SEL__REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_RANGE_SEL);
	*Range = data;


	return comres;
}


static int bma250_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int Bandwidth = 0;


	if (BW < 8) {
		switch (BW) {
		case 0:
			Bandwidth = BMA250_BW_7_81HZ;
			break;
		case 1:
			Bandwidth = BMA250_BW_15_63HZ;
			break;
		case 2:
			Bandwidth = BMA250_BW_31_25HZ;
			break;
		case 3:
			Bandwidth = BMA250_BW_62_50HZ;
			break;
		case 4:
			Bandwidth = BMA250_BW_125HZ;
			break;
		case 5:
			Bandwidth = BMA250_BW_250HZ;
			break;
		case 6:
			Bandwidth = BMA250_BW_500HZ;
			break;
		case 7:
			Bandwidth = BMA250_BW_1000HZ;
			break;
		default:
			break;
		}
		comres = bma250_smbus_read_byte(client,
				BMA250_BANDWIDTH__REG, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_BANDWIDTH,
				Bandwidth);
		comres += bma250_smbus_write_byte(client,
				BMA250_BANDWIDTH__REG, &data);
	} else{
		comres = -1;
	}


	return comres;
}

static int bma250_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client, BMA250_BANDWIDTH__REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_BANDWIDTH);
	if (data <= 8) {
		*BW = 0;
	} else{
		if (data >= 0x0F)
			*BW = 7;
		else
			*BW = data - 8;

	}


	return comres;
}

#if defined(BMA250_ENABLE_INT1) || defined(BMA250_ENABLE_INT2)
static int bma250_get_interruptstatus1(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS1_REG, &data);
	*intstatus = data;

	return comres;
}

#if 0
static int bma250_get_HIGH_first(struct i2c_client *client, unsigned char
						param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data;

	switch (param) {
	case 0:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}

static int bma250_get_HIGH_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_SIGN_S);
	*intstatus = data;

	return comres;
}

static int bma250_get_slope_first(struct i2c_client *client, unsigned char
	param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data;

	switch (param) {
	case 0:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_TAP_SLOPE_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_TAP_SLOPE_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma250_smbus_read_byte(client,
				BMA250_STATUS_TAP_SLOPE_REG, &data);
		data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}
static int bma250_get_slope_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_TAP_SLOPE_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_SIGN_S);
	*intstatus = data;

	return comres;
}
static int bma250_get_orient_status(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_S);
	*intstatus = data;

	return comres;
}

static int bma250_get_orient_flat_status(struct i2c_client *client, unsigned
		char *intstatus)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_FLAT_S);
	*intstatus = data;

	return comres;
}
#endif
#endif 
static int bma250_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_INT_MODE_SEL, Mode);
	comres = bma250_smbus_write_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);


	return comres;
}

static int bma250_get_Int_Mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_INT_MODE_SEL__REG, &data);
	data  = BMA250_GET_BITSLICE(data, BMA250_INT_MODE_SEL);
	*Mode = data;


	return comres;
}
static int bma250_set_slope_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_DUR, duration);
	comres = bma250_smbus_write_byte(client,
			BMA250_SLOPE_DUR__REG, &data);


	return comres;
}

static int bma250_get_slope_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_DUR);
	*status = data;


	return comres;
}

static int bma250_set_slope_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_THRES, threshold);
	comres = bma250_smbus_write_byte(client,
			BMA250_SLOPE_THRES__REG, &data);


	return comres;
}

static int bma250_get_slope_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data;


	comres = bma250_smbus_read_byte(client,
			BMA250_SLOPE_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_SLOPE_THRES);
	*status = data;


	return comres;
}
static int bma250_set_low_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOWG_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_LOWG_DUR, duration);
	comres = bma250_smbus_write_byte(client, BMA250_LOWG_DUR__REG, &data);

	return comres;
}

static int bma250_get_low_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOW_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_LOWG_DUR);
	*status = data;

	return comres;
}

static int bma250_set_low_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOWG_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_LOWG_THRES, threshold);
	comres = bma250_smbus_write_byte(client, BMA250_LOWG_THRES__REG, &data);

	return comres;
}

static int bma250_get_low_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_LOW_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_LOWG_THRES);
	*status = data;

	return comres;
}

static int bma250_set_high_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGHG_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_HIGHG_DUR, duration);
	comres = bma250_smbus_write_byte(client, BMA250_HIGHG_DUR__REG, &data);

	return comres;
}

static int bma250_get_high_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGH_DURN_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_DUR);
	*status = data;

	return comres;
}

static int bma250_set_high_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGHG_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_HIGHG_THRES, threshold);
	comres = bma250_smbus_write_byte(client, BMA250_HIGHG_THRES__REG,
			&data);

	return comres;
}

static int bma250_get_high_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_HIGH_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_HIGHG_THRES);
	*status = data;

	return comres;
}


static int bma250_set_tap_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_DUR__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_DUR, duration);
	comres = bma250_smbus_write_byte(client, BMA250_TAP_DUR__REG, &data);

	return comres;
}

static int bma250_get_tap_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_DUR);
	*status = data;

	return comres;
}

static int bma250_set_tap_shock(struct i2c_client *client, unsigned char setval)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_SHOCK_DURN__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_SHOCK_DURN, setval);
	comres = bma250_smbus_write_byte(client, BMA250_TAP_SHOCK_DURN__REG,
			&data);

	return comres;
}

static int bma250_get_tap_shock(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_SHOCK_DURN);
	*status = data;

	return comres;
}

static int bma250_set_tap_quiet(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_QUIET_DURN__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_QUIET_DURN, duration);
	comres = bma250_smbus_write_byte(client, BMA250_TAP_QUIET_DURN__REG,
			&data);

	return comres;
}

static int bma250_get_tap_quiet(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_QUIET_DURN);
	*status = data;

	return comres;
}

static int bma250_set_tap_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_THRES, threshold);
	comres = bma250_smbus_write_byte(client, BMA250_TAP_THRES__REG, &data);

	return comres;
}

static int bma250_get_tap_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_THRES);
	*status = data;

	return comres;
}

static int bma250_set_tap_samp(struct i2c_client *client, unsigned char samp)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_SAMPLES__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_TAP_SAMPLES, samp);
	comres = bma250_smbus_write_byte(client, BMA250_TAP_SAMPLES__REG,
			&data);

	return comres;
}

static int bma250_get_tap_samp(struct i2c_client *client, unsigned char *status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_TAP_THRES_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_TAP_SAMPLES);
	*status = data;

	return comres;
}

static int bma250_set_orient_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_MODE__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_MODE, mode);
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_MODE__REG,
			&data);

	return comres;
}

static int bma250_get_orient_mode(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_MODE);
	*status = data;

	return comres;
}

static int bma250_set_orient_blocking(struct i2c_client *client, unsigned char
		samp)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_BLOCK__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_BLOCK, samp);
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_BLOCK__REG,
			&data);

	return comres;
}

static int bma250_get_orient_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_BLOCK);
	*status = data;

	return comres;
}

static int bma250_set_orient_hyst(struct i2c_client *client, unsigned char
		orienthyst)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_HYST__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_ORIENT_HYST, orienthyst);
	comres = bma250_smbus_write_byte(client, BMA250_ORIENT_HYST__REG,
			&data);

	return comres;
}

static int bma250_get_orient_hyst(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_ORIENT_PARAM_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_ORIENT_HYST);
	*status = data;

	return comres;
}
static int bma250_set_theta_blocking(struct i2c_client *client, unsigned char
		thetablk)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_BLOCK__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_THETA_BLOCK, thetablk);
	comres = bma250_smbus_write_byte(client, BMA250_THETA_BLOCK__REG,
			&data);

	return comres;
}

static int bma250_get_theta_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_BLOCK_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_THETA_BLOCK);
	*status = data;

	return comres;
}

static int bma250_set_theta_flat(struct i2c_client *client, unsigned char
		thetaflat)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_FLAT__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_THETA_FLAT, thetaflat);
	comres = bma250_smbus_write_byte(client, BMA250_THETA_FLAT__REG, &data);

	return comres;
}

static int bma250_get_theta_flat(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_THETA_FLAT_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_THETA_FLAT);
	*status = data;

	return comres;
}

static int bma250_set_flat_hold_time(struct i2c_client *client, unsigned char
		holdtime)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_FLAT_HOLD_TIME__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_FLAT_HOLD_TIME, holdtime);
	comres = bma250_smbus_write_byte(client, BMA250_FLAT_HOLD_TIME__REG,
			&data);

	return comres;
}

static int bma250_get_flat_hold_time(struct i2c_client *client, unsigned char
		*holdtime)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_FLAT_HOLD_TIME_REG,
			&data);
	data  = BMA250_GET_BITSLICE(data, BMA250_FLAT_HOLD_TIME);
	*holdtime = data ;

	return comres;
}

static int bma250_write_reg(struct i2c_client *client, unsigned char addr,
		unsigned char *data)
{
	int comres = 0;
	comres = bma250_smbus_write_byte(client, addr, data);

	return comres;
}


static int bma250_set_offset_target_x(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
			BMA250_COMP_TARGET_OFFSET_X__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_X,
			offsettarget);
	comres = bma250_smbus_write_byte(client,
			BMA250_COMP_TARGET_OFFSET_X__REG, &data);

	return comres;
}

static int bma250_get_offset_target_x(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0 ;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_OFFSET_PARAMS_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_X);
	*offsettarget = data;

	return comres;
}

static int bma250_set_offset_target_y(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
			BMA250_COMP_TARGET_OFFSET_Y__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Y,
			offsettarget);
	comres = bma250_smbus_write_byte(client,
			BMA250_COMP_TARGET_OFFSET_Y__REG, &data);

	return comres;
}

static int bma250_get_offset_target_y(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_OFFSET_PARAMS_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Y);
	*offsettarget = data;

	return comres;
}

static int bma250_set_offset_target_z(struct i2c_client *client, unsigned char
		offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
			BMA250_COMP_TARGET_OFFSET_Z__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Z,
			offsettarget);
	comres = bma250_smbus_write_byte(client,
			BMA250_COMP_TARGET_OFFSET_Z__REG, &data);

	return comres;
}

static int bma250_get_offset_target_z(struct i2c_client *client, unsigned char
		*offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_OFFSET_PARAMS_REG,
			&data);
	data = BMA250_GET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Z);
	*offsettarget = data;

	return comres;
}

static int bma250_get_cal_ready(struct i2c_client *client, unsigned char
		*calrdy)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_OFFSET_CTRL_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_FAST_COMP_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma250_set_cal_trigger(struct i2c_client *client, unsigned char
		caltrigger)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_EN_FAST_COMP__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_EN_FAST_COMP, caltrigger);
	comres = bma250_smbus_write_byte(client, BMA250_EN_FAST_COMP__REG,
			&data);

	return comres;
}

static int bma250_set_selftest_st(struct i2c_client *client, unsigned char
		selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_EN_SELF_TEST__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_EN_SELF_TEST, selftest);
	comres = bma250_smbus_write_byte(client, BMA250_EN_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma250_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client, BMA250_NEG_SELF_TEST__REG,
			&data);
	data = BMA250_SET_BITSLICE(data, BMA250_NEG_SELF_TEST, stn);
	comres = bma250_smbus_write_byte(client, BMA250_NEG_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma250_set_ee_w(struct i2c_client *client, unsigned char eew)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
			BMA250_UNLOCK_EE_WRITE_SETTING__REG, &data);
	data = BMA250_SET_BITSLICE(data, BMA250_UNLOCK_EE_WRITE_SETTING, eew);
	comres = bma250_smbus_write_byte(client,
			BMA250_UNLOCK_EE_WRITE_SETTING__REG, &data);
	return comres;
}

static int bma250_set_ee_prog_trig(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data;
	unsigned char eeprog;
	eeprog = 0x01;

	comres = bma250_smbus_read_byte(client,
			BMA250_START_EE_WRITE_SETTING__REG, &data);
	data = BMA250_SET_BITSLICE(data,
				BMA250_START_EE_WRITE_SETTING, eeprog);
	comres = bma250_smbus_write_byte(client,
			BMA250_START_EE_WRITE_SETTING__REG, &data);
	return comres;
}

static int bma250_get_eeprom_writing_status(struct i2c_client *client,
							unsigned char *eewrite)
{
	int comres = 0;
	unsigned char data;

	comres = bma250_smbus_read_byte(client,
				BMA250_EEPROM_CTRL_REG, &data);
	data = BMA250_GET_BITSLICE(data, BMA250_EE_WRITE_SETTING_S);
	*eewrite = data;

	return comres;
}

static int bma250_read_accel_x(struct i2c_client *client, short *a_x)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_X_LSB__REG,
			data, 2);
	*a_x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB) |
		(BMA250_GET_BITSLICE(data[1],
				     BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
	*a_x = *a_x <<
		(sizeof(short)*8-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));
	*a_x = *a_x >>
		(sizeof(short)*8-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));

	return comres;
}
static int bma250_read_accel_y(struct i2c_client *client, short *a_y)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_Y_LSB__REG,
			data, 2);
	*a_y = BMA250_GET_BITSLICE(data[0], BMA250_ACC_Y_LSB) |
		(BMA250_GET_BITSLICE(data[1],
				     BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
	*a_y = *a_y <<
		(sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));
	*a_y = *a_y >>
		(sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));

	return comres;
}

static int bma250_read_accel_z(struct i2c_client *client, short *a_z)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client, BMA250_ACC_Z_LSB__REG,
			data, 2);
	*a_z = BMA250_GET_BITSLICE(data[0], BMA250_ACC_Z_LSB) |
		BMA250_GET_BITSLICE(data[1],
				BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN;
	*a_z = *a_z <<
		(sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));
	*a_z = *a_z >>
		(sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));

	return comres;
}


static int bma250_read_accel_xyz(struct i2c_client *client,
		struct bma250acc *acc)
{
	int comres;
	unsigned char data[6];

	comres = bma250_smbus_read_byte_block(client,
			BMA250_ACC_X_LSB__REG, data, 6);

	acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
		|(BMA250_GET_BITSLICE(data[1],
				BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
	acc->x = acc->x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
				+ BMA250_ACC_X_MSB__LEN));
	acc->x = acc->x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
				+ BMA250_ACC_X_MSB__LEN));
	acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
		| (BMA250_GET_BITSLICE(data[3],
				BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
	acc->y = acc->y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
				+ BMA250_ACC_Y_MSB__LEN));
	acc->y = acc->y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
				+ BMA250_ACC_Y_MSB__LEN));

	acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
		| (BMA250_GET_BITSLICE(data[5],
				BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN);
	acc->z = acc->z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
				+ BMA250_ACC_Z_MSB__LEN));
	acc->z = acc->z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
				+ BMA250_ACC_Z_MSB__LEN));


	return comres;
}

#ifdef CONFIG_BMA250_WAKE_OPTIONS

static DEFINE_MUTEX(pwrlock);
static DEFINE_MUTEX(picklock);
static struct input_dev * flick2wake_pwrdev;

extern void flick2wake_setdev(struct input_dev * input_device) {
        flick2wake_pwrdev = input_device;
        return;
}
EXPORT_SYMBOL(flick2wake_setdev);

static void flick2wake_presspwr(struct work_struct * flick2wake_presspwr_work) {
	if ( touchscreen_is_on()==0 && power_key_check_in_pocket(0) ) return; // don't wake if in pocket

	if (!mutex_trylock(&pwrlock))
	    return;

	vibrate(5 * 5);

	printk("sending event KEY_POWER 1\n");
	input_event(flick2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_sync(flick2wake_pwrdev);
	msleep(100);
	input_event(flick2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_sync(flick2wake_pwrdev);
	msleep(100);
	// to make sure it gets back to Normal mode upon power off too, set suspended = 1
//	suspended = 1;
	mutex_unlock(&pwrlock);
	return;
}
static DECLARE_WORK(flick2wake_presspwr_work, flick2wake_presspwr);

void flick2wake_pwrtrigger(void) {
	schedule_work(&flick2wake_presspwr_work);
	return;
}


// flick stuff
static unsigned int LAST_VER_T = 0;
static unsigned int VER_T = 0;
static unsigned int START_T = 0;
static unsigned int LAST_FLICK_T = 0;

static s16 last_z;
static s16 last_y;

// used to calculate if enough time passed between last Power Off, and next possible power on detection from IRQ wake
static unsigned int LAST_SLEEP_TRIGGER_T = 0;

static void flick_wake_detection_snap(s16 data_x, s16 data_y, s16 data_z)
{
	if (PICK_WAKE_ENABLED == 0 && touchscreen_is_on()==0) return;

	if (touchscreen_is_on()==0 && ( jiffies - LAST_SLEEP_TRIGGER_T < ((1+FLICK_WAKE_MIN_SLEEP_TIME)*80) ) ) return;

	if (

	    (FLICK_WAKE_SENSITIVITY == 0 && ((
		(data_z > 270 || data_z < - 270 || (last_z > 260 || last_z < -260) ) &&
		(data_y < -75 || last_y < -75)) && data_x > - 380 && data_x < 380))
	    ||
	    (FLICK_WAKE_SENSITIVITY == 1 && ((
		(data_z > 250 || data_z < - 250 || (last_z > 250 || last_z < -250) ) &&
		(data_y < -55 || last_y < -55)) && data_x > - 380 && data_x < 380))
	) { // Y phone's standing or laying (0 degree mean laying),
//X phone is rotated on the plane of screen or not (0 degree angle means not rotated)

// z overfloated will happen on snap, so check for values below -270
// also y overfloated will happen too, so check for values below -75
// x can be overloaded too, check for wide range
// checking the previous y and z overfloat is very accurate to detect snap

		VER_T = jiffies;
		if (LAST_VER_T == 0) {
		} else
		if ((VER_T - LAST_VER_T) < 55) {
			printk("BMA - =============== 3 FLICK SNAP DONE - POWER OFF =====================\n");
			LAST_FLICK_T = 0;
			LAST_VER_T = 0;
			LAST_SLEEP_TRIGGER_T = jiffies;
			flick2wake_pwrtrigger();
		} else {
			printk("BMA - == TIMED OUT: RESET ==\n");
			LAST_FLICK_T = 0;
			LAST_VER_T = 0;
			START_T = 0;
		}
	} else if (data_y > 110 && data_y < 260 && data_x > - 110 && data_x < 110) {
		VER_T = jiffies;
		{
			printk("BMA - =============== 1 FLICK POSSIBLE ================\n");
			LAST_VER_T = VER_T;
			START_T = jiffies;
		}
	}
	last_z = data_z;
	last_y = data_y;
}

// used to react on snap in sleep mode, through IRQ work calling this
static void flick_wake_detection_snap_irq(s16 data_x, s16 data_y, s16 data_z)
{
	if (jiffies - LAST_SLEEP_TRIGGER_T < ((1+FLICK_WAKE_MIN_SLEEP_TIME)*80) ) return;
	if (touchscreen_is_on()==1) return;
	if (1) {
			printk("BMA - =============== 3 FLICK SNAP DONE - POWER ON =====================\n");
			flick2wake_pwrtrigger();
	}
}


// pick stuff
static unsigned int START_LAYING_T = 0;
static unsigned int LAST_LAYING_T = 0;
static unsigned int LAYING_T = 0;
static int WAS_LAYING = 0;
static int MOVED = 1;
static unsigned int MOVE_STARTED = 0;

static unsigned int MIN_STILL_TIME_FOR_POWER_ON = 10;
static unsigned int MIN_STILL_TIME_FOR_LAYING = 100;
static unsigned int MAX_TIME_TO_WAIT_FOR_FACE_ALIGNMENT = 200;


static int break_pick2wake_count = 0;

static void pick2wake_count(struct work_struct * pick2wake_count_work) {
	unsigned int time_count = 0;
	unsigned int calc_time = 0;
	printk("BMA pick2wake_count - check ts on and pwp...\n");

	if ( touchscreen_is_on()==0 && power_key_check_in_pocket(0) ) return; // don't wake if in pocket

	printk("BMA pick2wake_count\n");
	if (!mutex_trylock(&picklock))
	    return;
	printk("BMA pick2wake_count inside lock\n");

	time_count = jiffies;
	while (1)
	{
		calc_time = jiffies - time_count;
		printk("BMA counted time so far: %d\n",calc_time);
		if (break_pick2wake_count == 1)
		{
			printk("[BMA] breaking count work...\n");
			break_pick2wake_count = 0;
			mutex_unlock(&picklock);
			return;
		}
		if (calc_time > MIN_STILL_TIME_FOR_POWER_ON)
		{
			printk("[BMA] counted to time!\n");
			break;
		}
		msleep(3);
	}
	vibrate(3 * 5);

	printk("BMA - pick2wake_count sending event KEY_POWER 1\n");
	input_event(flick2wake_pwrdev, EV_KEY, KEY_POWER, 1);
	input_sync(flick2wake_pwrdev);
	msleep(100);
	input_event(flick2wake_pwrdev, EV_KEY, KEY_POWER, 0);
	input_sync(flick2wake_pwrdev);
	msleep(100);
	// to make sure it gets back to Normal mode upon power off too, set suspended = 1
//	suspended = 1;
	mutex_unlock(&picklock);
	return;
}
static DECLARE_WORK(pick2wake_count_work, pick2wake_count);

void pick2wake_count_trigger(void) {
	schedule_work(&pick2wake_count_work);
	return;
}

static int is_laying(s16 data_x, s16 data_y, s16 data_z) {
	if (data_x > -44 && data_x < 44 && data_y > -44 && data_y < 44 && ( (data_z > -260 && data_z < -250) || (data_z < 260 && data_z > 250) )) {
		return 1;
	}
	return 0;
}


static s16 last_x = 0, last_y = 0, last_z = 0;
static s16 max_delta = 10;
static unsigned int still_time = 0;
static unsigned int last_still_time_jiffies = 0;

static int is_around(s16 old, s16 new, s16 max_delta) {
	s16 diff = old - new;
	if (diff < 0) diff = diff * -1;
	if (diff <= max_delta) return 1;
	return 0;
}

static unsigned int calculate_still_time(s16 data_x, s16 data_y, s16 data_z) {
	unsigned int curr_jiffies = jiffies;
	int is_still = 0;
	if (still_time == 0) {
		last_still_time_jiffies = jiffies;
		still_time = 1;
		last_x = data_x;
		last_y = data_y;
		last_z = data_z;
		return still_time;
	}
	is_still = (is_around(last_x, data_x, max_delta) && is_around(last_y, data_y, max_delta) && is_around(last_z, data_z, max_delta));
	if (is_still == 0) {
		still_time = 0;
		last_still_time_jiffies = 0;
		return still_time;
	} else {
		still_time += curr_jiffies - last_still_time_jiffies;
		last_still_time_jiffies = curr_jiffies;
		return still_time;
	}
}


static int is_face_aligned(s16 data_x, s16 data_y, s16 data_z) {
	if (data_x > -120 && data_x < 120 && data_y > 60 && data_y < 270 && data_z < 270 && data_z > 0 ) {
		return 1;
	}
	return 0;
}


static int pick_wake_triggered = 1;

#if 0
static unsigned int LAST_PICK_INTERRUPT_TIME = 0;
static unsigned int STARTING_AFTER_LAYING_AROUND = 0;
static unsigned int FACE_ALIGN_COUNTING = 0;

static void pick_wake_detection(struct bma250_data *bma250, s16 data_x, s16 data_y, s16 data_z)
{
	unsigned int calc_diff = 0;
	unsigned int still_time = 0;

	if (LAST_PICK_INTERRUPT_TIME == 0) {
		LAST_PICK_INTERRUPT_TIME = jiffies;
		return;
	}

	if (touchscreen_is_on()==1) return;


	if (jiffies - LAST_PICK_INTERRUPT_TIME > MIN_STILL_TIME_FOR_LAYING) {
		if (STARTING_AFTER_LAYING_AROUND == 0) {
			STARTING_AFTER_LAYING_AROUND = jiffies;
		}
		printk("BMA PICK - jiffies - LAST_PICK_INTERRUPT_TIME > MIN_STILL_TIME_FOR_LAYING)\n");
		if (is_face_aligned(data_x, data_y, data_z)) {
			still_time = calculate_still_time(data_x, data_y, data_z);
			if (still_time > 0 && FACE_ALIGN_COUNTING == 0)
			{
				printk("BMA PICK - FACE ALIGN - starting count\n");
				FACE_ALIGN_COUNTING = 1;
				break_pick2wake_count = 0;
				pick2wake_count_trigger();
			} else 
			{
				printk("BMA PICK - FACE NEWLY ALIGN - restarting count\n");
				FACE_ALIGN_COUNTING = 1;
				break_pick2wake_count = 1;
				msleep(5);
				break_pick2wake_count = 1;
				pick2wake_count_trigger();
			}
		} else {
			printk("BMA PICK - NOT ALIGNED TO FACE\n");
			FACE_ALIGN_COUNTING = 0;
			break_pick2wake_count = 1;
		}
	}
	if (FACE_ALIGN_COUNTING == 0) {
		calc_diff = jiffies - STARTING_AFTER_LAYING_AROUND;
		printk("BMA PICK - jiffies - STARTING_AFTER_LAYING_AROUND = %d, > %d ,\n",calc_diff, MAX_TIME_TO_WAIT_FOR_FACE_ALIGNMENT);
		if ( STARTING_AFTER_LAYING_AROUND == 0 || (calc_diff > MAX_TIME_TO_WAIT_FOR_FACE_ALIGNMENT) ) 
		{
			printk("BMA PICK - LAST_PICK_INTERRUPT_TIME = jiffies,\n");
			LAST_PICK_INTERRUPT_TIME = jiffies;
			STARTING_AFTER_LAYING_AROUND = 0;
		}
	}
}
#endif

static void pick_wake_detection_non_interrupt(struct bma250_data *bma250, s16 data_x, s16 data_y, s16 data_z)
{
	unsigned int still_time_detected = 0;
	if (touchscreen_is_on()==1) {
		// screen on, finish detection
		START_LAYING_T = 0;
		WAS_LAYING = 0;
		LAST_LAYING_T = 0;
		return;
	}

	LAYING_T = jiffies;
	if (WAS_LAYING) {
		if (MOVED == 0 && is_laying(data_x, data_y, data_z)) {
			LAST_LAYING_T = LAYING_T;
			//printk("BMA PICK - WAS_LAYING = 1, MOVED 0, STILL LAYING\n");
		} else {
			if (MOVED == 0) {
			//	printk("BMA PICK - WAS_LAYING = 1, MOVED -> 1, MOVE_STARTED\n");
				MOVED = 1;
				MOVE_STARTED = LAYING_T;
			} else {
				if (LAYING_T - MOVE_STARTED > MAX_TIME_TO_WAIT_FOR_FACE_ALIGNMENT) {
					// timeout
			//		printk("BMA PICK - WAS_LAYING = 1, MOVE = 1, BUT TIMEOUT === RESET\n");
					START_LAYING_T = 0;
					WAS_LAYING = 0;
					LAST_LAYING_T = 0;
					MOVED = 0;
					return;
				}
				if (is_face_aligned(data_x, data_y, data_z)) {
					still_time_detected = calculate_still_time(data_x, data_y, data_z);
			//		printk("BMA PICK - WAS_LAYING = 1, MOVED = 1, FACE ALIGN, CALC STILL TIME so far %d \n", still_time_detected);
					if (MIN_STILL_TIME_FOR_POWER_ON < still_time_detected) {
						if (pick_wake_triggered == 1) {
							// already waking, don't power on a second time
						} else
						if (touchscreen_is_on()==0) {
			//				printk("BMA PICK - WAS_LAYING = 1, MOVE = 1, FACE ALIGNED, ============= POWER_ON!\n");
							pick_wake_triggered = 1;
							flick2wake_pwrtrigger();
						}
					}
				} else {
			//		printk("BMA PICK - NOT FACE ALIGNED\n");
				}
			}
		}
	} else {
		// check if it's laying
		if (is_laying(data_x, data_y, data_z)) {
			LAST_LAYING_T = LAYING_T;
			if (START_LAYING_T == 0) {
				START_LAYING_T = LAYING_T;
			}
			if (LAYING_T - START_LAYING_T > MIN_STILL_TIME_FOR_LAYING) {
			//	printk("BMA PICK - WAS_LAYING = 1\n");
				WAS_LAYING = 1;
				pick_wake_triggered = 0;
			} else {
				WAS_LAYING = 0;
				MOVED = 0;
			}
		} else {
			START_LAYING_T = 0;
			WAS_LAYING = 0;
			LAST_LAYING_T = 0;
			MOVED = 0;
		}
	}
}

#endif

static void bma250_work_func(struct work_struct *work)
{
	struct bma250_data *bma250 = container_of((struct delayed_work *)work,
			struct bma250_data, work);
	static struct bma250acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma250->delay));
	s16 data_x = 0, data_y = 0, data_z = 0;
	s16 hw_d[3] = {0};

	bma250_read_accel_xyz(bma250->bma250_client, &acc);

	hw_d[0] = acc.x + bma250->offset_buf[0];
	hw_d[1] = acc.y + bma250->offset_buf[1];
	hw_d[2] = acc.z + bma250->offset_buf[2];

	data_x = ((bma250->pdata->negate_x) ? (-hw_d[bma250->pdata->axis_map_x])
		   : (hw_d[bma250->pdata->axis_map_x]));
	data_y = ((bma250->pdata->negate_y) ? (-hw_d[bma250->pdata->axis_map_y])
		   : (hw_d[bma250->pdata->axis_map_y]));
	data_z = ((bma250->pdata->negate_z) ? (-hw_d[bma250->pdata->axis_map_z])
		   : (hw_d[bma250->pdata->axis_map_z]));

#ifdef CONFIG_BMA250_WAKE_OPTIONS
//	printk("BMA - x %d y %d z %d\n", data_x, data_y, data_z);
	if ((FLICK_SLEEP_ENABLED == 1 && touchscreen_is_on()==1)||(touchscreen_is_on()==0 && PICK_WAKE_ENABLED == 1 && FLICK_WAKE_ENABLED == 1)) {
		flick_wake_detection_snap(data_x, data_y, data_z);
	}
	if (PICK_WAKE_ENABLED) pick_wake_detection_non_interrupt(bma250,data_x, data_y, data_z);
#endif
	input_report_abs(bma250->input, ABS_X, data_x);
	input_report_abs(bma250->input, ABS_Y, data_y);
	input_report_abs(bma250->input, ABS_Z, data_z);
	input_sync(bma250->input);
	mutex_lock(&bma250->value_mutex);
	bma250->value = acc;
	mutex_unlock(&bma250->value_mutex);
	//printk("BMA schedule delayed work - work_func\n");
	schedule_delayed_work(&bma250->work, delay);
}


static ssize_t bma250_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &address, &value);

	if (bma250_write_reg(bma250->bma250_client, (unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x3d];
	int i;

	for (i = 0 ; i < 0x3d; i++) {
		bma250_smbus_read_byte(bma250->bma250_client, i, reg+i);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;


}
static ssize_t bma250_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_range(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_range(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_bandwidth(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_bandwidth(bma250->bma250_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma250_chip_layout_show(struct device *dev,	
		struct device_attribute *attr, char *buf)
{
	if (gdata == NULL) {
		E("%s: gdata == NULL\n", __func__);
		return 0;
	}

	return sprintf(buf, "chip_layout = %d\n"
			    "axis_map_x = %d, axis_map_y = %d,"
			    " axis_map_z = %d\n"
			    "negate_x = %d, negate_y = %d, negate_z = %d\n",
			    gdata->chip_layout,
			    gdata->pdata->axis_map_x, gdata->pdata->axis_map_y,
			    gdata->pdata->axis_map_z,
			    gdata->pdata->negate_x, gdata->pdata->negate_y,
			    gdata->pdata->negate_z);
}


static ssize_t bma250_get_raw_data_show(struct device *dev,	
		struct device_attribute *attr, char *buf)
{
	struct bma250_data *bma250 = gdata;
	static struct bma250acc acc;

	if (bma250 == NULL) {
		E("%s: bma250 == NULL\n", __func__);
		return 0;
	}

	bma250_read_accel_xyz(bma250->bma250_client, &acc);

	return sprintf(buf, "x = %d, y = %d, z = %d\n",
			    acc.x, acc.y, acc.z);
}


static ssize_t bma250_set_k_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250 == NULL) {
		E("%s: bma250 == NULL\n", __func__);
		return 0;
	}

	return sprintf(buf, "gs_kvalue = 0x%x\n", bma250->pdata->gs_kvalue);
}

static ssize_t bma250_set_k_value_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	int i = 0;

	if (bma250 == NULL) {
		E("%s: bma250 == NULL\n", __func__);
		return count;
	}

	D("%s: Set buf = %s\n", __func__, buf);

	bma250->pdata->gs_kvalue = simple_strtoul(buf, NULL, 10);

	D("%s: bma250->pdata->gs_kvalue = 0x%x\n", __func__,
		bma250->pdata->gs_kvalue);

	if ((bma250->pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
		bma250->offset_buf[0] = 0;
		bma250->offset_buf[1] = 0;
		bma250->offset_buf[2] = 0;
	} else {
		bma250->offset_buf[0] = (bma250->pdata->gs_kvalue >> 16) & 0xFF;
		bma250->offset_buf[1] = (bma250->pdata->gs_kvalue >>  8) & 0xFF;
		bma250->offset_buf[2] =  bma250->pdata->gs_kvalue        & 0xFF;

		for (i = 0; i < 3; i++) {
			if (bma250->offset_buf[i] > 127) {
				bma250->offset_buf[i] =
					bma250->offset_buf[i] - 256;
			}
		}
	}

	return count;
}


static ssize_t bma250_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	I("bma250_mode_store\n");
	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_mode(bma250->bma250_client, (unsigned char) data) < 0)
	    return -EINVAL;

	return count;
}


static ssize_t bma250_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
	struct bma250acc acc_value;

	mutex_lock(&bma250->value_mutex);
	acc_value = bma250->value;
	mutex_unlock(&bma250->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma250_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->delay));

}

static ssize_t bma250_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA250_MAX_DELAY)
		data = BMA250_MAX_DELAY;
	atomic_set(&bma250->delay, (unsigned int) data);

	return count;
}

static ssize_t bma250_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->enable));

}

#ifdef CONFIG_BMA250_WAKE_OPTIONS

static ssize_t bma250_setup_interrupt_for_wake(struct bma250_data *bma250);

static int MIN_SLEEP_TIME_MAX = 5;
static char MIN_SLEEP_TIME_MAX_C = '5';

static ssize_t bma250_f2w_min_sleep_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	int time_scale = 0;

	while (1) {
		if (time_scale == FLICK_WAKE_MIN_SLEEP_TIME) {
			count += sprintf(&buf[count], "[%d] ", time_scale);
		} else {
			count += sprintf(&buf[count], "%d ", time_scale);
		}
		if (++time_scale > MIN_SLEEP_TIME_MAX) {
			count += sprintf(&buf[count], "\n");
			break;
		}
	}
	return count;
}

static ssize_t bma250_f2w_min_sleep_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if (buf[0] >= '0' && buf[0] <= MIN_SLEEP_TIME_MAX_C && buf[1] == '\n')
		if (FLICK_WAKE_MIN_SLEEP_TIME != buf[0] - '0') {
			FLICK_WAKE_MIN_SLEEP_TIME = buf[0] - '0';
		}
	if (FLICK_WAKE_MIN_SLEEP_TIME<0) FLICK_WAKE_MIN_SLEEP_TIME = 0;
	if (FLICK_WAKE_MIN_SLEEP_TIME>MIN_SLEEP_TIME_MAX) FLICK_WAKE_MIN_SLEEP_TIME = MIN_SLEEP_TIME_MAX;
	printk(KERN_INFO "BMA [FLICK_WAKE_MIN_SLEEP_TIME]: %d.\n", FLICK_WAKE_MIN_SLEEP_TIME);

	return count;
}

static DEVICE_ATTR(f2w_min_sleep_time, (S_IWUSR|S_IRUGO),
	bma250_f2w_min_sleep_time_show, bma250_f2w_min_sleep_time_store);


static ssize_t bma250_f2w_sensitivity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", FLICK_WAKE_SENSITIVITY);

	return count;
}

static ssize_t bma250_f2w_sensitivity_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (FLICK_WAKE_SENSITIVITY != buf[0] - '0') {
			FLICK_WAKE_SENSITIVITY = buf[0] - '0';
		}

	printk(KERN_INFO "BMA [FLICK_WAKE_SENSITIVITY]: %d.\n", FLICK_WAKE_SENSITIVITY);
	bma250_setup_interrupt_for_wake(bma250);

	return count;
}

static DEVICE_ATTR(f2w_sensitivity, (S_IWUSR|S_IRUGO),
	bma250_f2w_sensitivity_show, bma250_f2w_sensitivity_store);

static ssize_t bma250_f2w_sensitivity_values_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	if (FLICK_WAKE_SENSITIVITY == 0) {
		count += sprintf(buf, "[0] 1\n");
	} else {
		count += sprintf(buf, "0 [1]\n");
	}

	return count;
}

static ssize_t bma250_f2w_sensitivity_values_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (FLICK_WAKE_SENSITIVITY != buf[0] - '0') {
			FLICK_WAKE_SENSITIVITY = buf[0] - '0';
		}

	printk(KERN_INFO "BMA [FLICK_WAKE_SENSITIVITY]: %d.\n", FLICK_WAKE_SENSITIVITY);
	bma250_setup_interrupt_for_wake(bma250);

	return count;
}

static DEVICE_ATTR(f2w_sensitivity_values, (S_IWUSR|S_IRUGO),
	bma250_f2w_sensitivity_values_show, bma250_f2w_sensitivity_values_store);


static ssize_t bma250_flick2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", FLICK_WAKE_ENABLED);

	return count;
}

static ssize_t bma250_flick2wake_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (FLICK_WAKE_ENABLED != buf[0] - '0') {
			FLICK_WAKE_ENABLED = buf[0] - '0';
		}

	printk(KERN_INFO "BMA [FLICK_WAKE_ENABLED]: %d.\n", FLICK_WAKE_ENABLED);

	return count;
}

static DEVICE_ATTR(flick2wake, (S_IWUSR|S_IRUGO),
	bma250_flick2wake_show, bma250_flick2wake_store);

static ssize_t bma250_flick2sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", FLICK_SLEEP_ENABLED);

	return count;
}

static ssize_t bma250_flick2sleep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (FLICK_SLEEP_ENABLED != buf[0] - '0') {
			FLICK_SLEEP_ENABLED = buf[0] - '0';
		}

	printk(KERN_INFO "BMA [FLICK_SLEEP_ENABLED]: %d.\n", FLICK_SLEEP_ENABLED);

	return count;
}

static DEVICE_ATTR(flick2sleep, (S_IWUSR|S_IRUGO),
	bma250_flick2sleep_show, bma250_flick2sleep_store);

static ssize_t bma250_pick2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", PICK_WAKE_ENABLED);

	return count;
}


static ssize_t bma250_pick2wake_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);


	if (buf[0] >= '0' && buf[0] <= '1' && buf[1] == '\n')
		if (PICK_WAKE_ENABLED != buf[0] - '0') {
			PICK_WAKE_ENABLED = buf[0] - '0';
		}

	printk(KERN_INFO "BMA [PICK_WAKE_ENABLED]: %d.\n", PICK_WAKE_ENABLED);
	bma250_setup_interrupt_for_wake(bma250);
	return count;
}

static DEVICE_ATTR(pick2wake, (S_IWUSR|S_IRUGO),
	bma250_pick2wake_show, bma250_pick2wake_store);

static unsigned int SET_ENABLE_0_CALLED_T = 0;

#endif


static void bma250_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma250->enable);
	int i = 0;
	
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	printk("BMA set_enable %d\n", enable);
	if ( ( enable == 0 ) && ( keep_sensor_on() == 1 || (FLICK_SLEEP_ENABLED == 1 && screen_on == 1) ) ) {
		printk("BMA set_enable %d skipped, wake options need it enabled\n", enable);
		SET_ENABLE_0_CALLED_T = jiffies;
		return;
	}
#endif

	mutex_lock(&bma250->enable_mutex);
	if (enable) {
		if (bma250->pdata->power_LPM)
			bma250->pdata->power_LPM(0);

		if (pre_enable == 0) {
			bma250_set_mode(bma250->bma250_client,
					BMA250_MODE_NORMAL);
			printk("BMA schedule delayed work - enable\n");
			schedule_delayed_work(&bma250->work,
				msecs_to_jiffies(atomic_read(&bma250->delay)));
			atomic_set(&bma250->enable, 1);
		}

	} else {
#ifdef CONFIG_BMA250_WAKE_OPTIONS
		if (keep_sensor_on() == 0)
#endif
		if (pre_enable == 1) {
#ifdef CONFIG_BMA250_WAKE_OPTIONS
			suspended = 1;
#endif
			bma250_set_mode(bma250->bma250_client,
					BMA250_MODE_SUSPEND);
			//printk("BMA cancel delayed work - enable\n");
			cancel_delayed_work_sync(&bma250->work);
			atomic_set(&bma250->enable, 0);
		}

#ifdef CONFIG_CIR_ALWAYS_READY
		if (bma250->pdata->power_LPM && !cir_flag)
#else

		if (bma250->pdata->power_LPM)
#endif
			if (keep_sensor_on() == 0)
				bma250->pdata->power_LPM(1);
	}

	if ((bma250->pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
		bma250->offset_buf[0] = 0;
		bma250->offset_buf[1] = 0;
		bma250->offset_buf[2] = 0;
	} else {
		bma250->offset_buf[0] = (bma250->pdata->gs_kvalue >> 16) & 0xFF;
		bma250->offset_buf[1] = (bma250->pdata->gs_kvalue >>  8) & 0xFF;
		bma250->offset_buf[2] =  bma250->pdata->gs_kvalue        & 0xFF;

		for (i = 0; i < 3; i++) {
			if (bma250->offset_buf[i] > 127) {
				bma250->offset_buf[i] =
					bma250->offset_buf[i] - 256;
			}
		}
	}

	mutex_unlock(&bma250->enable_mutex);

}
#ifdef CONFIG_BMA250_WAKE_OPTIONS
struct device *gyroscope_dev = 0;

extern void gyroscope_enable(int enable) {
	if (gyroscope_dev == 0) return; // not yet inited
	if (enable) {
		screen_on = 1;
		if (PICK_WAKE_ENABLED ==1 || FLICK_SLEEP_ENABLED == 1) {
				struct i2c_client *client = to_i2c_client(gyroscope_dev);
				struct bma250_data *bma250 = i2c_get_clientdata(client);
				bma250_set_enable(gyroscope_dev, 1);
				// set delay to 66, as htc framework, so gyro sleep works fine
				atomic_set(&bma250->delay, (unsigned int) 66);
		}
	} else {
		screen_on = 0;
		// set last sleep time, because m7-display calls enable(0) when screen is going sleeping
		// this way, Flick2Wake won't happen after a power button or touchscreen sleep event either
		LAST_SLEEP_TRIGGER_T = jiffies;
		if (PICK_WAKE_ENABLED == 0) {
			if (jiffies - SET_ENABLE_0_CALLED_T < 80) {
				// if SET_ENABLE_0_CALLED_T is very close to current jiffies, it means that it's not an in call mipi power off,
				// but a normal screen off, which should really turn off gyroscope, instead of the skipping done when the
				// screen was yet on (bma250_set_enable(0) sometimes is called BEFORE the actual Mipi power off, that's
				// why this is needed, to prevent wakelocks happening)
				printk("BMA - gyroscope_enable 0 - very close to last skipped userspace bma250_enable(0) call - disable screen this time\n");
				bma250_set_enable(gyroscope_dev, 0);
			}
		}
	}
}
#endif


static ssize_t bma250_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		bma250_set_enable(dev, data);

	return count;
}

static ssize_t bma250_enable_int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int type, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &type, &value);

	if (bma250_set_Int_Enable(bma250->bma250_client, type, value) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_int_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_Int_Mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_int_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_Int_Mode(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_slope_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_slope_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_slope_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_slope_duration(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_slope_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_slope_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_slope_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_slope_threshold(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_high_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_high_g_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_high_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_high_g_duration(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_high_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_high_g_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_high_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_high_g_threshold(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_low_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_low_g_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_low_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_low_g_duration(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_low_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_low_g_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_low_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_low_g_threshold(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_tap_threshold(bma250->bma250_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_duration(bma250->bma250_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_tap_quiet_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_quiet(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_quiet_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_quiet(bma250->bma250_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_tap_shock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_shock(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_shock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_shock(bma250->bma250_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_tap_samp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_tap_samp(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_tap_samp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_tap_samp(bma250->bma250_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_mode(bma250->bma250_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_blocking_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_blocking(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_blocking_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_blocking(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_orient_hyst_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_orient_hyst(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_hyst_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_orient_hyst(bma250->bma250_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_orient_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_theta_blocking(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_orient_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_theta_blocking(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_flat_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_theta_flat(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_flat_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_theta_flat(bma250->bma250_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}
static ssize_t bma250_flat_hold_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_flat_hold_time(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_flat_hold_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_flat_hold_time(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma250_fast_calibration_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_offset_target_x(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_fast_calibration_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_x(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 1) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

	
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("x axis fast calibration finished\n");
	return count;
}

static ssize_t bma250_fast_calibration_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_offset_target_y(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_fast_calibration_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_y(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 2) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

	
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("y axis fast calibration finished\n");
	return count;
}

static ssize_t bma250_fast_calibration_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_offset_target_z(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_fast_calibration_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma250_set_offset_target_z(bma250->bma250_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma250_set_cal_trigger(bma250->bma250_client, 3) < 0)
		return -EINVAL;

	do {
		mdelay(2);
		bma250_get_cal_ready(bma250->bma250_client, &tmp);

	
		timeout++;
		if (timeout == 50) {
			I("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("z axis fast calibration finished\n");
	return count;
}

static ssize_t bma250_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->selftest_result));

}

static ssize_t bma250_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	unsigned long data;
	unsigned char clear_value = 0;
	int error;
	short value1 = 0;
	short value2 = 0;
	short diff = 0;
	unsigned long result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;


	if (data != 1)
		return -EINVAL;
	
	if (bma250_set_range(bma250->bma250_client, 0) < 0)
		return -EINVAL;

	bma250_write_reg(bma250->bma250_client, 0x32, &clear_value);

	bma250_set_selftest_st(bma250->bma250_client, 1); 
	bma250_set_selftest_stn(bma250->bma250_client, 0); 
	mdelay(10);
	bma250_read_accel_x(bma250->bma250_client, &value1);
	bma250_set_selftest_stn(bma250->bma250_client, 1); 
	mdelay(10);
	bma250_read_accel_x(bma250->bma250_client, &value2);
	diff = value1-value2;

	I("diff x is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);

	if (abs(diff) < 204)
		result |= 1;

	bma250_set_selftest_st(bma250->bma250_client, 2); 
	bma250_set_selftest_stn(bma250->bma250_client, 0); 
	mdelay(10);
	bma250_read_accel_y(bma250->bma250_client, &value1);
	bma250_set_selftest_stn(bma250->bma250_client, 1); 
	mdelay(10);
	bma250_read_accel_y(bma250->bma250_client, &value2);
	diff = value1-value2;
	I("diff y is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);
	if (abs(diff) < 204)
		result |= 2;


	bma250_set_selftest_st(bma250->bma250_client, 3); 
	bma250_set_selftest_stn(bma250->bma250_client, 0); 
	mdelay(10);
	bma250_read_accel_z(bma250->bma250_client, &value1);
	bma250_set_selftest_stn(bma250->bma250_client, 1); 
	mdelay(10);
	bma250_read_accel_z(bma250->bma250_client, &value2);
	diff = value1-value2;

	I("diff z is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);
	if (abs(diff) < 102)
		result |= 4;

	atomic_set(&bma250->selftest_result, (unsigned int)result);

	I("self test finished\n");


	return count;
}


static ssize_t bma250_eeprom_writing_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	int timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;



	if (data != 1)
		return -EINVAL;

	
	if (bma250_set_ee_w(bma250->bma250_client, 1) < 0)
		return -EINVAL;

	I("unlock eeprom successful\n");

	if (bma250_set_ee_prog_trig(bma250->bma250_client) < 0)
		return -EINVAL;
	I("start update eeprom\n");

	do {
		mdelay(2);
		bma250_get_eeprom_writing_status(bma250->bma250_client, &tmp);

		I("wait 2ms eeprom write status is %d\n", tmp);
		timeout++;
		if (timeout == 1000) {
			I("get eeprom writing status error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	I("eeprom writing is finished\n");

	
	if (bma250_set_ee_w(bma250->bma250_client, 0) < 0)
		return -EINVAL;

	I("lock eeprom successful\n");
	return count;
}

#ifdef CONFIG_CIR_ALWAYS_READY
ssize_t bma250_setup_interrupt_for_wake(struct bma250_data *bma250) {
	int error;
	    cir_flag = 1;

	    if(bma250->pdata->power_LPM)
		bma250->pdata->power_LPM(0);

	    error = bma250_set_Int_Mode(bma250->bma250_client, 1);

	    if (PICK_WAKE_ENABLED == 1) {
		error += bma250_set_slope_duration(bma250->bma250_client, 0x01);
		error += bma250_set_slope_threshold(bma250->bma250_client, 0x07); // original 0x07
		// X and Y (1,1,0)
		error += bma250_set_Int_Enable(bma250->bma250_client, 5, 1);
		error += bma250_set_Int_Enable(bma250->bma250_client, 6, 1);
		error += bma250_set_Int_Enable(bma250->bma250_client, 7, 0);
	    } else {
//	    error += bma250_set_slope_duration(bma250->bma250_client, 0x01);
		error += bma250_set_slope_duration(bma250->bma250_client, 0x01); // set it higher (3 samples of slope), for being less motion sensitive...
//	    error +  bma250_set_slope_threshold(bma250->bma250_client, 0x07); // original 0x07
		error += bma250_set_slope_threshold(bma250->bma250_client, FLICK_WAKE_SENSITIVITY==0?140:110); // higher threshold to only detect heavy motion through interrupt, less wake
		// only Y (0,1,0)
		error += bma250_set_Int_Enable(bma250->bma250_client, 5, 0);
		error += bma250_set_Int_Enable(bma250->bma250_client, 6, 1);
		error += bma250_set_Int_Enable(bma250->bma250_client, 7, 0);
	    }
	    error += bma250_set_int1_pad_sel(bma250->bma250_client, PAD_SLOP);

	    error += bma250_set_mode(bma250->bma250_client, BMA250_MODE_NORMAL);

	    if (error)
		return error;
	    I("Always Ready enable = 1 \n");

	return error;
}

static ssize_t bma250_enable_interrupt(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &enable);
		if (error)
		return error;
#ifdef CONFIG_BMA250_WAKE_OPTIONS
///////////////////
	enable = 1;
///////////////////
#endif
	I("bma250_enable_interrupt, power_key_pressed = %d\n", power_key_pressed);
	if(enable == 1 && !power_key_pressed){ 
	    error += bma250_setup_interrupt_for_wake(bma250);
	    if (error)
		return error;
	}  else if(enable == 0){

	    error += bma250_set_Int_Enable(bma250->bma250_client, 5, 0);
	    error += bma250_set_Int_Enable(bma250->bma250_client, 6, 0);
	    error += bma250_set_Int_Enable(bma250->bma250_client, 7, 0);
	
	    power_key_pressed = 0;
	    cir_flag = 0;
	    if (error)
		return error;
	    I("Always Ready enable = 0 \n");	   	    

	} 	return count;
}
static ssize_t bma250_clear_powerkey_pressed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long powerkey_pressed;
	int error;
	error = strict_strtoul(buf, 10, &powerkey_pressed);
	if (error)
	    return error;

	if(powerkey_pressed == 1) {
	    power_key_pressed = 1;
	}
	else if(powerkey_pressed == 0) {
	    power_key_pressed = 0;
	}
	return count;
}
static ssize_t bma250_get_powerkry_pressed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_key_pressed);
}
static DEVICE_ATTR(enable_cir_interrupt, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		NULL, bma250_enable_interrupt);
static DEVICE_ATTR(clear_powerkey_flag, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_get_powerkry_pressed, bma250_clear_powerkey_pressed);
#endif

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_range_show, bma250_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_bandwidth_show, bma250_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma250_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_enable_show, bma250_enable_store);
static DEVICE_ATTR(enable_int, S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bma250_enable_int_store);
static DEVICE_ATTR(int_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_int_mode_show, bma250_int_mode_store);
static DEVICE_ATTR(slope_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_slope_duration_show, bma250_slope_duration_store);
static DEVICE_ATTR(slope_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_slope_threshold_show, bma250_slope_threshold_store);
static DEVICE_ATTR(high_g_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_high_g_duration_show, bma250_high_g_duration_store);
static DEVICE_ATTR(high_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_high_g_threshold_show, bma250_high_g_threshold_store);
static DEVICE_ATTR(low_g_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_low_g_duration_show, bma250_low_g_duration_store);
static DEVICE_ATTR(low_g_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_low_g_threshold_show, bma250_low_g_threshold_store);
static DEVICE_ATTR(tap_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_tap_duration_show, bma250_tap_duration_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_tap_threshold_show, bma250_tap_threshold_store);
static DEVICE_ATTR(tap_quiet, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_tap_quiet_show, bma250_tap_quiet_store);
static DEVICE_ATTR(tap_shock, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_tap_shock_show, bma250_tap_shock_store);
static DEVICE_ATTR(tap_samp, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_tap_samp_show, bma250_tap_samp_store);
static DEVICE_ATTR(orient_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_orient_mode_show, bma250_orient_mode_store);
static DEVICE_ATTR(orient_blocking, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_orient_blocking_show, bma250_orient_blocking_store);
static DEVICE_ATTR(orient_hyst, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_orient_hyst_show, bma250_orient_hyst_store);
static DEVICE_ATTR(orient_theta, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_orient_theta_show, bma250_orient_theta_store);
static DEVICE_ATTR(flat_theta, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_flat_theta_show, bma250_flat_theta_store);
static DEVICE_ATTR(flat_hold_time, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_flat_hold_time_show, bma250_flat_hold_time_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_register_show, bma250_register_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_fast_calibration_x_show,
		bma250_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_fast_calibration_y_show,
		bma250_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_fast_calibration_z_show,
		bma250_fast_calibration_z_store);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_selftest_show, bma250_selftest_store);
static DEVICE_ATTR(eeprom_writing, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		NULL, bma250_eeprom_writing_store);

static DEVICE_ATTR(chip_layout,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_chip_layout_show,NULL);

static DEVICE_ATTR(get_raw_data, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_get_raw_data_show, NULL);

static DEVICE_ATTR(set_k_value, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_set_k_value_show, bma250_set_k_value_store);

static struct attribute *bma250_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable_int.attr,
	&dev_attr_int_mode.attr,
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_high_g_duration.attr,
	&dev_attr_high_g_threshold.attr,
	&dev_attr_low_g_duration.attr,
	&dev_attr_low_g_threshold.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_tap_duration.attr,
	&dev_attr_tap_quiet.attr,
	&dev_attr_tap_shock.attr,
	&dev_attr_tap_samp.attr,
	&dev_attr_orient_mode.attr,
	&dev_attr_orient_blocking.attr,
	&dev_attr_orient_hyst.attr,
	&dev_attr_orient_theta.attr,
	&dev_attr_flat_theta.attr,
	&dev_attr_flat_hold_time.attr,
	&dev_attr_reg.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_selftest.attr,
	&dev_attr_eeprom_writing.attr,
	&dev_attr_chip_layout.attr,
	&dev_attr_get_raw_data.attr,
	&dev_attr_set_k_value.attr,
#ifdef CONFIG_CIR_ALWAYS_READY
	&dev_attr_enable_cir_interrupt.attr,
#endif
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	&dev_attr_flick2wake.attr,
	&dev_attr_flick2sleep.attr,
	&dev_attr_f2w_min_sleep_time.attr,
	&dev_attr_f2w_sensitivity.attr,
	&dev_attr_f2w_sensitivity_values.attr,
	&dev_attr_pick2wake.attr,
#endif
	NULL
};

static struct attribute_group bma250_attribute_group = {
	.attrs = bma250_attributes
};


#ifdef CONFIG_CIR_ALWAYS_READY
#if defined(BMA250_ENABLE_INT1) || defined(BMA250_ENABLE_INT2)
unsigned char *orient_st[] = {"upward looking portrait upright",   \
	"upward looking portrait upside-down",   \
		"upward looking landscape left",   \
		"upward looking landscape right",   \
		"downward looking portrait upright",   \
		"downward looking portrait upside-down",   \
		"downward looking landscape left",   \
		"downward looking landscape right"};

static void bma250_irq_work_func(struct work_struct *work)
{
	struct bma250_data *bma250 = container_of((struct work_struct *)work,
			struct bma250_data, irq_work);

	unsigned char status = 0;
	

#ifdef CONFIG_BMA250_WAKE_OPTIONS

	// TODO try to determine if bma250_resume() should be called
	if (FLICK_WAKE_ENABLED || PICK_WAKE_ENABLED) {
	static struct bma250acc acc;
	s16 data_x = 0, data_y = 0, data_z = 0;
	s16 hw_d[3] = {0};

	printk("BMA INTERRUPT \n");
	bma250_read_accel_xyz(bma250->bma250_client, &acc);
	hw_d[0] = acc.x + bma250->offset_buf[0];
	hw_d[1] = acc.y + bma250->offset_buf[1];
	hw_d[2] = acc.z + bma250->offset_buf[2];

	data_x = ((bma250->pdata->negate_x) ? (-hw_d[bma250->pdata->axis_map_x])
		   : (hw_d[bma250->pdata->axis_map_x]));
	data_y = ((bma250->pdata->negate_y) ? (-hw_d[bma250->pdata->axis_map_y])
		   : (hw_d[bma250->pdata->axis_map_y]));
	data_z = ((bma250->pdata->negate_z) ? (-hw_d[bma250->pdata->axis_map_z])
		   : (hw_d[bma250->pdata->axis_map_z]));
		//printk("BMA - x %d y %d z %d\n", data_x, data_y, data_z);
	if (FLICK_WAKE_ENABLED == 1 && PICK_WAKE_ENABLED == 0) flick_wake_detection_snap_irq(data_x, data_y, data_z);
//	if (PICK_WAKE_ENABLED) pick_wake_detection(bma250,data_x, data_y, data_z);
	}
#endif

	bma250_get_interruptstatus1(bma250->bma250_client, &status);
	I("bma250_irq_work_func, status = 0x%x\n", status);
#if 0
	input_report_rel(bma250->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED);
	input_report_rel(bma250->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED);
	input_report_rel(bma250->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_X_HAPPENED);
	input_report_rel(bma250->input_cir,
		SLOP_INTERRUPT,
		SLOPE_INTERRUPT_Y_HAPPENED);
	input_sync(bma250->input_cir);
#endif
	enable_irq(bma250->IRQ);

}

static irqreturn_t bma250_irq_handler(int irq, void *handle)
{


	struct bma250_data *data = handle;

	disable_irq_nosync(data->IRQ);

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->bma250_client == NULL)
		return IRQ_HANDLED;


	schedule_work(&data->irq_work);

	return IRQ_HANDLED;


}
#endif
#endif 
static int bma250_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char tempvalue;
	struct bma250_data *data;
	struct input_dev *dev;
#ifdef CONFIG_CIR_ALWAYS_READY
	struct input_dev *dev_cir;
	struct class *bma250_powerkey_class = NULL;
	struct device *bma250_powerkey_dev = NULL;
	int res;
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		I("i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	
	tempvalue = i2c_smbus_read_byte_data(client, BMA250_CHIP_ID_REG);

	if ((tempvalue == BMA250_CHIP_ID) || (tempvalue == BMA250E_CHIP_ID)) {
		I("Bosch Sensortec Device detected! CHIP ID = 0x%x. "
				"BMA250 registered I2C driver!\n", tempvalue);
	} else{
		I("Bosch Sensortec Device not found"
				"i2c error %d \n", tempvalue);
		err = -ENODEV;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma250_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	bma250_set_bandwidth(client, BMA250_BW_SET);
	bma250_set_range(client, BMA250_RANGE_SET);

	data->pdata = kmalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (data->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto pdata_kmalloc_fail;
	}

	if (client->dev.platform_data != NULL) {
		memcpy(data->pdata, client->dev.platform_data,
		       sizeof(*data->pdata));
	}

	if (data->pdata) {
		data->chip_layout = data->pdata->chip_layout;
		data->pdata->gs_kvalue = gs_kvalue;
	} else {
		data->chip_layout = 0;
		data->pdata->gs_kvalue = 0;
	}
	I("BMA250 G-sensor I2C driver: gs_kvalue = 0x%X\n",
		data->pdata->gs_kvalue);

	gdata = data;
	D("%s: layout = %d\n", __func__, gdata->chip_layout);

#if defined(BMA250_ENABLE_INT1) || defined(BMA250_ENABLE_INT2)
#endif
#ifdef BMA250_ENABLE_INT1
	
#endif


#ifdef BMA250_ENABLE_INT2
	
	bma250_set_int2_pad_sel(client, PAD_LOWG);
	bma250_set_int2_pad_sel(client, PAD_HIGHG);
	bma250_set_int2_pad_sel(client, PAD_SLOP);
	bma250_set_int2_pad_sel(client, PAD_DOUBLE_TAP);
	bma250_set_int2_pad_sel(client, PAD_SINGLE_TAP);
	bma250_set_int2_pad_sel(client, PAD_ORIENT);
	bma250_set_int2_pad_sel(client, PAD_FLAT);
#endif

#if defined(BMA250_ENABLE_INT1) || defined(BMA250_ENABLE_INT2)
	data->IRQ = client->irq;
	err = request_irq(data->IRQ, bma250_irq_handler, IRQF_TRIGGER_RISING,
			"bma250", data);
	enable_irq_wake(data->IRQ); 
	if (err)
		E("could not request irq\n");

	INIT_WORK(&data->irq_work, bma250_irq_work_func);
#endif

	INIT_DELAYED_WORK(&data->work, bma250_work_func);
	atomic_set(&data->delay, BMA250_MAX_DELAY);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev)
	    return -ENOMEM;

#ifdef CONFIG_CIR_ALWAYS_READY

	dev_cir = input_allocate_device();
	if (!dev_cir) {
	    kfree(data);
	    input_free_device(dev);
	    return -ENOMEM;
	}
#endif
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;
#ifdef CONFIG_CIR_ALWAYS_READY
	dev_cir->name = "CIRSensor";
	dev_cir->id.bustype = BUS_I2C;

	input_set_capability(dev_cir, EV_REL, SLOP_INTERRUPT);
	input_set_drvdata(dev_cir, data);
#endif
	input_set_capability(dev, EV_ABS, ORIENT_INTERRUPT);
	input_set_capability(dev, EV_ABS, FLAT_INTERRUPT);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);

	if (err < 0) {
	    goto err_register_input_device;
	}


#ifdef CONFIG_CIR_ALWAYS_READY
	err = input_register_device(dev_cir);
	if (err < 0) {
	    goto err_register_input_cir_device;
	}
#endif

	data->input = dev;
#ifdef CONFIG_CIR_ALWAYS_READY
	data->input_cir = dev_cir;
#endif

#ifdef HTC_ATTR


#ifdef CONFIG_CIR_ALWAYS_READY
	bma250_powerkey_class = class_create(THIS_MODULE, "bma250_powerkey");
	if (IS_ERR(bma250_powerkey_class)) {
		err = PTR_ERR(bma250_powerkey_class);
		bma250_powerkey_class = NULL;
		E("%s: could not allocate bma250_powerkey_class\n", __func__);
		goto err_create_class;
	}

	bma250_powerkey_dev= device_create(bma250_powerkey_class,
				NULL, 0, "%s", "bma250");
	res = device_create_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);
	if (res) {
	        E("%s, create bma250_device_create_file fail!\n", __func__);
		goto err_create_bma250_device_file;
	}

#endif
	data->g_sensor_class = class_create(THIS_MODULE, "htc_g_sensor");
	if (IS_ERR(data->g_sensor_class)) {
		err = PTR_ERR(data->g_sensor_class);
		data->g_sensor_class = NULL;
		E("%s: could not allocate data->g_sensor_class\n", __func__);
		goto err_create_class;
	}

	data->g_sensor_dev = device_create(data->g_sensor_class,
				NULL, 0, "%s", "g_sensor");
	if (unlikely(IS_ERR(data->g_sensor_dev))) {
		err = PTR_ERR(data->g_sensor_dev);
		data->g_sensor_dev = NULL;
		E("%s: could not allocate data->g_sensor_dev\n", __func__);
		goto err_create_g_sensor_device;
	}

	dev_set_drvdata(data->g_sensor_dev, data);

	err = sysfs_create_group(&data->g_sensor_dev->kobj,
			&bma250_attribute_group);
	if (err < 0)
		goto error_sysfs;
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	gyroscope_dev = data->g_sensor_dev;
#endif

#else 

	err = sysfs_create_group(&data->input->dev.kobj,
			&bma250_attribute_group);
	if (err < 0)
		goto error_sysfs;

#endif 

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bma250_early_suspend;
	data->early_suspend.resume = bma250_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	I("%s: BMA250 BOSCH driver probe successful", __func__);

	return 0;
error_sysfs:
	device_unregister(data->g_sensor_dev);
err_create_g_sensor_device:
	class_destroy(data->g_sensor_class);
#ifdef CONFIG_CIR_ALWAYS_READY
	device_remove_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);
err_create_bma250_device_file:
	class_destroy(bma250_powerkey_class);
#endif
err_create_class:
#ifdef CONFIG_CIR_ALWAYS_READY
	input_unregister_device(data->input_cir);
err_register_input_cir_device:
#endif
	input_unregister_device(data->input);
err_register_input_device:
#ifdef CONFIG_CIR_ALWAYS_READY
	input_free_device(dev_cir);
#endif
	input_free_device(dev);

pdata_kmalloc_fail:
kfree_exit:
	kfree(data);
exit:
	return err;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);

	D("%s++\n", __func__);
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	if (keep_sensor_on()) return;
	suspended = 1;
#endif

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
	    I("suspend mode\n");
	    bma250_set_mode(data->bma250_client, BMA250_MODE_SUSPEND);
	    printk("BMA cancel delayed work - early suspend\n");
	    cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->enable_mutex);
}


static void bma250_late_resume(struct early_suspend *h)
{
	struct bma250_data *data =
		container_of(h, struct bma250_data, early_suspend);

	D("%s++\n", __func__);

#ifdef CONFIG_BMA250_WAKE_OPTIONS
	if (suspended == 0) return 0;
#endif

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		bma250_set_mode(data->bma250_client, BMA250_MODE_NORMAL);
		printk("BMA schedule delayed work - late resume\n");
		schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	suspended = 0;
#endif

}
#endif

static int __devexit bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	bma250_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}
#ifdef CONFIG_PM


static int bma250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bma250_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_BMA250_WAKE_OPTIONS
	if (keep_sensor_on()) return 0;
	suspended = 1;
#endif
	D("%s++\n", __func__);



	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
	    I("suspend mode\n");
		bma250_set_mode(data->bma250_client, BMA250_MODE_SUSPEND);
		printk("BMA cancel delayed work - suspend\n");
		cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->enable_mutex);

#ifdef CONFIG_CIR_ALWAYS_READY
	
	if (data && (data->pdata->power_LPM) && !cir_flag){
#else

	if (data && (data->pdata->power_LPM)){
#endif
		if (keep_sensor_on() == 0)
		{
			I("suspend + power_LPM\n");
			data->pdata->power_LPM(1);
		}
	}

	return 0;
}

static int bma250_resume(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_BMA250_WAKE_OPTIONS
	if (suspended == 0) return 0;
#endif
	D("%s++\n", __func__);


	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {

		bma250_set_mode(data->bma250_client, BMA250_MODE_NORMAL);
		printk("BMA schedule delayed work - resume\n");
		schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
#ifdef CONFIG_BMA250_WAKE_OPTIONS
	suspended = 0;
#endif

	return 0;
}

#else

#define bma250_suspend		NULL
#define bma250_resume		NULL

#endif 

static const struct i2c_device_id bma250_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
	},
	.suspend	= bma250_suspend,
	.resume		= bma250_resume,
	.id_table	= bma250_id,
	.probe		= bma250_probe,
	.remove		= __devexit_p(bma250_remove),

};

static int __init BMA250_init(void)
{
	return i2c_add_driver(&bma250_driver);
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA250 accelerometer sensor driver");
MODULE_LICENSE("GPL");

module_init(BMA250_init);
module_exit(BMA250_exit);

