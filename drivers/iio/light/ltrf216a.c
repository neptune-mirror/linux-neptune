/* Lite-On LTR-F216A Android / Linux Driver
 *
 * Copyright (C) 2020 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
//#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/version.h>


/* LTR-F216A Registers */
#define LTRF216A_MAIN_CTRL			0x00
#define LTRF216A_LS_MEAS_RATE		0x04
#define LTRF216A_LS_GAIN			0x05
#define LTRF216A_PART_ID			0x06
#define LTRF216A_MAIN_STATUS		0x07
#define LTRF216A_CLEAR_DATA_0		0x0A
#define LTRF216A_CLEAR_DATA_1		0x0B
#define LTRF216A_CLEAR_DATA_2		0x0C
#define LTRF216A_ALS_DATA_0			0x0D
#define LTRF216A_ALS_DATA_1			0x0E
#define LTRF216A_ALS_DATA_2			0x0F
#define LTRF216A_INT_CFG			0x19
#define LTRF216A_LS_THRES_UP_0		0x21
#define LTRF216A_LS_THRES_UP_1		0x22
#define LTRF216A_LS_THRES_UP_2		0x23
#define LTRF216A_LS_THRES_LOW_0		0x24
#define LTRF216A_LS_THRES_LOW_1		0x25
#define LTRF216A_LS_THRES_LOW_2		0x26
/* LTR-F216A Registers */


/* user-layer interface function enable flags */
#define ALS_ADC_EN					1
#define LS_EN						1
#define LS_RESO_SETTINGS			1
#define LS_GAIN_SETTINGS			1
#define LS_LO_THR_SETTINGS			1
#define LS_HI_THR_SETTINGS			1
/* user-layer interface function enable flags */


/* address 0x00 */
#define SW_RESET					(1 << 4)
#define LS_EN_ACT					(1 << 1)
#define LS_EN_STDBY					(0 << 1)
/* address 0x00 */

/* address 0x04 */
#define LS_RESO_20BIT				(0 << 4)
#define LS_RESO_19BIT				(1 << 4)
#define LS_RESO_18BIT				(2 << 4)
#define LS_RESO_17BIT				(3 << 4)
#define LS_RESO_16BIT				(4 << 4)
#define LS_RESO_13BIT				(5 << 4)
#define LS_MEAS_25MS				(0 << 0)
#define LS_MEAS_50MS				(1 << 0)
#define LS_MEAS_100MS				(2 << 0)
#define LS_MEAS_200MS				(3 << 0)
#define LS_MEAS_500MS				(4 << 0)
#define LS_MEAS_1000MS				(5 << 0)
#define LS_MEAS_2000MS				(6 << 0)
/* address 0x04 */


/* address 0x05 */
#define LS_GAIN_1					(0 << 0)
#define LS_GAIN_3					(1 << 0)
#define LS_GAIN_6					(2 << 0)
#define LS_GAIN_9					(3 << 0)
#define LS_GAIN_18					(4 << 0)
/* address 0x05 */


/* address 0x19 */
#define LS_INT_SEL_ALS_CHA			(1 << 4)
#define LS_THRES_INT_MODE			(0 << 3)
#define LS_VAR_INT_MODE				(1 << 3)
#define LS_INT_DIS					(0 << 2)
#define LS_INT_EN					(1 << 2)
/* address 0x19 */


#define PON_DELAY					600
#define ALS_MIN_VAL					0
#define ALS_MAX_VAL					1048575
#define DRIVER_VERSION				"1.00"
#define PARTID						0xB1
#define I2C_RETRY 					5
#define DEVICE_NAME 				"LTRF216A"


/* TODO: define the IOCTL here */
#define LTRF216A_IOCTL_MAGIC					'U'

/* IOCTLs for LTRF216A device */
#define LTRF216A_IOCTL_ALS_ENABLE				_IOR(LTRF216A_IOCTL_MAGIC, 1, int *)
/* TODO: define the IOCTL here */


struct ltrf216a_platform_data {
	/* interrupt */
	int pfd_gpio_int_no;
};


struct ltrf216a_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	
	struct workqueue_struct *workqueue;
	//struct early_suspend early_suspend;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;

	/* Interrupt */
	int irq;
	int gpio_int_no;
};


struct ltrf216a_data *sensor_info;

uint32_t ls_reso_max[6] = {1048575, 524287, 262143, 131071, 65535, 8191};
uint8_t  als_reso_store = 0x02, als_gain_store = 0x01; //this is als default setting


/* I2C Read */
// take note --------------------------------------- 
// for i2c read, need to send the register address follwed by buffer over to register.
// There should not be a stop in between register address and buffer.  
// There should not be release of lock in between register address and buffer. 
// take note ---------------------------------------
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


static int8_t read_als_adc_value(struct ltrf216a_data *ltrf216a, uint32_t *als_value)
{
	int8_t ret = -99;  // to hold returned function status
	uint32_t clear_val, als_val;
	uint8_t buffer[6], ls_data_status = 0;
	uint32_t lux_val;

	while (!(ls_data_status & 0x08)) {
		buffer[0] = LTRF216A_MAIN_STATUS;
		ret = I2C_Read(buffer, 1);
		if (ret < 0) {
			dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

			return ret;
		}
		ls_data_status = buffer[0];
	}

	buffer[0] = LTRF216A_ALS_DATA_0;

	ret = I2C_Read(buffer, 6);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* clear value */
	clear_val = (uint32_t)buffer[0];
	clear_val |= ((uint32_t)buffer[1] << 8);
	clear_val |= ((uint32_t)buffer[2] << 16);	
	input_report_abs(ltrf216a->als_input_dev, ABS_MISC, clear_val);
	input_sync(ltrf216a->als_input_dev);
	dev_info(&ltrf216a->i2c_client->dev, "%s: clear_val = %d\n", __func__, clear_val);
	/* clear value */

	/* ALS value */
	als_val = (uint32_t)buffer[3];
	als_val |= ((uint32_t)buffer[4] << 8);
	als_val |= ((uint32_t)buffer[5] << 16);	
	input_report_abs(ltrf216a->als_input_dev, ABS_MISC, als_val);
	input_sync(ltrf216a->als_input_dev);
	dev_info(&ltrf216a->i2c_client->dev, "%s: als_val = %d\n", __func__, als_val);
	/* ALS value */

	lux_val = als_val * 45 / 100;
	input_report_abs(ltrf216a->als_input_dev, ABS_MISC, lux_val);
	input_sync(ltrf216a->als_input_dev);
	dev_info(&ltrf216a->i2c_client->dev, "%s: lux_val = %d\n", __func__, lux_val);

	*als_value = lux_val;

	return ret;	
}

static int8_t ls_resolution_readback(uint8_t *retVal, struct ltrf216a_data *ltrf216a)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTRF216A_LS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x70;

	*retVal = value >> 4;

	return ret;
}


static int8_t ls_en_readback(uint8_t *retVal, struct ltrf216a_data *ltrf216a)
{
	int8_t ret =0;
	uint8_t buffer[2], value;

	buffer[0] = LTRF216A_MAIN_CTRL;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x02;

	*retVal = value >> 1;

	return ret;
}


static int8_t ls_thr_readback(uint32_t *lothr, uint32_t *hithr, struct ltrf216a_data *ltrf216a)
{
	int8_t ret = 0;
	uint8_t buffer[6];
	uint32_t v_low, v_high, temp;

	buffer[0] = LTRF216A_LS_THRES_UP_0;
	ret = I2C_Read(buffer, 6);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	v_high = buffer[2];
	v_high <<= 16;
	temp = buffer[1];
	temp <<= 8;
	temp &= 0x0FF00;
	v_high += temp;
	v_high += buffer[0];

	v_low = buffer[5];
	v_low <<= 16;
	temp = buffer[4];
	temp <<= 8;
	temp &= 0x0FF00;
	v_low += temp;
	v_low += buffer[3];

	*lothr = v_low;
	*hithr = v_high;

	return ret;
}


#if 0
static int8_t ls_measRate_readback(uint8_t *retVal, struct ltrf216a_data *ltrf216a)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTRF216A_LS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x07;

	*retVal = value;

	return ret;
}
#endif


static int8_t ls_gain_readback(uint8_t *retVal, struct ltrf216a_data *ltrf216a)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTRF216A_LS_GAIN;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x07;

	*retVal = value;

	return ret;
}


static int8_t ls_reso_readback(uint8_t *retVal, struct ltrf216a_data *ltrf216a)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTRF216A_LS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x70;

	*retVal = value >>4;

	return ret;
}


static int8_t set_ls_en (uint8_t ls_en, struct ltrf216a_data *ltrf216a)
{
	int8_t ret;
	uint8_t buffer[2], ls_en_tmp;

	buffer[0] = LTRF216A_MAIN_CTRL;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ls_en_tmp = buffer[0];
	ls_en_tmp &= 0xFD;

	ls_en_tmp |= (ls_en << 1);

	buffer[0] = LTRF216A_MAIN_CTRL;
	buffer[1] = ls_en_tmp;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	return ret;	
}

static int8_t set_ls_threshold (uint32_t lo_thresh, uint32_t hi_thresh)
{
	int8_t ret;
	uint8_t buffer[7];
	uint32_t temp;

	buffer[0] = LTRF216A_LS_THRES_UP_0;
	buffer[1] = hi_thresh & 0xFF;
	temp = hi_thresh & 0x0FF00;
	temp >>= 8;
	buffer[2] = temp;
	temp = hi_thresh & 0xF0000;
	temp >>= 16;
	buffer[3] = temp;

	buffer[4] = lo_thresh & 0xFF;
	temp = lo_thresh & 0x0FF00;
	temp >>= 8;
	buffer[5] = temp;
	temp = lo_thresh & 0xF0000;
	temp >>= 16;
	buffer[6] = temp;

	ret = I2C_Write(buffer, 7);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set ls threshold:0x%02x"
	                                       " - 0x%02x\n", __func__, lo_thresh, hi_thresh);

	return ret;
}

// LS_MEAS_25MS				(0 << 0)
// LS_MEAS_50MS				(1 << 0)
// LS_MEAS_100MS			(2 << 0)
// LS_MEAS_200MS			(3 << 0)
// LS_MEAS_500MS			(4 << 0)
// LS_MEAS_1000MS			(5 << 0)
// LS_MEAS_2000MS			(6 << 0)
static int8_t set_ls_meas_rate(uint8_t meas_rate_val, struct ltrf216a_data *ltrf216a)
{
	int8_t ret;
	uint8_t buffer[2], meas_rate;

	buffer[0] = LTRF216A_LS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	meas_rate = buffer[0];
	meas_rate &= 0xF8;

	meas_rate |= meas_rate_val;
	buffer[0] = LTRF216A_LS_MEAS_RATE;
	buffer[1] = meas_rate;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	return ret;
}


// LS_RESO_20BIT				(0 << 4)
// LS_RESO_19BIT				(1 << 4)
// LS_RESO_18BIT				(2 << 4)
// LS_RESO_17BIT				(3 << 4)
// LS_RESO_16BIT				(4 << 4)
// LS_RESO_13BIT				(5 << 4)
static int8_t set_ls_reso(uint8_t ls_reso_val, struct ltrf216a_data *ltrf216a)
{
	int8_t ret;
	uint8_t buffer[2], lsReso;

	buffer[0] = LTRF216A_LS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	lsReso = buffer[0];
	lsReso &= 0x8F;

	lsReso |= ls_reso_val;
	buffer[0] = LTRF216A_LS_MEAS_RATE;
	buffer[1] = lsReso;

	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	return ret;
}


// LS_GAIN_1					(0 << 0)
// LS_GAIN_3					(1 << 0)
// LS_GAIN_6					(2 << 0)
// LS_GAIN_9					(3 << 0)
// LS_GAIN_18				(4 << 0)
static int8_t set_ls_gain(uint8_t gain_value, struct ltrf216a_data *ltrf216a)
{
	int8_t ret;
	uint8_t buffer[2], gain_val;

	buffer[0] = LTRF216A_LS_GAIN;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	gain_val = buffer[0];
	gain_val &= 0xF8;

	gain_val |= gain_value;
	buffer[0] = LTRF216A_LS_GAIN;
	buffer[1] = gain_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	return ret;
}


static int8_t sw_reset(struct ltrf216a_data *ltrf216a)
{
	int8_t ret;
	uint8_t buffer[2], addr00_val;

	buffer[0] = LTRF216A_MAIN_CTRL;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	addr00_val = buffer[0];

	addr00_val |= SW_RESET;
	buffer[0] = LTRF216A_MAIN_CTRL;
	buffer[1] = addr00_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	return ret;
	
}

static uint8_t num_to_substract (uint8_t input_num)
{
	uint8_t ret = 0;
	if (input_num >= 65 && input_num <= 70)
		ret = 55;
	else if (input_num >= 97 && input_num <= 102)
		ret = 87;
	else if (input_num >= 48 && input_num <= 57)
		ret = 48;

	return ret;
}

//TBD
static void ltrf216a_schedwork(struct work_struct *work)
{
	int8_t ret;
	uint8_t status, interrupt_stat = 0, new_data = 0, buffer[2];
	//uint16_t adc_value;
	struct ltrf216a_data *ltrf216a = sensor_info;

	buffer[0] = LTRF216A_MAIN_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x02;
	new_data = status & 0x01;

	if (interrupt_stat) {
	}	

	enable_irq(ltrf216a->irq);
	
}

static DECLARE_WORK(irq_workqueue, ltrf216a_schedwork);


/* IRQ Handler */
static irqreturn_t ltrf216a_irq_handler(int irq, void *data)
{
	struct ltrf216a_data *ltrf216a = data;

	/* disable an irq without waiting */
	disable_irq_nosync(ltrf216a->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}


static int ltrf216a_gpio_irq(struct ltrf216a_data *ltrf216a)
{
	int rc = 0;

	rc = gpio_request(ltrf216a->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev,"%s: GPIO %d Request Fail (%d)\n", __func__, ltrf216a->gpio_int_no, rc);
		return rc;
	}

	rc = gpio_direction_input(ltrf216a->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Set GPIO %d as Input Fail (%d)\n", __func__, ltrf216a->gpio_int_no, rc);
		goto out1;
	}

	/* Configure an active low trigger interrupt for the device */
	//rc = request_irq(ltrf216a->irq, ltrf216a_irq_handler, IRQF_TRIGGER_FALLING, DEVICE_NAME, ltrf216a);
	rc = request_irq(ltrf216a->irq, ltrf216a_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Request IRQ (%d) for GPIO %d Fail (%d)\n", __func__, ltrf216a->irq,
		        ltrf216a->gpio_int_no, rc);
		goto out1;
	}

	return rc;

out1:
	gpio_free(ltrf216a->gpio_int_no);

	return rc;
	
}

static int8_t als_enable_init (struct ltrf216a_data *ltrf216a)
{
	int8_t rc = 0;
	uint8_t lsReso, lsGain;
	uint32_t loThr, hiThr;

	/* if device not enabled, enable it */
	if (ltrf216a->als_enable_flag) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS already enabled...\n", __func__);
		return rc;
	}

	rc = set_ls_meas_rate(LS_MEAS_100MS, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
		return rc;
	}

	// to set ALS threshold here

	lsReso = als_reso_store << 4;
	lsGain = als_gain_store;

	rc = set_ls_reso(lsReso, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS resolution setup Fail...\n", __func__);
		return rc;
	}

	rc = set_ls_gain(lsGain, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS gain setup Fail...\n", __func__);
		return rc;
	}

	rc = ls_reso_readback(&lsReso, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS resolution readback Fail...\n", __func__);
		return rc;
	}

	rc = ls_thr_readback(&loThr, &hiThr, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS threshold readback Fail...\n", __func__);
		return rc;
	}

	if (loThr > ls_reso_max[lsReso]) {
		loThr = ls_reso_max[lsReso];
	}

	if (hiThr > ls_reso_max[lsReso]) {
		hiThr = ls_reso_max[lsReso];
	}

	rc = set_ls_threshold(loThr, hiThr);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s : LS thresholds setting Fail...\n", __func__);
		return rc;
	}

	rc = set_ls_en(0x01, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS enable Fail...\n", __func__);
		return rc;
	}

	ltrf216a->als_enable_flag = 1;

	return rc;
}


static int8_t als_disable(struct ltrf216a_data *ltrf216a)
{
	int8_t rc = 0;

	if (ltrf216a->als_enable_flag == 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS already disabled...\n", __func__);
		return rc;
	}

	rc = set_ls_en(0x00, ltrf216a);
	if (rc < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS disable Fail...\n", __func__);
		return rc;
	}

	ltrf216a->als_enable_flag = 0;

	return rc;
}


ssize_t als_open (struct inode *inode, struct file *file)
{
	struct ltrf216a_data *ltrf216a = sensor_info;
	int8_t rc = 0;

	if (ltrf216a->als_opened) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	ltrf216a->als_opened = 1;

	return rc;
}


ssize_t als_release (struct inode *inode, struct file *file)
{
	struct ltrf216a_data *ltrf216a = sensor_info;

	ltrf216a->als_opened = 0;

	//return 0;
	return als_disable(ltrf216a);
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int als_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTRF216A_IOCTL_ALS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			/*pr_info("%s value = %d\n", __func__, val);*/
			rc = val ? als_enable_init(ltrf216a) : als_disable(ltrf216a);

			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			rc = -EINVAL;

	}

	return rc;

}


static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = als_ioctl
#else
	.unlocked_ioctl = als_ioctl
#endif
};


static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltrf216a_ls",
	.fops = &als_fops
};


#if ALS_ADC_EN
static ssize_t als_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint32_t alsVal = 0;
	//uint8_t lsResoVal, lsMeasRateVal, lsGainVal;
	//uint8_t lsResoVal = 0, lsGainVal = 0, alsReso = 0, alsGain = 0;
	uint8_t lsResoVal = 0, lsGainVal = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;	

	ret = ls_resolution_readback(&lsResoVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s", __func__);
		return ret;
	}	

	//ret = ls_measRate_readback(&lsMeasRateVal, ltrf216a);
	//if (ret < 0) {
	//	dev_err(&ltrf216a->i2c_client->dev, "%s", __func__);
	//	return ret;
	//}

	ret = ls_gain_readback(&lsGainVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s", __func__);
		return ret;
	}		

	ret = read_als_adc_value(ltrf216a, &alsVal);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s", __func__);
		return ret;
	}

	//ret = sprintf(buf, "%d %d %d %d\n", lsResoVal, lsMeasRateVal, lsGainVal, alsVal);
	ret = sprintf(buf, "%d %d %d\n", lsResoVal, lsGainVal, alsVal);

	return ret;
}

static DEVICE_ATTR(als_adc, 0660, als_adc_show, NULL);
#endif


#if LS_EN
static ssize_t ls_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	ret = ls_en_readback(&rdback_val, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ls_en_readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t ls_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret = 0;
	int param;
	uint8_t active;
	struct ltrf216a_data *ltrf216a = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltrf216a->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	active = param & 0x01;
	

	ret = set_ls_en (active, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ls_en setup Fail...\n", __func__);
		return (-1);
	}

	active = 0;
	ret = ls_en_readback(&active, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ls_en readback Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(ls_en, 0660, ls_en_show, ls_en_store);
#endif

#if LS_LO_THR_SETTINGS
static ssize_t ls_lo_thr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint32_t loVal = 0, hiVal = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	ret = ls_thr_readback(&loVal, &hiVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS thresh readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", loVal);

	return ret;
}


static ssize_t ls_lo_thr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t n_data, lsReso;
	uint32_t param = 0;
	uint32_t loVal, hiVal;

	struct ltrf216a_data *ltrf216a = sensor_info;

	for (n_data = 0; n_data < (count - 1); n_data++) {
		if (*buf >= 48 && *buf <= 57) {
			param = ((*buf) - num_to_substract(*buf)) + (param * 10);
			buf++;
		}
	}

	ret = ls_thr_readback(&loVal, &hiVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS thresh readback Fail...\n", __func__);
		return (-1);
	}

	ret = ls_reso_readback(&lsReso, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS resolution readback Fail...\n", __func__);
		return (-1);
	}

	if (param > ls_reso_max[lsReso]) {
		param = ls_reso_max[lsReso];
	}

	loVal = param;

	ret = set_ls_threshold(loVal, hiVal);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s : LS thresholds setting Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(ls_lo_thr, 0660, ls_lo_thr_show, ls_lo_thr_store);
#endif


#if LS_HI_THR_SETTINGS
static ssize_t ls_hi_thr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint32_t loVal = 0, hiVal = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	ret = ls_thr_readback(&loVal, &hiVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS thresh readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", hiVal);

	return ret;
}


static ssize_t ls_hi_thr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t n_data, lsReso;
	uint32_t param = 0;
	uint32_t loVal, hiVal;

	struct ltrf216a_data *ltrf216a = sensor_info;

	for (n_data = 0; n_data < (count - 1); n_data++) {
		if (*buf >= 48 && *buf <= 57) {
			param = ((*buf) - num_to_substract(*buf)) + (param * 10);
			buf++;
		}
	}

	ret = ls_thr_readback(&loVal, &hiVal, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS thresh readback Fail...\n", __func__);
		return (-1);
	}

	ret = ls_reso_readback(&lsReso, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS resolution readback Fail...\n", __func__);
		return (-1);
	}

	if (param > ls_reso_max[lsReso]) {
		param = ls_reso_max[lsReso];
	}

	hiVal = param;

	ret = set_ls_threshold(loVal, hiVal);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s : LS thresholds setting Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(ls_hi_thr, 0660, ls_hi_thr_show, ls_hi_thr_store);
#endif


#if LS_RESO_SETTINGS
static ssize_t ls_reso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	ret = ls_reso_readback(&rdback_val, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS resolution readback Fail...\n", __func__);
		return (-1);
	}

	if (rdback_val == 0) {
		rdback_val = 20;
	} else if (rdback_val == 1) {
		rdback_val = 19;
	} else if (rdback_val == 2) {
		rdback_val = 18;
	} else if (rdback_val ==3) {
		rdback_val = 17;
	} else if (rdback_val ==4) {
		rdback_val = 16;
	} else if (rdback_val ==5) {
		rdback_val = 13;
	}
	
	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t ls_reso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param = 0, n_data, lsReso;
	uint32_t loThr = 0, hiThr = 0;

	struct ltrf216a_data *ltrf216a = sensor_info;

	for (n_data = 0; n_data < (count - 1); n_data++) {
		if (*buf >= 48 && *buf <= 57) {
			param = ((*buf) - num_to_substract(*buf)) + (param * 10);
			buf++;
		}
	}

	dev_dbg(&ltrf216a->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	if (param <= 13) {
		lsReso = LS_RESO_13BIT;
	} else if ((param > 13) && (param <= 16)) {
		lsReso = LS_RESO_16BIT;
	} else if (param == 17) {
		lsReso = LS_RESO_17BIT;
	} else if (param == 18) {
		lsReso = LS_RESO_18BIT;
	} else if (param == 19) {
		lsReso = LS_RESO_19BIT;
	} else if (param >= 20) {
		lsReso = LS_RESO_20BIT;
	}

	ret = set_ls_reso(lsReso, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS resolution setup Fail...\n", __func__);
		return (-1);
	}

	ret = ls_reso_readback(&lsReso, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS resolution readback Fail...\n", __func__);
		return (-1);
	}

	ret = ls_thr_readback(&loThr, &hiThr, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS threshold readback Fail...\n", __func__);
		return (-1);
	}

	if (loThr > ls_reso_max[lsReso]) {
		loThr = ls_reso_max[lsReso];
	}

	if (hiThr > ls_reso_max[lsReso]) {
		hiThr = ls_reso_max[lsReso];
	}

	ret = set_ls_threshold(loThr, hiThr);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s : LS thresholds setting Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(ls_reso, 0660, ls_reso_show, ls_reso_store);
#endif


#if LS_GAIN_SETTINGS
static ssize_t ls_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltrf216a_data *ltrf216a = sensor_info;

	ret = ls_gain_readback(&rdback_val, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS gain readback Fail...\n", __func__);
		return (-1);
	}

	if (rdback_val == 0) {
		rdback_val = 1;
	} else if (rdback_val == 1) {
		rdback_val = 3;
	} else if (rdback_val == 2) {
		rdback_val = 6;
	} else if (rdback_val == 3) {
		rdback_val = 9;
	} else if (rdback_val == 4) {
		rdback_val = 18;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t ls_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param = 0, n_data, lsGain;

	struct ltrf216a_data *ltrf216a = sensor_info;

	for (n_data = 0; n_data < (count - 1); n_data++) {
		if (*buf >= 48 && *buf <= 57) {
			param = ((*buf) - num_to_substract(*buf)) + (param * 10);
			buf++;
		}
	}

	dev_dbg(&ltrf216a->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	if (param <= 1) {
		lsGain = LS_GAIN_1;
	} else if ((param > 1) && (param <= 3)) {
		lsGain = LS_GAIN_3;
	} else if ((param > 3) && (param <= 6)) {
		lsGain = LS_GAIN_6;
	} else if ((param > 6) && (param <= 9)) {
		lsGain = LS_GAIN_9;
	} else if (param > 9) {
		lsGain = LS_GAIN_18;
	}

	ret = set_ls_gain(lsGain, ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: LS gain setup Fail...\n", __func__);
		return (-1);
	}

	return count;
	
}

static DEVICE_ATTR(ls_gain, 0660, ls_gain_show, ls_gain_store);
#endif


static void sysfs_register_device(struct i2c_client *client) 
{
	int rc = 0;

#if ALS_ADC_EN
	rc += device_create_file(&client->dev, &dev_attr_als_adc);
#endif
#if LS_EN
	rc += device_create_file(&client->dev, &dev_attr_ls_en);
#endif
#if LS_LO_THR_SETTINGS
	rc += device_create_file(&client->dev, &dev_attr_ls_lo_thr);
#endif
#if LS_HI_THR_SETTINGS
	rc += device_create_file(&client->dev, &dev_attr_ls_hi_thr);
#endif
#if LS_RESO_SETTINGS
	rc += device_create_file(&client->dev, &dev_attr_ls_reso);
#endif
#if LS_GAIN_SETTINGS
	rc += device_create_file(&client->dev, &dev_attr_ls_gain);
#endif


	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}


static int als_setup (struct ltrf216a_data *ltrf216a)
{
	int ret;

	ltrf216a->als_input_dev = input_allocate_device();
	if (!ltrf216a->als_input_dev) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltrf216a->als_input_dev->name = "ltrf216a_als";
	set_bit(EV_ABS, ltrf216a->als_input_dev->evbit);
	input_set_abs_params(ltrf216a->als_input_dev, ABS_MISC, ALS_MIN_VAL, ALS_MAX_VAL, 0, 0);

	ret = input_register_device(ltrf216a->als_input_dev);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: ALS Register Misc Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}

	return ret;

err_als_register_misc_device:
	input_unregister_device(ltrf216a->als_input_dev);
err_als_register_input_device:
	input_free_device(ltrf216a->als_input_dev);

	return ret;
}

static uint8_t _check_part_id(struct ltrf216a_data *ltrf216a)
{
	uint8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTRF216A_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Read failure :0x%02X",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}

	return 0;
}


static int ltrf216a_setup(struct ltrf216a_data *ltrf216a)
{
	int ret = 0;

	/* Reset the devices */
	ret = sw_reset(ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Soft reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltrf216a->i2c_client->dev, "%s: Reset ltrf216a device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltrf216a) < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	ret = ltrf216a_gpio_irq(ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&ltrf216a->i2c_client->dev, "%s Requested interrupt\n", __func__);

	// if neccessary, set interrupt persist here

	// turn on ALS
	ret = als_enable_init(ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s Unable to enable ALS", __func__);
		goto err_out2;
	}
	dev_info(&ltrf216a->i2c_client->dev, "%s Turned on ambient light sensor\n", __func__);

	return ret;

err_out2:
	free_irq(ltrf216a->irq, ltrf216a);
	gpio_free(ltrf216a->gpio_int_no);

err_out1:
	dev_err(&ltrf216a->i2c_client->dev, "%s Unable to setup device\n", __func__);

	return ret;
}


static int ltrf216a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltrf216a_data *ltrf216a;

	struct ltrf216a_platform_data *platdata;

	ltrf216a = kzalloc(sizeof(struct ltrf216a_data), GFP_KERNEL);
	if (!ltrf216a) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = ltrf216a;

	/* Set initial defaults */
	ltrf216a->als_enable_flag = 0;
	
	ltrf216a->i2c_client = client;
	ltrf216a->irq = client->irq;

	i2c_set_clientdata(client, ltrf216a);

	/* Parse the platform data */
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	ltrf216a->gpio_int_no = platdata->pfd_gpio_int_no;
	
	if (_check_part_id(ltrf216a) < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev,"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}
	
	/* Create the workqueue for the interrup handler */
	ltrf216a->workqueue = create_singlethread_workqueue("ltrf216a_wq");
	if (!ltrf216a->workqueue) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}
	
	ret = ltrf216a_setup(ltrf216a);
	if (ret < 0) {
		dev_err(&ltrf216a->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_ltrf216a_setup;
	}

	/* Setup the suspend and resume functionality */
	//ltrf216a->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	//ltrf216a->early_suspend.suspend = ltrf216a_early_suspend;
	//ltrf216a->early_suspend.resume = ltrf216a_late_resume;
	//register_early_suspend(&ltrf216a->early_suspend);

	/* Register the sysfs files */
	//sysfs_register_device(client);

	dev_dbg(&ltrf216a->i2c_client->dev, "%s: probe complete\n", __func__);

	return ret;

err_ltrf216a_setup:
	destroy_workqueue(ltrf216a->workqueue);
err_out:
	kfree(ltrf216a);

	return ret;
}


static const struct i2c_device_id ltrf216a_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

static struct i2c_driver ltrf216a_driver = {
	.probe = ltrf216a_probe,
	.id_table = ltrf216a_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};


static int __init ltrf216a_init(void)
{
	return i2c_add_driver(&ltrf216a_driver);
}


static void __exit ltrf216a_exit(void)
{
	i2c_del_driver(&ltrf216a_driver);
}


module_init(ltrf216a_init);
module_exit(ltrf216a_exit);

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-F216A Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
