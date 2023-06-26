// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright(c) 2023, Valve Software. All rights reserved.
 *
 */
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include "anx7580_regs.h"

#define LOG_TAG "ANX"

typedef struct {
	struct i2c_client *client;
	u8 panel_read_data[512];
	int panel_read_reg_addr, panel_read_num_bytes;
	unsigned int read_sid, read_offset, read_len;
	uint16_t dp_vtotal;
	struct delayed_work work;
} anx7580_t;

/* i2c functions ported from analogix chicago android driver */

static int anx7580_i2c_write_byte( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_err( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if ( ret < 0 ) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_write_byte_data( anx->client, (u8)(offset_addr&0x00FF), data );
		if ( ret < 0 ) {
			pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
		}
	}
	return ret;
}

static int anx7580_i2c_write_block( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 length, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_info( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_write_i2c_block_data( anx->client, (u8)(offset_addr&0x00FF), length, p_data );
		if (ret < 0) {
			pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
		}
	}
	return ret;
}

static int anx7580_i2c_write_byte4( anx7580_t * anx, u8 slave_id, u16 offset_addr, u32 data )
{
	int ret = 0;
	u8	buf[4],i;
	for ( i=0; i<4; i++ ) {
		buf[i] = (u8)(data&0x000000FF);
		data = data >> 8;
	}

	ret = anx7580_i2c_write_block( anx, slave_id, offset_addr, 4, &buf[0] );

	return ret;
}

static int anx7580_i2c_read_byte( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_info( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = ( ANX7580_SLAVE_ID_ADDR >> 1 );
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
	}else{
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_read_byte_data( anx->client, (u8)(offset_addr&0x00FF) );
		if (ret < 0) {
			pr_err(" %s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
			return ret;
		}
		*p_data = (u8)ret;
	}
	return ret;
}

static int anx7580_i2c_read_block( anx7580_t * anx, u8 slave_id, u16 offset_addr, u8 length, u8 *p_data )
{
	int ret = 0;

	if ((((slave_id & 0x0F)!=0)&&((offset_addr&0xFF00)!=0))||((offset_addr&0xF000)!=0)) {
		pr_err( "%s %s: I2C slave_id or offset_addr ERROR!! %02x %04x\n",LOG_TAG,__func__,slave_id,offset_addr );
		return -EINVAL;
	}

	anx->client->addr = (ANX7580_SLAVE_ID_ADDR >> 1);
	ret = i2c_smbus_write_byte_data( anx->client, 0x00, (slave_id | (u8)((offset_addr&0x0F00)>>8)) );
	if (ret < 0) {
		pr_err( "%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr );
	} else {
		anx->client->addr = ( ANX7580_OFFSET_ADDR >> 1 );
		ret = i2c_smbus_read_i2c_block_data( anx->client, (u8)(offset_addr&0x00FF), length, p_data );
		if (ret < 0) {
			pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,__func__, anx->client->addr);
		}
	}
	return ret;
}

static int anx7580_i2c_read_byte4( anx7580_t * anx, u8 slave_id, u16 offset_addr, u32 *p_data )
{
	int ret = 0;
	u8	buf[4],i;
	
	ret = anx7580_i2c_read_block( anx, slave_id, offset_addr, 4, &buf[0] );

	if ( ret>=0 ) {
		*p_data = 0;
		for ( i=0; i<3; i++ ){
			*p_data += (u32)(buf[3-i]);
			*p_data = *p_data<<8;
		}
		*p_data += (u32)(buf[3-i]);
	}
	
	return ret;
}

static void anx7580_panel_dcs_cmd( anx7580_t * anx, uint8_t cmd )
{
	uint32_t reg_val = DCS_WRITE_NO_PARAM | cmd << 8;
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, reg_val );
}

static void anx7580_panel_write_immediate( anx7580_t * anx, uint8_t reg, uint8_t val )
{
	uint32_t reg_val = DCS_WRITE_ONE_PARAM | reg << 8 | val << 16;
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, reg_val );
}

static void anx7580_panel_write_long( anx7580_t * anx, void * _buf, uint8_t len )
{
    const uint8_t *buf = _buf;
    for ( uint32_t v = 0, l = len; l; ) {
      int i;

      // swap byte order in string to meet anx mipi port interface requirements
      for ( i = 0; i < 4 && l; i++, l-- ) {
        v = ( v >> 8 ) | ( *( buf++ ) << 24 );
      }
      anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_PLD_DATA, v >> ( 8 * ( 4 - i ) ) );
    }
    anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( ( len ) << 8 ) | DCS_WRITE_LONG );
}

//-----------------------------------------------------------------------------
// Reset mipi output block
// Must do this after panel read to clear buffer offset
static int anx7580_mipi_reset( anx7580_t * anx )
{
  anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, PWR_UP, 0 );
  return anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, PWR_UP, 1 );
};

//Minimum time between DCS transfers.
//Right now, we are going to indiscriminately wait, since the methodology for
//timing out and verifiying transaction seems to be fraught with peril.
//
//This seems to be pretty reliable in my tests.
#define DCS_INTERWRITE_DELAY_MS 34   

static uint32_t anx7580_dcs_read( anx7580_t * anx, uint8_t reg, uint8_t len, uint8_t *pay )
{
	uint32_t data, bytes_copied = 0, returned_bytes = 0;
	uint16_t dcs_timeout_msec = 0;
	uint32_t fifo_status;

	// request len number of bytes from reg
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( len << 8 ) | DCS_SET_RETURN_SIZE );
	anx7580_i2c_write_byte4( anx, SLAVEID_MIPI_PORT0, GEN_HDR, ( uint32_t )DCS_READ | ( reg << 8 ) );

	do
	{
		anx7580_i2c_read_byte4( anx, SLAVEID_MIPI_PORT0, CMD_PKT_STATUS, &fifo_status );
		msleep(1);
		dcs_timeout_msec++;
	}
	while (( fifo_status & 0x10 ) && ( dcs_timeout_msec < DCS_INTERWRITE_DELAY_MS ) );

	if ( dcs_timeout_msec >= DCS_INTERWRITE_DELAY_MS ) {
		pr_err( "anx7580: MIPI Read timeout\n" );
		return 0;
	}

	// get data from anx7580 fifo
	do
	{
		anx7580_i2c_read_byte4(anx, SLAVEID_MIPI_PORT0, GEN_PLD_DATA, &data );
		for ( uint8_t i = 0; i < 4 && ( i + returned_bytes ) < len; i++ )
		{
			pay[bytes_copied++] = ( data >> ( i * 8 ) ) & 0xFF;
		}
		returned_bytes += 4;
		anx7580_i2c_read_byte4( anx, SLAVEID_MIPI_PORT0, CMD_PKT_STATUS, &fifo_status );
	} while ( !( fifo_status & 0x10 ) );

	// must reset mipi buffers after read to correct offset
	anx7580_mipi_reset(anx);

	return bytes_copied;
}

static void anx7580_read_chip_id(anx7580_t * anx)
{
	u8 reg_temp;
	u16 reg_int;
	
	anx7580_i2c_read_byte( anx,SLAVEID_SPI, CHIP_ID_HIGH, &reg_temp );
	reg_int = ((((u16)reg_temp)<<8)&0xFF00);
	anx7580_i2c_read_byte( anx,SLAVEID_SPI, CHIP_ID_LOW, &reg_temp );
	reg_int |= (((u16)reg_temp)&0x00FF);
	pr_info( "anx7580: Chip ID = %04X\n", reg_int );
}

static u8 anx7580_check_ocm_status(anx7580_t * anx)
{
	u8 reg_temp;
	// Check OCM status
	anx7580_i2c_read_byte( anx, SLAVEID_SERDES,SERDES_POWER_CONTROL, &reg_temp );
	if ( OCM_LOAD_DONE == ( OCM_LOAD_DONE & reg_temp ) ) {
		return 1;
    }
    return 0;
}

static void anx7580_get_dp_vtotal(anx7580_t * anx, uint16_t * dp_vtotal)
{
	uint8_t high, low;
	anx7580_i2c_read_byte( anx, SLAVEID_MAIN_LINK, ADDR_VTOTAL_DBG, &high );
	anx7580_i2c_read_byte( anx, SLAVEID_MAIN_LINK, ADDR_VTOTAL_DBG+4, &low );
	*dp_vtotal = high << 8 | low;
}

// sysfs interface 
static ssize_t bmode_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int result;
	unsigned int mode;
	anx7580_t *anx = dev_get_drvdata(device);
	if ( !anx ) {
		return 0;
	}

    result = sscanf( buf, "%d", &mode );
	if( result == 1 ) {
		if ( mode == 1 ){
			anx7580_panel_write_immediate( anx, 0x53, 0xE0 ); // high brightness mode
		} else if (mode == 2){
			anx7580_panel_write_immediate( anx, 0x53, 0x28 ); // normal mode, update brightness within 32 frame
		} else if (mode == 3){
			anx7580_panel_write_immediate( anx, 0x53, 0x20 ); // normal mode, update brightness within 32 frame
		}
	}
	return count;
}

static ssize_t bmode_show(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	uint8_t brightness_mode_reg, count, bytes_copied = 0;
	anx7580_t *anx = dev_get_drvdata( device );
	// TODO return current brightness mode
	bytes_copied = anx7580_dcs_read( anx, 0x53, 1, &brightness_mode_reg );
	if ( brightness_mode_reg == 0xE0 ) {
		count = sprintf( buf, "High Brightness\n");
	} else if ( brightness_mode_reg == 0x28 ) {
		count = sprintf( buf, "Normal Brightness 32 frames\n" );
	} else if ( brightness_mode_reg == 0x20 ) {
		count = sprintf( buf, "Normal Brightness 1 frames\n" );
	} else {
		count = sprintf( buf, "Unknown Brightness mode %x\n", brightness_mode_reg );
	}
	return count;
}

static ssize_t brightness_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int result;
	unsigned int brightness;
	uint8_t cmd[3];
	anx7580_t *anx = dev_get_drvdata( device );

	result = sscanf( buf, "%d", &brightness );

    if ( result == 1 ) {
		// limit brightness to 16 bits
		brightness &= 0xFFFF;
		cmd[0] = 0x51;
		cmd[1] = ( brightness & 0xFF00 ) >> 8;
		cmd[2] = brightness & 0xFF;

		anx7580_panel_write_long( anx, cmd, 3 );
	}
	return count;
}

static ssize_t brightness_show( struct device *device, struct device_attribute *attr, char *buf )
{
	uint8_t brightness_reg[2];
	uint8_t count, bytes_copied = 0;
	anx7580_t *anx = dev_get_drvdata( device );
	// TODO return current brightness mode
	bytes_copied = anx7580_dcs_read( anx, 0x51, 2, brightness_reg );
	if (bytes_copied) {
		count = sprintf( buf, "Brightness %d\n", (int)(brightness_reg[0] << 8 | brightness_reg[1]) );
	} else {
		count = sprintf( buf, "Brightness Unknwn\n" );
	}
	return count;
}

static ssize_t panel_read_show( struct device *device, struct device_attribute *attr, char *buf )
{
	int ret;
	ssize_t len = 0;
	unsigned int i;
	anx7580_t *anx = dev_get_drvdata( device );
	ret = anx7580_dcs_read( anx, anx->panel_read_reg_addr, anx->panel_read_num_bytes, anx->panel_read_data );
	if ( ret < 0 ) {
		pr_err( "anx7580: failed to read panel reg %u\n", anx->panel_read_reg_addr );
		return ret;
	} else {
		for (i = 0; i < anx->panel_read_num_bytes; i++) {
				ret = scnprintf( buf + len, PAGE_SIZE - len, "%x ",
								anx->panel_read_data[i] );
				if ( ret < 0 )
						return ret;
				len += ret;
		}
		buf[len - 1] = '\n';
		return len;
	}
}

static ssize_t panel_read_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
	int ret;
	anx7580_t *anx = dev_get_drvdata( device );
	ret = sscanf( buf, "%2x %d", &anx->panel_read_reg_addr, &anx->panel_read_num_bytes );
	if(ret != 2) {
		pr_err( "anx7580: panel read got wrong number of input parameters from sscanf" );
		return -EINVAL;
	}
	return count;
}

static ssize_t panel_write_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
	int ret;
	int addr, len, i, v;
	char char_data[256];
	u8 payload[256];
	anx7580_t *anx = dev_get_drvdata(device);
	// get the base parameters
	ret = sscanf( buf, "%2x %d %s", &addr, &len, char_data );
	if(ret != 3) {
		pr_err( "anx7580: panel write got wrong number of input parameters (%d) from sscanf", ret );
		return -EINVAL;
	}
	// TODO: for now we limit writes to a total of 256 bytes
	if(len >256) {
		pr_err( "anx7580: panel write exceed max length %d\n", len );
		return -EINVAL;
	}
	payload[0] = addr;
	// get the data payload
	for ( i=0; i<len; i++ ) {
	    if ( sscanf( char_data + i * 2, "%2x", &v ) != 1 ) break;
        payload[i+1] = (unsigned char)v;
	}

    if ( len == 0 )
	    anx7580_panel_dcs_cmd( anx, payload[0] );
	if ( len == 1 )
		anx7580_panel_write_immediate( anx, payload[0], payload[1] );
	else
		anx7580_panel_write_long( anx, payload, len );

	pr_info( "anx7580: wrote %s to reg %x\n", char_data, addr );

	return count;
}

static ssize_t panel_write_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static ssize_t anx7580_read_reg_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret;
	anx7580_t *anx = dev_get_drvdata(device);
	ret = sscanf( buf, "%x %x %u", &anx->read_sid, &anx->read_offset, &anx->read_len);
	if ( ret != 3 ) {
		pr_err( "anx7580: invalid reg read params\n" );
		return -EINVAL;
	}
	return count;
}

static ssize_t anx7580_read_reg_show( struct device *device, struct device_attribute *attr, char *buf )
{
	ssize_t len = 0;
	unsigned int i, ret;
	uint8_t data[256];
	anx7580_t *anx = dev_get_drvdata(device);
	anx7580_i2c_read_block(anx, anx->read_sid, anx->read_offset, anx->read_len, data);
	for (i = 0; i < anx->read_len; i++) {
		ret = scnprintf( buf + len, PAGE_SIZE - len, "%x ", data[i] );
		if ( ret < 0 )
				return ret;
		len += ret;
	}
	buf[len - 1] = '\n';
	return len;
}

static ssize_t anx7580_write_reg_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret;
	unsigned int write_byte_sid, write_byte_offset, write_byte_data;
	anx7580_t *anx = dev_get_drvdata(device);
	ret = sscanf( buf, "%x %x %x", &write_byte_sid, &write_byte_offset, &write_byte_data);
	if ( ret != 3 ) {
		pr_err( "anx7580: invalid write byte params\n" );
		return -EINVAL;
	}
	anx7580_i2c_write_byte(anx, write_byte_sid, write_byte_offset, write_byte_data);
	return count;
}

static ssize_t anx7580_write_reg_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static ssize_t anx7580_write_reg4_store( struct device *device, struct device_attribute *attr,const char *buf, size_t count )
{
    int ret, i, v;
	unsigned int sid, offset;
	char char_data[4*2];
	unsigned int reg_value = 0;
	anx7580_t *anx = dev_get_drvdata(device);

	// get the base parameters
	ret = sscanf( buf, "%x %x %s", &sid, &offset, char_data );
	if(ret != 3) {
		pr_err( "anx7580: panel write got wrong number of input parameters (%d) from sscanf", ret );
		return -EINVAL;
	}

	// get the data payload
	for ( i=0; i<4; i++ ) {
	    if ( sscanf( char_data + i * 2, "%2x", &v ) != 1 ) break;
        reg_value = ( reg_value << (8) ) | (unsigned char)v;
	}
	// pr_info("anx7580: write4 %x %x = %x\n", sid, offset, reg_value);
	anx7580_i2c_write_byte4(anx, sid, offset, reg_value);
	return count;
}

static ssize_t anx7580_write_reg4_show( struct device *device, struct device_attribute *attr, char *buf )
{
	return 0;
}

static struct device_attribute anx7580_attrs[] = {
	__ATTR(bmode, 0664, bmode_show, bmode_store),
	__ATTR(brightness, 0664, brightness_show, brightness_store),
	__ATTR(panel_read, 0664, panel_read_show, panel_read_store),
	__ATTR(panel_write, 0664, panel_write_show, panel_write_store),
	__ATTR(anx7580_read_reg, 0664, anx7580_read_reg_show, anx7580_read_reg_store),
	__ATTR(anx7580_write_reg, 0664, anx7580_write_reg_show, anx7580_write_reg_store),
	__ATTR(anx7580_write_reg4, 0664, anx7580_write_reg4_show, anx7580_write_reg4_store),
};

static int anx7580_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	anx7580_t *anx;
	unsigned int i;
	int ret;

	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK) ) {
		pr_err("%s: i2c bus does not support the anx7580\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx = kzalloc(sizeof(anx7580_t), GFP_KERNEL);
	if ( !anx ) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata( &client->dev, anx );
    anx->client = client;

    // anx7580 is powered on by bios, so we assume ocm wait period is complete
    if ( anx7580_check_ocm_status(anx) )
    {
        // OCM load done
        // Read Chip ID
        anx7580_read_chip_id(anx);
		// store current dp vtotal for future mode changes
		anx7580_get_dp_vtotal(anx, &anx->dp_vtotal);
		pr_info("anx7580: initial dp vtotal %d\n", anx->dp_vtotal);
    }

	for ( i=0; i < ARRAY_SIZE(anx7580_attrs); i++ )
	{
		if ( device_create_file(&client->dev, &anx7580_attrs[i]) )
			pr_err( "anx7580: could not create sysfs index %d\n", i );
	}

exit:
    return 0;
}

static void anx7580_remove(struct i2c_client *client)
{
    anx7580_t *data;
	unsigned int i;
	data = i2c_get_clientdata(client);
	for (i=0; i < ARRAY_SIZE(anx7580_attrs); i++)
	{
		device_remove_file(&client->dev, &anx7580_attrs[i]);
	}
    kfree(data);
}

static int anx7580_runtime_suspend(struct device *dev)
{
	return 0;
}

static int anx7580_runtime_resume(struct device *dev)
{
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(anx7580_pm_ops, anx7580_runtime_suspend,
				 anx7580_runtime_resume, NULL);

static const struct acpi_device_id anx7580_acpi_match[] = {
        { "ANX7580A", 0 },
        { }
};
MODULE_DEVICE_TABLE(acpi, anx7580_acpi_match);

static const struct i2c_device_id anx7580_i2c_ids[] = {
	{ "anx7580", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, anx7580_i2c_ids);

static const struct of_device_id anx7580_of_ids[] = {
	{ .compatible = "analogix,anx7580", },
	{ }
};
MODULE_DEVICE_TABLE(of, anx7580_of_ids);

static struct i2c_driver anx7580_i2c_driver = {
        .driver = {
                .name   = "anx7580",
                .pm     = pm_ptr(&anx7580_pm_ops),
                .of_match_table = anx7580_of_ids,
                .acpi_match_table = anx7580_acpi_match,
        },
        .probe          = anx7580_probe,
        .remove         = anx7580_remove,
        .id_table       = anx7580_i2c_ids,
};
module_i2c_driver(anx7580_i2c_driver);

MODULE_AUTHOR("Keith Mikoleit <keithm@valvesoftware.com>");
MODULE_DESCRIPTION("Analogix ANX7580 DP to MIPI Bridge Interface");
MODULE_LICENSE("GPL");