/*
    Montage Technology DS3103 - DVBS/S2 Demodulator driver

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/firmware.h>

#include "dvb_frontend.h"
#include "ds3103.h"

static int debug;
static struct dvb_frontend_ops ds3103_ops;

#define dprintk(args...) \
	do { \
		if (debug) \
			printk(args); \
	} while (0)

#define DS3103_DEFAULT_FIRMWARE "dvb-fe-ds3103.fw"

#define DS3103_ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#define DS3103_DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
 #define DS3103_DIV_ROUND_CLOSEST(x, divisor)(           \
 {                                                       \
          typeof(x) __x = x;                             \
          typeof(divisor) __d = divisor;                 \
          (((typeof(x))-1) > 0 ||                        \
          ((typeof(divisor))-1) > 0 || (__x) > 0) ?      \
                 (((__x) + ((__d) / 2)) / (__d)) :       \
                 (((__x) - ((__d) / 2)) / (__d));        \
}                                                        \
)

struct ds3103_dev {
	struct i2c_client *client;
	struct ds3103_config config;
	const struct ds3103_config *cfg;
	struct dvb_frontend fe;
	enum fe_delivery_system delivery_system;
	enum fe_status fe_status;
	u32 dvbv3_ber; /* for old DVBv3 API read_ber */
	bool warm; /* FW running */
	struct i2c_adapter *i2c_adapter;
	/* auto detect chip id to do different config */
	u8 chip_id;
	/* main mclk is calculated for M88RS6000 dynamically */
	u32 mclk_khz;
	u64 post_bit_error;
	u64 post_bit_count;
	u16 prevUCBS2;
};

static int ds3103_writereg(struct ds3103_dev *dev, int reg, int data)
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .addr = dev->cfg->i2c_addr,
		.flags = 0, .buf = buf, .len = 2 };
	int err;

	dprintk("%s: write reg 0x%02x, value 0x%02x\n", __func__, reg, data);

	err = i2c_transfer(dev->i2c_adapter, &msg, 1);
	if (err != 1) {
		printk(KERN_ERR "%s: writereg error(err == %i, reg == 0x%02x,"
			 " value == 0x%02x)\n", __func__, err, reg, data);
		return -EREMOTEIO;
	}

	return 0;
}

/* I2C write for 8k firmware load */
static int ds3103_writeFW(struct ds3103_dev *dev, int reg,
				const u8 *data, u16 len)
{
	int i, ret = -EREMOTEIO;
	struct i2c_msg msg;
	u8 *buf;

	buf = kmalloc(33, GFP_KERNEL);
	if (buf == NULL) {
		printk(KERN_ERR "Unable to kmalloc\n");
		ret = -ENOMEM;
		goto error;
	}

	*(buf) = reg;

	msg.addr = dev->cfg->i2c_addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = 33;
   
	for (i = 0; i < len; i += 32) {
		memcpy(buf + 1, data + i, 32);

		dprintk("%s: write reg 0x%02x, len = %d\n", __func__, reg, len);

		ret = i2c_transfer(dev->i2c_adapter, &msg, 1);
		if (ret != 1) {
			printk(KERN_ERR "%s: write error(err == %i, "
				"reg == 0x%02x\n", __func__, ret, reg);
			ret = -EREMOTEIO;
		}
	}

error:
	kfree(buf);
	return ret;
}

static int ds3103_readreg(struct ds3103_dev *dev, u8 reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = dev->cfg->i2c_addr,
			.flags = 0,
			.buf = b0,
			.len = 1
		}, {
			.addr = dev->cfg->i2c_addr,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};

	ret = i2c_transfer(dev->i2c_adapter, msg, 2);

	if (ret != 2) {
		printk(KERN_ERR "%s: reg=0x%x(error=%d)\n", __func__, reg, ret);
		return ret;
	}

	dprintk("%s: read reg 0x%02x, value 0x%02x\n", __func__, reg, b1[0]);

	return b1[0];
}

static int _regmap_update_bits(struct ds3103_dev *dev, unsigned int reg,
                                unsigned int mask, unsigned int val,
                                bool *change)
{
         int ret = 0;
         unsigned int tmp, orig;
 
         orig = ds3103_readreg(dev,reg);
 
         tmp = orig & ~mask;
         tmp |= val & mask;
 
         if (tmp != orig) {
                 ret = ds3103_writereg(dev, reg, tmp);
                 if (change)
                         *change = true;
         } else {
                 if (change)
                         *change = false;
               
         }
 
         return ret;
}

static int regmap_update_bits(struct ds3103_dev *dev, unsigned int reg,
                                unsigned int mask, unsigned int val)
{
         return _regmap_update_bits(dev,reg,mask,val,NULL);
}

static int ds3103_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	
	if (enable)
		ds3103_writereg(dev, 0x03, 0x12);
	else
		ds3103_writereg(dev, 0x03, 0x02);
	
	return 0;
}

/*
 * Get the demodulator AGC PWM voltage setting supplied to the tuner.
 */
int ds3103_get_agc_pwm(struct dvb_frontend *fe, u8 *_agc_pwm)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	unsigned tmp;
	tmp = ds3103_readreg(dev, 0x3f);
	*_agc_pwm = tmp;
	return 0;
}
EXPORT_SYMBOL(ds3103_get_agc_pwm);

static int ds3103_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	unsigned int utmp;
	int ret;

	*status = 0;

	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}
	switch (c->delivery_system) {
	case SYS_DVBS:
		utmp = ds3103_readreg(dev, 0xd1);
		if ((utmp & 0x07) == 0x07)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				FE_HAS_VITERBI | FE_HAS_SYNC |
				FE_HAS_LOCK;

		break;
	case SYS_DVBS2:
		utmp = ds3103_readreg(dev, 0x0d);
		if ((utmp & 0x8f) == 0x8f)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				FE_HAS_VITERBI | FE_HAS_SYNC |
				FE_HAS_LOCK;

		break;
	default:
		return 1;
	}

	if (dev->cfg->set_lock_led)
		dev->cfg->set_lock_led(fe, *status == 0 ? 0 : 1);

	dprintk("%s: status = 0x%02x\n", __func__, utmp);

	return 0;
err:
	return -1;
}

static int ds3103_set_frontend(struct dvb_frontend *fe)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, len;
	const struct ds3103_reg_val *init;
	u8 u8tmp, u8tmp1 = 0, u8tmp2 = 0; /* silence compiler warning */
	u8 buf[3];
	u16 u16tmp, divide_ratio = 0;
	u32 tuner_frequency, target_mclk;
	s32 s32tmp;
	int i;

	printk(KERN_DEBUG
		"%s:delivery_system=%d modulation=%d frequency=%u symbol_rate=%d inversion=%d pilot=%d rolloff=%d\n",
		__func__,
		c->delivery_system, c->modulation, c->frequency, c->symbol_rate,
		c->inversion, c->pilot, c->rolloff);

	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}

	/* reset */
	ret = ds3103_writereg(dev, 0x07, 0x80);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0x07, 0x00);
	if (ret)
		goto err;

	/* Disable demod clock path */
	if (dev->chip_id == RS6000_CHIP_ID) {
		ret = ds3103_writereg(dev, 0x06, 0xe0);
		if (ret)
			goto err;
	}

	/* program tuner */
	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret)
			goto err;
	}


	if (fe->ops.tuner_ops.get_frequency) {
		ret = fe->ops.tuner_ops.get_frequency(fe, &tuner_frequency);
		if (ret)
			goto err;
	} else {
		/*
		 * Use nominal target frequency as tuner driver does not provide
		 * actual frequency used. Carrier offset calculation is not
		 * valid.
		 */
		tuner_frequency = c->frequency;
	}

	/* select M88RS6000 demod main mclk and ts mclk from tuner die. */
	if (dev->chip_id == RS6000_CHIP_ID) {
		if (c->symbol_rate > 45010000)
			dev->mclk_khz = 110250;
		else
			dev->mclk_khz = 96000;

		if (c->delivery_system == SYS_DVBS)
			target_mclk = 96000;
		else
			target_mclk = 144000;
		/* Enable demod clock path */
		ret = ds3103_writereg(dev, 0x06, 0x00);
		if (ret)
			goto err;
		msleep(2);
	} else {
	   /* set M88DS3103 mclk and ts mclk. */
		dev->mclk_khz = 96000;

		switch (dev->cfg->ts_mode) {
		case DS3103_TS_SERIAL:
		case DS3103_TS_SERIAL_D7:
			target_mclk = dev->cfg->ts_clk;
			break;
		case DS3103_TS_PARALLEL:
		case DS3103_TS_CI:
			if (c->delivery_system == SYS_DVBS)
				target_mclk = 96000;
			else {
				if (c->symbol_rate < 18000000)
					target_mclk = 96000;
				else if (c->symbol_rate < 28000000)
					target_mclk = 144000;
				else
					target_mclk = 192000;
			}
			break;
		default:
			printk(KERN_DEBUG "%s: invalid ts_mode\n",__func__);
			ret = -EINVAL;
			goto err;
		}

		switch (target_mclk) {
		case 96000:
			u8tmp1 = 0x02; /* 0b10 */
			u8tmp2 = 0x01; /* 0b01 */
			break;
		case 144000:
			u8tmp1 = 0x00; /* 0b00 */
			u8tmp2 = 0x01; /* 0b01 */
			break;
		case 192000:
			u8tmp1 = 0x03; /* 0b11 */
			u8tmp2 = 0x00; /* 0b00 */
			break;
		}
		ret = regmap_update_bits(dev, 0x22, 0xc0, u8tmp1 << 6);
		if (ret)
			goto err;
		ret = regmap_update_bits(dev, 0x24, 0xc0, u8tmp2 << 6);
		if (ret)
			goto err;
	}
	
	ret = ds3103_writereg(dev, 0xb2, 0x01);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0x00, 0x01);
	if (ret)
		goto err;

	switch (c->delivery_system) {
	case SYS_DVBS:
		if (dev->chip_id == RS6000_CHIP_ID) {
			len = DS3103_ARRAY_SIZE(rs6000_dvbs_init_reg_vals);
			init = rs6000_dvbs_init_reg_vals;
		} else {
			len = DS3103_ARRAY_SIZE(ds3103_dvbs_init_reg_vals);
			init = ds3103_dvbs_init_reg_vals;
		}
		break;
	case SYS_DVBS2:
		if (dev->chip_id == RS6000_CHIP_ID) {
			len = DS3103_ARRAY_SIZE(rs6000_dvbs2_init_reg_vals);
			init = rs6000_dvbs2_init_reg_vals;
		} else {
			len = DS3103_ARRAY_SIZE(ds3103_dvbs2_init_reg_vals);
			init = ds3103_dvbs2_init_reg_vals;
		}
		break;
	default:
		printk(KERN_DEBUG "%s: invalid delivery_system\n",__func__);
		ret = -EINVAL;
		goto err;
	}

	/* program init table */
	if (c->delivery_system != dev->delivery_system) {
		for (i = 0; i < len; i++){
			ret = ds3103_writereg(dev, init[i].reg,
					 init[i].val);
			if (ret)
				goto err;
		}
	}



	switch (dev->cfg->ts_mode) {
	case DS3103_TS_SERIAL:
		u8tmp1 = 0x00;
		u8tmp = 0x06;
		break;
	case DS3103_TS_SERIAL_D7:
		u8tmp1 = 0x20;
		u8tmp = 0x06;
		break;
	case DS3103_TS_PARALLEL:
		u8tmp = 0x02;
		break;
	case DS3103_TS_CI:
		u8tmp = 0x03;
		break;
	default:
		printk(KERN_DEBUG "%s: invalid ts_mode\n",__func__);
		ret = -EINVAL;
		goto err;
	}

	if (dev->cfg->ts_clk_pol)
		u8tmp |= 0x40;

	/* TS mode */
	ret = ds3103_writereg(dev, 0xfd, u8tmp);
	if (ret)
		goto err;

	switch (dev->cfg->ts_mode) {
	case DS3103_TS_SERIAL:
	case DS3103_TS_SERIAL_D7:
		ret = regmap_update_bits(dev, 0x29, 0x20, u8tmp1);
		if (ret)
			goto err;
		u8tmp1 = 0;
		u8tmp2 = 0;
		break;
	default:
		if (dev->cfg->ts_clk) {
			divide_ratio = DS3103_DIV_ROUND_UP(target_mclk, dev->cfg->ts_clk);
			u8tmp1 = divide_ratio / 2;
			u8tmp2 = DS3103_DIV_ROUND_UP(divide_ratio, 2);
		}
	}

	printk(KERN_DEBUG
		"target_mclk=%d ts_clk=%d divide_ratio=%d\n",
		target_mclk, dev->cfg->ts_clk, divide_ratio);

	u8tmp1--;
	u8tmp2--;
	/* u8tmp1[5:2] => fe[3:0], u8tmp1[1:0] => ea[7:6] */
	u8tmp1 &= 0x3f;
	/* u8tmp2[5:0] => ea[5:0] */
	u8tmp2 &= 0x3f;
	u8tmp = ds3103_readreg(dev, 0xfe);
	u8tmp = ((u8tmp  & 0xf0) << 0) | u8tmp1 >> 2;
	ret = ds3103_writereg(dev, 0xfe, u8tmp);
	if (ret)
		goto err;

	u8tmp = ((u8tmp1 & 0x03) << 6) | u8tmp2 >> 0;
	ret = ds3103_writereg(dev, 0xea, u8tmp);
	if (ret)
		goto err;

	if (c->symbol_rate <= 3000000)
		u8tmp = 0x20;
	else if (c->symbol_rate <= 10000000)
		u8tmp = 0x10;
	else
		u8tmp = 0x06;

	ret = ds3103_writereg(dev, 0xc3, 0x08);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0xc8, u8tmp);
	if (ret)
		goto err;
	ret = ds3103_writereg(dev,  0xc4, 0x08);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0xc7, 0x00);
	if (ret)
		goto err;
	u16tmp = DS3103_DIV_ROUND_CLOSEST((c->symbol_rate / 1000) << 15, dev->mclk_khz / 2);
	buf[0] = (u16tmp >> 0) & 0xff;
	buf[1] = (u16tmp >> 8) & 0xff;
	ret = ds3103_writereg(dev, 0x61, buf[0]);
	ret |= ds3103_writereg(dev, 0x62, buf[1]);
	if (ret)
		goto err;
	ret = regmap_update_bits(dev, 0x4d, 0x02, dev->cfg->spec_inv << 1);
	if (ret)
		goto err;

	ret = regmap_update_bits(dev, 0x30, 0x10, dev->cfg->agc_inv << 4);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0x33, dev->cfg->agc);
	if (ret)
		goto err;

	printk(KERN_DEBUG "carrier offset=%d\n",
		(tuner_frequency - c->frequency));
	s32tmp = 0x10000 * (tuner_frequency - c->frequency);
	s32tmp = DS3103_DIV_ROUND_CLOSEST(s32tmp, dev->mclk_khz);
	if (s32tmp < 0)
		s32tmp += 0x10000;

	buf[0] = (s32tmp >> 0) & 0xff;
	buf[1] = (s32tmp >> 8) & 0xff;
	ret = ds3103_writereg(dev, 0x5e, buf[0]);
	ret |= ds3103_writereg(dev, 0x5f, buf[1]);
	if (ret)
		goto err;

	ret = ds3103_writereg(dev, 0x00, 0x00);
	if (ret)
		goto err;
	ret = ds3103_writereg(dev, 0xb2, 0x00);
	if (ret)
		goto err;

	dev->delivery_system = c->delivery_system;

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n", __func__,ret);
	return ret;
}


static int ds3103_init(struct dvb_frontend *fe)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	unsigned int utmp;
	const struct firmware *fw = NULL;
	u8 *fw_file;

	printk(KERN_DEBUG "Entering %s\n", __func__);
	/* set cold state by default */
	dev->warm = false;
	
	/* wake up device from sleep */
	ret = regmap_update_bits(dev, 0x08, 0x01, 0x01);
	if (ret)
		goto err;
	ret = regmap_update_bits(dev, 0x04, 0x01, 0x00);
	if (ret)
		goto err;
	ret = regmap_update_bits(dev, 0x23, 0x10, 0x00);
	if (ret)
		goto err;

	/* firmware status */
	utmp = ds3103_readreg(dev, 0xb9);

	printk("firmware=%02x\n", utmp);
	
	if (utmp)
		goto skip_fw_download;

	/* global reset, global diseqc reset, golbal fec reset */
	ret = ds3103_writereg(dev, 0x07, 0xe0);
	if (ret)
		goto err;
	ret = ds3103_writereg(dev, 0x07, 0x00);
	if (ret)
		goto err;

	/* cold state - try to download firmware */
	printk(KERN_INFO "found a '%s' in cold state\n",
		 ds3103_ops.info.name);
	if (dev->chip_id == RS6000_CHIP_ID)
		fw_file = RS6000_FIRMWARE;
	else
		fw_file = DS3103_FIRMWARE;
	/* request the firmware, this will block and timeout */
	ret = request_firmware(&fw, fw_file, dev->i2c_adapter->dev.parent);
	if (ret) {
		printk(KERN_ERR "%s: firmare file '%s' not found\n", __func__,fw_file);
		goto err;
	}
	printk(KERN_INFO "downloading firmware from file '%s'\n",
			 fw_file);
	ret = ds3103_writereg(dev, 0xb2, 0x01);
	if (ret)
		goto error_fw_release;
	dprintk("%s\n", __func__);
	dprintk("Firmware is %zu bytes (%02x %02x .. %02x %02x)\n",
			fw->size,
			fw->data[0],
			fw->data[1],
			fw->data[fw->size - 2],
			fw->data[fw->size - 1]);

	// write the entire firmware 
	ds3103_writeFW(dev, 0xb0, fw->data, fw->size);

	ret = ds3103_writereg(dev, 0xb2, 0x00);
	if (ret)
		goto error_fw_release;
	release_firmware(fw);
	fw = NULL;
	utmp = ds3103_readreg(dev, 0xb9);
	
	if (!utmp) {
		printk(KERN_DEBUG "firmware did not run\n");
		ret = -EFAULT;
		goto err;
	}
	printk(KERN_INFO "found a '%s' in warm state\n",
		 ds3103_ops.info.name);
	printk(KERN_INFO "firmware version: %X.%X\n",
		 (utmp >> 4) & 0xf, (utmp >> 0 & 0xf));
skip_fw_download:
	/* warm state */
	dev->warm = true;

	/* init stats here in order signal app which stats are supported */
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_error.len = 1;
	c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_count.len = 1;
	c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	return 0;
error_fw_release:
	release_firmware(fw);
err:
	printk(KERN_ERR "%s: failed\n", __func__);
	return ret;	
}

/* Put device to sleep */
static int ds3103_sleep(struct dvb_frontend *fe)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	int ret;
	unsigned int utmp;

	printk(KERN_DEBUG "Entering %s\n", __func__);
	
	if (dev->cfg->set_lock_led)
		dev->cfg->set_lock_led(fe, 0);

	dev->fe_status = 0;
	dev->delivery_system = SYS_UNDEFINED;

	/* TS Hi-Z */
	if (dev->chip_id == RS6000_CHIP_ID)
		utmp = 0x29;
	else
		utmp = 0x27;
		
	ret = regmap_update_bits(dev, utmp, 0x01, 0x00);
	if (ret)
		goto err;

	/* sleep */
	ret = regmap_update_bits(dev, 0x08, 0x01, 0x00);
	if (ret)
		goto err;
	ret = regmap_update_bits(dev, 0x04, 0x01, 0x01);
	if (ret)
		goto err;
	ret = regmap_update_bits(dev, 0x23, 0x10, 0x10);
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n",__func__,ret);
	return ret;
}

static int ds3103_get_frontend(struct dvb_frontend *fe)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	u8 buf[3];

	printk(KERN_DEBUG "\n");

	if (!dev->warm || !(dev->fe_status & FE_HAS_LOCK)) {
		ret = 0;
		goto err;
	}

	switch (c->delivery_system) {
	case SYS_DVBS:
		buf[0] = ds3103_readreg(dev, 0xe0);
		buf[1] = ds3103_readreg(dev, 0xe6);


		switch ((buf[0] >> 2) & 0x01) {
		case 0:
			c->inversion = INVERSION_OFF;
			break;
		case 1:
			c->inversion = INVERSION_ON;
			break;
		}

		switch ((buf[1] >> 5) & 0x07) {
		case 0:
			c->fec_inner = FEC_7_8;
			break;
		case 1:
			c->fec_inner = FEC_5_6;
			break;
		case 2:
			c->fec_inner = FEC_3_4;
			break;
		case 3:
			c->fec_inner = FEC_2_3;
			break;
		case 4:
			c->fec_inner = FEC_1_2;
			break;
		default:
			printk(KERN_DEBUG "%s: invalid fec_inner\n",__func__);
		}

		c->modulation = QPSK;

		break;
	case SYS_DVBS2:
		buf[0] = ds3103_readreg(dev, 0x7e);
		buf[1] = ds3103_readreg(dev, 0x89);

		buf[2] = ds3103_readreg(dev, 0xf2);


		switch ((buf[0] >> 0) & 0x0f) {
		case 2:
			c->fec_inner = FEC_2_5;
			break;
		case 3:
			c->fec_inner = FEC_1_2;
			break;
		case 4:
			c->fec_inner = FEC_3_5;
			break;
		case 5:
			c->fec_inner = FEC_2_3;
			break;
		case 6:
			c->fec_inner = FEC_3_4;
			break;
		case 7:
			c->fec_inner = FEC_4_5;
			break;
		case 8:
			c->fec_inner = FEC_5_6;
			break;
		case 9:
			c->fec_inner = FEC_8_9;
			break;
		case 10:
			c->fec_inner = FEC_9_10;
			break;
		default:
			printk(KERN_DEBUG "%s: invalid fec_inner\n",__func__);
		}

		switch ((buf[0] >> 5) & 0x01) {
		case 0:
			c->pilot = PILOT_OFF;
			break;
		case 1:
			c->pilot = PILOT_ON;
			break;
		}

		switch ((buf[0] >> 6) & 0x07) {
		case 0:
			c->modulation = QPSK;
			break;
		case 1:
			c->modulation = PSK_8;
			break;
		case 2:
			c->modulation = APSK_16;
			break;
		case 3:
			c->modulation = APSK_32;
			break;
		default:
			printk(KERN_DEBUG "%s: invalid modulation\n",__func__);
		}

		switch ((buf[1] >> 7) & 0x01) {
		case 0:
			c->inversion = INVERSION_OFF;
			break;
		case 1:
			c->inversion = INVERSION_ON;
			break;
		}

		switch ((buf[2] >> 0) & 0x03) {
		case 0:
			c->rolloff = ROLLOFF_35;
			break;
		case 1:
			c->rolloff = ROLLOFF_25;
			break;
		case 2:
			c->rolloff = ROLLOFF_20;
			break;
		default:
			printk(KERN_DEBUG "%s: invalid rolloff\n",__func__);
		}
		break;
	default:
		printk(KERN_DEBUG "%s: invalid delivery_system\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	buf[0] = ds3103_readreg(dev, 0x6d);
	buf[1] = ds3103_readreg(dev, 0x6e);

	c->symbol_rate = 1ull * ((buf[1] << 8) | (buf[0] << 0)) *
			dev->mclk_khz * 1000 / 0x10000;

	return 0;
err:
	printk(KERN_DEBUG "%s: failed=%d\n", __func__,ret);
	return ret;
}


/* calculate DS3103 snr value in dB */
static int ds3103_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 snr_reading, snr_value;
	u32 dvbs2_signal_reading, dvbs2_noise_reading, tmp;
	static const u16 dvbs_snr_tab[] = { /* 20 x Table (rounded up) */
		0x0000, 0x1b13, 0x2aea, 0x3627, 0x3ede, 0x45fe, 0x4c03,
		0x513a, 0x55d4, 0x59f2, 0x5dab, 0x6111, 0x6431, 0x6717,
		0x69c9, 0x6c4e, 0x6eac, 0x70e8, 0x7304, 0x7505
	};
	static const u16 dvbs2_snr_tab[] = { /* 80 x Table (rounded up) */
		0x0000, 0x0bc2, 0x12a3, 0x1785, 0x1b4e, 0x1e65, 0x2103,
		0x2347, 0x2546, 0x2710, 0x28ae, 0x2a28, 0x2b83, 0x2cc5,
		0x2df1, 0x2f09, 0x3010, 0x3109, 0x31f4, 0x32d2, 0x33a6,
		0x3470, 0x3531, 0x35ea, 0x369b, 0x3746, 0x37ea, 0x3888,
		0x3920, 0x39b3, 0x3a42, 0x3acc, 0x3b51, 0x3bd3, 0x3c51,
		0x3ccb, 0x3d42, 0x3db6, 0x3e27, 0x3e95, 0x3f00, 0x3f68,
		0x3fcf, 0x4033, 0x4094, 0x40f4, 0x4151, 0x41ac, 0x4206,
		0x425e, 0x42b4, 0x4308, 0x435b, 0x43ac, 0x43fc, 0x444a,
		0x4497, 0x44e2, 0x452d, 0x4576, 0x45bd, 0x4604, 0x4649,
		0x468e, 0x46d1, 0x4713, 0x4755, 0x4795, 0x47d4, 0x4813,
		0x4851, 0x488d, 0x48c9, 0x4904, 0x493f, 0x4978, 0x49b1,
		0x49e9, 0x4a20, 0x4a57
	};

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		snr_reading = ds3103_readreg(dev, 0xff);
		snr_reading /= 8;
		if (snr_reading == 0)
			*snr = 0x0000;
		else {
			if (snr_reading > 20)
				snr_reading = 20;
			snr_value = dvbs_snr_tab[snr_reading - 1] * 10 / 23026;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = snr_value * 8 * 655;
		}
		dprintk("%s: raw / cooked = 0x%02x / 0x%04x\n", __func__,
				snr_reading, *snr);
		break;
	case SYS_DVBS2:
		dvbs2_noise_reading = (ds3103_readreg(dev, 0x8c) & 0x3f) +
				(ds3103_readreg(dev, 0x8d) << 4);
		dvbs2_signal_reading = ds3103_readreg(dev, 0x8e);
		tmp = dvbs2_signal_reading * dvbs2_signal_reading >> 1;
		if (tmp == 0) {
			*snr = 0x0000;
			return 0;
		}
		if (dvbs2_noise_reading == 0) {
			snr_value = 0x0013;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = 0xffff;
			return 0;
		}
		if (tmp > dvbs2_noise_reading) {
			snr_reading = tmp / dvbs2_noise_reading;
			if (snr_reading > 80)
				snr_reading = 80;
			snr_value = dvbs2_snr_tab[snr_reading - 1] / 1000;
			/* cook the value to be suitable for szap-s2
			human readable output */
			*snr = snr_value * 5 * 655;
		} else {
			snr_reading = dvbs2_noise_reading / tmp;
			if (snr_reading > 80)
				snr_reading = 80;
			*snr = -(dvbs2_snr_tab[snr_reading] / 1000);
		}
		dprintk("%s: raw / cooked = 0x%02x / 0x%04x\n", __func__,
				snr_reading, *snr);
		break;
	default:
		return 1;
	}

	return 0;
}

/* read DS3103 BER value */
static int ds3103_read_ber(struct dvb_frontend *fe, u32* ber)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 data;
	u32 ber_reading, lpdc_frames;

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		/* set the number of bytes checked during
		BER estimation */
		ds3103_writereg(dev, 0xf9, 0x04);
		/* read BER estimation status */
		data = ds3103_readreg(dev, 0xf8);
		/* check if BER estimation is ready */
		if ((data & 0x10) == 0) {
			/* this is the number of error bits,
			to calculate the bit error rate
			divide to 8388608 */
			*ber = (ds3103_readreg(dev, 0xf7) << 8) |
				ds3103_readreg(dev, 0xf6);
			/* start counting error bits */
			/* need to be set twice
			otherwise it fails sometimes */
			data |= 0x10;
			ds3103_writereg(dev, 0xf8, data);
			ds3103_writereg(dev, 0xf8, data);
		} else
			/* used to indicate that BER estimation
			is not ready, i.e. BER is unknown */
			*ber = 0xffffffff;
		break;
	case SYS_DVBS2:
		/* read the number of LPDC decoded frames */
		lpdc_frames = (ds3103_readreg(dev, 0xd7) << 16) |
				(ds3103_readreg(dev, 0xd6) << 8) |
				ds3103_readreg(dev, 0xd5);
		/* read the number of packets with bad CRC */
		ber_reading = (ds3103_readreg(dev, 0xf8) << 8) |
				ds3103_readreg(dev, 0xf7);
		if (lpdc_frames > 750) {
			/* clear LPDC frame counters */
			ds3103_writereg(dev, 0xd1, 0x01);
			/* clear bad packets counter */
			ds3103_writereg(dev, 0xf9, 0x01);
			/* enable bad packets counter */
			ds3103_writereg(dev, 0xf9, 0x00);
			/* enable LPDC frame counters */
			ds3103_writereg(dev, 0xd1, 0x00);
			*ber = ber_reading;
		} else
			/* used to indicate that BER estimation is not ready,
			i.e. BER is unknown */
			*ber = 0xffffffff;
		break;
	default:
		return 1;
	}

	return 0;
}

static int ds3103_set_tone(struct dvb_frontend *fe,
	enum fe_sec_tone_mode fe_sec_tone_mode)
{
	struct ds3103_dev *dev = fe->demodulator_priv;

	int ret;
	unsigned int utmp, tone, reg_a1_mask;
	
    printk(KERN_DEBUG "fe_sec_tone_mode=%d\n", fe_sec_tone_mode);

	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}

	
	switch (fe_sec_tone_mode) {
	case SEC_TONE_ON:
		tone = 0;
		reg_a1_mask = 0x47;
		break;
	case SEC_TONE_OFF:
		tone = 1;
		reg_a1_mask = 0x00;
		break;
	default:
		printk(KERN_DEBUG "%s: invalid fe_sec_tone_mode\n",__func__);
		ret = -EINVAL;
		goto err;
	}

	utmp = tone << 7 | dev->cfg->envelope_mode << 5;
	ret = regmap_update_bits(dev, 0xa2, 0xe0, utmp);
	if (ret)
		goto err;

	utmp = 1 << 2;
	ret = regmap_update_bits(dev, 0xa1, reg_a1_mask, utmp);
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n", __func__,ret);
	return ret;
}

static int ds3103_set_voltage(struct dvb_frontend *fe,
	enum fe_sec_voltage fe_sec_voltage)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	int ret;
	unsigned int utmp;
	bool voltage_sel, voltage_dis;

	printk(KERN_DEBUG "fe_sec_voltage=%d\n", fe_sec_voltage);

	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}

	switch (fe_sec_voltage) {
	case SEC_VOLTAGE_18:
		voltage_sel = true;
		voltage_dis = false;
		break;
	case SEC_VOLTAGE_13:
		voltage_sel = false;
		voltage_dis = false;
		break;
	case SEC_VOLTAGE_OFF:
		voltage_sel = false;
		voltage_dis = true;
		break;
	default:
		printk(KERN_DEBUG "%s: invalid fe_sec_voltage\n",__func__);
		ret = -EINVAL;
		goto err;
	}

	/* output pin polarity */
	voltage_sel ^= dev->cfg->lnb_hv_pol;
	voltage_dis ^= dev->cfg->lnb_en_pol;

	utmp = voltage_dis << 1 | voltage_sel << 0;
	ret = regmap_update_bits(dev, 0xa2, 0x03, utmp);
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n", __func__,ret);
	return ret;
}

static int ds3103_diseqc_send_master_cmd(struct dvb_frontend *fe,
				struct dvb_diseqc_master_cmd *diseqc_cmd)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	int ret;
	unsigned int utmp;
	unsigned int i;


   /* Dump DiSEqC message */
	dprintk("%s(", __func__);
	for (i = 0 ; i < diseqc_cmd->msg_len;) {
		dprintk("0x%02x", diseqc_cmd->msg[i]);
		if (++i < diseqc_cmd->msg_len)
			dprintk(", ");
	}
	
	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}
	
	if (diseqc_cmd->msg_len < 3 || diseqc_cmd->msg_len > 6) {
		ret = -EINVAL;
		goto err;
	}

	/* enable DiSEqC message send pin */
	utmp = dev->cfg->envelope_mode << 5;
	ret = regmap_update_bits(dev, 0xa2, 0xe0, utmp);
	if (ret)
		goto err;

	/* DiSEqC message */
	for (i = 0; i < diseqc_cmd->msg_len; i++){
		ret = ds3103_writereg(dev, 0xa3 + i, diseqc_cmd->msg[i]);
		if (ret)
			goto err;
	}

	ret = ds3103_writereg(dev, 0xa1,
			(diseqc_cmd->msg_len - 1) << 3 | 0x07);
	if (ret)
		goto err;

	/* wait up to 150ms for DiSEqC transmission to complete */
	for (i = 0; i < 15; i++) {
		utmp = ds3103_readreg(dev, 0xa1);
		if ((utmp & 0x40) == 0)
			break;
		msleep(10);
	}

	/* DiSEqC timeout after 150ms */
	if (i == 15) {
		ret = regmap_update_bits(dev, 0xa1, 0xc0, 0x40);
		ret |= regmap_update_bits(dev, 0xa2, 0xc0, 0x80);
		if (ret)
			goto err;
		ret = -ETIMEDOUT;
		goto err;
	}

	ret = regmap_update_bits(dev, 0xa2, 0xc0, 0x80);
	
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n", __func__,ret);
	return ret;
}

/* Send DiSEqC burst */
static int ds3103_diseqc_send_burst(struct dvb_frontend *fe,
				    enum fe_sec_mini_cmd fe_sec_mini_cmd)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	int i;
	int ret;
	unsigned int utmp, burst;

	dprintk("%s()\n", __func__);
	
	if (!dev->warm) {
		ret = -EAGAIN;
		goto err;
	}

	utmp = dev->cfg->envelope_mode << 5;
	ret = regmap_update_bits(dev, 0xa2, 0xe0, utmp);
	if (ret)
		goto err;

	switch (fe_sec_mini_cmd) {
	case SEC_MINI_A:
		burst = 0x02;
		break;
	case SEC_MINI_B:	
		burst = 0x01;
		break;
	default:
		printk(KERN_DEBUG "%s: invalid fe_sec_mini_cmd\n",__func__);
		ret = -EINVAL;
		goto err;
	}
	
	ret = ds3103_writereg(dev, 0xa1, burst);
	if (ret)
		goto err;
		
	msleep(13);
	for (i = 0; i < 5; i++) {
		utmp = ds3103_readreg(dev, 0xa1);
		if ((utmp & 0x40) == 0)
			break;
		msleep(1);
	}

	if (i == 5) {
		ret = regmap_update_bits(dev, 0xa1, 0xc0, 0x40);
		ret |= regmap_update_bits(dev, 0xa2, 0xc0, 0x80);
		if (ret)
		  goto err;
		
		ret = -ETIMEDOUT;
		goto err;
	}
	
	ret = regmap_update_bits(dev, 0xa2, 0xc0, 0x80);
	if (ret)
		goto err;
	

	return 0;
err:
	printk(KERN_ERR "%s: failed=%d\n", __func__,ret);
	return ret;

}

static int ds3103_get_tune_settings(struct dvb_frontend *fe,
	struct dvb_frontend_tune_settings *s)
{
	s->min_delay_ms = 3000;

	return 0;
}

static void ds3103_release(struct dvb_frontend *fe)
{
	struct ds3103_dev *dev = fe->demodulator_priv;

	if (dev->cfg->set_lock_led)
		dev->cfg->set_lock_led(fe, 0);

	dprintk("%s\n", __func__);
	kfree(dev);
}



/* read DS3103 uncorrected blocks */
static int ds3103_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct ds3103_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 data;
	u16 _ucblocks;

	dprintk("%s()\n", __func__);

	switch (c->delivery_system) {
	case SYS_DVBS:
		*ucblocks = (ds3103_readreg(dev, 0xf5) << 8) |
				ds3103_readreg(dev, 0xf4);
		data = ds3103_readreg(dev, 0xf8);
		/* clear packet counters */
		data &= ~0x20;
		ds3103_writereg(dev, 0xf8, data);
		/* enable packet counters */
		data |= 0x20;
		ds3103_writereg(dev, 0xf8, data);
		break;
	case SYS_DVBS2:
		_ucblocks = (ds3103_readreg(dev, 0xe2) << 8) |
				ds3103_readreg(dev, 0xe1);
		if (_ucblocks > dev->prevUCBS2)
			*ucblocks = _ucblocks - dev->prevUCBS2;
		else
			*ucblocks = dev->prevUCBS2 - _ucblocks;
		dev->prevUCBS2 = _ucblocks;
		break;
	default:
		return 1;
	}

	return 0;
}

static int ds3103_select(struct i2c_adapter *adap, void *mux_priv, u32 chan)
{
	struct ds3103_dev *dev = mux_priv;
	struct i2c_client *client = dev->client;
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = "\x03\x11",
	};

	/* Open tuner I2C repeater for 1 xfer, closes automatically */
	ret = __i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_warn(&client->dev, "i2c wr failed=%d\n", ret);
		if (ret >= 0)
			ret = -EREMOTEIO;
		return ret;
	}

	return 0;
}

#if 0
/*
 * XXX: That is wrapper to ds3103_probe() via driver core in order to provide
 * proper I2C client for legacy media attach binding.
 * New users must use I2C client binding directly!
 */
struct dvb_frontend *ds3103_attach(const struct ds3103_config *cfg,
		struct i2c_adapter *i2c, struct i2c_adapter **tuner_i2c_adapter)
{
	struct i2c_client *client;
	struct i2c_board_info board_info;
	struct ds3103_platform_data pdata;

	pdata.clk = cfg->clock;
	pdata.i2c_wr_max = cfg->i2c_wr_max;
	pdata.ts_mode = cfg->ts_mode;
	pdata.ts_clk = cfg->ts_clk;
	pdata.ts_clk_pol = cfg->ts_clk_pol;
	pdata.spec_inv = cfg->spec_inv;
	pdata.agc = cfg->agc;
	pdata.agc_inv = cfg->agc_inv;
	pdata.clk_out = cfg->clock_out;
	pdata.envelope_mode = cfg->envelope_mode;
	pdata.lnb_hv_pol = cfg->lnb_hv_pol;
	pdata.lnb_en_pol = cfg->lnb_en_pol;
	pdata.attach_in_use = true;

	memset(&board_info, 0, sizeof(board_info));
	strlcpy(board_info.type, "ds3103", I2C_NAME_SIZE);
	board_info.addr = cfg->i2c_addr;
	board_info.platform_data = &pdata;
	client = i2c_new_device(i2c, &board_info);
	if (!client || !client->dev.driver)
		return NULL;

	*tuner_i2c_adapter = pdata.get_i2c_adapter(client);
	return pdata.get_dvb_frontend(client);
}
#endif

struct dvb_frontend *ds3103_attach(const struct ds3103_config *cfg,
		struct i2c_adapter *i2c, struct i2c_adapter **tuner_i2c_adapter)
{
	struct ds3103_dev *dev = NULL;
	int ret;
	u8 val_01, val_02, val_b2;
	int utmp;

	printk("Entering %s\n", __func__);

	/* allocate memory for the internal dev */
	dev = kzalloc(sizeof(struct ds3103_dev), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "Unable to kmalloc\n");
		goto error2;
	}

	//dev->client = client;
	dev->config.clock = cfg->clock;
	dev->config.i2c_wr_max = cfg->i2c_wr_max;
	dev->config.ts_mode = cfg->ts_mode;
	dev->config.ts_clk = cfg->ts_clk;
	dev->config.ts_clk_pol = cfg->ts_clk_pol;
	dev->config.spec_inv = cfg->spec_inv;
	dev->config.agc_inv = cfg->agc_inv;
	dev->config.clock_out = cfg->clock_out;
	dev->config.envelope_mode = cfg->envelope_mode;
	dev->config.agc = cfg->agc;
	dev->config.lnb_hv_pol = cfg->lnb_hv_pol;
	dev->config.lnb_en_pol = cfg->lnb_en_pol;
	dev->config.i2c_addr = cfg->i2c_addr;
	dev->cfg = &dev->config;
	dev->i2c_adapter = i2c;

	/* 0x00: chip id[6:0], 0x01: chip ver[7:0], 0x02: chip ver[15:8] */
	utmp = ds3103_readreg(dev, 0x00);
	
	dev->chip_id = utmp >> 1;
		
	/* check if the demod is present */
	ret = utmp & 0xfe;
	if (ret != 0xe0) {
		printk(KERN_ERR "Invalid probe, probably not a DS3x0x\n");
		goto error3;
	}

	/* check demod chip ID */
	val_01 = ds3103_readreg(dev, 0x01);
	val_02 = ds3103_readreg(dev, 0x02);
	val_b2 = ds3103_readreg(dev, 0xb2);
	if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0xC0)) {
		printk("\tChip ID = [DS3103]!\n");
	} else if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0x00)) {
		printk("\tChip ID = [DS3002B]!\n");
	} else if ((val_02 == 0x00) && (val_01 == 0xC0)) {
		printk("\tChip ID = [DS300X]! Not supported by this module\n");
		goto error3;
	} else {
		printk("\tChip ID = unknow!\n");
		goto error3;
	}

	switch (dev->cfg->clock_out) {
	case DS3103_CLOCK_OUT_DISABLED:
		utmp = 0x80;
		break;
	case DS3103_CLOCK_OUT_ENABLED:
		utmp = 0x00;
		break;
	case DS3103_CLOCK_OUT_ENABLED_DIV2:
		utmp = 0x10;
		break;
	default:
		ret = -EINVAL;
		goto error3;
	}

	/* 0x29 register is defined differently for m88rs6000. */
	/* set internal tuner address to 0x21 */
	if (dev->chip_id == RS6000_CHIP_ID)
		utmp = 0x00;

	ret = ds3103_writereg(dev, 0x29, utmp);
	if (ret)
		goto error3;


	/* sleep */
	ret = regmap_update_bits(dev, 0x08, 0x01, 0x00);
	if (ret)
		goto error3;
	ret = regmap_update_bits(dev, 0x04, 0x01, 0x01);
	if (ret)
		goto error3;
	ret = regmap_update_bits(dev, 0x23, 0x10, 0x10);
	if (ret)
		goto error3;


	printk(KERN_INFO "DS3103 chip version: %d.%d attached.\n", val_02, val_01);

	memcpy(&dev->fe.ops, &ds3103_ops,
			sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = dev;
	return &dev->fe;
error3:
	kfree(dev);
error2:
	return NULL;
}	
EXPORT_SYMBOL(ds3103_attach);

static struct dvb_frontend_ops ds3103_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2 },
	.info = {
		.name = "Montage Technology DS3103",
		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_tolerance = 5000,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_8_9 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_RECOVER |
			FE_CAN_2G_MODULATION
	},

	.release = ds3103_release,

	.get_tune_settings = ds3103_get_tune_settings,
	.init = ds3103_init,
	.sleep = ds3103_sleep,

	.set_frontend = ds3103_set_frontend,
	.get_frontend = ds3103_get_frontend,

	.read_status = ds3103_read_status,
	
	.i2c_gate_ctrl = ds3103_i2c_gate_ctrl,
	.read_snr = ds3103_read_snr,
	.read_ber = ds3103_read_ber,
	.read_ucblocks = ds3103_read_ucblocks,
	
	.diseqc_send_master_cmd = ds3103_diseqc_send_master_cmd,
	.diseqc_send_burst = ds3103_diseqc_send_burst,

	.set_tone = ds3103_set_tone,
	.set_voltage = ds3103_set_voltage,
};

static struct dvb_frontend *ds3103_get_dvb_frontend(struct i2c_client *client)
{
	struct ds3103_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	return &dev->fe;
}

static struct i2c_adapter *ds3103_get_i2c_adapter(struct i2c_client *client)
{
	struct ds3103_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	return dev->i2c_adapter;
}

static int ds3103_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds3103_dev *dev = NULL;
	struct ds3103_platform_data *pdata = client->dev.platform_data;
	int ret;
	u8 val_01, val_02, val_b2;
	unsigned int utmp;

	printk(KERN_DEBUG "Entering %s\n", __func__);

	/* allocate memory for the internal dev */
	dev = kzalloc(sizeof(struct ds3103_dev), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "Unable to kmalloc\n");
		goto error2;
	}

	dev->client = client;
	dev->config.clock = pdata->clk;
	dev->config.i2c_wr_max = pdata->i2c_wr_max;
	dev->config.ts_mode = pdata->ts_mode;
	dev->config.ts_clk = pdata->ts_clk;
	dev->config.ts_clk_pol = pdata->ts_clk_pol;
	dev->config.spec_inv = pdata->spec_inv;
	dev->config.agc_inv = pdata->agc_inv;
	dev->config.clock_out = pdata->clk_out;
	dev->config.envelope_mode = pdata->envelope_mode;
	dev->config.agc = pdata->agc;
	dev->config.lnb_hv_pol = pdata->lnb_hv_pol;
	dev->config.lnb_en_pol = pdata->lnb_en_pol;
	dev->cfg = &dev->config;

	/* 0x00: chip id[6:0], 0x01: chip ver[7:0], 0x02: chip ver[15:8] */
	utmp = ds3103_readreg(dev, 0x00);

	dev->chip_id = utmp >> 1;
		
	/* check if the demod is present */
	ret = utmp & 0xfe;
	if (ret != 0xe0) {
		printk(KERN_ERR "Invalid probe, probably not a DS3x0x\n");
		goto error3;
	}

	/* check demod chip ID */
	val_01 = ds3103_readreg(dev, 0x01);
	val_02 = ds3103_readreg(dev, 0x02);
	val_b2 = ds3103_readreg(dev, 0xb2);
	if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0xC0)) {
		printk("\tChip ID = [DS3103]!\n");
	} else if((val_02 == 0x00) &&
			(val_01 == 0xD0) && ((val_b2 & 0xC0) == 0x00)) {
		printk("\tChip ID = [DS3002B]!\n");
	} else if ((val_02 == 0x00) && (val_01 == 0xC0)) {
		printk("\tChip ID = [DS300X]! Not supported by this module\n");
		goto error3;
	} else {
		printk("\tChip ID = unknow!\n");
		goto error3;
	}

	switch (dev->cfg->clock_out) {
	case DS3103_CLOCK_OUT_DISABLED:
		utmp = 0x80;
		break;
	case DS3103_CLOCK_OUT_ENABLED:
		utmp = 0x00;
		break;
	case DS3103_CLOCK_OUT_ENABLED_DIV2:
		utmp = 0x10;
		break;
	default:
		ret = -EINVAL;
		goto error3;
	}

	/* 0x29 register is defined differently for m88rs6000. */
	/* set internal tuner address to 0x21 */
	if (dev->chip_id == RS6000_CHIP_ID)
		utmp = 0x00;

	ret = ds3103_writereg(dev, 0x29, utmp);
	if (ret)
		goto error3;

	/* sleep */
	ret = regmap_update_bits(dev, 0x08, 0x01, 0x00);
	if (ret)
		goto error3;
	ret = regmap_update_bits(dev, 0x04, 0x01, 0x01);
	if (ret)
		goto error3;
	ret = regmap_update_bits(dev, 0x23, 0x10, 0x10);
	if (ret)
		goto error3;

#if 0
	/* create mux i2c adapter for tuner */
	dev->i2c_adapter = i2c_add_mux_adapter(client->adapter,
					       dev, 0, 0, ds3103_select,
					       NULL);
	if (dev->i2c_adapter == NULL) {
		ret = -ENOMEM;
		goto error3;
	}
#endif
	printk(KERN_INFO "DS3103 chip version: %d.%d attached.\n", val_02, val_01);

	memcpy(&dev->fe.ops, &ds3103_ops,
			sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = dev;
	return &dev->fe;

error3:
	kfree(dev);
error2:
	return NULL;
}	

static int ds3103_remove(struct i2c_client *client)
{
	struct ds3103_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	//i2c_del_mux_adapter(dev->i2c_adapter);

	kfree(dev);
	return 0;
}

static const struct i2c_device_id ds3103_id_table[] = {
	{"ds3103", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ds3103_id_table);

static struct i2c_driver ds3103_driver = {
	.driver = {
		.name	= "ds3103",
		.suppress_bind_attrs = true,
	},
	.probe		= ds3103_probe,
	.remove		= ds3103_remove,
	.id_table	= ds3103_id_table,
};

module_i2c_driver(ds3103_driver);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates frontend debugging (default:0)");

MODULE_DESCRIPTION("DVB Frontend module for Montage Technology "
			"DS3103 hardware");
MODULE_AUTHOR("Tomazzo Muzumici");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(DS3103_DEFAULT_FIRMWARE);
