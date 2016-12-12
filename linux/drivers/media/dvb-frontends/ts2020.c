/*
     Driver for Montage ts2022 DVBS/S2 Silicon tuner

     Copyright (C) 2012 Tomazzo Muzumici

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

#include "dvb_frontend.h"
#include "ts2020.h"

static int debug;
static int nDebugWriteByFct = 0;
#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "ts2020: " args); \
	} while (0)

#define TS2020_XTAL_FREQ   27000 /* in kHz */
#define FREQ_OFFSET_LOW_SYM_RATE 3000

#define TS2020_min(x, y) ({                      \
         typeof(x) _min1 = (x);                  \
         typeof(y) _min2 = (y);                  \
         (void) (&_min1 == &_min2);              \
         _min1 < _min2 ? _min1 : _min2; })
 
#define TS2020_max(x, y) ({                      \
         typeof(x) _max1 = (x);                  \
         typeof(y) _max2 = (y);                  \
         (void) (&_max1 == &_max2);              \
         _max1 > _max2 ? _max1 : _max2; })

/**
  * clamp - return a value clamped to a given range with strict typechecking
  * @val: current value
  * @lo: lowest allowable value
  * @hi: highest allowable value
  *
  * This macro does strict typechecking of lo/hi to make sure they are of the
  * same type as val.  See the unnecessary pointer comparisons.
 */
#define TS2020_clamp(val, lo, hi) TS2020_min((typeof(val))TS2020_max(val, lo), hi)
 
#define TS2020_min_t(type, x, y) ({             \
         type __min1 = (x);                     \
         type __min2 = (y);                     \
         __min1 < __min2 ? __min1: __min2; })
 
#define TS2020_max_t(type, x, y) ({             \
         type __max1 = (x);                     \
         type __max2 = (y);                     \
         __max1 > __max2 ? __max1: __max2; })
 
 /**
  * clamp_t - return a value clamped to a given range using a given type
  * @type: the type of variable to use
  * @val: current value
  * @lo: minimum allowable value
  * @hi: maximum allowable value
  *
  * This macro does no typechecking and uses temporary variables of type
  * 'type' to make all the comparisons.
  */
#define TS2020_clamp_t(type, val, lo, hi) TS2020_min_t(type, TS2020_max_t(type, val, lo), hi)
 
 
#define TS2020_ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
 #define TS2020_DIV_ROUND_CLOSEST(x, divisor)(           \
 {                                                       \
          typeof(x) __x = x;                             \
          typeof(divisor) __d = divisor;                 \
          (((typeof(x))-1) > 0 ||                        \
          ((typeof(divisor))-1) > 0 || (__x) > 0) ?      \
                 (((__x) + ((__d) / 2)) / (__d)) :       \
                 (((__x) - ((__d) / 2)) / (__d));        \
}                                                        \
)

struct ts2020_priv {
	struct i2c_client *client;
	struct dvb_frontend *fe;
	struct delayed_work stat_work;
	int (*get_agc_pwm)(struct dvb_frontend *fe, u8 *_agc_pwm);
	/* i2c details */
	struct i2c_adapter *i2c;
	int i2c_address;
	bool loop_through:1;
	u8 clk_out:2;
	u8 clk_out_div:5;
	bool dont_poll:1;
	u32 frequency_div; /* LO output divider switch frequency */
	u32 frequency_khz; /* actual used LO frequency */
#define TS2020_M88TS2020 0
#define TS2020_M88TS2022 1
	u8 tuner;
};

struct ts2020_reg_val {
	u8 reg;
	u8 val;
};

static void ts2020_stat_work(struct work_struct *work);

static int ts2020_release(struct dvb_frontend *fe)
{
	//kfree(fe->tuner_priv);
	//fe->tuner_priv = NULL;
	struct ts2020_priv *priv = fe->tuner_priv;
	struct i2c_client *client = priv->client;
	i2c_unregister_device(client);
	return 0;
}

static int ts2020_writereg(struct dvb_frontend *fe, int reg, int data)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	u8 buf[] = { reg, data };
	struct i2c_msg msg[] = {
		{
			.addr = priv->i2c_address,
			.flags = 0,
			.buf = buf,
			.len = 2
		}
	};
	int err;

	if(1 == nDebugWriteByFct){
		printk("%s: write reg 0x%02x, value 0x%02x\n", __func__, reg, data);
	}
	
	dprintk("%s: write reg 0x%02x, value 0x%02x\n", __func__, reg, data);
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	err = i2c_transfer(priv->i2c, msg, 1);
	if (err != 1) {
		printk("%s: writereg error(err == %i, reg == 0x%02x,"
		" value == 0x%02x)\n", __func__, err, reg, data);
		return -EREMOTEIO;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;
}

static int ts2020_readreg(struct dvb_frontend *fe, u8 reg)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = priv->i2c_address,
			.flags = 0,
			.buf = b0,
			.len = 1
		}, {
			.addr = priv->i2c_address,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = i2c_transfer(priv->i2c, msg, 2);
	
	if (ret != 2) {
		printk(KERN_ERR "%s: reg=0x%x(error=%d)\n", __func__, reg, ret);
		return ret;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	dprintk("%s: read reg 0x%02x, value 0x%02x\n", __func__, reg, b1[0]);
	
	return b1[0];
}

static int ts2020_sleep(struct dvb_frontend *fe)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	int ret;
	u8 u8tmp;

	if (priv->tuner == TS2020_M88TS2020)
		u8tmp = 0x0a; /* XXX: probably wrong */
	else
		u8tmp = 0x00;

	ret = ts2020_writereg(fe, u8tmp, 0x00);
	if (ret < 0)
		return ret;

	/* stop statistics polling */
	if (!priv->dont_poll)
		cancel_delayed_work_sync(&priv->stat_work);
	return 0;
}

static int ts2020_init(struct dvb_frontend *fe)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	int i;
	u8 u8tmp;
    
	dprintk(KERN_INFO "Entering %s\n", __func__);
	
	if (priv->tuner == TS2020_M88TS2020) {
		ts2020_writereg(fe, 0x42, 0x73);
		ts2020_writereg(fe,0x05, priv->clk_out_div);
		ts2020_writereg(fe, 0x20, 0x27);
		ts2020_writereg(fe, 0x07, 0x02);
		ts2020_writereg(fe, 0x11, 0xff);
		ts2020_writereg(fe, 0x60, 0xf9);
		ts2020_writereg(fe, 0x08, 0x01);
		ts2020_writereg(fe, 0x00, 0x41);
	} else {
	static const struct ts2020_reg_val reg_vals[] = {
			{0x7d, 0x9d},
			{0x7c, 0x9a},
			{0x7a, 0x76},
			{0x3b, 0x01},
			{0x63, 0x88},
			{0x61, 0x85},
			{0x22, 0x30},
			{0x30, 0x40},
			{0x20, 0x23},
			{0x24, 0x02},
			{0x12, 0xa0},
		};

	ts2020_writereg(fe, 0x00, 0x01);
	ts2020_writereg(fe, 0x00, 0x03);

	switch (priv->clk_out) {
		case TS2020_CLK_OUT_DISABLED:
			u8tmp = 0x60;
			break;
		case TS2020_CLK_OUT_ENABLED:
			u8tmp = 0x70;
			ts2020_writereg(fe, 0x05, priv->clk_out_div);
			break;
		case TS2020_CLK_OUT_ENABLED_XTALOUT:
			u8tmp = 0x6c;
			break;
		default:
			u8tmp = 0x60;
			break;
		}

	ts2020_writereg(fe, 0x42, u8tmp);

		if (priv->loop_through)
			u8tmp = 0xec;
		else
			u8tmp = 0x6c;

	ts2020_writereg(fe, 0x62, u8tmp);

	for (i = 0; i < TS2020_ARRAY_SIZE(reg_vals); i++)
		ts2020_writereg(fe,  reg_vals[i].reg,
				 reg_vals[i].val);
	}
	dprintk(KERN_INFO "%s: Initializing TS2020_M88TS2022\n", __func__);
	return 0;
}
static int ts2020_tuner_gate_ctrl(struct dvb_frontend *fe, u8 offset)
{
	int ret;
	ret = ts2020_writereg(fe, 0x51, 0x1f - offset);
	ret |= ts2020_writereg(fe, 0x51, 0x1f);
	ret |= ts2020_writereg(fe, 0x50, offset);
	ret |= ts2020_writereg(fe, 0x50, 0x00);
	msleep(20);
	return ret;
}

static int ts2020_set_tuner_rf(struct dvb_frontend *fe)
{
	int ret;
	unsigned int utmp;

	utmp = ts2020_readreg(fe, 0x3d);

	utmp &= 0x7f;
	if (utmp < 0x16)
		utmp = 0xa1;
	else if (utmp == 0x16)
		utmp = 0x99;
	else
		utmp = 0xf9;

	ret = ts2020_writereg(fe, 0x60, utmp);
	ret = ts2020_tuner_gate_ctrl(fe, 0x08);

	return ret;
}

static int ts2020_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct ts2020_priv *priv = fe->tuner_priv;
	int ret;
	unsigned int utmp;
	u32 f3db, gdiv28;
	u16 u16tmp, value, lpf_coeff;
	u8 buf[3], reg10, lpf_mxdiv, mlpf_max, mlpf_min, nlpf;
	unsigned int f_ref_khz, f_vco_khz, div_ref, div_out, pll_n;
	unsigned int frequency_khz = c->frequency;

	dprintk(KERN_INFO "Entering %s\n", __func__);
	//nDebugWriteByFct = 1;

	/*
	 * Integer-N PLL synthesizer
	 * kHz is used for all calculations to keep calculations within 32-bit
	 */
	f_ref_khz = TS2020_XTAL_FREQ;
	div_ref = TS2020_DIV_ROUND_CLOSEST(f_ref_khz, 2000);


	/* select LO output divider */
	if (frequency_khz < priv->frequency_div) {
		div_out = 4;
		reg10 = 0x10;
	} else {
		div_out = 2;
		reg10 = 0x00;
	}
	f_vco_khz = frequency_khz * div_out;
	pll_n = f_vco_khz * div_ref / f_ref_khz;
	pll_n += pll_n % 2;
	priv->frequency_khz = pll_n * f_ref_khz / div_ref / div_out;
	
	dprintk(KERN_INFO "%s:frequency=%u offset=%d f_vco_khz=%u pll_n=%u div_ref=%u div_out=%u\n",
		 __func__,priv->frequency_khz, priv->frequency_khz - c->frequency,
		 f_vco_khz, pll_n, div_ref, div_out);


	if (priv->tuner == TS2020_M88TS2020) {
		lpf_coeff = 2766;
		reg10 |= 0x01;
		ret = ts2020_writereg(fe, 0x10, reg10);
	} else {
		lpf_coeff = 3200;
		reg10 |= 0x0b;
		ret = ts2020_writereg(fe, 0x10, reg10);
		ret |= ts2020_writereg(fe, 0x11, 0x40);
	}

	u16tmp = pll_n - 1024;
	buf[0] = (u16tmp >> 8) & 0xff;
	buf[1] = (u16tmp >> 0) & 0xff;
	buf[2] = div_ref - 8;

	ret |= ts2020_writereg(fe, 0x01, buf[0]);
	ret |= ts2020_writereg(fe, 0x02, buf[1]);
	ret |= ts2020_writereg(fe, 0x03, buf[2]);

	ret |= ts2020_tuner_gate_ctrl(fe, 0x10);
	if (ret < 0)
		return -ENODEV;

	ret |= ts2020_tuner_gate_ctrl(fe, 0x08);

	/* Tuner RF */
	if (priv->tuner == TS2020_M88TS2020)
		ret |= ts2020_set_tuner_rf(fe);
	gdiv28 = (TS2020_XTAL_FREQ / 1000 * 1694 + 500) / 1000;
	ret |= ts2020_writereg(fe, 0x04, gdiv28 & 0xff);
	ret |= ts2020_tuner_gate_ctrl(fe, 0x04);
	if (ret < 0)
		return -ENODEV;

	if (priv->tuner == TS2020_M88TS2022) {
		ret = ts2020_writereg(fe, 0x25, 0x00);
		ret |= ts2020_writereg(fe, 0x27, 0x70);
		ret |= ts2020_writereg(fe, 0x41, 0x09);
		ret |= ts2020_writereg(fe, 0x08, 0x0b);
		if (ret < 0)
			return -ENODEV;
	}
	utmp =  ts2020_readreg(fe, 0x26);

	f3db = (c->bandwidth_hz / 1000 / 2) + 2000;
	f3db += FREQ_OFFSET_LOW_SYM_RATE; /* FIXME: ~always too wide filter */
	f3db = TS2020_clamp(f3db, 7000U, 40000U);

	gdiv28 = gdiv28 * 207 / (value * 2 + 151);
	mlpf_max = gdiv28 * 135 / 100;
	mlpf_min = gdiv28 * 78 / 100;
	if (mlpf_max > 63)
		mlpf_max = 63;

	nlpf = (f3db * gdiv28 * 2 / lpf_coeff /
		(TS2020_XTAL_FREQ / 1000)  + 1) / 2;
	if (nlpf > 23)
		nlpf = 23;
	if (nlpf < 1)
		nlpf = 1;

	lpf_mxdiv = (nlpf * (TS2020_XTAL_FREQ / 1000)
		* lpf_coeff * 2  / f3db + 1) / 2;

	if (lpf_mxdiv < mlpf_min) {
		nlpf++;
		lpf_mxdiv = (nlpf * (TS2020_XTAL_FREQ / 1000)
			* lpf_coeff * 2  / f3db + 1) / 2;
	}

	if (lpf_mxdiv > mlpf_max)
		lpf_mxdiv = mlpf_max;

	ret = ts2020_writereg(fe, 0x04, lpf_mxdiv);
	ret |= ts2020_writereg(fe, 0x06, nlpf);

	ret |= ts2020_tuner_gate_ctrl(fe, 0x04);

	ret |= ts2020_tuner_gate_ctrl(fe, 0x01);

	msleep(80);

	//nDebugWriteByFct = 0;
	dprintk(KERN_INFO "Exiting %s\n", __func__);
	
	return (ret < 0) ? -EINVAL : 0;
}

static int ts2020_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency_khz;
	return 0;
}

static int ts2020_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	*frequency = 0; /* Zero-IF */
	return 0;
}

/*
 * Get the tuner gain.
 * @fe: The front end for which we're determining the gain
 * @v_agc: The voltage of the AGC from the demodulator (0-2600mV)
 * @_gain: Where to store the gain (in 0.001dB units)
 *
 * Returns 0 or a negative error code.
 */
static int ts2020_read_tuner_gain(struct dvb_frontend *fe, unsigned v_agc,
				  __s64 *_gain)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	unsigned long gain1, gain2, gain3;
	unsigned utmp;
	dprintk(KERN_INFO "Entering %s\n", __func__);
		
	/* Read the RF gain */
	utmp = ts2020_readreg(fe, 0x3d);
	
	gain1 = utmp & 0x1f;

	/* Read the baseband gain */
	utmp = ts2020_readreg(fe, 0x21);
	gain2 = utmp & 0x1f;

	switch (priv->tuner) {
	case TS2020_M88TS2020:
		gain1 = TS2020_clamp_t(long, gain1, 0, 15);
		gain2 = TS2020_clamp_t(long, gain2, 0, 13);
		v_agc = TS2020_clamp_t(long, v_agc, 400, 1100);

		*_gain = -(gain1 * 2330 +
			   gain2 * 3500 +
			   v_agc * 24 / 10 * 10 +
			   10000);
		/* gain in range -19600 to -116850 in units of 0.001dB */
		break;

	case TS2020_M88TS2022:
		utmp = ts2020_readreg(fe, 0x66);
		gain3 = (utmp >> 3) & 0x07;

		gain1 = TS2020_clamp_t(long, gain1, 0, 15);
		gain2 = TS2020_clamp_t(long, gain2, 2, 16);
		gain3 = TS2020_clamp_t(long, gain3, 0, 6);
		v_agc = TS2020_clamp_t(long, v_agc, 600, 1600);

		*_gain = -(gain1 * 2650 +
			   gain2 * 3380 +
			   gain3 * 2850 +
			   v_agc * 176 / 100 * 10 -
			   30000);
		/* gain in range -47320 to -158950 in units of 0.001dB */
		dprintk(KERN_INFO "%s: TS2020_M88TS2022 gain %l\n", __func__,_gain);
		break;
	}

	return 0;
}

/*
 * Get the AGC information from the demodulator and use that to calculate the
 * tuner gain.
 */
static int ts2020_get_tuner_gain(struct dvb_frontend *fe, __s64 *_gain)
{
	struct ts2020_priv *priv = fe->tuner_priv;
	int v_agc = 0, ret;
	u8 agc_pwm;

	dprintk(KERN_INFO "Entering %s\n", __func__);
	
	/* Read the AGC PWM rate from the demodulator */
	if (priv->get_agc_pwm) {
		ret = priv->get_agc_pwm(fe, &agc_pwm);
		if (ret < 0)
			return ret;

		switch (priv->tuner) {
		case TS2020_M88TS2020:
			v_agc = (int)agc_pwm * 20 - 1166;
			break;
		case TS2020_M88TS2022:
			v_agc = (int)agc_pwm * 16 - 670;
			break;
		}

		if (v_agc < 0)
			v_agc = 0;
	}

	return ts2020_read_tuner_gain(fe, v_agc, _gain);
}

/*
 * Gather statistics on a regular basis
 */
static void ts2020_stat_work(struct work_struct *work)
{
	struct ts2020_priv *priv = container_of(work, struct ts2020_priv,
					       stat_work.work);
	struct i2c_client *client = priv->client;
	struct dtv_frontend_properties *c = &priv->fe->dtv_property_cache;
	int ret;

	dev_dbg(&client->dev, "\n");

	ret = ts2020_get_tuner_gain(priv->fe, &c->strength.stat[0].svalue);
	if (ret < 0)
		goto err;

	c->strength.stat[0].scale = FE_SCALE_DECIBEL;

	if (!priv->dont_poll)
		schedule_delayed_work(&priv->stat_work, msecs_to_jiffies(2000));
	return;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
}

/*
 * Read TS2020 signal strength in v3 format.
 */
static int ts2020_read_signal_strength(struct dvb_frontend *fe,
				       u16 *_signal_strength)
{
	dprintk(KERN_INFO "Entering %s\n", __func__);
	
	int sig_reading = 0; 
	u8 rfgain, bbgain, nngain;
	u8 rfagc;
	u32 gain = 0;
	dprintk("KERN_INFO %s()\n", __func__);
	
	rfgain = ts2020_readreg(fe, 0x3d) & 0x1f;
	bbgain = ts2020_readreg(fe, 0x21) & 0x1f;
	rfagc = ts2020_readreg(fe, 0x3f);
	sig_reading = rfagc * 16 -670;
	if (sig_reading<0)
		sig_reading =0;
	nngain =ts2020_readreg(fe, 0x66);
	nngain = (nngain >> 3) & 0x07;
	
	if (rfgain < 0)
		rfgain = 0;
	if (rfgain > 15)
		rfgain = 15;
	if (bbgain < 2)
		bbgain = 2;
	if (bbgain > 16)
		bbgain = 16;
	if (nngain < 0)
		nngain = 0;
	if (nngain > 6)
		nngain = 6;
	
	if (sig_reading < 600)
		sig_reading = 600;
	if (sig_reading > 1600)
		sig_reading = 1600;
	
	gain = (u16) rfgain * 265 + (u16) bbgain * 338 + (u16) nngain * 285 + sig_reading * 176 / 100 - 3000;
	
	
	*_signal_strength = gain*100;
	
	dprintk(KERN_INFO "%s: raw / cooked = 0x%04x / 0x%04x\n", __func__, sig_reading, *_signal_strength);
	
	return 0;
}

static struct dvb_tuner_ops ts2020_tuner_ops = {
	.info = {
		.name = "TS2020",
		.frequency_min = 950000,
		.frequency_max = 2150000
	},
	.init = ts2020_init,
	.release = ts2020_release,
	.sleep = ts2020_sleep,
	.set_params = ts2020_set_params,
	.get_frequency = ts2020_get_frequency,
	.get_if_frequency = ts2020_get_if_frequency,
	.get_rf_strength = ts2020_read_signal_strength,
};

struct dvb_frontend *ts2020_attach(struct dvb_frontend *fe,
					const struct ts2020_config *config,
					struct i2c_adapter *i2c)
{
	struct i2c_client *client;
	struct i2c_board_info board_info;

	/* This is only used by ts2020_probe() so can be on the stack */
	struct ts2020_config pdata;

	memcpy(&pdata, config, sizeof(pdata));
	pdata.fe = fe;
	pdata.attach_in_use = true;

	memset(&board_info, 0, sizeof(board_info));
	strlcpy(board_info.type, "ts2020", I2C_NAME_SIZE);
	board_info.addr = config->tuner_address;
	board_info.platform_data = &pdata;
	client = i2c_new_device(i2c, &board_info);
	if (!client || !client->dev.driver)
		return NULL;

	return fe;
}
EXPORT_SYMBOL(ts2020_attach);
	

static int ts2020_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ts2020_config *pdata = client->dev.platform_data;
	struct dvb_frontend *fe = pdata->fe;
	struct ts2020_priv *dev;
	int ret;
	u8 u8tmp;
	unsigned int utmp;
	char *chip_str;
	
	dprintk(KERN_INFO "Entering %s\n", __func__);
	
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err;
	}

	

	dev->i2c = client->adapter;
	dev->i2c_address = client->addr;
	dev->loop_through = pdata->loop_through;
	dev->clk_out = pdata->clk_out;
	dev->clk_out_div = pdata->clk_out_div;
	dev->dont_poll = pdata->dont_poll;
	dev->frequency_div = pdata->frequency_div;
	dev->fe = fe;
	dev->get_agc_pwm = pdata->get_agc_pwm;
	fe->tuner_priv = dev;
	dev->client = client;
	INIT_DELAYED_WORK(&dev->stat_work, ts2020_stat_work);

	/* check if the tuner is there */
	utmp = ts2020_readreg(fe, 0x00);

	if ((utmp & 0x03) == 0x00) {
		ret = ts2020_writereg(fe, 0x00, 0x01);
		if (ret)
			goto err_kfree;

		msleep(2);
	}

	ret = ts2020_writereg(fe, 0x00, 0x03);
	if (ret)
		goto err_kfree;

		msleep(2);

	utmp = ts2020_readreg(fe, 0x00);

	dev_dbg(&client->dev, "chip_id=%02x\n", utmp);

	switch (utmp) {
	case 0x01:
	case 0x41:
	case 0x81:
		dev->tuner = TS2020_M88TS2020;
		chip_str = "TS2020";
		printk(KERN_INFO "%s: Found tuner TS2020!\n", __func__);
		if (!dev->frequency_div)
			dev->frequency_div = 1060000;
		break;
	case 0xc3:
	case 0x83:
		dev->tuner = TS2020_M88TS2022;
		printk(KERN_INFO "%s: Found tuner TS2022!\n", __func__);
		chip_str = "TS2022";
		if (!dev->frequency_div)
			dev->frequency_div = 1103000;
		break;
	default:
		ret = -ENODEV;
		goto err_kfree;
	}

	if (dev->tuner == TS2020_M88TS2022) {
		switch (dev->clk_out) {
		case TS2020_CLK_OUT_DISABLED:
			u8tmp = 0x60;
			break;
		case TS2020_CLK_OUT_ENABLED:
			u8tmp = 0x70;
			ret = ts2020_writereg(fe, 0x05, dev->clk_out_div);
			if (ret)
				goto err_kfree;
			break;
		case TS2020_CLK_OUT_ENABLED_XTALOUT:
			u8tmp = 0x6c;
			break;
		default:
			ret = -EINVAL;
			goto err_kfree;
		}

		ret = ts2020_writereg(fe, 0x42, u8tmp);
		if (ret)
			goto err_kfree;

		if (dev->loop_through)
			u8tmp = 0xec;
		else
			u8tmp = 0x6c;

		ret = ts2020_writereg(fe, 0x62, u8tmp);
		if (ret)
			goto err_kfree;
	}

	/* sleep */
	ret = ts2020_writereg(fe, 0x00, 0x00);
	if (ret)
		goto err_kfree;

	dev_info(&client->dev,
		 "Montage Technology %s successfully identified\n", chip_str);

	memcpy(&fe->ops.tuner_ops, &ts2020_tuner_ops,
			sizeof(struct dvb_tuner_ops));
	if (!pdata->attach_in_use)
		fe->ops.tuner_ops.release = NULL;

	i2c_set_clientdata(client, dev);
	return 0;
err_kfree:
	kfree(dev);
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int ts2020_remove(struct i2c_client *client)
{
	struct ts2020_priv *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	
	kfree(dev);
	return 0;
}

static const struct i2c_device_id ts2020_id_table[] = {
	{"ts2020", 0},
	{"ts2022", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ts2020_id_table);

static struct i2c_driver ts2020_driver = {
	.driver = {
		.name	= "ts2020",
	},
	.probe		= ts2020_probe,
	.remove		= ts2020_remove,
	.id_table	= ts2020_id_table,
};

module_i2c_driver(ts2020_driver);
	

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("DVB ts2020 driver");
MODULE_AUTHOR("Tomazzo Muzumici");
MODULE_LICENSE("GPL");
