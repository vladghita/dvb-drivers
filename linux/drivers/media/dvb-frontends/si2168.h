/*
 * Silicon Labs Si2168 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef SI2168_H
#define SI2168_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

struct si2168_config {
	/* i2c addr
	 * possible values: 0x64,0x65,0x66,0x67 */
	u8 i2c_addr;

	/* TS mode */
#define SI2168_TS_PARALLEL	0x06
#define SI2168_TS_SERIAL	0x03
	u8 ts_mode;

	/* TS clock inverted */
	bool ts_clock_inv;

	/* TS clock gapped */
	bool ts_clock_gapped;
	
	/* Tuner control pins */
#define SI2168_MP_NOT_USED	1
#define SI2168_MP_A		2
#define SI2168_MP_B		3
#define SI2168_MP_C		4
#define SI2168_MP_D		5
	int agc_pin;
	bool agc_inv;
	int fef_pin;
	bool fef_inv;
	
	struct dvb_frontend **fe;
	struct i2c_adapter **i2c_adapter;
};

#if defined(CONFIG_DVB_SI2168) || \
		(defined(CONFIG_DVB_SI2168_MODULE) && defined(MODULE))
extern struct dvb_frontend *si2168_attach(const struct si2168_config *config,
						struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *si2168_attach(
		const struct si2168_config *config, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif


#endif
