  /*
     Driver for Montage TS2022 DVBS/S2 Silicon tuner

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

#ifndef TS2020_H
#define TS2020_H

#include <linux/kconfig.h>
#include <linux/dvb/frontend.h>

struct ts2020_config {
	u8 tuner_address;
	u32 frequency_div;

	/*
	 * RF loop-through
	 */
	bool loop_through:1;

	/*
	 * clock output
	 */
#define TS2020_CLK_OUT_DISABLED        0
#define TS2020_CLK_OUT_ENABLED         1
#define TS2020_CLK_OUT_ENABLED_XTALOUT 2
	u8 clk_out:2;

	/*
	 * clock output divider
	 * 1 - 31
	 */
	u8 clk_out_div:5;

	/* Set to true to suppress stat polling */
	bool dont_poll:1;

	/*
	 * pointer to DVB frontend
	 */
	struct dvb_frontend *fe;

	/*
	 * driver private, do not set value
	 */
	u8 attach_in_use:1;

	/* Operation to be called by the ts2020 driver to get the value of the
	 * AGC PWM tuner input as theoretically output by the demodulator.
	 */
	int (*get_agc_pwm)(struct dvb_frontend *fe, u8 *_agc_pwm);
};

/**
 * Attach a ts2020 tuner to the supplied frontend structure.
 *
 * @param fe Frontend to attach to.
 * @param addr i2c address of the tuner.
 * @param i2c i2c adapter to use.
 * @return FE pointer on success, NULL on failure.
 */
extern struct dvb_frontend *ts2020_attach(
	struct dvb_frontend *fe,
	const struct ts2020_config *config,
	struct i2c_adapter *i2c);

#endif /* TS2020_H */
