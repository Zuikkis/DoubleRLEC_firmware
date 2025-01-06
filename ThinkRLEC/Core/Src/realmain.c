/*
 * realmain.c
 *
 *  Created on: Nov 19, 2024
 *      Author: tsuikki
 *
 *  Main level loop that reads chip data into the table.
 *
 */


#include "main.h"
#include <stdio.h>
#include <math.h>



/**
 *  Real main. Never returns
 */

void realmain(void) {
	uint8_t x, balance_cnt1=0, balance_cnt2=0, debug=0,ch;
	uint16_t *vcells,balance_v;
	int8_t *vgpio;
	uint32_t start, stop;
	L9963E_RegisterUnionTypeDef adcv_conv_reg = {0};

	adcv_conv_reg.ADCV_CONV.SOC            = 1;
	adcv_conv_reg.ADCV_CONV.GPIO_CONV      = 1;
	adcv_conv_reg.ADCV_CONV.GPIO_TERM_CONV = 1;
	adcv_conv_reg.ADCV_CONV.BAL_TERM_CONV  = 1;
	adcv_conv_reg.ADCV_CONV.CELL_TERM_CONV = 1;
	adcv_conv_reg.ADCV_CONV.HWSC           = 1;
	adcv_conv_reg.ADCV_CONV.ADC_FILTER_SOC = 0b010;

	HAL_Delay(1000);
	HAL_IWDG_Refresh(&hiwdg);

	flash_read();

	DBG ("can init\n");
	can_init();

	DBG ("L9963E init\n");
	L9963E_utils_init();

	while (1) {
		start=HAL_GetTick();
		HAL_IWDG_Refresh(&hiwdg);

		if (debug&1) {
			for (x=0;x<6;x++)
				DBG("%04x: %02x %02x %02x %02x %02x %02x %02x %02x\n",x+0x7e1, can_broadcast[x][0], can_broadcast[x][1], can_broadcast[x][2], can_broadcast[x][3], can_broadcast[x][4], can_broadcast[x][5], can_broadcast[x][6], can_broadcast[x][7]);

			balance_v=(can_broadcast[4][2]<<8) + can_broadcast[4][3];
			DBG ("Min cell volt balance limit: %d mV\n", balance_v*2440/1000);

			balance_v=(can_broadcast[4][4]<<8) + can_broadcast[4][5];
			DBG ("Low cell volt balance limit: %d mV\n", balance_v*2440/1000);

			balance_v=(can_broadcast[2][0]<<8) + can_broadcast[2][1];
			DBG ("balance target upper: %d mV\n", balance_v*2440/1000);

			balance_v=(can_broadcast[2][2]<<8) + can_broadcast[2][3];
			DBG ("balance target lower: %d mV\n", balance_v*2440/1000);

			debug&=0xff-1;
		}

		for (x=0;x<num_rlecs;x++) {
			// balance one module every cycle, except not always. :)
			if ((x==balance_cnt1) && !balance_cnt2)
				L9963E_set_balance(x);

			L9963E_utils_read_cells(x, debug & 8);

			if (debug&2) {
				vcells=info[rlec_ids[x]].vcells;
				vgpio=info[rlec_ids[x]].vgpio;

				DBG ("RLEC %d (=%d):\n",x,rlec_ids[x]);

				DBG("  Volts: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",
					 vcells[0]*2440/1000, vcells[1]*2440/1000, vcells[2]*2440/1000, vcells[3]*2440/1000, vcells[4]*2440/1000, vcells[5]*2440/1000,
					 vcells[6]*2440/1000, vcells[7]*2440/1000, vcells[8]*2440/1000, vcells[9]*2440/1000, vcells[10]*2440/1000, vcells[11]*2440/1000);

				DBG("  Temps: %4d %4d %4d %4d %4d %4d %4d\n",
					vgpio[0], vgpio[1], vgpio[2], vgpio[3], vgpio[4], vgpio[5], vgpio[6]);

				DBG("  faults: %02x   timeout: %d\n", info[rlec_ids[x]].fault, info[rlec_ids[x]].timeout);
			}

			// start voltage conversion with all possible tests
			// MLEC polls at 100ms interval and full conversion is only 45ms, and all RLECs run in parallel
			L9963E_DRV_reg_write(&(h9l.drv_handle), x+1 , L9963E_ADCV_CONV_ADDR, &adcv_conv_reg, 10);
		}

		debug&=0xff-2;

		// there's two kinds of Cell open diagnostics. Toggle bit so we test both cases.
		if (adcv_conv_reg.ADCV_CONV.ADC_CROSS_CHECK)
			adcv_conv_reg.ADCV_CONV.ADC_CROSS_CHECK=0;
		else
			adcv_conv_reg.ADCV_CONV.ADC_CROSS_CHECK=1;

		// if data available in UART, read and set debug modes
		if (DEBUG) {
			ch=0;
			HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 1);
			if ((ch>='1') && (ch<='8')) debug|=(1<<(ch-'1'));
			else if (ch) {
				DBG("Debug usage, press 1-8 to enable various debug\n");
			}
		}

		// increment balance counter
		if (balance_cnt2++ > 20) {
			balance_cnt2=0;
			if (balance_cnt1++ >= num_rlecs) balance_cnt1=0;
		}

		// aim for about 90-99ms loop time
		// will be slightly slower if all 16 modules
		stop=HAL_GetTick();
		if (stop-start < 90) {
			HAL_Delay(99-(stop-start));
		}
		if (debug&4) {
			DBG("Loop execution time %ld ms\n",stop-start);
			debug&=0xff-4;

		}

	}


}


