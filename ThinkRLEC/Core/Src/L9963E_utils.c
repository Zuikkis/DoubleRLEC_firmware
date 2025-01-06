/*
 * L9963E_utils.c
 *
 *  Created on: Dec 1, 2024
 *      Author: tsuikki
 *
 *  Heavily based on squadracorsepolito's work
 *
 *  Uses a lot of DRV functions directly, without going through L9963E.c upper level functions.
 *  Only function used from L9963E.c is L9963E_addressing_procedure().
 *  GCC does not link the unused functions so this saves a lot of space on my tiny stm32f042 with
 *  only 32k of flash. Could fit into 16k now if debug printf is disabled (in main.h).
 *
 *  There is a hardware issue in the current boards, L9963T's TXEN-pin is not connected so it always
 *  transmits. This prevents the use of burst mode, but otherwise is not an issue.
 */

#include "main.h"

#include <stdio.h>
#include <string.h>


// some driver level functions we use directly
L9963E_StatusTypeDef _L9963E_DRV_build_frame(uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t, int32_t);
L9963E_StatusTypeDef _L9963E_DRV_spi_transmit(L9963E_DRV_HandleTypeDef *, uint8_t *, uint8_t, uint8_t);
L9963E_StatusTypeDef _L9963E_DRV_wait_and_receive(union L9963E_DRV_FrameUnion *, L9963E_DRV_HandleTypeDef *, uint8_t, int32_t, uint8_t);
void _L9963E_DRV_switch_endianness(uint8_t *, uint8_t *);


L9963E_HandleTypeDef h9l;


struct module_info info[16];

L9963E_IF_PinState GPIO_ReadPin(L9963E_IF_PINS pin) {
    L9963E_IF_PinState state = L9963E_IF_GPIO_PIN_RESET;
    switch (pin) {
        case L9963E_IF_CS:
            state = HAL_GPIO_ReadPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
            break;
        case L9963E_IF_TXEN:
            state = HAL_GPIO_ReadPin(TXEN_GPIO_Port, TXEN_Pin);
            break;
        case L9963E_IF_BNE:
            state = HAL_GPIO_ReadPin(BNE_GPIO_Port, BNE_Pin);
            break;
        case L9963E_IF_ISOFREQ:
            state = HAL_GPIO_ReadPin(ISOFREQ_GPIO_Port, ISOFREQ_Pin);
            break;
        case L9963E_IF_DIS:
            state = HAL_GPIO_ReadPin(DIS_GPIO_Port, DIS_Pin);
            break;
    }

    return state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET : GPIO_PIN_SET;  //convert lib state to stm state
}

L9963E_StatusTypeDef GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    GPIO_PinState stm_state = state == L9963E_IF_GPIO_PIN_RESET ? GPIO_PIN_RESET
                                                                : GPIO_PIN_SET;  //convert lib state to stm state
    switch (pin) {
        case L9963E_IF_CS:
            HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, stm_state);
            break;
        case L9963E_IF_TXEN:
            HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, stm_state);
            break;
        case L9963E_IF_BNE:
            HAL_GPIO_WritePin(BNE_GPIO_Port, BNE_Pin, stm_state);
            break;
        case L9963E_IF_ISOFREQ:
            HAL_GPIO_WritePin(ISOFREQ_GPIO_Port, ISOFREQ_Pin, stm_state);
            break;
        case L9963E_IF_DIS:
            HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, stm_state);
            break;
        default:
            return L9963E_ERROR;
    }
    return L9963E_OK;
}
L9963E_StatusTypeDef SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Receive(&hspi1, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}
L9963E_StatusTypeDef SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    HAL_StatusTypeDef errorcode;

    errorcode = HAL_SPI_Transmit(&hspi1, data, size, timeout_ms);

    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}

const L9963E_IfTypeDef interface = {
    .L9963E_IF_DelayMs = HAL_Delay,
    .L9963E_IF_GetTickMs = HAL_GetTick,
    .L9963E_IF_GPIO_ReadPin = GPIO_ReadPin,
    .L9963E_IF_GPIO_WritePin = GPIO_WritePin,
    .L9963E_IF_SPI_Receive = SPI_Receive,
    .L9963E_IF_SPI_Transmit = SPI_Transmit
};


void L9963E_utils_init(void) {
	L9963E_RegisterUnionTypeDef reg;
	uint8_t x;

	h9l.slave_n = num_rlecs;
	h9l.out_res_tx_iso = 0b11;
	L9963E_DRV_init(&h9l.drv_handle, interface);

	x=0;
	while (L9963E_addressing_procedure(&h9l, 0, 0, 0b11, 0) != L9963E_OK) {
    	DBG("Addressing procedure failed\n");

		// this is often needed if chain is modified
		reg.generic = 0;
		reg.FSM.GO2SLP= 0b10;
		reg.FSM.SW_RST= 0b10;
		L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_FSM_ADDR, &reg, 10);
		HAL_Delay(5);

		x++;
		if (x>10) break;   // continue after a few attempts. It's likely there are boards missing, no waiting will fix that
    }


	// Calibration data is normally read only after power up. Any fault there will NOT fix
	// unless chip is removed from battery, or manually forcing rewrite (as below).
	reg.generic = 0;
	reg.Bal_3.trimming_retrigger= 1;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_3_ADDR, &reg, 10);
	HAL_Delay(15);

	reg.generic = 0;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_3_ADDR, &reg, 10);

	// set GPIO7&8 as ADC inputs
	reg.generic = L9963E_GPIO9_3_CONF_DEFAULT;
	reg.GPIO9_3_CONF.GPIO7_CONFIG = 0;
	reg.GPIO9_3_CONF.GPIO8_CONFIG = 0;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_GPIO9_3_CONF_ADDR, &reg, 10);

	// disable over/undervolt detection. Handled by MLEC, stock RLECs don't have this either
	reg.generic = L9963E_VCELL_THRESH_UV_OV_DEFAULT;
	reg.VCELL_THRESH_UV_OV.threshVcellOV = 0xff;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_VCELL_THRESH_UV_OV_ADDR, &reg, 10);

	reg.generic = L9963E_VBATT_SUM_TH_DEFAULT;
	reg.VBATT_SUM_TH.VBATT_SUM_OV_TH = 0xff;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_VBATT_SUM_TH_ADDR, &reg, 10);

	reg.generic = 0x7f;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_CSA_GPIO_MSK_ADDR, &reg, 10);

	// enable VTREF
    reg.generic = L9963E_NCYCLE_PROG_2_DEFAULT;
    reg.NCYCLE_PROG_2.VTREF_EN = 1;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_NCYCLE_PROG_2_ADDR, &reg, 10);

	// set comm timeout
    reg.generic = L9963E_FASTCH_BALUV_DEFAULT;
    reg.fastch_baluv.CommTimeout = _2048MS;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_fastch_baluv_ADDR, &reg, 10);

	// set enabled cells
    reg.generic = 0x3F3F;
	L9963E_DRV_reg_write(&(h9l.drv_handle), L9963E_DEVICE_BROADCAST, L9963E_VCELLS_EN_ADDR, &reg, 10);

}

/*
 * set balance bits and restart balancing
 *
 * doesn't need to be called very often. Every 10-60 sec?
 */
void L9963E_set_balance(uint8_t device) {
  L9963E_RegisterUnionTypeDef balcell6_1act_reg;
  L9963E_RegisterUnionTypeDef balcell14_7act_reg;
  L9963E_RegisterUnionTypeDef bal_1_reg;
  uint16_t balance_v;
  struct module_info *mod=&info[rlec_ids[device]];

  // devices start really from 1
  device++;

  // stop previous balancing
  bal_1_reg.generic = L9963E_BAL_1_DEFAULT;
  bal_1_reg.Bal_1.bal_start=0;
  bal_1_reg.Bal_1.bal_stop=1;

  L9963E_DRV_reg_write(&(h9l.drv_handle), device, L9963E_Bal_1_ADDR, &bal_1_reg, 10);

  // set balancing.
  // if any active faults, balancing is disabled
  // hard limit 2500mV
  balance_v=(can_broadcast[2][2]<<8) + can_broadcast[2][3];
  if (!balance_v || mod->fault) balance_v=0xffff;
  if (balance_v < (2500000/2440)) balance_v=2500000/2440;
  //DBG ("balance target: %d mV\n", balance_v*2440/1000);

  balcell6_1act_reg.generic=0;
  if (mod->vcells[0]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL1=0b10;
  if (mod->vcells[1]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL2=0b10;
  if (mod->vcells[2]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL3=0b10;
  if (mod->vcells[3]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL4=0b10;
  if (mod->vcells[4]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL5=0b10;
  if (mod->vcells[5]>balance_v) balcell6_1act_reg.BalCell6_1act.BAL6=0b10;

  balcell14_7act_reg.generic=0;
  if (mod->vcells[6]>balance_v)  balcell14_7act_reg.BalCell14_7act.BAL9= 0b10;
  if (mod->vcells[7]>balance_v)  balcell14_7act_reg.BalCell14_7act.BAL10=0b10;
  if (mod->vcells[8]>balance_v)  balcell14_7act_reg.BalCell14_7act.BAL11=0b10;
  if (mod->vcells[9]>balance_v)  balcell14_7act_reg.BalCell14_7act.BAL12=0b10;
  if (mod->vcells[10]>balance_v) balcell14_7act_reg.BalCell14_7act.BAL13=0b10;
  if (mod->vcells[11]>balance_v) balcell14_7act_reg.BalCell14_7act.BAL14=0b10;

  L9963E_DRV_reg_write(&(h9l.drv_handle), device, L9963E_BalCell6_1act_ADDR, &balcell6_1act_reg, 10);
  L9963E_DRV_reg_write(&(h9l.drv_handle), device, L9963E_BalCell14_7act_ADDR, &balcell14_7act_reg, 10);

  // start balancing, if needed
  if (balcell6_1act_reg.generic | balcell14_7act_reg.generic) {
	  bal_1_reg.Bal_1.bal_start=1;
	  bal_1_reg.Bal_1.bal_stop=0;

	  L9963E_DRV_reg_write(&(h9l.drv_handle), device, L9963E_Bal_1_ADDR, &bal_1_reg, 10);
  }

}

/*
 * simple NTC approximation
 *
 * This is +-1C accurate between -20C .. 60C and more inaccurate elsewhere.
 */

int8_t ntcconv(uint16_t adc)
{
	uint32_t resistance;

	// calculate resistance in ohm
	// Vadc = adc*89uV (from datasheet)
	// Vref = 4.958V   (from datasheet)
	// adc_max = 55708 (4958000/89)
	// Rntc = (Vadc * 49900ohm) / (Vref - Vadc)
	// Rntc = (adc * 49900ohm) / (adc_max - adc)

	if (adc>=55708) return -99;

	resistance= 2*(adc*49900U / (55708-adc));

	// below code is for 100K ntc so we multiple by 2 above..

	if (resistance >= 540000) { 										// -10°C
		return -10- (10 * (resistance - 540000)) / (940000 - 540000);
	} else if (resistance >= 320000) { 									// 0°C
		return 0  - (10 * (resistance - 320000)) / (540000 - 320000);
	} else if (resistance >= 198000) { 									// 10°C
		return 10 - (10 * (resistance - 198000)) / (320000 - 198000);
	} else if (resistance >= 125000) { 									// 20°C
		return 20 - (10 * (resistance - 125000)) / (198000 - 125000);
	} else if (resistance >= 80000) {  									// 30°C
		return 30 - (10 * (resistance - 80000)) / (125000 - 80000);
	} else if (resistance >= 53300) {  									// 40°C
		return 40 - (10 * (resistance - 53300)) / (80000 - 53300);
	} else if (resistance >= 35800) {  									// 50°C
		return 50 - (10 * (resistance - 35800)) / (53300 - 35800);
	} else {															// above 50°C
		return 50 + (10 * (35800 - resistance)) / (35800 - 24700);
	}

}

/*
 * transmit and receive at the same time. Receive lags one cycle behind.
 */
uint8_t prevaddr;

int32_t L9963E_utils_txrx(uint8_t device, uint8_t addr, uint8_t first) {
	L9963E_StatusTypeDef e;
	uint8_t tx[5], rx[5];
	uint32_t current_tick;
	union L9963E_DRV_FrameUnion frame;

	_L9963E_DRV_build_frame(tx, 1, 0, device, addr, 0);

	L9963E_DRV_CS_LOW(&(h9l.drv_handle));
	e = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 10);
	L9963E_DRV_CS_HIGH(&(h9l.drv_handle));

    if (e != L9963E_OK) {
    	DBG("SPI error\n");
    	return -1;
    }

	// wait for RX data
	current_tick=L9963E_DRV_GETTICK(&(h9l.drv_handle));
	while (L9963E_DRV_BNE_READ(&(h9l.drv_handle)) == L9963E_IF_GPIO_PIN_RESET) {
		if (L9963E_DRV_GETTICK(&(h9l.drv_handle)) - current_tick >= 10) {
			DBG("Timeout:    device: %02d, addr: %02x\n",device, addr);
			return -1;
		}
	}

	// nothing to receive if first call
	if (first) {
		prevaddr=addr;
		return 0;
	}

	_L9963E_DRV_switch_endianness(rx, (uint8_t *)&frame.val);

	if (frame.cmd.crc != L9963E_DRV_crc_calc(frame.val)) {
		DBG("CRC error:  device: %02d, prevaddr: %02x, addr: %02x, crc %02x\n", device, prevaddr,frame.cmd.addr,frame.cmd.crc);
		return -1;
	}

	if (frame.cmd.addr != prevaddr) {
		DBG("addr error: device: %02d, prevaddr: %02x, addr: %02x, crc %02x\n", device, prevaddr,frame.cmd.addr,frame.cmd.crc);
		return -1;
	}
	prevaddr=addr;

	return frame.cmd.data;
}

// calibration data for cell voltages. Mainly highest cell is incorrect because it supplies L9963E power,
// but I measured many boards with multimeter and there is few mV consistent error in other cells too.
// RLEC has only 2.44mV accuracy anyway so this can't be too exact..
// These are in 2.44mV units.
static int8_t calib_v[12] = { 0, 0, 0, 0, 0, -1, 2, 0, 0, 0, 0, 11 };

/*
 * Read all cell voltages and other info, convert to RLEC format.
 */
void L9963E_utils_read_cells(uint8_t device, uint8_t debug) {
  uint8_t x, addr, not_fresh=0;
  int32_t res, adcv_conv=0;
  struct module_info *mod=&info[rlec_ids[device]];

  // devices start really from 1
  device++;

  // increment timeout counter. it's cleared on successfull read.
  // this way we can exit on any failure without cleanup
  mod->timeout++;
  mod->fault=0;

  // if timeout>20 (about two seconds) return fail
  if (mod->timeout > 20) {
	  mod->fault|=0x10;		// --> Cell voltage A/D fault
	  mod->timeout=21;
  }

  // read all needed registers. we use HAL_SPI_TransmitReceive to transmit and receive at the same time.
  // Receive is lagging one cycle behind of transmit.
  // data is converted to RLEC CAN format on the fly.

  res=L9963E_utils_txrx(device, 0x0D, 1);	// ADCV_CONV
  if (res<0) return;
  res=L9963E_utils_txrx(device, 0x10, 0);	// BalCell14_7act
  if (res<0) return;

  adcv_conv=res;

  res=L9963E_utils_txrx(device, 0x11, 0);	// BalCell6_1act
  if (res<0) return;

  mod->balcell =	(((res>>14)&3)==2 ? 0x800 : 0) +
					(((res>>12)&3)==2 ? 0x400 : 0) +
					(((res>>10)&3)==2 ? 0x200 : 0) +
					(((res>> 8)&3)==2 ? 0x100 : 0) +
					(((res>> 6)&3)==2 ? 0x080 : 0) +
					(((res>> 4)&3)==2 ? 0x040 : 0);

  res=L9963E_utils_txrx(device, 0x21, 0);	// Vcell1
  if (res<0) return;

  mod->balcell|=	(((res>>14)&3)==2 ? 0x020 : 0) +
					(((res>>12)&3)==2 ? 0x010 : 0) +
					(((res>>10)&3)==2 ? 0x008 : 0) +
					(((res>> 8)&3)==2 ? 0x004 : 0) +
					(((res>> 6)&3)==2 ? 0x002 : 0) +
					(((res>> 4)&3)==2 ? 0x001 : 0);

  for (x=0;x<12 ;x++) {
		if (x<5)
			addr=x + 0x22;				// Vcell2-6
		else if (x<11)
			addr=x + 0x24;				// Vcell9-14
		else
			addr=0x34;					// GPIO3_MEAS

		res=L9963E_utils_txrx(device, addr, 0);
		if (res<0) return;

		if (!(res & 0x10000)) not_fresh++;

		mod->vcells[x]=(res & 0xffff)*89/2440 + calib_v[x];  // L9963E volts are 89uV per unit, RLEC volts are 2440uV per unit
  }

  for (x=0;x<7 ;x++) {
		addr=x + 0x35;				// GPIO4-9_MEAS, TempChip

		res=L9963E_utils_txrx(device, addr, 0);
		if (res<0) return;

		if (!(res & 0x10000)) not_fresh++;

		mod->vgpio[x]=ntcconv(res & 0xffff);
  }

  res=L9963E_utils_txrx(device, 0x41, 0);	// VBATTDIV
  if (res<0) return;

  mod->tempchip=(((int8_t)(res&0xff))*13828+997330)/10000;

  res=L9963E_utils_txrx(device, 0x3C, 0);	// Faults1
  if (res<0) return;

  mod->vtot=(res & 0xffff)*133/1220;		// L9963E volts are 1330uV per unit, RLEC volts are 12200uV per unit

  res=L9963E_utils_txrx(device, 0x1E, 0);	// Faultmask2
  if (res<0) return;

  // if OVR_LATCH or FAULTL, read all fault codes as well
  // these are mapped to RLEC fault in some way
  if ((adcv_conv & 0x4000) || (res&8)) {
	  if (debug) DBG("DEVICE %d ERROR:\n" , device);
	  if (debug) DBG("  0d: %05lx\n", adcv_conv);
	  if (debug) DBG("  3c: %05lx\n", res);

	  res=L9963E_utils_txrx(device, 0x3D, 0);
	  if (res<0) return;

	  if (debug) DBG("  1e: %05lx\n", res);

	  if (res&0x15400) mod->fault|=0x32;						// calibration data corruct, all AD invalid

	  for (addr=0x3E;addr<0x4E ;addr++) {
		  res=L9963E_utils_txrx(device, addr, 0);
		  if (res<0) return;

		  if (((addr-1) == 0x3E) && (res&0x0fffc)) mod->fault|=0x08;		// BAL_OPEN                 --> Cell voltage connection fault
		  if (((addr-1) == 0x3F) && (res&0x0fffc)) mod->fault|=0x08;		// BAL_SHORT                --> Cell voltage connection fault
		  if (((addr-1) == 0x42) && (res&0x03fff)) mod->fault|=0x08;		// CELL_OPEN                --> Cell voltage connection fault
		  if (((addr-1) == 0x47) && (res&0x03f80)) mod->fault|=0x02;		// GPIOx_OPEN               --> Cell temp A/D fault
		  if (((addr-1) == 0x48) && (res&0x03fff)) mod->fault|=0x10;		// MUX_BIST_FAIL            --> Cell voltage A/D fault
		  if (((addr-1) == 0x49) && (res&0x38000)) mod->fault|=0x20;		// VBAT/VREG/VCOM_BIST_FAIL --> Module voltage A/D fault
		  if (((addr-1) == 0x49) && (res&0x04000)) mod->fault|=0x02;		// VTREF_COMP_BIST_FAIL     --> Cell temp A/D fault
		  if (((addr-1) == 0x49) && (res&0x03FFF)) mod->fault|=0x10;		// BIST_BAL_COMP_x          --> Cell voltage A/D fault
		  if (((addr-1) == 0x4A) && (res&0x03FFF)) mod->fault|=0x10;		// OPEN_BIST_FAIL           --> Cell voltage A/D fault
		  if (((addr-1) == 0x4B) && (res&0x000FF)) mod->fault|=0x02;		// GPIO_BIST_FAIL           --> Cell temp A/D fault

		  if (debug) DBG("  %02x: %05lx\n",addr-1,res);
	  }

  }

  // if new data received successfully, clear timeout
  if (!not_fresh) {
	  mod->timeout=0;
  } else {
	  DBG("not fresh\n");
  }
}


