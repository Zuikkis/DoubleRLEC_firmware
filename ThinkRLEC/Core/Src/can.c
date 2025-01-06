/*
 * can.c
 *
 *  Created on: Nov 19, 2024
 *      Author: zuikkis
 *
 *  All canbus logic is here. Can is entirely interrupt-driven, all TX actions are triggered by RX.
 *
 *  Most of this is based on "RLEC_programmers_guide.pdf" with some changes based on actual MLEC/RLEC behaviour.
 *
 */


#include "main.h"

#include <stdio.h>

CAN_RxHeaderTypeDef rxHeader; 			//CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; 			//CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  	//CAN Bus Receive Buffer
uint8_t canTX[8] = {0,0,0,0,0,0,0,0};  	//CAN Bus Receive Buffer
uint32_t txMailbox;

CAN_FilterTypeDef canfil; 				//CAN Bus Filter

uint8_t can_challenge=0;				// rlec security challenge state

uint8_t can_rlec=0;						// rlec number currently in use, 0-15
uint8_t can_msgmask=0;					// message bitmask for receiver to calculate when all 4 packets received
uint16_t can_txid=0;					// currently transmitting this id

uint8_t can_broadcast[6][8];			// 0x7e1-0x7e6 broadcast messages are stored here

uint8_t num_rlecs=16;					// number of rlecs (2-16), must be even

uint8_t rlec_ids[16]= {0, 1, 3, 2, 8, 9, 0xB, 0xA, 0xC, 0xD, 0xF, 0xE, 4, 5, 7, 6};

/**
 * Flash read&write
 *
 * RLEC id's are stored in the last page of the 32kB flash memory.
 * There is a simple xor check so invalid data is not used.
 */
void flash_write() {
	int x;
	HAL_FLASH_Unlock();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = 0x08007c00;
	FLASH->CR |= FLASH_CR_STRT;
	while ((FLASH->SR & FLASH_SR_BSY) != 0)
	{/* For robust implementation, add here time-out management */}

	if ((FLASH->SR & FLASH_SR_EOP) != 0)
	{
		FLASH->SR |= FLASH_SR_EOP;
	}

	FLASH->CR &= ~FLASH_CR_PER;


	for (x=0;x<16;x++) {
		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x08007C00+x*2, ((rlec_ids[x]<<8) | (rlec_ids[x]))^0x55AA );
	}

	HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x08007C20, ((num_rlecs<<8) | (num_rlecs))^0x55AA );

	HAL_FLASH_Lock();
}

void flash_read() {
	int x,error=0;
	uint16_t num=0;
	uint16_t data[16];

	DBG("Reading configuration from flash\n");

	num=*((uint16_t *) 0x08007C20) ^0x55AA;
	if ((num>>8) != (num & 0xff)) error=1;

	for (x=0;x<16;x++) {
		data[x]=*((uint16_t *) (0x08007C00+x*2)) ^0x55AA;

		if ((data[x]>>8) != (data[x] & 0xff)) error=1;
	}

	if (error) {
		DBG ("Invalid data, reverting to defaults\n");
	} else {
		num_rlecs=num&0xff;
		for (x=0;x<num_rlecs;x++) {
			rlec_ids[x]=data[x]&0xff;
			DBG("  RLEC id %02d: %02x\n",x,rlec_ids[x]);
		}

	}

}

/**
 *  CAN init
 */

void can_init(void) {

	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfil.FilterIdHigh = 0x7e0<<5;
	canfil.FilterMaskIdHigh = 0xfe8<<5;
	canfil.FilterIdLow = 0x400<<5;
	canfil.FilterMaskIdLow = 0xe00<<5;
	canfil.FilterScale = CAN_FILTERSCALE_16BIT;
	canfil.FilterActivation = ENABLE;

	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_ConfigFilter(&hcan,&canfil);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_TX_MAILBOX_EMPTY);
}

/*
 * CAN transmitter. Called initially from RX, then automatically increments id when previous packet is finished.
 */

void can_send(void) {
	uint16_t offs=0,x;
	int16_t minvolt, maxvolt;

	if (!can_txid) return;

	if (can_txid<0x600) {
		txHeader.StdId=can_txid;

		switch (can_txid & 0xf) {
			case 3:
			case 7:
			case 10:
				offs+=4;

			case 2:
			case 6:
			case 9:
				offs+=4;

			case 1:				// 1-3: cell voltages
			case 5:				// 5-7: unfiltered cell voltages (not used by anyone)
			case 8:				// 8-10: raw ADC cell voltages (not used by anyone)
				canTX[0]=info[can_rlec].vcells[offs + 0]>>8;
				canTX[1]=info[can_rlec].vcells[offs + 0] & 0xff;
				canTX[2]=info[can_rlec].vcells[offs + 1]>>8;
				canTX[3]=info[can_rlec].vcells[offs + 1] & 0xff;
				canTX[4]=info[can_rlec].vcells[offs + 2]>>8;
				canTX[5]=info[can_rlec].vcells[offs + 2] & 0xff;
				canTX[6]=info[can_rlec].vcells[offs + 3]>>8;
				canTX[7]=info[can_rlec].vcells[offs + 3] & 0xff;

				break;

			case 4:				// maxvolt/minvolt, rlec board temp, balance info, faults
				minvolt=0x7fff;
				maxvolt=-0x7fff;
				for(x=0;x<12;x++) {
					if (info[can_rlec].vcells[x]>maxvolt) maxvolt=info[can_rlec].vcells[x];
					if (info[can_rlec].vcells[x]<minvolt) minvolt=info[can_rlec].vcells[x];
				}
				canTX[0]=maxvolt >> 8;							// maxvolts
				canTX[1]=maxvolt &  0xff;
				canTX[2]=minvolt >> 8;							// minvolts
				canTX[3]=minvolt &  0xff;
				canTX[4]=info[can_rlec].tempchip;				// RLEC temp
				canTX[5]=info[can_rlec].balcell>>8;				// balance info
				canTX[6]=info[can_rlec].balcell & 0xff;
				canTX[7]=info[can_rlec].fault;					// fault status
				break;

			case 11:
				canTX[0]=info[can_rlec].vcells[0]>>8;			// redundant cell 1 volts
				canTX[1]=info[can_rlec].vcells[0] & 0xff;
				canTX[2]=0;										// zero cap voltage raw adc
				canTX[3]=0;
				canTX[4]=info[can_rlec].vtot >> 8;				// module voltage raw adc
				canTX[5]=info[can_rlec].vtot & 0xff;
				canTX[6]=info[can_rlec].vtot >> 8;				// filtered module voltage
				canTX[7]=info[can_rlec].vtot & 0xff;
				break;

			case 12:							// cell temperatures
				canTX[0]=info[can_rlec].vgpio[0];
				canTX[1]=info[can_rlec].vgpio[0];
				canTX[2]=info[can_rlec].vgpio[1];
				canTX[3]=info[can_rlec].vgpio[1];
				canTX[4]=info[can_rlec].vgpio[2];
				canTX[5]=info[can_rlec].vgpio[2];
				canTX[6]=info[can_rlec].vgpio[3];
				canTX[7]=info[can_rlec].vgpio[3];
				break;

			case 13:							// cell temperatures
				minvolt=0x7fff;
				maxvolt=-0x7fff;
				for(x=0;x<6;x++) {
					if (info[can_rlec].vgpio[x]>maxvolt) maxvolt=info[can_rlec].vgpio[x];
					if (info[can_rlec].vgpio[x]<minvolt) minvolt=info[can_rlec].vgpio[x];
				}
				canTX[0]=info[can_rlec].vgpio[4];
				canTX[1]=info[can_rlec].vgpio[4];
				canTX[2]=info[can_rlec].vgpio[5];
				canTX[3]=info[can_rlec].vgpio[5];
				canTX[4]=maxvolt;				// max temp
				canTX[5]=minvolt;				// min temp
				canTX[6]=0;						// filtered heater temp
				canTX[7]=13;					// RLEC software build number (12=stock, 13=us)
				break;

		}

	// RLEC id changing responses
	} else {
		txHeader.StdId=0x600;

		switch (can_txid) {
			case 0x600:
				canTX[0]=0x0d;
				canTX[1]=0x01;
				canTX[2]=0xaa;
				canTX[3]=0x12;					// security challenge, always 1234 because I'm lazy
				canTX[4]=0x34;
				canTX[5]=0;
				canTX[6]=0;
				canTX[7]=0;
				break;

			case 0x601:
				canTX[0]=0x0d;
				canTX[1]=0x02;
				canTX[2]=0xaa;
				canTX[3]=0;
				canTX[4]=0;
				canTX[5]=0;
				canTX[6]=0;
				canTX[7]=0;
				break;

			case 0x602:
				canTX[0]=0x04;
				canTX[1]=0x15;
				canTX[2]=0xaa;
				canTX[3]=0;
				canTX[4]=0;
				canTX[5]=0;
				canTX[6]=0;
				canTX[7]=0;
				break;
		}

	}

	HAL_CAN_AddTxMessage(&hcan, &txHeader, canTX, &txMailbox);

	can_txid++;
	if ((can_txid&0xfe0f)>=14) can_txid=0;

}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan1) {
	can_send();
}

/*
 * CAN receiver.
 *
 * We pretend to be multiple RLECs. Current identity is stored in "can_rlec" variable.
 *
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	int new_rlec,x,y;

	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);

	//DBG("%04x:  %02x %02x %02x %02x %02x %02x %02x %02x\n",(int)rxHeader.StdId, canRX[0],canRX[1],canRX[2],canRX[3],canRX[4],canRX[5],canRX[6],canRX[7]);

	// 0x7e0 = RLEC id setting.
	// there's a simple key challenge protection to prevent accidental setting.
	// We are using always key 0x1234 which should give result 0xbedc.
	// Real RLECs have random key.
	//
	// The last message originally contains only single RLEC id, x=0-15:
	// 7E0 04 15 0x 00 00 00 00 00
	//            ^
	//
	// we use expanded syntax to set IDs for entire chain:
	// 7E0 04 15 0n ab cd ef gh AA    <-- AA=mark of the new format
	//            ^ -- -- -- --
	// number of  |            \ 1-8 board IDs, 4 bits per each.
	// DoubleRLEC
	// boards in chain (1-8)
	//
	// The command only sets the RLEC id closest to the connector.
	// The other RLEC id is set by standard Enerdel RLEC numbering:
	//
	// -------   -------     <-- front of the car
	// |6   7|   |0   1|
	// -------   -------
	// |5   4|   |3   2|
	// -------   -------
	// |E   F|   |8   9|
	// -------   -------
	// |D   C|   |B   A|
	// -------   -------
	//
	// So, only legal RLEC id's for this function are 038BCF47.

	// example to set full 8 modules in standard order;
	// 7E0 04 15 08 03 8B CF 47 AA
	//
	// this is also the default if nothing else is programmed.

	if ((rxHeader.StdId==0x7e0) && (canRX[0]==0xd) && (canRX[1]==0x1))
	{
		can_challenge=1;
		can_txid=0x600;
		can_send();
	}
	else if ((rxHeader.StdId==0x7e0) && (canRX[0]==0xd) && (canRX[1]==0x2) && (canRX[2]==0xbe) && (canRX[3]==0xdc) && (can_challenge==1))
	{
		can_challenge=2;
		can_txid=0x601;
		can_send();
	}
	else if ((rxHeader.StdId==0x7e0) && (canRX[0]==0x4) && (canRX[1]==0x15) && (can_challenge==2) )
	{
		can_challenge=0;
		can_txid=0;
		if (canRX[7]==0xAA) {
			can_txid=0x602;
			num_rlecs=canRX[2]*2;

			rlec_ids[0]= canRX[3] >> 4;
			rlec_ids[2]= canRX[3] & 0xf;
			rlec_ids[4]= canRX[4] >> 4;
			rlec_ids[6]= canRX[4] & 0xf;
			rlec_ids[8]= canRX[5] >> 4;
			rlec_ids[10]=canRX[5] & 0xf;
			rlec_ids[12]=canRX[6] >> 4;
			rlec_ids[14]=canRX[6] & 0xf;

		} else if (!canRX[3] && !canRX[4] && !canRX[5] && !canRX[6] && !canRX[7]) {
			can_txid=0x602;
			num_rlecs=2;
			rlec_ids[0]=canRX[2];
		}

		// set odd rlec numbers, send response, write flash
		if (can_txid) {
			for (x=0;x<16;x+=2) rlec_ids[x+1] = (rlec_ids[x]&1) ? rlec_ids[x]-1 : rlec_ids[x]+1;
			can_send();
			flash_write();
		}
	}

	//
	// MLEC sends bunch of broadcast messages with CAN id 0x7e1-0x7e6. These contain information needed for cell balancing.
	// We store these messages. Main loop picks up the data from there.
	//
	else if (rxHeader.StdId>0x7e0)
	{
		x=rxHeader.StdId-0x7e1;
		if (x<6) for (y=0;y<8;y++) can_broadcast[x][y] = canRX[y];
	}

	//
	// Then MLEC sends four "data request messages" to each RLEC. After receiving all four messages, RLEC starts sending its payload in 13 messages.
	// MLEC data request messages contain no useful information, so we just them record them in can_msgmask so we know when all four have arrived.
	//
	else if ((rxHeader.StdId & 0xe00) == 0x400)
	{

		new_rlec=(rxHeader.StdId & 0x1e0)>>5;

		// If RLEC number changes, clear can_msgmask and start waiting for new 4 messages
		// It is assumed MLEC only talks to one RLEC at a time.
		// Only accept RLECs that are on our rlec_ids list.
		if (can_rlec!=new_rlec) {
			for (x=0;x<num_rlecs;x++) {
				if (rlec_ids[x]==new_rlec) {
					can_rlec=new_rlec;
				}
			}
			can_msgmask=0;
		}

		if (can_rlec==new_rlec) {
			if ((rxHeader.StdId & 0xf) == 0x6) can_msgmask|=1;
			if ((rxHeader.StdId & 0xf) == 0xa) can_msgmask|=2;
			if ((rxHeader.StdId & 0xf) == 0xb) can_msgmask|=4;
			if ((rxHeader.StdId & 0xf) == 0xc) can_msgmask|=8;
		}

		// all 4 received, start sending our response
		if (can_msgmask==15) {
			can_msgmask=0;
			can_txid=(can_rlec<<5)+1;
			//DBG("sending 0x%04x\n",can_txid);
			can_send();
		}

	}
}

