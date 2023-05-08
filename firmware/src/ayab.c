/**
  ******************************************************************************
  * @file    ayab.c
  * @brief   This file provides code for the interface to the AYAB software.
  * @author  matei
  ******************************************************************************
  */

/* Includes */
#include "ayab.h"
#include "string.h"
#include "usbd_cdc_if.h"

extern uint8_t *ayabBuf;
extern uint8_t *bitPattern;

extern uint8_t EOL_result[2];

extern uint8_t  carriageType;
extern uint8_t  machineType;
extern uint8_t  machineState;
extern uint8_t  machineInitialized;
extern uint8_t  machineStart;

extern uint8_t  startNeedle;
extern uint8_t  stopNeedle;
extern uint8_t  currentDir;
extern uint8_t  currentPosition;
extern uint8_t  currentRow;
extern uint8_t  lastRow;
extern uint8_t  crc8_cs;

char debugString[128];

/**
 * @brief   Parse the data sent from AYAB software.
 * @param   none
 * @retval  none
 * 
 * This function interacts with the AYAB API v6 to receive serial data
 * from the PC and then configure the state of the hardware.
*/
void rxAYAB(void)
{
	switch (*ayabBuf)
	{
	case reqStart:
        /* AYAB software requests to start a new image. 
            Provide machine type and worked needles */
		machineType =   ayabBuf[1];
		startNeedle =   ayabBuf[2];
		stopNeedle =    ayabBuf[3];
        txAYAB(cnfStart);
		break;

	case cnfLine:
        /* Send information about a new row from the PC. */
		currentRow =     ayabBuf[1];
        /* Don't want to use pointer because next packet will clobber ayabBuf. */
		memcpy(bitPattern, &ayabBuf[2], 25); 
		lastRow =       ayabBuf[27];
		crc8_cs =       ayabBuf[28];
		break;

	case reqInfo:
        /* First part of handshake, request firmware information */
		txAYAB(cnfInfo);
		break;

	default:
		break;
	}
}

/**
 * @brief   Send data to the AYAB software.
 * @param   A command byte from the API table indicating what is being sent.
 * @retval  none
 * 
 * This function provides data back to the AYAB software. This data can be
 * requested by the software or the controller can request new information
 * from the PC (i.e. getting a new row).
*/
void txAYAB(uint8_t cmd)
{
    /* Should we check if the USB tx is busy? */
    uint8_t ayabPacket[64]; /* Don't use more than one USB packet. */
    ayabPacket[0] = cmd;    /* First byte of the message is the ID */

	switch (cmd)
	{
	case cnfStart:
        /* Confirm that the pattern parameters were received. 
        as long as the needles are within real values (initialized to 0xFF)
        then we can just return that a valid config was received. */
        if(startNeedle <= 198 && stopNeedle <= 200){
            ayabPacket[1] = 0x01;
            machineState = 1; /* HOMING */
        }
        else{
            ayabPacket[1] = 0x00;
        }

        slipSend((uint8_t *)ayabPacket, 2);
		break;

	case reqLine:
		/* Request a new line from the software */
        ayabPacket[1] = currentRow + 1;
        slipSend((uint8_t *)ayabPacket, 2);
        break;

	case cnfInfo:
        /* Respond to reqInfo with firmware version identifier, major version, minor version. */
        ayabPacket[1] = versionAPI;
        ayabPacket[2] = versionMajor;
        ayabPacket[3] = versionMinor;
        slipSend((uint8_t *)ayabPacket, 4);
		break;

	case indState:
        /* Indicate if initialized or not. */
        ayabPacket[1] = machineInitialized;

        /* EOL sensor states */
        ayabPacket[2] = EOL_result[0] >> 8;
        ayabPacket[3] = EOL_result[0] & 0xFF;
        ayabPacket[4] = EOL_result[1] >> 8;
        ayabPacket[5] = EOL_result[1] & 0xFF;

        /* Machine information */
        ayabPacket[6] = carriageType;
        ayabPacket[7] = machineType;
        ayabPacket[8] = currentDir;

        slipSend((uint8_t *)ayabPacket, 9);
		break;

	case debug:
        /* Send the debug string to the software */
        ayabPacket[1] = 0xFF;
		break;

	default:
		break;
	}
}

void slipSend(uint8_t *packet, uint8_t len){
    /* The pointer should be to first item... so we can just add bytes at the end */
    packet[len] = '\r';
    packet[len+1] = '\n';
    CDC_Transmit_FS((uint8_t *)packet, (len+2));
}