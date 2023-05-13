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

void slipSend(uint8_t*, uint8_t);

extern uint8_t *ayabRX;
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
 * from the PC and then configure the state of the hardware. ayabRX is 
 * a global pointer in main.c that is attached to the USB CDC rx buffer.
 * It should only be called from CDC_Recieve_FS (CDC rx callback). It's 
 * not meant for you to call this in your own code.
*/
void rxAYAB(void)
{
	switch (*ayabRX)
	{
	case reqStart:
        /* AYAB software requests to start a new image. 
            Provide machine type and worked needles */
		machineType =   ayabRX[1];
		startNeedle =   ayabRX[2];
		stopNeedle =    ayabRX[3];
        txAYAB(cnfStart);
		break;

	case cnfLine:
        /* Send information about a new row from the PC. */
		currentRow =     ayabRX[1];

        /* Don't want to use pointer because next packet will clobber ayabRX. */
		memcpy(bitPattern, &ayabRX[2], 25); 
		lastRow =       ayabRX[27];
		crc8_cs =       ayabRX[28];
		break;

	case reqInfo:
        /* First part of handshake, request firmware information */
		txAYAB(cnfInfo);
		break;

    case reqInit:
        txAYAB(cnfInit);
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
    uint8_t ayabIndex = 0;

    /* The SLIP packet is double ended, so we start with an end signal (0xC0). */
    ayabPacket[ayabIndex++] = slipFrameEnd;
    ayabPacket[ayabIndex++] = cmd;    /* First real byte of the message is the ID */

	switch (cmd)
	{
	case cnfStart:
        /* Confirm that the pattern parameters were received. 
        as long as the needles are within real values (initialized to 0xFF)
        then we can just return that a valid config was received. */
        if(startNeedle <= 198 && stopNeedle <= 200){
            ayabPacket[ayabIndex++] = 0x00;
            machineState = 1; /* HOMING */
        }
        else{
            ayabPacket[ayabIndex++] = 0xFF;
        }
		break;

	case reqLine:
		/* Request a new line from the software */
        ayabPacket[ayabIndex++] = currentRow + 1;
        break;

	case cnfInfo:
        /* Respond to reqInfo with firmware version identifier, major version, minor version. */
        ayabPacket[ayabIndex++] = versionAPI;
        ayabPacket[ayabIndex++] = versionMajor;
        ayabPacket[ayabIndex++] = versionMinor;
		break;

    case cnfInit:   
        /* Should probably add something here to check if init went smooth.
        default return 0 for success. There are error codes in the API. */
        ayabPacket[ayabIndex++] = 0x00;
        break;

	case indState:
        /* Indicate if initialized or not. */
        ayabPacket[ayabIndex++] = machineInitialized;

        /* EOL sensor states */
        ayabPacket[ayabIndex++] = EOL_result[0] >> 8;
        ayabPacket[ayabIndex++] = EOL_result[0] & 0xFF;
        ayabPacket[ayabIndex++] = EOL_result[1] >> 8;
        ayabPacket[ayabIndex++] = EOL_result[1] & 0xFF;

        /* Machine information */
        ayabPacket[ayabIndex++] = carriageType;
        ayabPacket[ayabIndex++] = machineType;
        ayabPacket[ayabIndex++] = currentDir;
		break;

	case debug:
        /* Send the debug string to the software */
        ayabPacket[ayabIndex++] = 0xFF;
		break;

	default:
		break;
	}

    ayabPacket[ayabIndex++] = slipFrameEnd;
    CDC_Transmit_FS((uint8_t *)ayabPacket, ayabIndex);
}
