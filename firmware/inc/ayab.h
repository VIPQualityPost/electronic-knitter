/**
 ******************************************************************************
* @file    ayab.h
* @brief   This file contains all the function prototypes for
*          the ayab.c file.
* @author  matei
******************************************************************************
*/

#ifndef __AYAB_H__
#define __AYAB_H__

/* Includes */
#include "main.h"

/* AYAB firmware defines */
#define versionIdentifier 6
#define versionMajor 0
#define versionMinor 95
#define versionAPI   6

/* AYAB API commands v6 */

#define  reqStart 0x01
#define  cnfStart 0xc1
#define  reqLine 0x82
#define  cnfLine 0x42
#define  reqInfo 0x03
#define  cnfInfo 0xc3
#define  reqTest 0x04
#define  cnfTest 0xc4
#define  indState 0x84
#define  helpCmd 0x25
#define  sendCmd 0x26
#define  beepCmd 0x27
#define  setSingleCmd 0x28
#define  setAllCmd 0x29
#define  readEOLsensorsCmd 0x2a
#define  readEncodersCmd 0x2b
#define  autoReadCmd 0x2c
#define  autoTestCmd 0x2d
#define  stopCmd 0x2e
#define  quitCmd 0x2f
#define  reqInit 0x30
#define  cnfInit 0x31
#define  testRes 0xe0
#define  debug 0x99
#define  slipFrameEnd 0xC0

/* Prototypes */

/* For parsing received CDC packet */
void rxAYAB(void);

/* For sending/requesting data from AYAB software */
void txAYAB(uint8_t);

#endif