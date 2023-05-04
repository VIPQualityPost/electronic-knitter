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
#define versionIdentifier 0x00
#define versionMajor 0
#define versionMinor 95
#define versionAPI   6

/* AYAB API commands v6 */
#define unknown -2
#define none -1
#define reqInfo 0x03
#define cnfInfo 0xC3
#define reqTest 0x04
#define cnfTest 0xC4
#define reqStart 0x01
#define cnfStart 0xC1
#define reqLine 0x82
#define cnfLine 0x42
#define indState 0x84
#define helpCmd 0x26
#define sendCmd 0x27
#define beepCmd 0x28
#define readCmd 0x29
#define autoCmd 0x2a
#define testCmd 0x2b
#define quitCmd 0x2c
#define reqInit 0x30
#define cnfInit 0x31
#define setCmd 0x2d
#define testRes 0xee
#define debug 0x9f
#define slipFrameEnd 0xc0

/* Prototypes */

/* For parsing received CDC packet */
void rxAYAB(void);

/* For sending/requesting data from AYAB software */
void txAYAB(uint8_t);

#endif