/*
 * x-ble.h
 *
 *  Created on: Mar 21, 2021
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

#ifndef MAIN_X_BLE_H_
#define MAIN_X_BLE_H_

/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
#include "utility.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */
#define DEVICE_NAME "SmartDream"
#define MAX_SINGLE_SIZE	20

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
void enable_ble( void );
void disable_ble( void );
void bleprintf(const char* format, ...);
int is_ble_msg(void);
int get_ble_msg (char *buf);
int get_ble_state(void);

int bt_wr( int len, char* data );
void prs_bt_js( char *js );

#endif /* MAIN_X_BLE_H_ */
