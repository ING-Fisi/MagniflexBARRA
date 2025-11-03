/*
 * main.h
 *
 *  Created on: Mar 2, 2020
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#include "mqtt_client.h"
//#include <cstdint>

/* Firmware version used formatted as string to be used also with 	*
 * Thingsboard for OTA configuration and control 					*/
#define VERSION_MAJOR	3
#define VERSION_MINOR	3
#define VERS_CHAR		v
#define STRINGIFY(x)	#x
#define VERSION_STR(A,B,C) 	STRINGIFY(C) STRINGIFY(A) "." STRINGIFY(B)

// FEATURES CONTROL DEFINES
// --------------------------------------------
//#define PUB_DBG
//#define DBG_LED
//#define DBG_STATS
//#define MQTTCMD_DBG
//#define DBG_TOUCHPAD
//#define DBG_BT_JS_PARSE
//#define DBG_RGBLED
//#define TEST_SNSMEMS
#define USE_PERIOD_CIRCBUF
//#define SIM_DATA			// To use for remote development purpose.
//#define EN_OTA
//#define NEW_OTAURL
//#define EN_HEAP_TASK_INFO
//#define OCTAVE_SERIAL
#define CIRC_LPF

// FUNCTIONAL PARAMETERS CONTROL DEFINES
// --------------------------------------------
#define DEFAULT_PUBINT 	(300) // Default publish interval time in seconds.
#define RESET_GPIO		(23)
#define THRSH_BLOBS_TAG	"thrsh"



/////////////////////////////////// MAGNIFLEX DEVICE ////////////////////////////////
#define MAX_NSNS 	10

typedef enum parameter_type {
	BODY_P = 0,
	TEMP,
	HUM,
	COMPASS,
	SLEEP_T,
	BREATH_R,
	HEART_R,
	GOOD_K,
	TEMP_A,
	HUM_A,
	NPARAM
} ptyp_t;

// FIXME: Test data storage structures, only for platform debug.
//		  [will be done the assumption to have 3 AFE sensors SENSMEMS]
#define NSNS 3 // Assumed to has 3 sensors on system.
#define RNGM 3 // Statistics length [min, acg, max]

typedef struct {
	uint32_t snssize; // Specify if related to single or more SNSMEMS board.
	uint32_t rangesize; // Specify if support range data mode. If yes = 3. [min, avg, max]
	union {
		float *fbuf;
		uint32_t *ibuf;
	};
} pdata_t;

typedef struct {
	char type; // Specify data parameter type. ['f'|'i']
	pdata_t val; // Pointer to data parameter value buffer.
} param_t;

//#define NMODE 		2
typedef enum { MEAN = 0, RANGE, NMODE } data_mode_t;

//#define NCMD 		4
typedef enum { DATAINT = 0, DATAMODE, DATAREQ, FORCE_PUB, NCMD } cmd_t;

typedef struct {
	uint16_t indx;
	uint16_t iscomm;
} snsmems_t;

typedef struct {
	uint32_t cnt_nsns;
	snsmems_t snsmems[MAX_NSNS]; 	// Array to hold SNSMEMS I2C indexes.
	data_mode_t data_mode; 			// Data mode type. TODO: define enum.
	uint8_t presence; 					// TODO: evaluate to give 0 or time a body has been detected.
	long t_hold[NPARAM]; 			// Time holding variables.
	uint32_t acq_int[NPARAM]; 			// Data acquisition time intervals.
	uint32_t pub_int[NPARAM]; 			// Data publish time intervals.
	uint8_t data_req[NPARAM]; 			// Request data flags.
	param_t params[NPARAM]; 		// Parameter structure. Has to be initialized.
	SemaphoreHandle_t smph;			// Device structure semaphore.
	float prsnc_trsh[MAX_NSNS*2];	// TODO: implement more sensors routines.
	esp_mqtt_client_handle_t mqttc; // Client MQTT.
	long start_sleep_t;				// Start sleep time.
} magniflex_reg_t;

const char *fw_ver_str;


void ota_check(void);

////////////////////////////////////////////////////////////////////////////////////

#endif /* MAIN_MAIN_H_ */
