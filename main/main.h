/*
 * main.h
 *
 *  Created on: Mar 2, 2020
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#include "mqtt_client.h"

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
#define GIOTCP_PUB
//#define XPHASE_PUB
//#define OCTAVE_SERIAL
#define CIRC_LPF

// FUNCTIONAL PARAMETERS CONTROL DEFINES
// --------------------------------------------
#define DEFAULT_PUBINT 	(300) // Default publish interval time in seconds.
#define RESET_GPIO		(23)
#define THRSH_BLOBS_TAG	"thrsh"


#define PUBSTR_SIZE 1024
#define DATASTR_SIZE 1024
/////////////////////////////////// MAGNIFLEX DEVICE ////////////////////////////////
#define MAX_NSNS 	10


//#define OFFSET_TEMP 10

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
	u32 snssize; // Specify if related to single or more SNSMEMS board.
	u32 rangesize; // Specify if support range data mode. If yes = 3. [min, avg, max]
	union {
		float *fbuf;
		u32 *ibuf;
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
	u16 indx;
	u16 iscomm;
} snsmems_t;

typedef struct {
	u32 cnt_nsns;
	snsmems_t snsmems[MAX_NSNS]; 	// Array to hold SNSMEMS I2C indexes.
	data_mode_t data_mode; 			// Data mode type. TODO: define enum.
	u8 presence; 					// TODO: evaluate to give 0 or time a body has been detected.
	s64 t_hold[NPARAM]; 			// Time holding variables.
	u32 acq_int[NPARAM]; 			// Data acquisition time intervals.
	u32 pub_int[NPARAM]; 			// Data publish time intervals.
	u8 data_req[NPARAM]; 			// Request data flags.
	param_t params[NPARAM]; 		// Parameter structure. Has to be initialized.
	SemaphoreHandle_t smph;			// Device structure semaphore.
	float prsnc_trsh[MAX_NSNS*2];	// TODO: implement more sensors routines.
	esp_mqtt_client_handle_t mqttc; // Client MQTT.
	long start_sleep_t;				// Start sleep time.
} magniflex_reg_t;

const char *fw_ver_str;

int chck_req_periodic_pub( magniflex_reg_t *dev, char* pub_js, char* data_js );


////////////////////////////////////////////////////////////////////////////////////

#endif /* MAIN_MAIN_H_ */
