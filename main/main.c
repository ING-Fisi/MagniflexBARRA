/* MQTT over SSL Template with GCP (Google Core Platform) support
 *
 *  Created on: Mar 4, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
// ESP-IDF
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
// #include "tcpip_adapter.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"
#include "esp_tls.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>

// Custom
#include "fsntp.h"
#include "gble.h"
#include "gcpjwt.h"
#include "i2c-driver.h"
#include "jsn.h"
#include "main.h"
#include "mems/mems.h"
#include "mqtt.h"
#include "snsmems.h"
#include "utility.h"

#include "Fisitron.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */
// #define MAX_NSNS 	60

//********************************************************************************************************//
//***************************************** VERSIONE TOTEM
//***********************************************//
//********************************************************************************************************//

//****************** GPIO **********************//

#define GPIO_OUTPUT_IO_0 2
#define GPIO_OUTPUT_IO_1 25
#define GPIO_OUTPUT_IO_2 26
#define GPIO_OUTPUT_PIN_SEL                                                    \
	((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1) |                 \
	 (1ULL << GPIO_OUTPUT_IO_2))
#define GPIO_INPUT_IO_0 RESET_GPIO
// #define GPIO_INPUT_IO_1     5
// #define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) |
// (1ULL<<GPIO_INPUT_IO_1))
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
static const char *TAG = "MAIN";

bool wifi_connected = false;
bool connected_sns = false;

#define PLABEL "certs"

extern int bt_snd_rsp_flag; // TODO: fix on the go, into bt.c

int used_heap = 0;

char ts[50];
char js[PUBSTR_SIZE];
char pjsdata[DATASTR_SIZE];
int force_publish = 0;

const char ptyp_str[NPARAM][10] = {
	{"body_p"},	  {"temp"},	   {"hum"},	   {"compass"}, {"sleep_t"},
	{"breath_r"}, {"heart_r"}, {"good_k"}, {"temp_a"},	{"hum_a"},
};

const char data_mode_str[NMODE][10] = {
	{"mean"},
	{"range"},
};

const char cmd_str[NCMD][10] = {
	{"data_int"},
	{"data_mode"},
	{"data_req"},
	{"force_pub"},
};

magniflex_reg_t curdev; // Main device register structure.

char macstr[20];

/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
#ifdef EN_HEAP_TASK_INFO
static void esp_dump_per_task_heap_info(void);
#endif

///////////////////////////////////////////////////////////////////////////////
// TODO: System initialization:
//		 1. NVS read or AFE enumeration if first startup.
//		 2. Memory allocation.
//		 3. Device interrogation and NVS save.

// Function to allocate value buffer structure specific for 'type'.
// with given nsns = 0 and rngm = x, parameters will be initialized as a
// component parameter [always prompted 3 values!]
int alloc_param_val(param_t *par, u32 nsns, u32 rngm) {
	par->val.snssize = nsns;   // Assign parameter sensor size.
	par->val.rangesize = rngm; // Assign parameter range mode.
	u32 len = nsns * rngm;
	len = len == 0 ? 3 : len;
	ESP_LOGV(TAG, "type: '%c', nsns: %d, rngm: %d", par->type, par->val.snssize,
			 par->val.rangesize);
	switch (par->type) {
	case 'f': {
		par->val.fbuf = (float *)malloc(len * sizeof(float));
		if (par->val.fbuf == NULL) {
			return -1;
		}
		memset(par->val.fbuf, 0, len * sizeof(float));
		for (int j = 0; j < len; j++) {
			ESP_LOGV(TAG, "[%i] %f", j, par->val.fbuf[j]);
		}
	} break;
	case 'i': {
		par->val.ibuf = (u32 *)malloc(len * sizeof(u32));
		if (par->val.ibuf == NULL) {
			return -1;
		}
		memset(par->val.ibuf, 0, len * sizeof(u32));
		for (int j = 0; j < len; j++) {
			ESP_LOGV(TAG, "[%i] %d", j, par->val.ibuf[j]);
		}
	} break;
	default: {
	} break;
	}
	return 0;
}

// Parameters initialization function.
void init_param_val(param_t *par, ptyp_t pty) {
	switch (pty) {
	case BODY_P: { // Support range. Each sensor.
		par->type = 'f';
		if (alloc_param_val(par, NSNS, RNGM) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case TEMP: { // Support range. Each sensor.
		par->type = 'f';
		//			if ( alloc_param_val( par, NSNS, RNGM ) < 0 ) {
		if (alloc_param_val(par, 1, 1) < 0) { // Average value of sensors.
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case HUM: { // Support range. Single.
		par->type = 'f';
		if (alloc_param_val(par, 1, RNGM) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case COMPASS: { // Support range. Single.
		par->type = 'i';
		//			if ( alloc_param_val( par, 0, RNGM ) < 0 ) { // Set
		//parameter as components parameter, nsns = 0.
		if (alloc_param_val(par, 1, 1) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case SLEEP_T: { // Single.
		par->type = 'i';
		if (alloc_param_val(par, 1, 1) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case BREATH_R: { // Support range. Single [Get value with better good_k
					 // among acquired values].
		par->type = 'f';
		if (alloc_param_val(par, 1, RNGM) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case HEART_R: { // Support range. Single [Get value with better good_k among
					// acquired values].
		par->type = 'f';
		if (alloc_param_val(par, 1, RNGM) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case GOOD_K: { // Support range. Single [Get value with better good_k among
				   // acquired values].
		par->type = 'f';
		if (alloc_param_val(par, 1, RNGM) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case TEMP_A: { // Support range. Single [Get value with better good_k among
				   // acquired values].
		par->type = 'f';
		if (alloc_param_val(par, 1, 1) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	case HUM_A: { // Support range. Single [Get value with better good_k among
				  // acquired values].
		par->type = 'f';
		if (alloc_param_val(par, 1, 1) < 0) {
			ESP_LOGE(TAG, "error: %s value buffer allocation.", ptyp_str[pty]);
		}
	} break;
	default: {
		ESP_LOGW(TAG, "parameters type not recognized.");
	} break;
	}
	ESP_LOGV(TAG, "New [%s]: type: %c, nsns: %d, rngm: %d", ptyp_str[pty],
			 (*par).type, (*par).val.snssize, (*par).val.rangesize);
}

// Main device initialization function.
void init_magniflex_device(magniflex_reg_t *dev) {

	memset(dev, 0, sizeof(magniflex_reg_t));
	// Default state and data parameter values.
	dev->data_mode = MEAN;
	dev->presence = 0;
	char tmpstr[20];
	sprintf(tmpstr, "{\"presence\":%d}", dev->presence);

	// Set publish time intervals.
	dev->pub_int[BODY_P] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[TEMP] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[HUM] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[COMPASS] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[SLEEP_T] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[BREATH_R] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[HEART_R] = dev->pub_int[BREATH_R]; // DBG: different values to
													// see different data JSON.
	dev->pub_int[GOOD_K] = dev->pub_int[BREATH_R];	// DBG: different values to
													// see different data JSON.
	dev->pub_int[TEMP_A] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.
	dev->pub_int[HUM_A] =
		DEFAULT_PUBINT; // DBG: different values to see different data JSON.

	for (int i = 0; i < NPARAM; i++) {
		ESP_LOGV(TAG, "Initialize parameter %d [%s]", i, ptyp_str[i]);
		dev->t_hold[i] = get_curtimestamp();
		dev->data_req[i] = 0; // Reset parameter request flags.
		init_param_val(&(dev->params[i]), i);
	}

	dev->data_req[TEMP] = 1;
	dev->data_req[HUM] = 1;
	dev->data_req[TEMP_A] = 1;
	dev->data_req[HUM_A] = 1;
	dev->data_req[COMPASS] = 1;
	dev->data_req[BODY_P] = 1;
	dev->data_req[SLEEP_T] = 1;
	dev->data_req[BREATH_R] = 1;
	dev->data_req[HEART_R] = 1;
	dev->data_req[GOOD_K] = 1;

	dev->smph = xSemaphoreCreateBinary();
	if (dev->smph == NULL) {
		ESP_LOGE(TAG, "error: creating device semaphore.");
	}
}

void print_mgnflx_regs(magniflex_reg_t *dev) {

	// Print enumerated devices on bus.
	ESP_LOGI(TAG, " ----------------------------------- \n"
				  "Print Magniflex configured registers.\n"
				  " ----------------------------------- \n"
				  " ''''''''''''''''''''''''''''''''''' \n"
				  "Enumereated SNSMENS AFE on bus:");
	for (int i = 0; i < dev->cnt_nsns; i++) {
		ESP_LOGI(TAG, "snsmens[%d]: %d", i,
				 dev->snsmems[i].indx); // Assign I2C bus address.
	}

	// Default state and data parameter values.
	ESP_LOGI(TAG,
			 " ''''''''''''''''''''''''''''''''''' \n"
			 "Control parameters:\n"
			 "data mode: %s\n"
			 "presence: %d",
			 data_mode_str[dev->data_mode], dev->presence);

	ESP_LOGI(TAG, " ''''''''''''''''''''''''''''''''''' \n"
				  "Data parameters handling:");
	for (int i = 0; i < NPARAM; i++) {
		ESP_LOGI(TAG, " ------------------------- ");
		ESP_LOGI(TAG, "Parameters[%d]: %s", i, ptyp_str[i]);
		ESP_LOGI(TAG, "t_hold: %lld", dev->t_hold[i]);
		ESP_LOGI(TAG, "pub_int: %d", dev->pub_int[i]);
		ESP_LOGI(TAG, "data_req: %d", dev->data_req[i]);
		ESP_LOGI(TAG, "type: %c, sns: %d, rngs: %d, len: %d",
				 dev->params[i].type, dev->params[i].val.snssize,
				 dev->params[i].val.rangesize,
				 (dev->params[i].val.rangesize * dev->params[i].val.snssize));
		if (dev->params[i].type == 'f') {
			esp_log_buffer_hex_internal(
				TAG, dev->params[i].val.fbuf,
				(dev->params[i].val.rangesize * dev->params[i].val.snssize) *
					sizeof(float),
				ESP_LOG_VERBOSE);
			//			float *pf =(float*) &(dev->params[i].val.fbuf);
			for (int j = 0; j < (dev->params[i].val.rangesize *
								 dev->params[i].val.snssize);
				 j++) {
				ESP_LOGI(TAG, "[%i] %.2f", j, dev->params[i].val.fbuf[j]);
			}
		} else if (dev->params[i].type == 'i') {
			esp_log_buffer_hex_internal(
				TAG, dev->params[i].val.ibuf,
				(dev->params[i].val.rangesize * dev->params[i].val.snssize) *
					sizeof(float),
				ESP_LOG_VERBOSE);
			//			u32 *pi =(u32*) &(dev->params[i].val.ibuf);
			for (int j = 0; j < (dev->params[i].val.rangesize *
								 dev->params[i].val.snssize);
				 j++) {
				ESP_LOGI(TAG, "[%i] %d", j, dev->params[i].val.ibuf[j]);
			}
			if ((dev->params[i].val.rangesize * dev->params[i].val.snssize ==
				 0)) {
				ESP_LOGI(TAG, "Components type.");
				for (int k = 0; k < 3; k++) {
					ESP_LOGI(TAG, "[%i] %d", k, dev->params[i].val.ibuf[k]);
				}
			}
		} else {
			ESP_LOGE(TAG, "error: parameter type not recognized.");
		}
	}

	ESP_LOGI(TAG, " ----------------------------------- \n"
				  " ----------------------------------- \n");
}

// Function that populate data JSON.
void param_add2_json(param_t *par, char *pname, data_mode_t m, char *s) {
	u32 nsns = par->val.snssize, rngs = par->val.rangesize;
	int avgindx = rngs <= 1 ? 0 : 1; // Get index of the average elements. [min, avg, max]
	jsn_add_key(s, pname);
	switch (par->type) {
	case 'f': {
		switch (m) {
		case MEAN: {
			if (nsns == 0) { // Component value! Add all 3 component to JSON.
				jsn_set_float_key(s, (par->val.fbuf), 3, 1, 1, 1);
			} else if (nsns == 1) { // Single sensor parameter.
				jsn_set_float_key(s, (par->val.fbuf + avgindx), 1, 1, 1, 1);
			} else {
				jsn_set_float_key(s, (par->val.fbuf + avgindx), nsns * rngs,
								  rngs, 1, 1);
			}
		} break;
		case RANGE: {
			if (nsns == 0) { // Component value! Add all 3 component to JSON.
				jsn_set_float_key(s, (par->val.fbuf), 3, 1, 1, 1);
			} else if (nsns == 1) { // Single sensor parameter.
				jsn_set_float_key(s, (par->val.fbuf), rngs, 1, 1, 1);
			} else {
				jsn_set_float_key(s, par->val.fbuf, nsns, 1, rngs, 1);
			}
		} break;
		default: {
		} break;
		}
	} break;
	case 'i': {
		switch (m) {
		case MEAN: {
			if (nsns == 0) { // Component value! Add all 3 component to JSON.
				jsn_set_int_key(s, (int *)(par->val.ibuf), 3, 1, 1, 1);
			} else if (nsns == 1) { // Single sensor parameter.
				jsn_set_int_key(s, (int *)(par->val.ibuf + avgindx), 1, 1, 1,
								1);
			} else {
				jsn_set_int_key(s, (int *)(par->val.ibuf + avgindx),
								nsns * rngs, rngs, 1, 1);
			}
		} break;
		case RANGE: {
			if (nsns == 0) { // Component value! Add all 3 component to JSON.
				jsn_set_int_key(s, (int *)(par->val.ibuf), 3, 1, 1, 1);
			} else if (nsns == 1) { // Single sensor parameter.
				jsn_set_int_key(s, (int *)(par->val.ibuf), rngs, 1, 1, 1);
			} else {
				jsn_set_int_key(s, (int *)(par->val.ibuf), nsns, 1, rngs, 1);
			}
		} break;
		default: {
		} break;
		}
	} break;
	default: {
	} break;
	}
	//	if ( strlen(s) == 0 )  {
	//		strcpy(s,"{");
	//	}
	//	strcat(s,tmpstr);
}

// Function that checks if a parameter has to be published and create the
// publish JSON. It checks both time intervals and pending requests.
void param_chck_pub(magniflex_reg_t *dev, char *js_str) {

	// if(dev->presence == 1)
	if (true) {
		for (int i = 0; i < NPARAM; i++) {
			if (dev->data_req[i] == 1) {
				param_add2_json(&(dev->params[i]), (char *)ptyp_str[i],dev->data_mode, js_str);
			}
		}
		int slen = strlen(js_str);

		if (slen > 0) {
			if (js_str[slen - 1] == ']') {
				strcat(js_str, "}");
			} else {
				js_str[slen - 1] = '}'; // Close data JSON.
				js_str[slen] = 0;		// Close data JSON string.
			}
		}
	}
}

// function that check if JSON data id available and publish it.
int chck_req_periodic_pub(magniflex_reg_t *dev, char *pub_js, char *data_js) {
	int ret = 0;
	param_chck_pub(dev, data_js);

	if (strlen(data_js) == 0) { // No data available.
		ESP_LOGI(TAG, "No data available");
	} else {
		ESP_LOGI(TAG, "data_js (%d):%s  [%d]\n", strlen(data_js), data_js,curdev.presence);
		pub_js[0] = 0;
		data_js[0] = 0;
	}

	return ret;
}

void dbg_sim_data(magniflex_reg_t *dev) {
	for (int i = 0; i < dev->cnt_nsns * RNGM; i++) {
		//		dev->params[BODY_P].val.fbuf[i] = rand_int_decimal( 1, 2 ); //
		//+- 10;
		dev->params[BODY_P].val.fbuf[i] = 0.01f; // +- 10;
	}
	dev->params[TEMP].val.fbuf[0] = (20.0f + rand_int_decimal(5, 1));
	//	for ( int i = 0 ; i < dev->cnt_nsns*RNGM ; i++  ) {
	//		dev->params[TEMP].val.fbuf[i] = (20.0f + rand_int_decimal( 2, 1 ));
	//	}
	for (int i = 0; i < RNGM; i++) {
		dev->params[HUM].val.fbuf[i] = (50.0f + rand_int_decimal(2, 1));
	}
	//	for ( int i = 0 ; i < RNGM ; i++  ) {
	//		dev->params[MAG].val.ibuf[i] = (u32) (500 + rand_int_decimal( 10, 0
	//));
	//	}
	dev->params[COMPASS].val.ibuf[0] = (u32)(0 + rand_int_decimal(360, 0));
	//	dev->params[SLEEP_T].val.ibuf[0] = (u32) (0 + rand_int_decimal( 3600*12,
	//0 ));
	for (int i = 0; i < RNGM; i++) {
		dev->params[BREATH_R].val.fbuf[i] = (12.00f + rand_int_decimal(1, 2));
	}
	for (int i = 0; i < RNGM; i++) {
		dev->params[HEART_R].val.fbuf[i] = (60.00f + rand_int_decimal(1, 2));
	}
	for (int i = 0; i < RNGM; i++) {
		dev->params[GOOD_K].val.fbuf[i] = (0.0f + rand_int_decimal(1, 2));
	}
	dev->params[TEMP_A].val.fbuf[0] = (20.0f + rand_int_decimal(5, 1));
	dev->params[HUM_A].val.fbuf[0] = (50.0f + rand_int_decimal(10, 1));
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
#ifdef DBG_STATS
/* FreeRTOS Real Time Stats Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

#define NUM_OF_SPIN_TASKS 6
#define SPIN_ITER                                                              \
	500000 // Actual CPU cycles used will depend on compiler optimization
#define SPIN_TASK_PRIO 2
#define STATS_TASK_PRIO 3
#define STATS_TICKS pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET                                                      \
	5 // Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

static char task_names[NUM_OF_SPIN_TASKS][configMAX_TASK_NAME_LEN];
static SemaphoreHandle_t sync_spin_task;
static SemaphoreHandle_t sync_stats_task;

/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @note    If any tasks are added or removed during the delay, the stats of
 *          those tasks will not be printed.
 * @note    This function should be called from a high priority task to minimize
 *          inaccuracies with delays.
 * @note    When running in dual core mode, each core will correspond to 50% of
 *          the run time.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_NO_MEM        Insufficient memory to allocated internal arrays
 *  - ESP_ERR_INVALID_SIZE  Insufficient array size for uxTaskGetSystemState.
 * Trying increasing ARRAY_SIZE_OFFSET
 *  - ESP_ERR_INVALID_STATE Delay duration too short
 */
static esp_err_t print_real_time_stats(TickType_t xTicksToWait) {
	TaskStatus_t *start_array = NULL, *end_array = NULL;
	UBaseType_t start_array_size, end_array_size;
	uint32_t start_run_time, end_run_time;
	esp_err_t ret;

	// Allocate array to store current task states
	start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
	if (start_array == NULL) {
		ret = ESP_ERR_NO_MEM;
		goto exit;
	}
	// Get current task states
	start_array_size =
		uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
	if (start_array_size == 0) {
		ret = ESP_ERR_INVALID_SIZE;
		goto exit;
	}

	vTaskDelay(xTicksToWait);

	// Allocate array to store tasks states post delay
	end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
	if (end_array == NULL) {
		ret = ESP_ERR_NO_MEM;
		goto exit;
	}
	// Get post delay task states
	end_array_size =
		uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
	if (end_array_size == 0) {
		ret = ESP_ERR_INVALID_SIZE;
		goto exit;
	}

	// Calculate total_elapsed_time in units of run time stats clock period.
	uint32_t total_elapsed_time = (end_run_time - start_run_time);
	if (total_elapsed_time == 0) {
		ret = ESP_ERR_INVALID_STATE;
		goto exit;
	}

	printf("| Task | Run Time | Percentage\n");
	// Match each task in start_array to those in the end_array
	for (int i = 0; i < start_array_size; i++) {
		int k = -1;
		for (int j = 0; j < end_array_size; j++) {
			if (start_array[i].xHandle == end_array[j].xHandle) {
				k = j;
				// Mark that task have been matched by overwriting their handles
				start_array[i].xHandle = NULL;
				end_array[j].xHandle = NULL;
				break;
			}
		}
		// Check if matching task found
		if (k >= 0) {
			uint32_t task_elapsed_time =
				end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
			uint32_t percentage_time =
				(task_elapsed_time * 100UL) /
				(total_elapsed_time * portNUM_PROCESSORS);
			printf("| %s | %d | %d%%\n", start_array[i].pcTaskName,
				   task_elapsed_time, percentage_time);
		}
	}

	// Print unmatched tasks
	for (int i = 0; i < start_array_size; i++) {
		if (start_array[i].xHandle != NULL) {
			printf("| %s | Deleted\n", start_array[i].pcTaskName);
		}
	}
	for (int i = 0; i < end_array_size; i++) {
		if (end_array[i].xHandle != NULL) {
			printf("| %s | Created\n", end_array[i].pcTaskName);
		}
	}
	ret = ESP_OK;

exit: // Common return path
	free(start_array);
	free(end_array);
	return ret;
}

static void spin_task(void *arg) {
	xSemaphoreTake(sync_spin_task, portMAX_DELAY);
	while (1) {
		// Consume CPU cycles
		for (int i = 0; i < SPIN_ITER; i++) {
			__asm__ __volatile__("NOP");
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

static void stats_task(void *arg) {
	xSemaphoreTake(sync_stats_task, portMAX_DELAY);

	// Start all the spin tasks
	for (int i = 0; i < NUM_OF_SPIN_TASKS; i++) {
		xSemaphoreGive(sync_spin_task);
	}

	// Print real time stats periodically
	while (1) {
		printf("\n\nGetting real time stats over %d ticks\n", STATS_TICKS);
		if (print_real_time_stats(STATS_TICKS) == ESP_OK) {
			printf("Real time stats obtained\n");
		} else {
			printf("Error getting real time stats\n");
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
#endif
///////////////////////////////////////////////////////////

void env_tsk(void *vargs) {

	ESP_LOGI(TAG, "Run env tasks.");

	while (1) {
		if (connected_sns == true) {

			acq_snsmems_env_data(&curdev);
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

//*********************************************************************************************//
void ctrl_tsk(void *vargs) {

	ESP_LOGI(TAG, "Run working tasks.");
	// Time variables.
	long print_log_t = T_US;

	curdev.cnt_nsns = snsmems_initilaize(curdev.snsmems);

	if (curdev.cnt_nsns < 2) {
		ESP_LOGW(TAG, "no snsmems detected, try enumaration.");
	} else {
		ESP_LOGI("snsmems_acq_tsk", "detected: %d SNSMEMS", curdev.cnt_nsns);
		for (int i = 0; i < curdev.cnt_nsns; i++) {
			ESP_LOGI(TAG, "sns_addr[%d]: %02x(%d)", i, curdev.snsmems[i].indx,
					 curdev.snsmems[i].indx);
		}

		// Get saved threshold values.
		snsmems_nvs_get_thrsh(curdev.prsnc_trsh);

		ESP_LOGI("snsmems_nvs_get_thrsh", "prsnc_trsh: %f %f %f",
				 curdev.prsnc_trsh[0], curdev.prsnc_trsh[1],
				 curdev.prsnc_trsh[2]);
		connected_sns = true;

		xTaskCreatePinnedToCore(env_tsk, "env_tsk", 1024 * 5, NULL, 4, NULL,
								1 /*tskNO_AFFINITY*/);
	}

#ifdef USE_PERIOD_CIRCBUF
	period_buf_init();
#endif

	while (1) {

		float t = 0.0f, h = 0.0f;
		// Acquire environment parameters.
		MEMS_ENV_SENSOR_GetValue(MEMS_HTS221_0, ENV_TEMPERATURE, &t);
		MEMS_ENV_SENSOR_GetValue(MEMS_HTS221_0, ENV_HUMIDITY, &h);

		ESP_LOGI(TAG, "Run working tasks. [%f] [%f]",t,h);

		if (curdev.cnt_nsns < 2) {
			ESP_LOGW(TAG, "no snsmems detected, try enumaration.");
		} else {
			acq_snsmems_data(&curdev);
		}


//**************************************************************************************/
//********************* Modificare questi offset per le temperature e umidita **********/

		/*******************************************************************************/
		//curdev.params[HUM].val.fbuf[1] = h;
		//curdev.params[HUM_A].val.fbuf[0] = h;// - 5;
		//curdev.params[TEMP_A].val.fbuf[0] = curdev.params[TEMP].val.fbuf[0];// - 3;
		/*******************************************************************************/
		
		/*******************************************************************************/
		curdev.params[HUM].val.fbuf[1] = h;
		curdev.params[HUM_A].val.fbuf[0] = curdev.params[HUM].val.fbuf[1];
		
		curdev.params[TEMP_A].val.fbuf[0] = curdev.params[TEMP].val.fbuf[0]+2;
		//curdev.params[TEMP_A].val.fbuf[0] = 10;
		/*******************************************************************************/
	

		memset(js, 0, sizeof(js));
		memset(pjsdata, 0, sizeof(pjsdata));
		chck_req_periodic_pub(&curdev, js, pjsdata);

		gpio_set_level(GPIO_OUTPUT_IO_0, 100);
		vTaskDelay(20 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_OUTPUT_IO_0, 0);
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

//*********************************************************************//

static void IRAM_ATTR gpio_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t)arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg) {
	uint32_t io_num;
	for (;;) {
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

			switch (io_num) {
			case RESET_GPIO: {
				ESP_LOGI(TAG, "RESET PARAMETRI --> REBOOT");
				float tmpf[MAX_NSNS] = {0};
				memset(tmpf, 0, sizeof(tmpf));
				snsmems_nvs_save_thrsh(tmpf, MAX_NSNS);
				// ESP_LOGI(TAG,"Erase NVS flash partition.");
				// nvs_flash_erase();
				// ESP_LOGI(TAG,"Reboot system.  CAZZZO");
				esp_restart();
			} break;

			default:
				break;
			}

			// printf("GPIO[%d] intr, val: %d\n", io_num,
			// gpio_get_level(io_num));
		}
	}
}

void gpio_init(void) {
	// zero-initialize the config structure.
	gpio_config_t io_conf = {};
	// disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	// set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	// bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	// disable pull-down mode
	io_conf.pull_down_en = 0;
	// disable pull-up mode
	io_conf.pull_up_en = 0;
	// configure GPIO with the given settings
	gpio_config(&io_conf);

	// interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	// bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	// set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	// enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	// change gpio intrrupt type for one pin
	gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

	// create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	// start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	// install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	// hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
						 (void *)GPIO_INPUT_IO_0);
	// hook isr handler for specific gpio pin
	// gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*)
	// GPIO_INPUT_IO_1);

	// remove isr handler for gpio number.
	// gpio_isr_handler_remove(GPIO_INPUT_IO_0);
	// hook isr handler for specific gpio pin again
	// gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*)
	// GPIO_INPUT_IO_0);

	printf("Minimum free heap size: %d bytes\n",
		   esp_get_minimum_free_heap_size());

	//    int cnt = 0;
	//    while(1) {
	//        printf("cnt: %d\n", cnt++);
	//        vTaskDelay(1000 / portTICK_RATE_MS);
	//        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
	//        gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
	//    }
}

// #define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 128

//********************************************************************************************************//

void app_main(void) {

	used_heap = esp_get_free_heap_size(); // Memory debug variable.

	/* Setup components log levels without rebuild whole IDF *
	 * ----------------------------------------------------- */
	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("XMQTT", ESP_LOG_DEBUG);
	esp_log_level_set("XBLE", ESP_LOG_DEBUG);
	esp_log_level_set("MAIN", ESP_LOG_DEBUG);
	//	esp_log_level_set("XOTA", ESP_LOG_DEBUG);
	//	esp_log_level_set("XWIFI", ESP_LOG_DEBUG);
	//	esp_log_level_set("XSNSMEMS", ESP_LOG_DEBUG);
	//	esp_log_level_set("XGCPJWT", ESP_LOG_DEBUG);
	//	esp_log_level_set("X_RGBLED", ESP_LOG_DEBUG);
	//	esp_log_level_set("MAIN", ESP_LOG_DEBUG);
	//    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
	//  esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_DEBUG);
	//  esp_log_level_set("TRANSPORT_TCP", ESP_LOG_DEBUG);
	//    esp_log_level_set("TRANS_SSL", ESP_LOG_VERBOSE);
	//    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
	//    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
	//  esp_log_level_set("XSNTP", ESP_LOG_VERBOSE);

	/* Main application initial chip and system information  *
	 * ----------------------------------------------------- */
	ESP_LOGI(TAG, "Startup..");
	ESP_LOGI(TAG, "Free memory: %d bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

	print_chip_info();

	/* Initialize device main features */
	ESP_LOGD(TAG, "DEV_INIT: initialize hardware features");
	esp_err_t err = nvs_flash_init(); // NVS
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	// DBG SPIFFS READ CERTS
	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
								  .partition_label = NULL,
								  .max_files = 5,
								  .format_if_mount_failed = true};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",
					 esp_err_to_name(ret));
		}
		return;
	}

	ESP_LOGI(TAG, "Performing SPIFFS_check().");
	ret = esp_spiffs_check(conf.partition_label);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
		return;
	} else {
		ESP_LOGI(TAG, "SPIFFS_check() successful");
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(
			TAG,
			"Failed to get SPIFFS partition information (%s). Formatting...",
			esp_err_to_name(ret));
		esp_spiffs_format(conf.partition_label);
		return;
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}

	// Check consistency of reported partiton size info.
	if (used > total) {
		ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. "
					  "Performing SPIFFS_check().");
		ret = esp_spiffs_check(conf.partition_label);
		// Could be also used to mend broken files, to clean unreferenced pages,
		// etc. More info at
		// https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
			return;
		} else {
			ESP_LOGI(TAG, "SPIFFS_check() successful");
		}
	}

	//	//**********************************************************//
	//	// strip newline
	//	char* pos = strchr(line, '\n');
	//	if (pos) {
	//		*pos = '\0';
	//	}
	//	ESP_LOGI(TAG, "Read from file: '%s'", line);

	// All done, unmount partition and disable SPIFFS
	// esp_vfs_spiffs_unregister(conf.partition_label);
	// ESP_LOGI(TAG, "SPIFFS unmounted");

	//*****************************************************************//

	//************************* GPIO INIT ********************************//
	gpio_init();
	//************************* SENS INIT ********************************//

	// Initialize HTS221 data acquisition.
	if (mems_i2c_master_init() != ESP_OK) {
		ESP_LOGE(TAG, "error: init i2c master mode");
	}
	if (MEMS_ENV_SENSOR_Init(MEMS_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY) !=
		BSP_ERROR_NONE) {
		ESP_LOGE(TAG, "error: init HTS221 temperature");
	}

	get_mac_str(macstr);

	//********************************************************//

#ifdef TEST_SNSMEMS
	// SENSMEMS Initialization and enumeration.
	int snsnum = snsmems_initilaize(curdev.snsmems);
	ESP_LOGI(TAG, "///////////////////// TEST SNSMEMS /////////////////////");
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "SNSMEMS detected: %d.", snsnum);
	ESP_LOGI(TAG, "");
	u32 buf;
	for (int i = 0; i < snsnum; i++) {
		ESP_LOGI(TAG, "-------------------- address: %d.", curdev.snsmems[i]);
		if (snsmems_rd(curdev.snsmems[i], SNSMEMS_REG(Status_REG), (u8 *)&buf,
					   REG_LEN) == ESP_OK) {
			snsmems_print_stat(buf);
			snsmems_print_ver(curdev.snsmems[i]);
		}
		ESP_LOGI(TAG, "");
		//		ESP_LOGI(TAG,"sns_addr[%d]: %02x(%d)", i, curdev.snsmems[i],
		//curdev.snsmems[i]);
	}
	//	snsmems_en_cmd(0);
	return;
#endif

	//************************* MAGNI INIT ********************************//
	init_magniflex_device(&curdev); // Initialize device.

	// TODO: do better initialization.
	for (int i = 0; i < NSNS * 2; i++) {
		curdev.prsnc_trsh[i] = 0.00f;
	}

	//*************************************************************************//
	xTaskCreatePinnedToCore(ctrl_tsk, "ctrl_tsk", 1024 * 5, NULL, 4, NULL,
							1 /*tskNO_AFFINITY*/);

	vTaskDelete(NULL);
}
