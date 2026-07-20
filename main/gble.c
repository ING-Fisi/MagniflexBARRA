/*
 * x-ble.c
 *
 *  Created on: Mar 21, 2021
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
// STD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
// ESP-XPHS
#include "gble.h"
#include "utility.h"

#define VISHID __attribute__ ((visibility("hidden"))

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
static const char *TAG = "XBLE";


extern bool wifi_connected;

#define GATTS_NOTIFY_LEN    490
static SemaphoreHandle_t gatts_semaphore;
static bool can_send_notify = false;
static uint8_t indicate_data[GATTS_NOTIFY_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a};

#define BLE_MSGBUFF_SIZE	5
#define BLE_MSG_LEN			250

typedef struct msg {
	char msg[BLE_MSG_LEN];
	int len;
} ble_msg_t;

int msgindxin = 0;
int msgindxout = 0;
int msgcnt = 0;
ble_msg_t blemsg_buff[BLE_MSGBUFF_SIZE];
bool is_ble_conn = false;

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

//#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val = {
		.attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
		.attr_len     = sizeof(char1_str),
		.attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

char ble_dev_name[20];


/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
static uint8_t adv_service_uuid128[32] = {
		/* LSB <--------------------------------------------------------------------------------> MSB */
		//first uuid, 16bit, [12],[13] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
		.set_scan_rsp = false,
		.include_name = true,
		.include_txpower = true,
		.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
		.max_interval = 0x000C, //slave connection max interval, Time = max_interval * 1.25 msec
		.appearance = 0x00,
		.manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
		.p_manufacturer_data =  NULL, //&test_manufacturer[0],
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = 32,
		.p_service_uuid = adv_service_uuid128,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
		.set_scan_rsp = true,
		.include_name = true,
		.include_txpower = true,
		.min_interval = 0x0006,
		.max_interval = 0x000C,
		.appearance = 0x00,
		.manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
		.p_manufacturer_data =  NULL, //&test_manufacturer[0],
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = 32,
		.p_service_uuid = adv_service_uuid128,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
		.adv_int_min        = 0x20,
		.adv_int_max        = 0x40,
		.adv_type           = ADV_TYPE_IND,
		.own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
		//.peer_addr            =
		//.peer_addr_type       =
		.channel_map        = ADV_CHNL_ALL,
		.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
		[PROFILE_A_APP_ID] = {
				.gatts_cb = gatts_profile_a_event_handler,
				.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		},
};

typedef struct {
	uint8_t                 *prepare_buf;
	int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);









/////////////////////////////////////////////////////////////////////////////
//#define NBTCMD 	2
#define BTCMDSTRLEN	10
#define NBTS 		1
#define BTSSTRLEN	10

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
int bt_snd_rsp_flag = 0; // TODO: fix on the go.

char str[2000];
char *pstr = str;
int totbtl = 0;
int sendlen = 100;

uint32_t bt_conn_h;



/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
wifi_ap_record_t ap_records[MAX_APs];

//! BLE recognized commands enum.
/*! Each enum member correspont to a single recognized BLE command
 *  and corresponds to the correct index of string tag name buffer. */
enum bt_cmd_num {
	BT_CMD = 0,		/*!< BLE command to query visible WIFI access points. */
	BT_WIFI,		/*!< BLE command to set WIFI credentials, followed by parameters. */
	BT_STATUS,		/*!< BLE command to query WIFI connection status. */
	BT_MAC,			/*!< BLE command to query DEVICE MAC address. */
	NBTCMD
};
char bt_cmd[NBTCMD-1][BTCMDSTRLEN] = { // BT command strings.
		"wifi",
		"status",
		"mac",
};

enum bt_status_num {BT_S_WIFI = 0};
char bt_status[NBTS][BTSSTRLEN] = {
		"wifi",
};
char *auth_names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
int bt_wr( int len, char* data ) {
	//	return esp_spp_write(bt_conn_h, len,(uint8_t*) data);
	int sended = 0;
	int sindx = 0;
	while (len > 0) {
		int tosend = len > MAX_SINGLE_SIZE ? MAX_SINGLE_SIZE : len;
		if ( esp_ble_gatts_send_indicate(
				gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
				gl_profile_tab[PROFILE_A_APP_ID].conn_id,
				gl_profile_tab[PROFILE_A_APP_ID].char_handle,
				tosend, (uint8_t*) &data[sindx], false ) < 0) {
			return -1;
		}
		//		esp_log_buffer_hexdump_internal(TAG, &data[sindx], tosend, ESP_LOG_WARN);
		sended += tosend;
		sindx += tosend;
		len -= tosend;
		vTaskDelay(1);
	}
	return sended;
}


char custom_ssid[30];
char custom_password[30];

void prs_bt_js( char *js ) {
	int prs_s = 0;
	char s[400];
	char *savep;
	char *p;
	savep = js;

	while ((p = strtok_r(savep, "{},:'", &savep))) { // JSON parsing cycle.


		ESP_LOGI(TAG,"token: %s", p);

		switch (prs_s) {

		case BT_CMD: { // parse CMD.
			for( int i = 0; i < NBTCMD; i++) {
				if ( strncmp(p,bt_cmd[i],strlen(bt_cmd[i])) == 0 )
				{
					//						ESP_LOGI(TAG,"rest: %s", savep);
					if ( strncmp(savep, "}", strlen("}")) == 0 ) {
						ESP_LOGI(TAG,"find command without parameter[%d]: %s", i, bt_cmd[i]);
						prs_s = i+100+1;
						break;
					}
					prs_s = i+1;
					ESP_LOGI(TAG,"find command: %s", bt_cmd[i]);
					break;
				}
			}
		} break;

		case BT_WIFI: { // parse 'wifi'.
							ESP_LOGD(TAG,"wifi cmd str: %s.", p);
							ESP_LOGI(TAG,"keys value: %s.", strtok_r(savep, "{},:'", &savep));
			ESP_LOGI(TAG,"BT_WIFI COMMAND CATCH");


			if ( strncmp(p, "ssid", strlen("ssid") ) == 0 ) {
				ESP_LOGI(TAG,"ssid: %s.", (p = strtok_r(savep, "{},:'", &savep)));
				memset(custom_ssid,0,sizeof(custom_ssid));
				memcpy(custom_ssid,p,sizeof(custom_ssid));
			}
			if ( strncmp(p, "psswd", strlen("psswd") ) == 0 ) {
				ESP_LOGI(TAG,"psswd: %s.", (p = strtok_r(savep, "{},:'", &savep)));
				memset(custom_password,0,sizeof(custom_password));
				memcpy(custom_password,p,sizeof(custom_password));

				esp_bridge_wifi_set(custom_ssid,custom_password);
				esp_wifi_disconnect();
				esp_wifi_connect();

				int t = T_US;
				while ( (T_US - t) < 30*SEC ) {
					if ( wifi_connected == true) {
						sprintf(str,"{'wifi':'success'}\r\n");
						totbtl = strlen(str);
						//    		ESP_LOGI(TAG,"%s(%d)",str, totbtl);
						bt_wr(totbtl, str);
						return;
					}
					vTaskDelay(1000/portTICK_PERIOD_MS);
				}
				sprintf(str,"{'wifi':'fail'}\r\n");
				totbtl = strlen(str);
				//    		ESP_LOGI(TAG,"%s(%d)",str, totbtl);
				bt_wr(totbtl, str);
			}
		} break;

		case BT_STATUS: { // parse 'wifi'.
			ESP_LOGI(TAG,"wifi cmd key: %s.", p);
			ESP_LOGI(TAG,"keys value: %s.", strtok_r(savep, "{},:'", &savep));
		} break;

		case BT_MAC: { 		// parse 'mac'.
			char macstr[50];
			get_mac_str(macstr);
			sprintf(str,"{'mac':'%s'}\r\n", macstr);
			totbtl = strlen(str);
			bt_wr(totbtl, str);
		} break;

		}
	}

	switch (prs_s) {

	case (BT_WIFI+100): { 	// parse 'wifi'.
		ESP_LOGI(TAG,"wifi cmd without action.");
		s[0] = 0;
		str[0] = 0;
		// Replay wifi nets.
		int indx = 0;

		wifi_ap_record_t *temp_ap_records_p=malloc(MAX_APs * sizeof(wifi_ap_record_t));
		memset(temp_ap_records_p,0, MAX_APs * sizeof(wifi_ap_record_t));

		wifi_scan(temp_ap_records_p,MAX_APs);

		if(temp_ap_records_p==NULL){
			break;
		}

		memset(ap_records,0, MAX_APs * sizeof(wifi_ap_record_t));
		memcpy(ap_records, temp_ap_records_p, MAX_APs * sizeof(wifi_ap_record_t));
		strcat(str,"{'wifi':[");

		for(int i=0;i<MAX_APs;i++)
		{
			sprintf(s, "{'ssid':'%s','security':'WPA %s','rssi':'%d'}",(char*) ap_records[i].ssid,auth_names[ap_records[i].authmode],ap_records[i].rssi);
			ESP_LOGI(TAG,"%s\r\n",s);
			if(i < MAX_APs-1)
				strcat(s,",");
			strcat(str,s);
		}

		strcat(str,"]}\r\n");

		totbtl = strlen(str);
		ESP_LOGI(TAG,"%s(%d)",str, totbtl);
		bt_wr(totbtl, str);
	} break;


	case (BT_STATUS+100):{ 	// parse 'status'.
		ESP_LOGD(TAG,"status cmd without action.");
		s[0] = 0;
		str[0] = 0;
		// Replay wifi nets.
		int indx = 0;
		strcat(str,"{'status':[");
		while ( indx < NBTS ) { // Populate JSON with SSIDs.
			switch (indx) {
			case BT_S_WIFI:{
				if (wifi_connected == false) {
					sprintf(s, "{'%s':{'status':'disconnected'}},",
							bt_status[indx] );
					strcat(str,s);
				}
				else {
//					sprintf(s, "{'%s':{'status':'connected','ssid':'%s'}},",
//							bt_status[indx],
//							libxphase_get_curr_ssid() );
//					strcat(str,s);
				}
			} break;
			}
			indx++;
		}
		if ( indx > 0 ) {
			str[strlen(str)-1] = 0;
		}
		strcat(str,"]}\r\n");
		totbtl = strlen(str);
		//    		ESP_LOGI(TAG,"%s(%d)",str, totbtl);
		bt_wr(totbtl, str);
	} break;

	case (BT_MAC+100):{ 		// parse 'mac'.
		char macstr[50];
		get_mac_str(macstr);
		sprintf(str,"{'mac':'%s'}\r\n", macstr);
		totbtl = strlen(str);
		bt_wr(totbtl, str);
	} break;

	}
}
/////////////////////////////////////////////////////////////////////////////




//static uint8_t check_sum(uint8_t *addr, uint16_t count) {
//    uint32_t sum = 0;
//
//    if (addr == NULL || count == 0) {
//        return 0;
//    }
//
//    for(int i = 0; i < count; i++) {
//        sum = sum + addr[i];
//    }
//
//    while (sum >> 8) {
//        sum = (sum & 0xff) + (sum >> 8);
//    }
//
//    return (uint8_t)~sum;
//}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event) {
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(TAG, "Advertising start failed\n");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(TAG, "Advertising stop failed\n");
		}
		else {
			ESP_LOGI(TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
				param->update_conn_params.timeout);
		break;
	default:
		break;
	}
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
	esp_gatt_status_t status = ESP_GATT_OK;
	if (param->write.need_rsp) {
		if (param->write.is_prep) {
			if (prepare_write_env->prepare_buf == NULL) {
				prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
				prepare_write_env->prepare_len = 0;
				if (prepare_write_env->prepare_buf == NULL) {
					ESP_LOGE(TAG, "Gatt_server prep no mem\n");
					status = ESP_GATT_NO_RESOURCES;
				}
			} else {
				if(param->write.offset > PREPARE_BUF_MAX_SIZE ||
						prepare_write_env->prepare_len > param->write.offset) {
					status = ESP_GATT_INVALID_OFFSET;
				} else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
					status = ESP_GATT_INVALID_ATTR_LEN;
				}
			}

			esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
			gatt_rsp->attr_value.len = param->write.len;
			gatt_rsp->attr_value.handle = param->write.handle;
			gatt_rsp->attr_value.offset = param->write.offset;
			gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
			memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
			esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);

			if (response_err != ESP_OK) {
				ESP_LOGE(TAG, "Send response error\n");
			}
			free(gatt_rsp);
			if (status != ESP_GATT_OK) {
				return;
			}
			memcpy(prepare_write_env->prepare_buf + param->write.offset,
					param->write.value,
					param->write.len);
			prepare_write_env->prepare_len += param->write.len;

		}else {
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
		}
	}
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
	if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
		esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
	}else{
		ESP_LOGI(TAG,"ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf) {
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	switch (event) {
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
		gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;
		gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
		//        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(ble_dev_name);
		char name[100];
		sprintf(name,"%s-%d", DEVICE_NAME, id_from_mac());
		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(name);
		if (set_dev_name_ret){
			ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
		}
		//config adv data
		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret){
			ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
		}
		adv_config_done |= adv_config_flag;
		//config scan response data
		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret){
			ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
		}
		adv_config_done |= scan_rsp_config_flag;

		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
		break;
	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = 4;
		rsp.attr_value.value[0] = 0xde;
		rsp.attr_value.value[1] = 0xed;
		rsp.attr_value.value[2] = 0xbe;
		rsp.attr_value.value[3] = 0xef;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
				ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
		if (!param->write.is_prep){
			ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hexdump_internal(TAG, param->write.value, param->write.len, ESP_LOG_DEBUG);
			//				esp_log_buffer_char_internal(TAG, param->write.value,param->write.len,ESP_LOG_WARN);

			// Control that all characters are ASCII printable and save into message buffer
			int i = 0;
			for (i = 0 ; i<param->write.len; i++) {
				if ( param->write.value[i] > 126 ) {
					//						bleprintf("error: not printable character into string");
					ESP_LOGW(TAG,"no printable character");
					//						break; // exit for cycle
				}
				else {
					if ( param->write.value[i] != '\r' ) {
						blemsg_buff[msgindxin].msg[blemsg_buff[msgindxin].len + i] = param->write.value[i];
						//							ESP_LOGD(TAG,"%c:%d", blemsg_buff[msgindxin].msg[blemsg_buff[msgindxin].len + i], blemsg_buff[msgindxin].len + i);
					}
					else {
						blemsg_buff[msgindxin].len += i;
						blemsg_buff[msgindxin].len++;
						blemsg_buff[msgindxin].msg[blemsg_buff[msgindxin].len] = 0; 	// string ending
						ESP_LOGD(TAG,"new BLE message detected: %s", blemsg_buff[msgindxin].msg);
						msgindxin = (msgindxin+1)%BLE_MSGBUFF_SIZE;
						msgcnt++;
						i = 0;
						break;
					}
				}
			}
			blemsg_buff[msgindxin].len += i;

			if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
				uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
				if (descr_value == 0x0001){
					blemsg_buff[msgindxin].len = 0;
					if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
						ESP_LOGD(TAG, "notify enable");
						//							can_send_notify = true;
						//							xSemaphoreGive(gatts_semaphore);
					}
				}
				//					else if (descr_value == 0x0002){
				//						if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
				//							ESP_LOGI(TAG, "indicate enable");
				//							uint8_t indicate_data[600];
				//							for (int i = 0; i < sizeof(indicate_data); ++i)
				//							{
				//								indicate_data[i] = i%0xff;
				//							}
				//
				//							for (int j = 0; j < 1000; j++) {
				//								//the size of indicate_data[] need less than MTU size
				//								esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
				//										sizeof(indicate_data), indicate_data, true);
				//							}
				//						}
				//					}
				else if (descr_value == 0x0000){
					blemsg_buff[msgindxin].len = 0;
					//						can_send_notify = false;
					//						a_property = 0;
					ESP_LOGD(TAG, "notify/indicate disable ");
				}
				//					else{
				//						ESP_LOGE(TAG, "unknown descr value");
				//						esp_log_buffer_hex(TAG, param->write.value, param->write.len);
				//					}

			}
		}
		example_write_event_env(gatts_if, &a_prepare_write_env, param);
	} break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(TAG,"ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&a_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
		gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
		a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				a_property,
				&gatts_demo_char1_val, NULL);
		if (add_char_ret){
			ESP_LOGE(TAG, "add char failed, error code =%x",add_char_ret);
		}
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {
		uint16_t length = 0;
		const uint8_t *prf_char;

		ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
		gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
		if (get_attr_ret == ESP_FAIL){
			ESP_LOGE(TAG, "ILLEGAL HANDLE");
		}

		ESP_LOGI(TAG, "the gatts demo char length = %x\n", length);
		for(int i = 0; i < length; i++){
			ESP_LOGI(TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
		}
		esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		if (add_descr_ret){
			ESP_LOGE(TAG, "add char descr failed, error code =%x", add_descr_ret);
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:

		gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		is_ble_conn = true;
		esp_ble_conn_update_params_t conn_params = {0};
		memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
		/* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
		conn_params.latency = 0;
		conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
		conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
		conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
		ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
				param->connect.conn_id,
				param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
				param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
		gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
		//start sent the update connection parameters to the peer device.
		//esp_ble_gap_update_conn_params(&conn_params);
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		is_ble_conn = false;
		ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT");
		esp_ble_gap_start_advertising(&adv_params);
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
		break;
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		if (param->congest.congested) {
			can_send_notify = false;
		} else {
			can_send_notify = true;
			xSemaphoreGive(gatts_semaphore);
		}
		break;
	default:
		break;
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
		} else {
			ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id,
					param->reg.status);
			return;
		}
	}

	/* If the gatts_if equal to profile A, call profile A cb handler,
	 * so here call each profile's callback */
	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == gl_profile_tab[idx].gatts_if) {
				if (gl_profile_tab[idx].gatts_cb) {
					gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

//void throughput_server_task(void *param) {
//    vTaskDelay(2000 / portTICK_PERIOD_MS);
//    uint8_t sum = check_sum(indicate_data, sizeof(indicate_data) - 1);
//    // Added the check sum in the last data value.
//    indicate_data[GATTS_NOTIFY_LEN - 1] = sum;
//
//    while(1) {
//        if (!can_send_notify) {
//            int res = xSemaphoreTake(gatts_semaphore, portMAX_DELAY);
//            assert(res == pdTRUE);
//        } else {
//            if (is_ble_conn) {
//                esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_A_APP_ID].gatts_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
//                                            gl_profile_tab[PROFILE_A_APP_ID].char_handle,
//                                            sizeof(indicate_data), indicate_data, false);
//            }
//        }
//
//    }
//}

void enable_ble( void ) {
	int ret = 0;

	// Get device Bluetooth MAC address
	uint8_t macaddr[6];
	esp_efuse_mac_get_default(macaddr);
	ESP_LOGI(TAG,"current dev MAC: %02x%02x%02x%02x%02x%02x",
			macaddr[0],macaddr[1],macaddr[2],macaddr[3],macaddr[4],macaddr[5]);
	ble_dev_name[sprintf(ble_dev_name,"AIT-BT-%02x%02x%02x%02x%02x%02x",
			macaddr[0],macaddr[1],macaddr[2],macaddr[3],macaddr[4],macaddr[5])] = 0;

	esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
	if (ret) {
		ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(TAG, "gap register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
	if (ret){
		ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret){
		ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}
}

void disable_ble( void ) {
	int ret = 0;
	ret = esp_ble_gatts_app_unregister(PROFILE_A_APP_ID);
	if (ret){
		ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	ret = esp_bluedroid_disable();
	if (ret) {
		ESP_LOGE(TAG, "%s disable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_deinit();
	if (ret) {
		ESP_LOGE(TAG, "%s deinit bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bt_controller_disable();
	if (ret) {
		ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bt_controller_deinit();
	if (ret) {
		ESP_LOGE(TAG, "%s deinitialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	while ( esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE );
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
}

void bleprintf(const char* format, ...){
	va_list args;
	va_start(args,format);
	if ( is_ble_conn == true ) {
		char buffer[BUFSIZ];
		int32_t TmpBytesToWrite;
		TmpBytesToWrite = vsprintf (buffer, format, args );
		esp_ble_gatts_send_indicate(
				gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
				gl_profile_tab[PROFILE_A_APP_ID].conn_id,
				gl_profile_tab[PROFILE_A_APP_ID].char_handle,
				TmpBytesToWrite,(uint8_t*) buffer, false );
	}
	va_end(args);
}

int is_ble_msg(void) {
	return msgcnt;
}

int get_ble_msg ( char *buf ) {
	int ret = blemsg_buff[msgindxout].len;
	if (msgcnt>0) {
		memcpy(buf, blemsg_buff[msgindxout].msg, ret);
		msgcnt--;
		msgindxout = (msgindxout+1)%BLE_MSGBUFF_SIZE;
	}
	else {
		return -1;
	}
	return ret;
}

int get_ble_state(void) {
	return is_ble_conn;
}
