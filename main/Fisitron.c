/*
 * Fisitron.c
 *
 *  Created on: 8 ott 2024
 *      Author: Utente
 */

#include "Fisitron.h"

#include "esp_log.h"

#include "main.h"

#include "mqtt.h"

static const char *TAG = "FISITRON_DEBUG";

esp_mqtt_client_handle_t mqtt_fisitron;

char fisitron_cfg_dev_id[100];
char fisitron_data_topic[100];
char fisitron_data_topic_sub[100];
extern char macstr[20];
extern bool wifi_connected;
extern bool tare_request;

static void log_error_if_nonzero(const char *message, int error_code) {
	if (error_code != 0) {
		ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	}
}

void fisitron_mqtt_event_handler(void *handler_args, esp_event_base_t base,
								 int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base,
			 event_id);
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id = 0;
	switch ((esp_mqtt_event_id_t)event_id) {
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		msg_id = esp_mqtt_client_subscribe(client, fisitron_data_topic_sub, 1);
		set_mqtt_service_state(MQTT_SERV_CONNECTED);
		break;
		// msg_id = esp_mqtt_client_publish(client, "/fisitron_topic/log",
		// "data_fisi", 0, 1, 0);
		ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
		break;
	case MQTT_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

		//		if (wifi_connected == true) {
		//			fisitron_mqtt_app_start();
		//		} else {
		//			esp_restart();
		//		}

		break;

	case MQTT_EVENT_SUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		// msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0,
		// 0);
		ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
		printf("DATA=%.*s\r\n", event->data_len, event->data);

		ESP_LOGI(TAG, "[%d]->%s", event->data_len, event->data);

		if (strncmp(event->data, "RESET_BARRA", event->data_len) == 0) {
			ESP_LOGI(TAG, "COMMAND_RESET");
			esp_restart();
		}

		if (strncmp(event->data, "TARA_BARRA", event->data_len) == 0) {
			ESP_LOGI(TAG, "COMMAND_TARA");
			tare_request = true;
		}

		if (strncmp(event->data, "UPGRADE_BARRA", event->data_len) == 0) {
			ESP_LOGI(TAG, "UPGRADE_BARRA");
			ota_check();
		}

		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
			log_error_if_nonzero("reported from esp-tls",
								 event->error_handle->esp_tls_last_esp_err);
			log_error_if_nonzero("reported from tls stack",
								 event->error_handle->esp_tls_stack_err);
			log_error_if_nonzero("captured as transport's socket errno",
								 event->error_handle->esp_transport_sock_errno);
			ESP_LOGI(TAG, "Last errno string (%s)",
					 strerror(event->error_handle->esp_transport_sock_errno));
		}
		break;
	default:
		ESP_LOGI(TAG, "Other event id:%d", event->event_id);
		break;
	}
}

int send_fisitron_message(char *pub_js) {

	int ret = 0;

	ret = esp_mqtt_client_publish(mqtt_fisitron, fisitron_data_topic, pub_js, 0,
								  1, 0);

	return ret;
}

void fisitron_mqtt_app_start(void) {

	sprintf(fisitron_cfg_dev_id, FISITRON_CLIENT_ID_TEMPLATE, macstr);
	sprintf(fisitron_data_topic, FISITRON_DATA_TOPIC_TEMPLATE, macstr);
	sprintf(fisitron_data_topic_sub, FISITRON_DATA_TOPIC_SUB_TEMPLATE, macstr);

	ESP_LOGI(TAG, "fisitron_cfg_dev_id %s", fisitron_cfg_dev_id);
	ESP_LOGI(TAG, "fisitron_data_topic %s", fisitron_data_topic);
	ESP_LOGI(TAG, "fisitron_data_topic_sub %s", fisitron_data_topic_sub);

	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = FISITRON_BROKER_URI,
		.client_id = fisitron_cfg_dev_id,
		.username = FISITRON_USER,
		.password = FISITRON_PSW,
	};

	mqtt_fisitron = esp_mqtt_client_init(&mqtt_cfg);
	/* The last argument may be used to pass data to the event handler, in this
	 * example mqtt_event_handler */
	esp_mqtt_client_register_event(mqtt_fisitron, ESP_EVENT_ANY_ID,
								   fisitron_mqtt_event_handler, NULL);
	esp_mqtt_client_start(mqtt_fisitron);
}
