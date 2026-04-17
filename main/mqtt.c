
/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
// ESP-IDF
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "freertos/FreeRTOS.h"
#include "mqtt_client.h"
// Custom
#include "gcpjwt.h"
#include "mqtt.h"
#include "sdkconfig.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
static const char *TAG = "MQTT CLOUD";

// IOT Core topic definition
char device_path[200];
mqtt_serv_state_t mqtt_service_state = 0; // state holder.

char giotc_cfg_dev_id[50];
char giotc_data_topic[50];
char giotc_data_topic_sub[50];


#define JWT_SIZE 1000
char jwt[JWT_SIZE];

esp_err_t mqtt_app_start(esp_mqtt_client_handle_t *mqtt_client,
						 esp_mqtt_client_config_t *mqtt_ext_cfg) {
							
	esp_err_t ret = ESP_FAIL;
	esp_mqtt_client_config_t mqtt_cfg;

	memcpy(&mqtt_cfg, mqtt_ext_cfg,
		   sizeof(esp_mqtt_client_config_t)); // Copy external configuration.

	if (mqtt_ext_cfg->uri == NULL) {
		ESP_LOGE(TAG,
				 "error: missing URI into external esp_mqtt_client_config_t.");
		return ESP_FAIL;
	}

	memset(jwt,0,JWT_SIZE);
	if (strcmp(mqtt_ext_cfg->uri, GCPIOT_BROKER_URI) == 0)
	{ 
		// JWT ****************************************************//
		if (xgiotc_gen_JWT(jwt, JWT_SIZE, 3600) < 0) {
			return ESP_FAIL;
		}

		mqtt_cfg.uri = GCPIOT_BROKER_URI;
		mqtt_cfg.client_id = giotc_cfg_dev_id;
		mqtt_cfg.cert_pem = (const char *)roots_pem_start;
		mqtt_cfg.username = "device";
		mqtt_cfg.password = (const char *)jwt;

	} else {
		ESP_LOGW(TAG, "Don't use GIOTC.");
	}

	printf("JWT TOCKEN GENERTATED %s\n\r", jwt);
	*mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

	if (*mqtt_client == NULL) {
		ESP_LOGE(TAG, "error: mqtt client NULL.");
		return ESP_FAIL;
	}

	ret = esp_mqtt_client_start(*mqtt_client);

	return ret;
}

mqtt_serv_state_t get_mqtt_service_state(void) { return mqtt_service_state; }

void set_mqtt_service_state(mqtt_serv_state_t s) {
	// TODO: protect with semaphore
	mqtt_service_state = s;
}
