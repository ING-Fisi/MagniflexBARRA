/*
 * Fisitron.h
 *
 *  Created on: 8 ott 2024
 *      Author: Utente
 */

#ifndef MAIN_FISITRON_H_
#define MAIN_FISITRON_H_

//*********************************************WIFI FISITRON BARRE************************************************* */

#define FISITRON_WIFI_SSID	"FisitronHUB"//FisitronBarre"
#define FISITRON_WIFI_PSW	"Fisitron319086"//B4rr3M4gn1!"
//*********************************************MQTT FISICONNECT************************************************* */

#define FISITRON_BROKER_URI	"mqtts://mqtt.fisitron.com:8883"
#define FISITRON_PROJECT_ID	"magniflex"

#define FISITRON_USER	"Fisitron4SmartDream!"
#define FISITRON_PSW	"Fisitron4SmartDream!"

#define FISITRON_CLIENT_ID_TEMPLATE				"apps/magniflex/registries/SmartDream/devices/%s"
#define FISITRON_DATA_TOPIC_TEMPLATE			"/apps/magniflex/registries/SmartDream/devices/%s/events"
#define FISITRON_DATA_TOPIC_SUB_TEMPLATE		"/apps/magniflex/registries/SmartDream/devices/%s/commands/#"


void fisitron_mqtt_app_start(void);
int send_fisitron_message(char *pub_js);


#endif /* MAIN_FISITRON_H_ */
