/*
 * Fisitron.h
 *
 *  Created on: 8 ott 2024
 *      Author: Utente
 */

#ifndef MAIN_FISITRON_H_
#define MAIN_FISITRON_H_


#define FISITRON_BROKER_URI	"mqtts://mqtt.fisitron.com:8883"
#define FISITRON_PROJECT_ID	"magniflex"

#define FISITRON_USER	"Fisitron4Debug!"
#define FISITRON_PSW	"Fisitron4Debug!"

#define FISITRON_CLIENT_ID_TEMPLATE				"apps/magniflex/registries/barre/devices/%s"
#define FISITRON_DATA_TOPIC_TEMPLATE			"/apps/magniflex/registries/barre/devices/%s/events"
#define FISITRON_DATA_TOPIC_SUB_TEMPLATE		"/apps/magniflex/registries/barre/devices/%s/commands/#"


void fisitron_mqtt_app_start(void);
int send_fisitron_message(char *pub_js);


#endif /* MAIN_FISITRON_H_ */
