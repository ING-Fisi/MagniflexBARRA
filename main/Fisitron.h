/*
 * Fisitron.h
 *
 *  Created on: 8 ott 2024
 *      Author: Utente
 */

#ifndef MAIN_FISITRON_H_
#define MAIN_FISITRON_H_


#define FISITRON_BROKER_URI	"ws://mqtt.fisitron.com:8084/mqtt"
#define FISITRON_PROJECT_ID	"e86beacc441c"

#define FISITRON_USER	"MagniFisitron1234!"
#define FISITRON_PSW	"MagniFlexFisi1234!"

#define FISITRON_CLIENT_ID_TEMPLATE				"apps/magniflex/registries/barre/devices/e86beacc441c"
#define FISITRON_DATA_TOPIC_TEMPLATE			"/apps/magniflex/registries/barre/devices/e86beacc441c/events"
#define FISITRON_DATA_TOPIC_SUB_TEMPLATE		"/apps/magniflex/registries/barre/devices/e86beacc441c/commands/#"


void fisitron_mqtt_app_start(void);


#endif /* MAIN_FISITRON_H_ */
