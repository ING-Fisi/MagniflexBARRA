/*
 * gcjwt.h
 *
 *  Created on: Mar 4, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

#ifndef MAIN_X_GCPJWT_H_
#define MAIN_X_GCPJWT_H_

/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */

/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
// Server certificates
//extern const uint8_t roots_pem_start[] asm("_binary_roots_pem_start");
//extern const uint8_t roots_pem_end[] asm("_binary_roots_pem_end");
//// Client RSA private key
//extern const uint8_t private_key_pem_start[] asm("_binary_rsa_private_pem_start");
//extern const uint8_t private_key_pem_end[] asm("_binary_rsa_private_pem_end");
//extern const uint8_t private_key_pem_start[];
//extern const uint8_t private_key_pem_end[];

// Server certificates
extern const uint8_t roots_pem_start[] asm("_binary_roots_pem_start");
extern uint8_t private_key_pem[2000];
extern size_t privateKeySize;

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */
#define GCPIOT_BROKER_URI	"mqtts://europe-west1-mqtt.clearblade.com:8883"
#define GCPIOT_PROJECT_ID	"magniflex-291516"
#define GCPIOT_CLIENT_ID_TEMPLATE	"projects/magniflex-291516/locations/europe-west1/registries/smart-tech-lite-v1/devices/Device-%s"
#define DATA_TOPIC_TEMPLATE			"/devices/Device-%s/events"
#define DATA_TOPIC_SUB_TEMPLATE		"/devices/Device-%s/commands/#"

/**
 * @brief Google IOT Core initialization structure.
 *
 * Structure to fill in the needed parameters for GIOTC that will create the correct
 * path to device, along with device identification ID.
 */
typedef struct {
	/*@{*/
	char dev_id[30];	/**< GIOTC device identification name. */
	char prj_id[30]; 	/**< GIOTC project identification name. */
	char loc[30]; 		/**< GIOTC project geographical location. */
	char reg_id[30]; 	/**< GIOTC device registry identification name. */
	/*@}*/
	/**
	 * @name Topic Strings
	 */
	/*@{*/
	char data_topic[30]; 	/**< GIOTC data topic name. */
	/*@}*/
} giotc_cfg_t;



/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
// Server certificates
//extern const uint8_t root_pem_start[] asm("_binary_root_pem_start");
//extern const uint8_t root_pem_end[] asm("_binary_root_pem_end");
//// Client RSA private key
//extern const uint8_t private_key_pem_start[] asm("_binary_rsa_private_pem_start");
//extern const uint8_t private_key_pem_end[] asm("_binary_rsa_private_pem_end");



/* --------------------- FUNCTIONS ----------------------- *
 * ------------------------------------------------------- */
/* Create a JWT token for GCP (Google Core Platform).						*
 * JWT is built using two string: header and payload in JSON format.		*
 * Both are converted into base64 (normal/simple base64 encoding) then		*
 * concatenated together separated by a ".".								*
 * Resulting string is then signed using RSASSA which basically produces	*
 * an SHA256 message digest that is then signed. The resulting binary is	*
 * then itself converted into base64 and concatenated with the previously 	*
 * built base64 combined header and payload.								*
 *
 * @param projectId 		: GCP project id.								*
 * @param privateKey 		: The PEM or DER of the private key.			*
 * @param privateKeySize	: The size in bytes of the private key.			*
 *																			*
 * @return char* 			: A JWT token for transmission to GCP			*/
char* createGCPJWT(const char* projectId, uint8_t* privateKey, uint32_t exp_time_s, size_t privateKeySize);

//void setup_gcpiot_struct(void);
// TODO: Update description and documentation.
char *get_gcpiot_dev_id(void);
char *get_gcpiot_sub_topic_cmd(void);
char *get_gcpiot_pub_topic_event(void);
char *get_gcpiot_pub_topic_state(void);
void xgiotc_print_cfg(giotc_cfg_t* cfg);
long get_expire_t ( void );

/**
 * @brief Google IOT Core initialization function.
 *
 * Function to initialize Google IOT Core internal structure and connection.
 * @code
 * giotc_cfg_t *new_gciot_cfg = {
 * 	dev_id = "dev_id";
 * 	prj_id = "prj_id";
 * 	loc = "location";
 * 	reg_id = "red_id";
 * };
 * int ret = xgiotc_init(&new_gciot_cfg);
 * @endcode
 * @param giotc_cfg_t GIOTC initialization structure.
 * @return return value 0 for SUCCESS or ERROR_CODE.
 */
int xgiotc_init(giotc_cfg_t* cfg);

/**
 * @brief Function to generate Google IOT Core JWT access key.
 *
 * JWT is built using two string: header and payload in JSON format.
 * Both are converted into base64 (normal/simple base64 encoding) then
 * concatenated together separated by a ".".
 * Resulting string is then signed using RSASSA which basically produces
 * an SHA256 message digest that is then signed. The resulting binary is
 * then itself converted into base64 and concatenated with the previously
 * built base64 combined header and payload.
 * @code
 * char jwt[1500];
 * int ret = xgiotc_gen_JWT(jwt, 1500, (3600*1)); // 3600 seconds * 1 hour.
 * @endcode
 * @param char JWT holding string buffer.
 * @param uint32_t JWT holding string buffer length.
 * @param uint32_t expiring time in seconds.
 * @return return value 0 for SUCCESS or ERROR_CODE.
 * @warning Give a buffer with a length >= 1200.
 */
int xgiotc_gen_JWT(char *jwtstr, uint32_t len, uint32_t exp_time_s);

/**
 * @brief Function to generate Google IOT Core device path.
 *
 * Create and copy Google IOT Core device path string using the
 * initialized parameters.
 * @code
 * char device_path[200];
 * xgiotc_gen_devpath(device_path);
 * @endcode
 * @param char holding device path string buffer.
 * @return return value 0 for SUCCESS or ERROR_CODE.
 */
char * xgiotc_gen_devpath(void);

#endif /* MAIN_X_GCPJWT_H_ */
