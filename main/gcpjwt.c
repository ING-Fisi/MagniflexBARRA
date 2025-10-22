
/* --------------------- INCLUDES ------------------------ *
 * ------------------------------------------------------- */
// Standard
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
// ESP-IDF
#include "esp_tls.h"
#include "mbedtls/pk.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
// Custom
#include "base64url.h"
#include "gcpjwt.h"
#include "esp_log.h"

#include "utility.h"

/* --------------------- DEFINES ------------------------- *
 * ------------------------------------------------------- */




/* --------------------- VARIABLES ----------------------- *
 * ------------------------------------------------------- */
static const char* TAG = "XGCPJWT";

long expire_t = 0; // TODO: put into GIOTC structure.

uint8_t *oBuf;

/* Return a string representation of an mbedtls error code */
static char* mbedtlsError(int errnum) {
    static char buffer[200];
    mbedtls_strerror(errnum, buffer, sizeof(buffer));
    return buffer;
}

long get_expire_t ( void ) {
	return expire_t;
}

int xgiotc_gen_JWT(char *jwtstr, uint32_t len, uint32_t exp_time_s) {


	//const uint8_t *privateKey = private_key_pem;


	//printf("PRIVATE KEY %s [%d]",privateKey,privateKeySize);

    char base64Header[100];
    const char header[] = "{\"alg\":\"RS256\",\"typ\":\"JWT\"}";
    ESP_LOGD(TAG,"JWT header: %s", header);
    base64url_encode( (unsigned char *)header, strlen(header), base64Header);

    // Save expire connection time stamp.
    expire_t = get_curtimestamp() + exp_time_s;

    time_t now;
    time(&now);
    uint32_t iat = now - 10;             	// Set the time now.
    uint32_t exp = iat + exp_time_s;	// Set the expire time.

    char payload[100];
    ESP_LOGI(TAG,"{\"iat\":%d,\"exp\":%d,\"aud\":\"%s\"}", iat, exp, GCPIOT_PROJECT_ID);
    sprintf(payload, "{\"iat\":%d,\"exp\":%d,\"aud\":\"%s\"}", iat, exp, GCPIOT_PROJECT_ID);
    ESP_LOGD(TAG,"JWT payload token: %s", payload);

    char base64Payload[100];
    base64url_encode( (unsigned char *)payload,  strlen(payload), base64Payload);        // Base64 encoded data.

    uint8_t headerAndPayload[800];
    sprintf((char*)headerAndPayload, "%s.%s", base64Header, base64Payload);

    // At this point we have created the header and payload parts, converted both to base64 and concatenated them
    // together as a single string.  Now we need to sign them using RSASSA
    mbedtls_pk_context pk_context;
    mbedtls_pk_init(&pk_context);
    int rc = mbedtls_pk_parse_key(&pk_context, private_key_pem, privateKeySize, NULL, 0);
    if (rc != 0) {
        ESP_LOGE(TAG,"Failed to mbedtls_pk_parse_key: %d (-0x%x): %s\n", rc, -rc, mbedtlsError(rc));
        return -1;
    }

//    uint8_t oBuf[5000];
    oBuf = (uint8_t*) malloc(8000);
    if (oBuf == NULL) {
    	ESP_LOGE(TAG,"error: oBuf null");
    	free(oBuf);
    	return -1;
    }

    uint8_t digest[32];
    rc = mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), headerAndPayload, strlen((char*)headerAndPayload), digest);
    if (rc != 0) {
    	ESP_LOGE(TAG,"Failed to mbedtls_md: %d (-0x%x): %s\n", rc, -rc, mbedtlsError(rc));
    	free(oBuf);
        return -1;
    }

    size_t retSize;
    rc = mbedtls_pk_sign(&pk_context, MBEDTLS_MD_SHA256, digest, sizeof(digest), oBuf, &retSize, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG,"Failed to mbedtls_pk_sign: %d (-0x%x): %s\n", rc, -rc, mbedtlsError(rc));
    	free(oBuf);
        return -1;
    }


    char base64Signature[600];
    base64url_encode((unsigned char *)oBuf, retSize, base64Signature);

    int rqlen = 0;
    if ( (rqlen = (strlen((char*)headerAndPayload) + 1 + strlen((char*)base64Signature) + 1)) >= len ) {
    	ESP_LOGE(TAG,"error: too short external JWT buffer, needed: %d, given: %d.", rqlen, len);
    	free(oBuf);
        return -1;
    }
    sprintf(jwtstr, "%s.%s", headerAndPayload, base64Signature);

    mbedtls_pk_free(&pk_context);

	free(oBuf);
    return 0;
}


