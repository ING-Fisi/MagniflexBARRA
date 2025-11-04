/*
 * magniflex_dec.c
 *
 *  Created on: May 10, 2020
 */

/* --------------------- INCLUDES ------------------------- *
 * -------------------------------------------------------- */
// ESP-IDF
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
// Custom
#include "snsmems.h"
#include "i2c-driver.h"
#include "mqtt.h"
#include "gcpjwt.h"
#include "main.h"



/* --------------------- DEFINES -------------------------- *
 * -------------------------------------------------------- */
typedef struct {
	float max;
	float avg;
	float min;
	float k1;
	float k2;
	float ratio;
} bpm_data_t;

typedef enum {
	LIS2DW12_POS = 0,
	HTS221_POS,
	LIS2MDL_POS,
	LSM6DS3_POS,
	STTS22H_POS,
	LSM6DSL_POS,
	SENSOR_NUM		// Automatic sensor number.
} mems_type_t;

// Periods Circular buffer.
#define PERIODS_BUFLEN 64

#define NCHECK_PRES	2

volatile int no_presence_cnt = 0;
volatile long t_snsmems_wdt = 0;	// Class watchdog.
extern bool connected_sns;

/* ---------------------- VARIABLES ----------------------- *
 * -------------------------------------------------------- */
static const char *TAG = "SNSMEMS";
extern esp_mqtt_client_handle_t mqttc; // FIXME: remove.

char mems_name_str[SENSOR_NUM][10] = {
		"LIS2DW12",
		"HTS221",
		"LIS2MDL",
		"LSM6DS3",
		"STTS22H",
		"LSM6DSL",
};

#ifdef GET_RAWDATA
// I2C data buffer
IRAM_ATTR uint8_t i2c_buffered_data[6000*sizeof(float)]; // TODO: disable if not connected to MQTT or BT enable.
#endif

// BPM holding structure.
bpm_data_t bpm[NSNS][6];

//*****************************************//
char js_speed[PUBSTR_SIZE];
char pjsdata_speed[DATASTR_SIZE];

//****************************

int magindx = 0; // Index to Mag board.

#ifdef USE_PERIOD_CIRCBUF
// Circular buffers.
uint16_t 	circ_azh[NSNS][PERIODS_BUFLEN], circ_azl[NSNS][PERIODS_BUFLEN],
circ_gxh[NSNS][PERIODS_BUFLEN], circ_gxl[NSNS][PERIODS_BUFLEN],
circ_gyh[NSNS][PERIODS_BUFLEN], circ_gyl[NSNS][PERIODS_BUFLEN];
// In buffer indexes
int azh_in[NSNS], azl_in[NSNS],
gxh_in[NSNS], gxl_in[NSNS],
gyh_in[NSNS], gyl_in[NSNS];
// In buffer indexes
int azh_n[NSNS], azl_n[NSNS],
gxh_n[NSNS], gxl_n[NSNS],
gyh_n[NSNS], gyl_n[NSNS];

// Initialize buffers variables;
void period_buf_init(void) {
	// Indexes.
	memset(azh_in, 0, NSNS*sizeof(int));
	memset(azl_in, 0, NSNS*sizeof(int));
	memset(gxh_in, 0, NSNS*sizeof(int));
	memset(gxl_in, 0, NSNS*sizeof(int));
	memset(gyh_in, 0, NSNS*sizeof(int));
	memset(gyl_in, 0, NSNS*sizeof(int));
	// Element counter.
	memset(azh_n, 0, NSNS*sizeof(int));
	memset(azl_n, 0, NSNS*sizeof(int));
	memset(gxh_n, 0, NSNS*sizeof(int));
	memset(gxl_n, 0, NSNS*sizeof(int));
	memset(gyh_n, 0, NSNS*sizeof(int));
	memset(gyl_n, 0, NSNS*sizeof(int));
}
// Push data function for periods buffers..
void period_buf_push( uint16_t* buf, int *idx, int *n, uint16_t val ) {
	buf[*idx] = val;
	*idx = (*idx+1)%PERIODS_BUFLEN;
	if ( *n < PERIODS_BUFLEN ) {
		*n += 1;
	}
}
#endif


#ifdef CIRC_LPF
#define FILT_DIM	30

// Circular LPF
typedef struct {
	float buf[FILT_DIM];
	u8 in;
	u8 dim;
	u8 div;
	u8 cnt;
} circfilt_t;

void init_cirf(circfilt_t *cf) {
	memset(cf->buf,0,sizeof(float)*FILT_DIM);
	cf->in = 0;
	cf->dim = FILT_DIM;
	cf->cnt = 0;
}

void push_circf_val(circfilt_t *cf, float val) {
	cf->buf[cf->in] = val;
	cf->in = (cf->in+1)%cf->dim;
	if (cf->cnt < cf->dim) {
		cf->cnt++;
	}
}

float get_circf_val(circfilt_t *cf) {
	float acc = 0;
	for (int i = 0; i < cf->cnt; i++) {
		acc += cf->buf[i];
	}
	return (float) (acc/cf->cnt);
}

// filters.
circfilt_t bpm_filt;
#endif



/* ---------------------- FUNCTIONS ----------------------- *
 * -------------------------------------------------------- */
int snsmems_initilaize(snsmems_t *sns) {
	int nsns = 0;
	set_gpio_output(SNSMEMS_EN_CMD, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE);
	vTaskDelay(1000/portTICK_PERIOD_MS);
	snsmems_en_cmd(1);
	vTaskDelay(1000/portTICK_PERIOD_MS);
	if ( i2c_master_init() != ESP_OK ) {
		ESP_LOGE(TAG, "error: init i2c master mode");
		return -1;
	}
	int a = 0;
	u32 buf;
	while ( a < 100 ) {
		if ( snsmems_rd(a, SNSMEMS_REG(Status_REG),(u8 *) &buf, REG_LEN) == ESP_OK ) {
			snsmems_print_stat(buf);
			snsmems_print_ver(a);
			sns[nsns].indx = a;	// assign SNSMEMS I2C address to buffer location.
			nsns++;
		}
		a++;
		//		vTaskDelay(1/portTICK_PERIOD_MS);
	}
#ifdef CIRC_LPF
	init_cirf(&bpm_filt);
#endif
	return nsns;
}

void snsmems_en_cmd( uint32_t en ) {
	gpio_set_level(SNSMEMS_EN_CMD,en);
}

int snsmems_rd(u16 snsadd, u16 offs, u8 *rxbuf, u16 nrd) {
	return i2c_master_read_slave_reg(I2C_NUM_1, snsadd, offs, rxbuf, nrd);
}

int snsmems_wr(u16 snsadd, u16 offs, u8 *wxbuf, u16 nwr) {
	return i2c_master_write_slave_reg(I2C_NUM_1, snsadd, offs, wxbuf, nwr);
}

void snsmems_print_stat(u32 stat) {
	ESP_LOGI(TAG,"Status: 0x%x", stat);
	for ( int j = 0; j<SENSOR_NUM; j++ ) {
		if ( (stat&(1<<j)) == (1<<j) ) {
			ESP_LOGI(TAG,"find: %s",mems_name_str[j]);
		}
	}
}

void snsmems_print_ver(u16 addr) {
	u32 buf;
	if ( snsmems_rd(addr, SNSMEMS_REG(Version_REG),(u8 *) &buf, REG_LEN) != ESP_OK ) {
		ESP_LOGE(TAG,"error: read SNSMEMS version at address %x(%d).", addr, addr);
		return;
	}
	ESP_LOGI(TAG,"Version: SW%dHW%d", (buf>>16)&0xff, buf&0xff);
}

/* Porting of Octave script for SNSMEMS data acquisition, encapsulated into a while cycle 	*
 * with a state machine in order to continuously produce new data. 							*
 * Get an index buffer as parameter, or a magniflex_dev_t.									*/
int acq_snsmems_raw_data (magniflex_reg_t *dev) {

	// Data acquisition variables.
	uint8_t rtbuff[100];
	int axis = 3, freqsamp = 208, blen = 5000;
	int err_cnt = 0;

	// Set data acquisition counters register.
	//ESP_LOGI(TAG,"Try one-shot.");
	memset(rtbuff, 0, 4);
	u16 *lendata = (uint16_t*) &rtbuff[0];
	*lendata = (u16) blen;
	rtbuff[3] = 1; 	// Get one T/H
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
		rtbuff[2] = (dev->snsmems[i].indx == magindx) ? 16 : 0; 	// Get 16 MAG data.
		if ( i2c_master_write_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, 6*4, rtbuff, 4 ) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c write fail. [device %d]", dev->snsmems[i].indx);
			dev->snsmems[i].iscomm = 0;
			err_cnt ++;
			if ( err_cnt == dev->cnt_nsns ) {
				ESP_LOGE(TAG,"error: no sensor reply.");
				return -1;
			}
		}
		dev->snsmems[i].iscomm = 1;
	}
	// Reset trigger [on rising edge].
	//ESP_LOGI(TAG,"Reset trigger.");
	memset(rtbuff, 0, 4);
	err_cnt = 0;
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
		if ( i2c_master_write_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, 7*4, rtbuff, 4) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c write fail. [device %d]", dev->snsmems[i].indx);
			dev->snsmems[i].iscomm = 0;
			err_cnt ++;
			if ( err_cnt == dev->cnt_nsns ) {
				ESP_LOGE(TAG,"error: no sensor reply.");
				return -1;
			}
		}
		dev->snsmems[i].iscomm = 0;
	}
	// Trigger broadcast.
	//ESP_LOGI(TAG,"Broadcast trigger acquisition.");
#ifdef GET_RAWDATA
	memset(i2c_buffered_data,0,sizeof(blen));
#endif
	rtbuff[0] = 1;
	if ( i2c_master_write_slave_reg(I2C_PORT_NUM, 0, 7*4, rtbuff, 4) != ESP_OK ) {
		ESP_LOGE(TAG,"error: i2c write fail. [BROADCAST]");
		return -2;
	}

	// Wait for acquisition without I2C	transaction to avoid electrical noise.
	int wait = (int) (blen/(freqsamp*axis) + 1);
	ESP_LOGE(TAG,"Wait for acquisition: %d s.", wait);
	long tmt = T_US;
	while ( (long) (T_US - tmt) < (long) (wait * SEC) ) {
		t_snsmems_wdt = T_US;
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	long tmt_wsns = T_US;
	while ( 1 ) {
		int chck = 0;
		err_cnt = 0;
		for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
			if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, (8*4), rtbuff, 4) != ESP_OK ) {
				ESP_LOGE(TAG,"error: i2c write fail. [device %d]", dev->snsmems[i].indx);
				dev->snsmems[i].iscomm = 0;
				err_cnt ++;
				if ( err_cnt == dev->cnt_nsns ) {
					ESP_LOGE(TAG,"error: no sensor reply.");
					return -1;
				}
			}
			dev->snsmems[i].iscomm = 1;
			ESP_LOGV(TAG,"chck: %d", chck);
			ESP_LOGV(TAG,"ret [%d]: %d %d %d %d", dev->snsmems[i].indx, rtbuff[0], rtbuff[1], rtbuff[2], rtbuff[3] );
			if ( (rtbuff[0] & 1) == 0 ) {
				ESP_LOGD(TAG,"[%d] done!", dev->snsmems[i].indx);
				chck += 1;
			}
		}
		if ( chck >= dev->cnt_nsns ) {
			ESP_LOGI(TAG,"SNSMEMS acquisition completed.");
			break;
		}
		if ( chck_time_int(&tmt_wsns, 5) == 1 ) {
			ESP_LOGW(TAG,"snsmems %d data acuqisition timeout.", dev->snsmems[chck].indx);
			break;
		}
		vTaskDelay(100);
	}

#ifdef GET_RAWDATA
	ESP_LOGI(TAG,"Get Raw SNSMEMS data.");
	err_cnt = 0;
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
		if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i], BUF_ADDR, (uint8_t*) i2c_buffered_data , blen*4) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c read fail. [device %d]", dev->snsmems[i]);
			err_cnt ++;
			if ( err_cnt == dev->cnt_nsns ) {
				ESP_LOGE(TAG,"error: no sensor reply.");
				return -1;
			}
		}
	}
#endif

	return 0;
}

void acq_snsmems_env_data ( magniflex_reg_t *dev ) {

	// Acquisition variables.
	float rtbuff[20];
	int rdlen = 0;
	float tmp_temp = 0.0f;
	float angle[dev->cnt_nsns], temp[dev->cnt_nsns], hum[dev->cnt_nsns];
	int32_t accx[dev->cnt_nsns], accy[dev->cnt_nsns], accz[dev->cnt_nsns],
	magx[dev->cnt_nsns], magy[dev->cnt_nsns], magz[dev->cnt_nsns];

	//ESP_LOGI(TAG, "acq_snsmems_env_data -> num sensori [%d]",dev->cnt_nsns);
	// Read environmental data.
	rdlen = 4*12;
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {

		if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, 10*4, (uint8_t*) &rtbuff[0], rdlen) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c write fail. [device %d]", dev->snsmems[i].indx);
			dev->snsmems[i].iscomm = 0;
			continue;	// Ignore sensor that didn't replayed.
		}
		dev->snsmems[i].iscomm = 1;
		//		esp_log_buffer_hexdump_internal(TAG,rtbuff,rdlen,ESP_LOG_WARN);

		//riporto l'angolo tra -pi/2 e pi/2 nel caso in cui il materasso sia stato messo sottosopra
		if(rtbuff[0] < -1.57){
			angle[i]=-(rtbuff[0] + 3.14);
		}else if(rtbuff[0] > 1.57){
			angle[i]=-(rtbuff[0] - 3.14);
		}else{
			angle[i] = rtbuff[0];
		}

		accx[i] = *((int32_t*)&rtbuff[1]);
		accy[i] = *((int32_t*)&rtbuff[2]);
		accz[i] = *((int32_t*)&rtbuff[3]);
		temp[i] = *((int32_t*)&rtbuff[7]);
		hum[i] = *((int32_t*)&rtbuff[8]);
		magx[i] = *((int32_t*)&rtbuff[9]);
		magy[i] = *((int32_t*)&rtbuff[10]);
		magz[i] = *((int32_t*)&rtbuff[11]);

//		ESP_LOGI(TAG, "[%d] angl:%.02f|acc:%d,%d,%d|temp:%.01f|hum:%.01f|mag:%d,%d,%d",
//				dev->snsmems[i].indx, angle[i], accx[i], accy[i], accz[i], temp[i], hum[i],
//				magx[i], magy[i], magz[i] );

		// Assign BODY_P parameter for each current SNSMEMS.
		// ------------------------------------------------.
		if (  angle[i] < dev->params[BODY_P].val.fbuf[NSNS*i] ) { // Check for MIN
			ESP_LOGD(TAG,"BODY_P value MIN found.");
			dev->params[BODY_P].val.fbuf[NSNS*i] = angle[i];
		}
		if ( angle[i] > dev->params[BODY_P].val.fbuf[NSNS*i+2] ) { // Check for MAX.
			ESP_LOGD(TAG,"BODY_P value MAX found.");
			dev->params[BODY_P].val.fbuf[NSNS*i+2] = angle[i];
		}
		dev->params[BODY_P].val.fbuf[NSNS*i+1] = angle[i]; // Assign AVG.

		// Assign TEMP parameter for each current SNSMEMS.
		// ----------------------------------------------.
		//		if (  temp[i] < dev->params[TEMP].val.fbuf[NSNS*i] ) { // Check for MIN
		//			ESP_LOGD(TAG,"TEMP value MIN found.");
		//			dev->params[TEMP].val.fbuf[NSNS*i] = temp[i];
		//		}
		//		if ( temp[i] > dev->params[TEMP].val.fbuf[NSNS*i+2] ) { // Check for MAX.
		//			ESP_LOGD(TAG,"TEMP value MAX found.");
		//			dev->params[TEMP].val.fbuf[NSNS*i+2] = temp[i];
		//		}
		//		dev->params[TEMP].val.fbuf[NSNS*i+1] = temp[i]; // Assign AVG.
		tmp_temp += temp[i]/(dev->cnt_nsns);

		if ( (i+1) == magindx ) { // Only for MAG equipped SNSMEMS. Should be central on strips.
			// FIXME: For this release humidity will be acquired only from MAG equipped device.

			// Assign HUM parameter for each current SNSMEMS.
			// ---------------------------------------------.
			if (  hum[i] < dev->params[HUM].val.fbuf[0] ) { // Check for MIN
				ESP_LOGD(TAG,"HUM value MIN found.");
				dev->params[HUM].val.fbuf[0] = hum[i];
			}
			if ( hum[i] > dev->params[HUM].val.fbuf[2] ) { 	// Check for MAX.
				ESP_LOGD(TAG,"HUM value MAX found.");
				dev->params[HUM].val.fbuf[2] = hum[i];
			}
			dev->params[HUM].val.fbuf[1] = hum[i]; 			// Assign AVG.

			// Calculate Compass from magnetometer components.
			// ----------------------------------------------.
			//			dev->params[MAG].val.ibuf[0] = magx[i];
			//			dev->params[MAG].val.ibuf[1] = magy[i];
			//			dev->params[MAG].val.ibuf[2] = magz[i];
			//--- Heading calculation
			float heading = atan2f((float) magy[i], (float) magx[i]) * (180.0f / 3.14f);
			if ( heading < 0 ) {
				heading += 360.0f;
			}
			dev->params[COMPASS].val.ibuf[0] = (int) heading;
			ESP_LOGW(TAG, "heading: %d", dev->params[COMPASS].val.ibuf[0]);
		}

		if ( dev->prsnc_trsh[i*2] == 0.0f ) { // Not initialized.
			dev->prsnc_trsh[i*2] = angle[i] - PRESENCE_THRESH;
			dev->prsnc_trsh[i*2+1] = angle[i] + PRESENCE_THRESH;
			ESP_LOGW(TAG,"Save threshold values [%.3f,%.3f].",
					angle[i] - PRESENCE_THRESH, angle[i] + PRESENCE_THRESH);
			//					dev->prsnc_trsh[i*2], dev->prsnc_trsh[i*2+1]);
			snsmems_nvs_save_thrsh(dev->prsnc_trsh, MAX_NSNS*2);
		}

	}

	dev->params[TEMP].val.fbuf[0] = tmp_temp;
	//ESP_LOGW(TAG,"temp: %.2f", dev->params[TEMP].val.fbuf[0]);
	// Count replying sensors.
	int rply_sns_cnt = 0;
	for (int l = 0; l < dev->cnt_nsns; l++) {
		if ((dev->snsmems[l].iscomm == 1)) {
			rply_sns_cnt++;
		}
	}

	// Check for Presence.
	if ( (dev->presence == 0) ) {
		for (int e = 0; e < dev->cnt_nsns ; e++ ) {
			if ( (dev->snsmems[e].iscomm = 1) && ((angle[e] <= dev->prsnc_trsh[e*2]) || (angle[e] >= dev->prsnc_trsh[e*2+1])) ) {
				ESP_LOGW(TAG,"Presence for threshold --------------------------------> %d  [%.02f] [%.02f] [%.02f]", e,angle[e],dev->prsnc_trsh[e*2],dev->prsnc_trsh[e*2+1]);
				dev->presence = 1;
				//rgbled_set_c_presence(0x050000, dev->presence);
				init_cirf(&bpm_filt);
				if ( dev->start_sleep_t == 0 ) {
					dev->start_sleep_t = get_curtimestamp(); // Get time sleep start.
					ESP_LOGW(TAG,"Presence detected, get time.");
#ifdef USE_PERIOD_CIRCBUF
					// Reset periods buffers.
					period_buf_init();
#endif
				}
				break;
			}
		}

		dev->params[HEART_R].val.fbuf[1] = 0;
		dev->params[BREATH_R].val.fbuf[1] = 0;
	}
	else if ( dev->presence == 1 ) {
		int chck = 0;
		for (int e = 0; e < dev->cnt_nsns ; e++ ) {
			//ESP_LOGW(TAG,"angle[%d]: %.2f", e, angle[e]);
			if ( (dev->snsmems[e].iscomm = 1) && (angle[e] > dev->prsnc_trsh[e*2]) && (angle[e] < dev->prsnc_trsh[e*2+1]) ) {
				//ESP_LOGW(TAG,"Threshold[%d]: %.2f, %.2f, %.2f.", e, angle[e], dev->prsnc_trsh[e*2], dev->prsnc_trsh[e*2+1]);
				chck++;
			}else{
				//ESP_LOGE(TAG,"Threshold[%d]: %.2f, %.2f, %.2f.", e, angle[e], dev->prsnc_trsh[e*2], dev->prsnc_trsh[e*2+1]);
			}
		}
		if (chck == rply_sns_cnt) {
			ESP_LOGW(TAG, "no_press_cnt: %d", no_presence_cnt);
			if ( no_presence_cnt++ > NCHECK_PRES ) {
				no_presence_cnt = 0;
				dev->presence = 0;
				//rgbled_set_c_presence(0x050000, dev->presence);
				dev->start_sleep_t = 0; // Reset sleep timer.
				ESP_LOGW(TAG,"Presence not detected, reset time.");
			}
		}
	}


	memset(js_speed,0,sizeof(js_speed));
	memset(pjsdata_speed,0,sizeof(pjsdata_speed));
	chck_req_periodic_pub(dev, js_speed, pjsdata_speed);


}

int get_periods_data ( magniflex_reg_t *dev, int i, uint16_t *buf, int len, int addr, char *str ) {
	if (len > 0) {
		int nread = ceil((float) len/2.0f);
		i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, addr, (uint8_t*) buf, 4*nread);
		//		esp_log_buffer_hexdump_internal(TAG,buf,4*nread,ESP_LOG_WARN);
		int dlen = nread*2;
		ESP_LOGV(TAG,"len to read: %d [given %d]", nread, len);
		if ( nread != len/2 ) {
			dlen -= 1;
		}
		ESP_LOGV(TAG,"%s\t| RAW\t| BPM\t|.", str);
		ESP_LOGV(TAG,"////////////////////////.");
		for ( int k = 0 ; k < dlen ; k++ ) {
			ESP_LOGV(TAG,"\t|%d\t|%.02f\t|.", buf[k], RAW2BPM(buf[k]));
		}
		return dlen;
	}
	return 0;
}

// Function for calculating variance
float variance(float *a, int n, float mean) {
	float sqDiff = 0.0f;
	float tmpf = 0.0f;
	for (int i = 0 ; i < n ; i++) {
		tmpf = (a[i] - mean)*(a[i] - mean);
		sqDiff += tmpf;
	}
	return (float) sqDiff/((float) (n-1));
}

void get_bpm_stats( bpm_data_t *bpm, uint16_t *buf, int len ) {
	float PHY[PERIODS_BUFLEN];
	bpm->max = RAW2BPM(buf[0]);
	bpm->min = RAW2BPM(buf[0]);
	int imin = 0, imax = 0;
	int l = 0;
	float acc = 0;
	float tmpf = 0.0f;
	ESP_LOGV(TAG,"Input vector");
	ESP_LOGV(TAG,"-----------------");
	for ( int j = 0 ; j < len ; j++ ) {
		tmpf = RAW2BPM(buf[j]);
		ESP_LOGV(TAG,"%.03f", tmpf);
		if ( ( tmpf >= 30) && (tmpf <= 140) ) { // Remove out scale values.
			if ( tmpf < bpm->min ) {
				bpm->min = tmpf;
				imin = l;
			}
			if ( tmpf > bpm->max ) {
				bpm->max = tmpf;
				imax = l;
			}
			PHY[l] = tmpf;
			l++;
			acc += tmpf;
		}
	}
	if ( l <= 0 ) {
		ESP_LOGW(TAG,"all data is out of range.");
		return;
	}
	ESP_LOGV(TAG,"min find: %d -> %f", imin, bpm->min);
	ESP_LOGV(TAG,"max find: %d -> %f", imax, bpm->max);
	//	esp_log_buffer_hexdump_internal(TAG,PHY,l*2,ESP_LOG_WARN);
	if (len > 2) {
		if ( imin > imax ) {
			acc -= bpm->min;
			acc -= bpm->max;
			bpm->avg = ((float) acc/((float)(l-2)));
			ESP_LOGV(TAG,"imin > imax");
			ESP_LOGV(TAG,"l: %d", l);
			ESP_LOGV(TAG,"(l-imin-1): %d", (l-imin-1));
			ESP_LOGV(TAG,"(l-imax-1): %d", (l-imin-2));
			memmove(&PHY[imin],&PHY[imin+1],(l-imin-1)*sizeof(float));
			memmove(&PHY[imax],&PHY[imax+1],(l-imax-2)*sizeof(float));
			l -= 2;
		}
		else if ( imin == imax ) {
			ESP_LOGV(TAG,"imin == imax");
			acc -= bpm->max;
			memmove(&PHY[imax],&PHY[imax+1],(l-imax-1)*sizeof(float));
			l--;
		}
		else {
			acc -= bpm->min;
			acc -= bpm->max;
			bpm->avg = ((float) acc/((float)(l-2)));
			ESP_LOGV(TAG,"imin < imax");
			ESP_LOGV(TAG,"l: %d", l);
			ESP_LOGV(TAG,"(l-imin-1): %d", (l-imin-1));
			ESP_LOGV(TAG,"(l-imax-2): %d", (l-imax-2));
			memmove(&PHY[imax],&PHY[imax+1],(l-imax-1)*sizeof(float));
			memmove(&PHY[imin],&PHY[imin+1],(l-imin-2)*sizeof(float));
			l -= 2;
		}
	}
	else {
		bpm->avg = ((float) acc/((float)l));
	}
	ESP_LOGV(TAG,"l: %d", l);
	ESP_LOGV(TAG,"Statistics vector");
	ESP_LOGV(TAG,"-------------------");
	for ( int i = 0 ; i < l ; i++ ) {
		ESP_LOGV(TAG,"%.03f",PHY[i]);
	}
	float var = variance(PHY, l, bpm->avg);
	float scart = sqrtf(var); // Good measure has low standard deviation.
	ESP_LOGD(TAG,"Statistics:\n\tavg: %f,\n\tvar: %f,\n\tscart: %f", bpm->avg, var, scart);
	bpm->k2 = ((float) l/scart); // Should be as big as possible.
	bpm->k1 = ((float) scart/l); // Should be as low as possible.
	//	bpm->k1 = var; // Should be as low as possible.
	bpm->ratio = ((float) scart/bpm->avg*100);
}

bpm_data_t sel_best_bpm ( bpm_data_t *bpm ) {
	bpm_data_t selbpm;
	memcpy(&selbpm, &bpm[0], sizeof(bpm_data_t));
	for ( int i = 0 ; i < 6 ; i++ ) {
		if ( (bpm[i].k1 < selbpm.k1) && (bpm[i].k1 != NAN) && (bpm[i].k2 != NAN) ) {
			memcpy(&selbpm, &bpm[i], sizeof(bpm_data_t));
		}
	}
	ESP_LOGI(TAG,"//// BPM ////\n"
			//"\t\tdev:\t|%d\t|\n"
			"\t\tmax:\t|%.02f\t|\n"
			"\t\tavg:\t|%.02f\t|\n"
			"\t\tmin:\t|%.02f\t|\n"
			"\t\tk1:\t|%.02f\t|\n"
			"\t\tk2:\t|%.02f\t|\n"
			"\t\tR: \t|%.02f\t|\n"
			"\t\tdif:\t|%.02f\t|\n",
			selbpm.max, selbpm.avg, selbpm.min,
			selbpm.k1, selbpm.k2, selbpm.ratio, fabsf(selbpm.k1-selbpm.k2) );
	//	ESP_LOGI(TAG,"dev:\t|%d\t|", seldev );
	//	ESP_LOGI(TAG,"max:\t|%.02f\t|", selbpm.max );
	//	ESP_LOGI(TAG,"avg:\t|%.02f\t|", selbpm.avg );
	//	ESP_LOGI(TAG,"min:\t|%.02f\t|", selbpm.min );
	//	ESP_LOGI(TAG,"k1:\t|%.02f\t|", selbpm.k1 );
	//	ESP_LOGI(TAG,"k2:\t|%.02f\t|", selbpm.k2 );
	//	ESP_LOGI(TAG,"ratio:\t|%.02f\t|", selbpm.ratio );
	return selbpm;
}

void print_bpm_vectors( bpm_data_t *bpm ) {
	ESP_LOGI(TAG,"////////////////////// BPM Vectors //////////////////////");
	ESP_LOGI(TAG,"    \t|azh\t|azl\t|gxh\t|gxl\t|gyh\t|gyl\t|");
	ESP_LOGI(TAG,"max:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].max, bpm[1].max, bpm[2].max, bpm[3].max, bpm[4].max, bpm[5].max );
	ESP_LOGI(TAG,"avg:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].avg, bpm[1].avg, bpm[2].avg, bpm[3].avg, bpm[4].avg, bpm[5].avg );
	ESP_LOGI(TAG,"min:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].min, bpm[1].min, bpm[2].min, bpm[3].min, bpm[4].min, bpm[5].min );
	ESP_LOGI(TAG,"k1:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].k1, bpm[1].k1, bpm[2].k1, bpm[3].k1, bpm[4].k1, bpm[5].k1 );
	ESP_LOGI(TAG,"k2:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].k2, bpm[1].k2, bpm[2].k2, bpm[3].k2, bpm[4].k2, bpm[5].k2 );
	ESP_LOGI(TAG,"R:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			bpm[0].ratio, bpm[1].ratio, bpm[2].ratio, bpm[3].ratio, bpm[4].ratio, bpm[5].ratio );
	ESP_LOGI(TAG,"dif:\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|%.02f\t|",
			fabsf(bpm[0].k2-bpm[0].k1), fabsf(bpm[1].k2-bpm[1].k1), fabsf(bpm[2].k2-bpm[2].k1), fabsf(bpm[3].k2-bpm[3].k1), fabsf(bpm[4].k2-bpm[4].k1), fabsf(bpm[5].k2-bpm[5].k1));

}

int acq_snsmems_data( magniflex_reg_t *dev ) {

	// Acquisition variables.
	uint8_t rtbuff[100];
	bpm_data_t best_bpm={0};
	int best_bpm_dev = 0;

	// Reset best_bpm each cycle.
	best_bpm.min = 200.0f;
	best_bpm.avg = 200.0f;
	best_bpm.max = 0.0f;
	best_bpm.k1 = 10000.0f; 	// High value because it has to be as low as possible.
	best_bpm.k2 = 10000.0f;

	if (magindx == 0) { 		// If magindx already assigned, device are already initialized.

		// 1. SENSORS STATUS
		// ------------------------------------------------------------------
		ESP_LOGI(TAG,"Read status registers.");
		for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
			if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, 0, rtbuff, 4*2) != ESP_OK ) {
				ESP_LOGE(TAG,"error: i2c write fail. [device %d]", dev->snsmems[i].indx);
				dev->snsmems[i].iscomm = 0;
			}
			dev->snsmems[i].iscomm = 1;
			ESP_LOGI(TAG,"Status [%d]: 0x%x", i, *((uint32_t*) &rtbuff[0]) );
			ESP_LOGI(TAG,"Version [%d]: 0x%x", i, *((uint32_t*) &rtbuff[4]) );
			if ( (rtbuff[0] & 4) == 4 ) {
				magindx = dev->snsmems[i].indx;
				ESP_LOGD(TAG,"Device with magnetometer: %d", magindx);
			}
		}

		// 2. SETUP SENSORS REGISTERS
		// ------------------------------------------------------------------
		// Enable print AX GX GY
		ESP_LOGI(TAG,"Enable print AX GX GY, T/H and Mag.");
		memset(rtbuff,0,sizeof(rtbuff));
		rtbuff[0] = 0x10;
		for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
			if ( i2c_master_write_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, 9*4, rtbuff, 4) != ESP_OK ) {
				ESP_LOGE(TAG,"error: i2c write fail. [device %d]", (dev->snsmems[i].indx));
				dev->snsmems[i].iscomm = 0;
			}
			dev->snsmems[i].iscomm = 1;
		}

	}

	// 3. READ RAW DATA
	// ------------------------------------------------------------------
	if ( acq_snsmems_raw_data(dev) ) { 	// No transaction during acquisition time.
		ESP_LOGE(TAG, "error: sensor read.");
		dev->presence = 0;
		//rgbled_set_c_presence(0x050000, dev->presence);
		return -1;
	}



	//acq_snsmems_env_data(dev);

	// 4. READ READS PERIODS NUMBER
	// ------------------------------------------------------------------
	// Periods.
	uint8_t raw_periods[dev->cnt_nsns][12];
	memset( raw_periods, 0, sizeof(raw_periods));
	uint16_t *periods[dev->cnt_nsns]; // Pointers to data.
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
		if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, OFFS_AZ_CNT*4, &raw_periods[i][0] , 4*3) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c read fail. [device %d]", dev->snsmems[i].indx);
			dev->snsmems[i].iscomm = 0;
		}
		dev->snsmems[i].iscomm = 1;
		periods[i] = (uint16_t*) &raw_periods[i][0]; // Point data.
	}
	// Min/Max.
	uint8_t raw_minmax[dev->cnt_nsns][24];
	memset( raw_minmax, 0, sizeof(raw_minmax));
	int32_t *minmax[dev->cnt_nsns];
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {
		if ( i2c_master_read_slave_reg(I2C_PORT_NUM, dev->snsmems[i].indx, OFFS_AZ_MAX*4, (uint8_t*) raw_minmax[i], 4*6) != ESP_OK ) {
			ESP_LOGE(TAG,"error: i2c read fail. [device %d]", dev->snsmems[i].indx);
			dev->snsmems[i].iscomm = 0;
		}
		dev->snsmems[i].iscomm = 1;
		minmax[i] = (int32_t*) &raw_minmax[i][0];
	}

	// 4. GET DATA FOR EACH PERIODS
	// ------------------------------------------------------------------
	// Repeat for each sensor.
	for ( int i = 0 ; i < dev->cnt_nsns ; i++ ) {

		if (dev->snsmems[i].iscomm == 0) {
			ESP_LOGW(TAG,"SNSMEMS %d didn't replayed, goes to next iteration.", dev->snsmems[i].indx);
			continue;
		}

		//		ESP_LOGI(TAG, "------------------- board %d ----------------------", dev->snsmems[i].indx);
		//		for ( int j = 0 ; j < 6 ; j++ ) { // DBG.
		//			ESP_LOGI(TAG, "period[%d][%d] = %d.", i, j, periods[i][j]);
		//		}
		//		for ( int j = 0 ; j < 6 ; j++ ) { // DBG.
		//			ESP_LOGI(TAG, "Min/Max[%d][%d] = %d.", i, j, minmax[i][j]);
		//		}

		//		ESP_LOGI(TAG,"periods:\n"
		//				"\tAccZHigh %d \tAccZLow %d\n"
		//				"\tGyroXHigh %d \tGyroXLow %d\n"
		//				"\tGyroYHigh %d \tGyroYLow %d",
		//				periods[i][0], periods[i][1],
		//				periods[i][2], periods[i][3],
		//				periods[i][4], periods[i][5]);
		//		ESP_LOGI(TAG,"Min/Max:\n"
		//				"\tAccZMax %d \tAccZMin %d \tDelta %d\n"
		//				"\tGyroXMax %d \tGyroXMin %d \tDelta %d\n"
		//				"\tGyroYMax %d \tGyroYMin %d \tDelta %d",
		//				minmax[i][0], minmax[i][1], minmax[i][0]-minmax[i][1],
		//				minmax[i][2], minmax[i][3], minmax[i][2]-minmax[i][3],
		//				minmax[i][4], minmax[i][5], minmax[i][4]-minmax[i][5]);

		// BPM holding variables.
		int nazh = periods[i][0];
		int nazl = periods[i][1];
		int ngxh = periods[i][2];
		int ngxl = periods[i][3];
		int ngyh = periods[i][4];
		int ngyl = periods[i][5];
		uint16_t azh[33];
		uint16_t azl[33];
		uint16_t gxh[33];
		uint16_t gxl[33];
		uint16_t gyh[33];
		uint16_t gyl[33];

		// Get raw BPM data from registers.
		nazh = get_periods_data(dev, i, azh, nazh, PERIODS_AZ_HIGH_ADDR, "azh");
		nazl = get_periods_data(dev, i, azl, nazl, PERIODS_AZ_LOW_ADDR, "azl");
		ngxh = get_periods_data(dev, i, gxh, ngxh, PERIODS_GX_HIGH_ADDR, "gxh");
		ngxl = get_periods_data(dev, i, gxl, ngxl, PERIODS_GX_LOW_ADDR, "gxl");
		ngyh = get_periods_data(dev, i, gyh, ngyh, PERIODS_GY_HIGH_ADDR, "gyh");
		ngyl = get_periods_data(dev, i, gyl, ngyl, PERIODS_GY_LOW_ADDR, "gyl");

		if(	(dev->presence == 0) ||
				(
						(minmax[i][0]-minmax[i][1]) > 500
						//					||
						//					(minmax[i][0]-minmax[i][1]) < 4
						||
						(minmax[i][2]-minmax[i][3])  > 5000
						||
						//					(minmax[i][2]-minmax[i][3])  < 600
						//					||
						(minmax[i][4]-minmax[i][5])  > 5000
						//					||
						//					(minmax[i][4]-minmax[i][5])  < 600
				)
		) {
			//			ESP_LOGW(TAG,"No Heart Rate detected device %d", dev->snsmems[i]);
			// If last device and the previous didn't assigned best_bpm, return and don't assign new values.
			if ( i == (dev->cnt_nsns - 1) && (best_bpm.avg == 0.0f) ) {
				return 0;
			}
			continue; // Go to next iteration.
		}

#ifdef USE_PERIOD_CIRCBUF
		// Get BPM statistics.
		// TODO: get statistics from circular periods buffer.
		ESP_LOGD(TAG,"circ_azh %d -> n: %d, idx: %d", i, azh_n[i], azh_in[i]);
		ESP_LOGD(TAG,"circ_azl %d -> n: %d, idx: %d", i, azl_n[i], azl_in[i]);
		ESP_LOGD(TAG,"circ_gxh %d -> n: %d, idx: %d", i, gxh_n[i], gxh_in[i]);
		ESP_LOGD(TAG,"circ_gxl %d -> n: %d, idx: %d", i, gxl_n[i], gxl_in[i]);
		ESP_LOGD(TAG,"circ_gyh %d -> n: %d, idx: %d", i, gyh_n[i], gyh_in[i]);
		ESP_LOGD(TAG,"circ_gyl %d -> n: %d, idx: %d", i, gyl_n[i], gyl_in[i]);

		// Fill circular buffers.
		for ( int k = 0 ; k < nazh ; k++ ) {
			period_buf_push(circ_azh[i], &azh_in[i], &azh_n[i], azh[k]);
		}
		for ( int k = 0 ; k < nazl ; k++ ) {
			period_buf_push(circ_azl[i], &azl_in[i], &azl_n[i], azl[k]);
		}
		for ( int k = 0 ; k < ngxh ; k++ ) {
			period_buf_push(circ_gxh[i], &gxh_in[i], &gxh_n[i], gxh[k]);
		}
		for ( int k = 0 ; k < ngxl ; k++ ) {
			period_buf_push(circ_gxl[i], &gxl_in[i], &gxl_n[i], gxl[k]);
		}
		for ( int k = 0 ; k < ngyh ; k++ ) {
			period_buf_push(circ_gyh[i], &gyh_in[i], &gyh_n[i], gyh[k]);
		}
		for ( int k = 0 ; k < ngyl ; k++ ) {
			period_buf_push(circ_gyl[i], &gyl_in[i], &gyl_n[i], gyl[k]);
		}

		ESP_LOGD(TAG,"Log circ_azh buffer.");
		for ( int a = 0 ; a < azh_n[i] ; a++ ) {
			ESP_LOGD(TAG,"%d", circ_azh[i][a]);
		}

		// Calculate statistics on circular buffer stored periods.
		get_bpm_stats( &bpm[i][0], circ_azh[i], azh_n[i]);
		get_bpm_stats( &bpm[i][1], circ_azl[i], azl_n[i]);
		get_bpm_stats( &bpm[i][2], circ_gxh[i], gxh_n[i]);
		get_bpm_stats( &bpm[i][3], circ_gxl[i], gxl_n[i]);
		get_bpm_stats( &bpm[i][4], circ_gyh[i], gyh_n[i]);
		get_bpm_stats( &bpm[i][5], circ_gyl[i], gyl_n[i]);
#else
		get_bpm_stats( &bpm[i][0], azh, nazh);
		get_bpm_stats( &bpm[i][1], azl, nazl);
		get_bpm_stats( &bpm[i][2], gxh, ngxh);
		get_bpm_stats( &bpm[i][3], gxl, ngxl);
		get_bpm_stats( &bpm[i][4], gyh, ngyh);
		get_bpm_stats( &bpm[i][5], gyl, ngyl);
#endif

		//print_bpm_vectors( bpm[i] );
		// Find Best BPM values among devices.
		// FIXME -> this function evaluate also 0 value from empty buffer.
		bpm_data_t tmpbpm = sel_best_bpm( bpm[i] );
		//		if ( (tmpbpm.k1 != 0) && tmpbpm.k1 < best_bpm.k1 ) {
		//			memcpy(&best_bpm,&tmpbpm,sizeof(bpm_data_t));
		//			best_bpm_dev = dev->snsmems[i];
		//		}

		if ( 	(tmpbpm.k1 != 0) &&
				(tmpbpm.avg + tmpbpm.min)/2 < (best_bpm.avg + best_bpm.min)/2) {
			memcpy(&best_bpm, &tmpbpm, sizeof(bpm_data_t));
			best_bpm_dev = dev->snsmems[i].indx;
		}
	}

	if ( (best_bpm.avg + best_bpm.min)/2 >= 200 ) {
		return 0;
	}

	//	ESP_LOGW(TAG,"//// Best BPM ////\n"
	//			"\t\tdev:\t|%d\t|\n"
	//			"\t\tmax:\t|%.02f\t|\n"
	//			"\t\tavg:\t|%.02f\t|\n"
	//			"\t\tmin:\t|%.02f\t|\n"
	//			"\t\tk1:\t|%.02f\t|\n"
	//			"\t\tk2:\t|%.02f\t|\n"
	//			"\t\tR:\t|%.02f\t|\n"
	//			"\t\tdif:\t|%.02f\t|\n",
	//			best_bpm_dev, best_bpm.max, best_bpm.avg, best_bpm.min, best_bpm.k1, best_bpm.k2, best_bpm.ratio, fabsf(best_bpm.k1-best_bpm.k2) );

	// Assign HEART parameter for each SNSMEMS.
	// ----------------------------------------
	//	if (  best_bpm.avg < dev->params[HEART_R].val.fbuf[0] ) { // Check for MIN
	////		ESP_LOGD(TAG,"Position value MIN found.");
	//		dev->params[HEART_R].val.fbuf[0] = best_bpm.avg;
	//	}
	//	if ( best_bpm.avg > dev->params[HEART_R].val.fbuf[2] ) { // Check for MAX.
	////		ESP_LOGD(TAG,"Position value MAX found.");
	//		dev->params[HEART_R].val.fbuf[2] = best_bpm.avg;
	//	}
	dev->params[HEART_R].val.fbuf[0] = best_bpm.min;
#ifdef CIRC_LPF
	push_circf_val(&bpm_filt, (best_bpm.avg+best_bpm.min)/2);
	//	push_circf_val(&bpm_filt, (best_bpm.avg+best_bpm.min-best_bpm.k1*best_bpm.min)/2);
	dev->params[HEART_R].val.fbuf[1] = get_circf_val(&bpm_filt);
#else
	dev->params[HEART_R].val.fbuf[1] = (best_bpm.avg+best_bpm.min)/2; // Assign AVG+MIN/2.
#endif
	//	dev->params[HEART_R].val.fbuf[1] = best_bpm.avg; // Assign AVG.
	dev->params[HEART_R].val.fbuf[2] = best_bpm.max;

	// Assign BREATH_R parameter for each SNSMEMS.
	// -------------------------------------------
	// TODO: For now get from breath division with randomized denominator [5 : 7].
	if ( (best_bpm.avg/(5+rand_int_decimal(2,1))) < dev->params[BREATH_R].val.fbuf[0] ) { // Check for MIN
		ESP_LOGD(TAG,"Position value MIN found.");
		dev->params[BREATH_R].val.fbuf[0] = (best_bpm.avg/(5+rand_int_decimal(2,1)));
	}
	if ( (best_bpm.avg/(5+rand_int_decimal(2,1))) > dev->params[BREATH_R].val.fbuf[2] ) { // Check for MAX.
		ESP_LOGD(TAG,"Position value MAX found.");
		dev->params[BREATH_R].val.fbuf[2] = (best_bpm.avg/(5+rand_int_decimal(2,1)));
	}
	dev->params[BREATH_R].val.fbuf[1] = (best_bpm.avg/(5+rand_int_decimal(2,1))); // Assign AVG.

	// Assign GOOD_K parameter for each current SNSMEMS.
	// -------------------------------------------------
	if (  best_bpm.k1 < dev->params[GOOD_K].val.fbuf[0] ) { // Check for MIN
		ESP_LOGD(TAG,"Position value MIN found.");
		dev->params[GOOD_K].val.fbuf[0] = best_bpm.k1;
	}
	if ( best_bpm.k1 > dev->params[GOOD_K].val.fbuf[2] ) { // Check for MAX.
		ESP_LOGD(TAG,"Position value MAX found.");
		dev->params[GOOD_K].val.fbuf[2] = best_bpm.k1;
	}
	dev->params[GOOD_K].val.fbuf[1] = best_bpm.k1; // Assign AVG.

	return 0;
}

u8 en_acq_snsmemes = 0;

void snsmems_acq_en( u8 val) {
	en_acq_snsmemes = val;
}

u8 snsmems_get_acq_en(void) {
	return en_acq_snsmemes;
}

extern magniflex_reg_t curdev; // FIXME: use better method to acccess object.

void snsmems_acq_tsk( void *vargs ) {
	//esp_task_wdt_add(NULL);
	ESP_LOGI(TAG, "snsmems_acq_tsk tasks START");

	while (1) {

		if(connected_sns == true)
		{
			acq_snsmems_env_data(&curdev);
		}

		//ESP_LOGI(TAG, "snsmems_acq_tsk tasks");

		vTaskDelay(1000/portTICK_PERIOD_MS);

	}

}

int snsmems_nvs_save_thrsh(float *thresh, int size) {
	nvs_handle nvsh;
	size = size*sizeof(float);
	int err = nvs_open("storage", NVS_READWRITE, &nvsh);
	if ( err != ESP_OK) {
		ESP_LOGE(TAG ,"Error (%s) opening NVS handle!\n", esp_err_to_name(err) );
		return -1;
	}
	else {
		// Write
		ESP_LOGD(TAG, "Save presence threshold values");

		ESP_LOGD(TAG,"nvs_open(): %s", esp_err_to_name(err));
		if (err != ESP_OK) return err;
		ESP_LOGW(TAG,"size: %d",size);
		err = nvs_set_blob(nvsh, THRSH_BLOBS_TAG, thresh, size);
		ESP_LOGD(TAG,"nvs_set_blob(): %s", esp_err_to_name(err));
		ESP_LOGD(TAG, "Committing updates in NVS");
		if ( (err = nvs_commit(nvsh)) != ESP_OK ) {
			ESP_LOGE(TAG, "fail committing NVS count updates ");
		}
		// Close
		nvs_close(nvsh);
	}
	ESP_LOGD(TAG, "update NVS blobs success.");
	return 0;
}

int snsmems_nvs_get_thrsh(float *thresh) {
	nvs_handle nvsh;
	int err = nvs_open("storage", NVS_READWRITE, &nvsh);
	if ( err != ESP_OK) {
		ESP_LOGE(TAG ,"Error (%s) opening NVS handle!\n", esp_err_to_name(err) );
		return -1;
	}
	else {
		// Read.
		size_t required_size = 0;
		err = nvs_get_blob(nvsh, THRSH_BLOBS_TAG, NULL, &required_size);
		ESP_LOGW(TAG,"required_size: %d", required_size);
		if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
			ESP_LOGD(TAG,"nvs_get_blob: %s", esp_err_to_name(err));
			return err;
		}
		if (required_size == 0) {
			ESP_LOGD(TAG,"Floar threshold not yet saved, return.");
			return 0;
		} else {
			err = nvs_get_blob(nvsh, THRSH_BLOBS_TAG, thresh, &required_size); // node structure
			if (err != ESP_OK) {
				ESP_LOGD(TAG,"nvs_get_blob: %s", esp_err_to_name(err));
				return err;
			}
		}
		nvs_close(nvsh);
	}
	return 0;
}

long snsmems_get_wdt(void) {
	return (long) (T_US - t_snsmems_wdt);
}

//////////////////////////////////////////////////////////////////////////////////
