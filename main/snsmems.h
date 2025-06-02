/*
 * magniflex_dev.h
 *
 *  Created on: May 10, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

#ifndef MAIN_SNSMEMS_H_
#define MAIN_SNSMEMS_H_


/* --------------------- INCLUDES ------------------------- *
 * -------------------------------------------------------- */
// Custom
#include "utility.h"
#include "main.h"

/* --------------------- DEFINES -------------------------- *
 * -------------------------------------------------------- */
#define MAX_AFE_N 100

#define max_periods 32
#define OFFS_AZ_CNT 22
#define OFFS_GX_CNT 23
#define OFFS_GY_CNT 24
#define OFFS_AZ_MAX 25
#define PERIODS_AZ_HIGH_ADDR	224
#define PERIODS_AZ_LOW_ADDR		228
#define PERIODS_GX_HIGH_ADDR	232
#define PERIODS_GX_LOW_ADDR 	236
#define PERIODS_GY_HIGH_ADDR	240
#define PERIODS_GY_LOW_ADDR 	244

#define MEM_WORDS32 (33)
#define MEM_BYTES	(MEM_WORDS32<<2)

#define FIFO_ADDR	(248)
#define BUF_ADDR	(252)

#define NSNS 3
#define FREQSAMP 208
#define PRESENCE_THRESH (3*0.005f)

#define RAW2BPM(x) ((float)FREQSAMP/x)*60
#define BPM2RAW(x) ((float)FREQSAMP/x)*60.0f

#define REG_LEN	4
#define SNSMEMS_REG(x) (x*REG_LEN)

#define SNSMEMS_EN_CMD	17

typedef enum {
	Status_REG = 0,			//0
	Version_REG,
	McuId0_REG,
	McuId1_REG,
	McuId2_REG,
	FwCrc_REG,				//5
	Aux1_REG,
	Aux2_REG,
	Aux3_REG,
	Aux4_REG,
	Angle_REG,				//10
	AccX_REG,
	AccY_REG,
	AccZ_REG,
	GyroX_REG,
	GyroY_REG,				//15
	GyroZ_REG,
	Temperature_REG,
	Humidity_REG,
	MagnX_REG,
	MagnY_REG,				//20
	MagnZ_REG,
	periodsNumbersAz_REG,	// 16 bit every counter LSB: high MSB: low
	periodsNumbersGx_REG,	// 16 bit every counter LSB: high MSB: low
	periodsNumbersGy_REG,	// 16 bit every counter LSB: high MSB: low
	AzMax_REG,				//25
	AzMin_REG,
	GxMax_REG,
	GxMin_REG,
	GyMax_REG,
	GyMin_REG,				//30
	Aux5_REG,
	Aux6_REG,
} snsmems_reg_t;

typedef volatile struct { // Registers can be externally modified.
	union{
		struct {
			u32 Status;			//0
			u32 Version;
			u32 McuId0;
			u32 McuId1;
			u32 McuId2;
			u32 FwCrc;			//5
			u32 Aux1;
			u32 Aux2;
			u32 Aux3;
			u32 Aux4;
			u32 Angle;			//10
			u32 AccX;
			u32 AccY;
			u32 AccZ;
			u32 GyroX;
			u32 GyroY;			//15
			u32 GyroZ;
			u32 Temperature;
			u32 Humidity;
			u32 MagnX;
			u32 MagnY;			//20
			u32 MagnZ;
			u32 periodsNumbersAz;	//16 bit every counter LSB: high MSB: low
			u32 periodsNumbersGx;	//16 bit every counter LSB: high MSB: low
			u32 periodsNumbersGy;	//16 bit every counter LSB: high MSB: low
			u32 AzMax;			//25
			u32 AzMin;
			u32 GxMax;
			u32 GxMin;
			u32 GyMax;
			u32 GyMin;			//30
			u32 Aux5;
			u32 Aux6;
		};
		u32 array[MEM_WORDS32];
	};
} memspace_t;

/* ---------------------- VARIABLES ----------------------- *
 * -------------------------------------------------------- */

/* ---------------------- FUNCTIONS ----------------------- *
 * -------------------------------------------------------- */
/* Detect and initialize I2C sensors.						*
 * 															*
 * @void 						.							*
 *															*
 * @return													*
 * 		int		: <0 error code. >0 number of sensors,		*/
int snsmems_initilaize(snsmems_t *sns) ;

/* Function to read SNSMEMS internal memory space.			*
 * 															*
 * @parameters												*
 * 		snsadd		: I2C SENSMES address.					*
 * 		off			: Memory space offset specified by enum	*
 * 					  snsmems_reg_t.						*
 * 		rxbuf		: Data buffer pointer					*
 * 		nrd			: Number of byte to read.				*
 *															*
 * @return													*
 * 		int		: <0 ERROR code. =0 SUCCESS,				*/
int snsmems_rd( u16 snsadd, u16 offs, u8 *rxbuf, u16 nrd);

/* Function to write SNSMEMS internal memory space.			*
 * 															*
 * @parameters												*
 * 		snsadd		: I2C SENSMES address.					*
 * 		off			: Memory space offset specified by enum	*
 * 					  snsmems_reg_t.						*
 * 		wxbuf		: Data buffer pointer					*
 * 		nwr			: Number of byte to read.				*
 *															*
 * @return													*
 * 		int		: <0 ERROR code. =0 SUCCESS,				*/
int snsmems_wr( u16 snsadd, u16 offs, u8 *wxbuf, u16 nwr);

/* Function to console print SNSMENS status register.		*
 * Print out detected MEMS.									*
 * 															*
 * @parameters												*
 * 		stat		: SNSMEMS state register value.			*
 *															*
 * @return													*
 * 		<void>												*/
void snsmems_print_stat(u32 stat);

/* Function to console print SNSMENS version register.		*
 * Print out format SW<v>HW<v>.								*
 * 															*
 * @parameters												*
 * 		addr	: I2C SENSMES address.						*
 *															*
 * @return													*
 * 		<void>												*/
void snsmems_print_ver(u16 addr);

int snsmems_nvs_save_thrsh(float *thresh, int size);

int snsmems_nvs_get_thrsh(float *thresh);

void snsmems_en_cmd( uint32_t en );

// TODO: tidy up.
void snsmems_acq_en(u8 val);
u8 snsmems_get_acq_en(void);
void snsmems_acq_tsk(void *vargs);
long snsmems_get_wdt(void);

void period_buf_init(void);
int acq_snsmems_data( magniflex_reg_t *dev );
void acq_snsmems_env_data ( magniflex_reg_t *dev );

#endif /* MAIN_SNSMEMS_H_ */
