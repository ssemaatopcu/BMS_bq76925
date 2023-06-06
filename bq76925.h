/*
 * bq76925.h
 *
 *  Created on: 23 May 2023
 *      Author: sematopcu
 */

#ifndef INC_BQ76925_H_
#define INC_BQ76925_H_

#include "i2c_interface.h"


// bq76925 I2C register address macros
#define I2C_GROUP_ADDR         (0x04)

/* Defines of BQ76925 addresses */
#define CHIP_ID_ADDR           (0x07)  /* Read Only */
#define STATUS_ADDR            (0x00)  /* Read/Write */

#define CELL_CTL_ADDR          (0x01)  /* Read/Write */
#define BAL_CTL_ADDR           (0x02)  /* Read/Write */
#define CONFIG_1_ADDR          (0x03)  /* Read/Write */
#define CONFIG_2_ADDR          (0x04)  /* Read/Write */
#define POWER_CTL_ADDR         (0x05)  /* Read/Write */


#define VREF_CAL_ADDR          (0x10)  /* EEPROM */
#define VC1_CAL_ADDR           (0x11)  /* EEPROM */
#define VC2_CAL_ADDR           (0x12)  /* EEPROM */
#define VC3_CAL_ADDR           (0x13)  /* EEPROM */
#define VC4_CAL_ADDR           (0x14)  /* EEPROM */
#define VC5_CAL_ADDR           (0x15)  /* EEPROM */
#define VC6_CAL_ADDR           (0x16)  /* EEPROM */
#define VC_CAL_EXT_1_ADDR      (0x17)  /* EEPROM */
#define VC_CAL_EXT_2_ADDR      (0x18)  /* EEPROM */
#define VREF_CAL_EXT_ADDR      (0x1B)  /* EEPROM */

/* Masks for Calibration Structure  BURAYA TEKRAR BAK DATASHEETTEN */

#define OFFSET_CORR	           (0xF0)
#define GAIN_CORR	           (0x0F)

#define VC1_OC_4	           (0x80)
#define VC1_GC_4	           (0x40)
#define VC2_OC_4	           (0x20)
#define VC2_GC_4	           (0x10)
#define VC3_OC_4	           (0x80)
#define VC3_GC_4	           (0x40)
#define VC4_OC_4	           (0x20)
#define VC4_GC_4	           (0x10)
#define VC5_OC_4	           (0x08)
#define VC5_GC_4	           (0x04)
#define VC6_OC_4	           (0x02)
#define VC6_GC_4	           (0x01)
#define VREF_OC5	           (0x04)
#define VREF_OC4	           (0x02)
#define VREF_GC4	           (0x01)

/*  REG_STATUS */
/* Read status of alert, CRC-Error and the power-on reset flag */
#define STATUS_ALERT	       (0x04)	/* ‘1’ = over-current */
#define STATUS_CRC_ER	       (0x02)	/* ‘1’ = CRC error */
#define STATUS_POR		       (0x01)	/* Set on each power-up and wake-up from sleep */

/* REG_BAL_CTL */
/* Default: no cell balancing */
#define BAL_1	               (0x01)
#define BAL_2	               (0x02)
#define BAL_3	               (0x04)
#define BAL_4	               (0x08)
#define BAL_5	               (0x10)
#define BAL_6	               (0x20)

/* REG_CONFIG_1 */
/* Default:	Comparator threshold(0): 25 mV
 *			I_COMP_POL(0): trips on discharge current (SENSEP > SENSEN)
 			I_AMP_CAL(0): current amplifier reports SENSEN with respect to VSS.
 			I_GAIN(0): 4 */
#define INIT_REG_CONFIG_1      (0x01)

#define INIT_REG_CONFIG_2	   (BIT_CONFIG2_REFSEL)

/* Bits for toggling */
#define BIT_CONFIG1_IGAIN	   (0x01)
#define BIT_CONFIG1_IAMPCAL	   (0x04)
#define BIT_CONFIG1_ICOMPOL	   (0x08)
#define BIT_CONFIG1_ITRESH1	   (0x10)
#define BIT_CONFIG1_ITRESH2	   (0x20)
#define BIT_CONFIG1_ITRESH3	   (0x40)
#define BIT_CONFIG1_ITRESH4	   (0x80)

#define BIT_CONFIG2_REFSEL	   (0x01)
#define BIT_CONFIG2_CRCEN	   (0x80)


typedef struct{                // Create a structure that will hold the
        uint8_t data;         // most recent received or transmitted data
        uint8_t error;
} i2c_trans;

/* Battery Stack data structure  */

typedef struct
{
	uint32_t cellVoltage[6];
	uint32_t vRef100;
	uint32_t vRef085;
	uint32_t vRef050;
} Battery_type;

/*  Calibration data structure */

typedef struct
{
	int8_t vcOffset_Corr[6];
	int8_t vcGain_Corr[6];
	int8_t vrefOffset_Corr;
	int8_t vrefGain_Corr;
	int8_t REF_SEL; /* 1.5 V if 0 - 3.0 V if 1*/
} Calibration_type;

typedef struct BQ76925{
	I2C_HandleTypeDef* hi2c;
}tBQ76925;


/* General Functions */
void BQ76925_Init(tBQ76925 *hi2c, tBQ76925* bq76925);
tI2C_Status writeRegister(uint8_t reg_addr, uint8_t command, tBQ76925 *bq76925);
tI2C_Status readRegister(uint8_t reg_addr, uint8_t r_data, tBQ76925 *bq76925);
tI2C_Status toggleBits(uint8_t reg_addr, uint8_t numBits, tBQ76925 *bq76925);

/* Voltage reading Functions */

float BQ76925_GetVoltage(tBQ76925 *hi2c, tBQ76925* bq76925);
// ADC Functions
uint16_t get_ADC12bits_channel(ADC_TypeDef *ADCx, uint8_t channel);


void request_VCout(uint8_t command, tBQ76925* bq76925);
void voltage_Measuring(ADC_TypeDef *ADCx, Battery_type *bat, Calibration_type *cal);
void corrected_Voltage(uint8_t cellNumber, uint16_t ADC_Count, uint16_t vRef, Calibration_type *cal, Battery_type *bat);
void getCalParamVCOUT(Calibration_type *cal, tBQ76925* bq76925);

/* ADC Functions */
//uint8_t calibrate_ADC(ADC_TypeDef *ADCx);
//uint16_t get_ADC12bits_channel(ADC_TypeDef *ADCx, uint8_t channel);
//void calibrateVRef(ADC_TypeDef *ADCx, Battery_type *bat, Calibration_type *cal);









#endif /* INC_BQ76925_H_ */
