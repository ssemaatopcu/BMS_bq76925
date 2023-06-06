/*
 * bq76925.c
 *
 *  Created on: 23 May 2023
 *      Author: sematopcu
 */


#include "bq76925.h"
#include "adc.h"
#include "i2c_interface.h"



tI2C_Status writeRegister(uint8_t reg_addr, uint8_t command, tBQ76925 *bq76925){

uint8_t w_data[2];
    w_data[0] = reg_addr;
    w_data[1] = command;

    if (HAL_I2C_Master_Transmit(bq76925->hi2c, (0x04 << 1), w_data, 2, 100) != HAL_OK) {
    	return I2C_FAIL;
    }
    return I2C_SUCCESS;
}


tI2C_Status readRegister(uint8_t reg_addr, uint8_t r_data, tBQ76925 *bq76925){

	 if (HAL_I2C_Mem_Read(bq76925->hi2c, (0x04 << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, &r_data, 1, 100) != HAL_OK) {
		 return I2C_FAIL;
		     }
		     return I2C_SUCCESS;
}


tI2C_Status toggleBits(uint8_t reg_addr, uint8_t numBits, tBQ76925 *bq76925){
	uint8_t reg_data;

	    if (HAL_I2C_Mem_Read(bq76925->hi2c, (0x04 << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100) == HAL_OK) {
	        reg_data ^= (1 << numBits);  // Belirtilen bitleri ters çevirdik

	        if (HAL_I2C_Mem_Write(bq76925->hi2c, (0x04 << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100) != HAL_OK) {
	        	return I2C_FAIL;
	        	    }
	        	    return I2C_SUCCESS;
	    }
	}




void BQ76925_Init(tBQ76925 *hi2c, tBQ76925 *bq76925 )
{
    bq76925->hi2c = hi2c;

    writeRegister(0x03, 0x01, &bq76925);
    writeRegister(0x04, BIT_CONFIG2_REFSEL, &bq76925);
}

uint16_t get_ADC12bits_channel(ADC_TypeDef *ADCx, uint8_t channel)
{
ADC_ChannelConfTypeDef sConfig = {0};



    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5; //bilgi: ADC örnekleme süresi makroları ADC_SAMPLETIME_xCYCLES_5 gibi bir formatta kullanılır, burada x örnekleme süresini belirtir ve 1 ila 239 çevrim arasında değişebilir.
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {

        return 0;
    }


    if (HAL_ADC_Start(&hadc) != HAL_OK)
    {

        return 0;
    }


    if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) != HAL_OK)
    {

        return 0;
    }


    uint16_t adcValue = HAL_ADC_GetValue(&hadc);

    return adcValue;
}

float BQ76925_GetVoltage(tBQ76925 *hi2c, tBQ76925 *bq76925)
{

    uint16_t adcValue;
    float voltage;

    adcValue = get_ADC12bits_channel(ADC1, 1); // ADC1 kanal 1'dan veri okunuyor (ADC2_Voltage ???)

    // Gerilim hesaplama ve dönüşüm işlemleri
    voltage = (float)adcValue * 3.3f / 4095.0f; // ADC değerini gerilime dönüştürme (3.3V referans gerilimi ve 12-bit çözünürlük ? bulamadım)

    return voltage;
}






void request_VCout(uint8_t command, tBQ76925* bq76925) {

    writeRegister(VC_CAL_EXT_2_ADDR, command, &bq76925);
}

void voltage_Measuring(ADC_TypeDef* ADCx, Battery_type *bat, Calibration_type *cal) {
    for (uint8_t i = 0; i < 6; i++) {
        uint16_t ADC_Count = get_ADC12bits_channel(ADCx, i);
        corrected_Voltage(i, ADC_Count, bat->vRef100, cal, bat);
    }
}

void corrected_Voltage(uint8_t cellNumber, uint16_t ADC_Count, uint16_t vRef, Calibration_type *cal, Battery_type *bat) {
    int8_t vcOffset = cal->vcOffset_Corr[cellNumber];
    int8_t vcGain = cal->vcGain_Corr[cellNumber];

    // ADC sayımını gerilime dönüştürme işlemi
    float voltage = ADC_Count * (vRef / 4096.0);

    // Düzeltme faktörlerini uygulama işlemi
    voltage = voltage * (1.0 + (vcGain / 10.0)) + (vcOffset / 100.0);

    // Hesaplanan gerilimi ilgili hücrenin gerilim değeri olarak güncelleme
    bat->cellVoltage[cellNumber] = voltage;
}

void getCalParamVCOUT(Calibration_type *cal, tBQ76925* bq76925) {

    uint8_t vc1_cal, vc2_cal, vc3_cal, vc4_cal, vc5_cal, vc6_cal;
    uint8_t vref_cal_ext_1, vref_cal_ext_2;

    readRegister(0x11, &vc1_cal, bq76925);
    readRegister(0x12, &vc2_cal, bq76925);
    readRegister(0x13, &vc3_cal, bq76925);
    readRegister(0x14, &vc4_cal, bq76925);
    readRegister(0x15, &vc5_cal, bq76925);
    readRegister(0x16, &vc6_cal, bq76925);
    readRegister(0x17, &vref_cal_ext_1, bq76925);
    readRegister(0x18, &vref_cal_ext_2, bq76925);

    // Okunan değerleri cal yapısındaki ilgili alanlara atama (OFFSET_CORR=0xF0, GAIN_CORR=0x0F)
    cal->vcOffset_Corr[0] = (vc1_cal & 0xF0) >> 4;
    cal->vcGain_Corr[0] = vc1_cal & 0x0F;
    cal->vcOffset_Corr[1] = (vc2_cal & 0xF0) >> 4;
    cal->vcGain_Corr[1] = vc2_cal & 0x0F;
    cal->vcOffset_Corr[2] = (vc3_cal & 0xF0) >> 4;
    cal->vcGain_Corr[2] = vc3_cal & 0x0F;
    cal->vcOffset_Corr[3] = (vc4_cal & 0xF0) >> 4;
    cal->vcGain_Corr[3] = vc4_cal & 0x0F;
    cal->vcOffset_Corr[4] = (vc5_cal & 0xF0) >> 4;
    cal->vcGain_Corr[4] = vc5_cal & 0x0F;
    cal->vcOffset_Corr[5] = (vc6_cal & 0xF0) >> 4;
    cal->vcGain_Corr[5] = vc6_cal & 0x0F;
    cal->vrefOffset_Corr = (vref_cal_ext_1 & 0xF0) >> 4;
    cal->vrefGain_Corr = vref_cal_ext_1 & 0x0F;
    cal->REF_SEL = vref_cal_ext_2 & 0x01; //BIT_CONFIG2_REFSEL
}
