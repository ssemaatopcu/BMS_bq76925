# BMS_bq76925
//It includes adc reading and voltage conversions using STM32L051C6Tx microcontroller with bq76925 module.
// bq76925 + STM32L051C67x Demo Code

/* 
writeRegister() -> function used for writing data
readRegister() -> function used for reading data
voltage_Measuring() -> function used for voltage calculation equations and conversions
corrected_Voltage() -> converting ADC count to voltage, process of applying correction factors, Update the calculated voltage as the voltage value of the corresponding cell
BQ76925_GetVoltage() -> voltage values getting function
get_ADC12bits_channel() -> adc values getting function
getCalParamVCOUT() -> assigning cal parameters function
*/
 * The following connections between the bq76925 and STM32x are assumed:
 *
 *  | bq76925 pin name | STM32x pin name |
 *  | ---------------- | --------------- |
 *  |        SCL       |       PB6       |
 *  |        SDA       |       PB7       |
 *  |     ADC_Temp     |       PA0       |
 *  |    ADC_Voltage   |       PA1       |
 *  |    ADC_Current   |       PA2       |
 *-------------------------------------------------------------------------
    
