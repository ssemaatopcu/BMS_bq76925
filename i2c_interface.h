/*
 * i2c_interface.h
 *
 *  Created on: 25 May 2023
 *      Author: sematopcu
 */

#ifndef INC_I2C_INTERFACE_H_
#define INC_I2C_INTERFACE_H_

#include "main.h"

typedef enum
{
	I2C_SUCCESS = 	(0x00U),
	I2C_FAIL = 		(0x01U),
	I2C_BUSY =	 	(0x02U),
	I2C_TIMEOUT =	(0x03U)
}tI2C_Status;


#endif /* INC_I2C_INTERFACE_H_ */
