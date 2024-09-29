/*
 * ADC2VoltageValues.h
 *
 *  Created on: Oct 15, 2023
 *      Author: pedro
 */

#ifndef INC_ADC2VOLTAGEVALUES_H_
#define INC_ADC2VOLTAGEVALUES_H_

#include "stm32g4xx_hal.h"

typedef struct{
   float VoltageA;
   float VoltageB;
   float VoltageC;
}VoltageValues;

VoltageValues ADC2GPIOVoltage( VoltageValues ADCvalues, float ADC_offset);
VoltageValues GPIO2RealVoltage( VoltageValues GPIOVoltage, uint32_t ratio_numerator, uint32_t ratio_denominator);
VoltageValues ADC2RealValues( VoltageValues ADCvalues, uint32_t ratio_numerator, uint32_t ratio_denominator, float ADC_offset);


#endif /* INC_ADC2VOLTAGEVALUES_H_ */
