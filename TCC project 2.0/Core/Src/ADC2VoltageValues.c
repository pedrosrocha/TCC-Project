/*
 * ADC2VoltageValues.c
 *
 *  Created on: Oct 15, 2023
 *      Author: pedro
 */
#include "ADC2VoltageValues.h"


float max_GPIO_volt = 3.3;
uint16_t ADC_resolution = 4096;
float Voltage_per_bit = 0.0008056640625; //max_GPIO_volt/ADC_resolution

VoltageValues ADC2GPIOVoltage( VoltageValues ADCvalues, float ADC_offset){
	VoltageValues output;

	output.VoltageA = Voltage_per_bit * ADCvalues.VoltageA - ADC_offset;
	output.VoltageB = Voltage_per_bit * ADCvalues.VoltageB - ADC_offset;
	output.VoltageC = Voltage_per_bit * ADCvalues.VoltageC - ADC_offset;

	return output;
}

VoltageValues GPIO2RealVoltage( VoltageValues GPIOVoltage, uint32_t ratio_numerator, uint32_t ratio_denominator){
	VoltageValues output;
	output.VoltageA = GPIOVoltage.VoltageA * ratio_denominator/ratio_numerator;
	output.VoltageB = GPIOVoltage.VoltageB * ratio_denominator/ratio_numerator;
	output.VoltageC = GPIOVoltage.VoltageC * ratio_denominator/ratio_numerator;

	return output;
}

VoltageValues ADC2RealValues( VoltageValues ADCvalues, uint32_t ratio_numerator, uint32_t ratio_denominator, float ADC_offset){
	VoltageValues output;

	//output = ADC2GPIOVoltage(ADCvalues, ADC_offset);
	//output = GPIO2RealVoltage(output, ratio_numerator, ratio_denominator);
	float offset_adjusted = ADC_offset * Voltage_per_bit - 0.10f;


	output.VoltageA = (ADCvalues.VoltageA * Voltage_per_bit - offset_adjusted) * 293.3330f; // 220/12 * 16
	output.VoltageB = (ADCvalues.VoltageB * Voltage_per_bit - offset_adjusted) * 293.3330f; // 220/12 * 16
	output.VoltageC = (ADCvalues.VoltageC * Voltage_per_bit - offset_adjusted) * 293.3330f; // 220/12 * 16

	return output;
}
VoltageValues ADCtoVoltage( VoltageValues ADCvalues, float ADC_offset){
	VoltageValues output;
	output.VoltageA = (ADCvalues.VoltageA * Voltage_per_bit - ADC_offset) * 293.3330f; //
	output.VoltageB = (ADCvalues.VoltageB * Voltage_per_bit - ADC_offset) * 293.3330f; //
	output.VoltageC = (ADCvalues.VoltageC * Voltage_per_bit - ADC_offset) * 293.3330f; //

	return output;
}



