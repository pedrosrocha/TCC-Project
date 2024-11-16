#include "ClarkeParkTransformation.h"


float _1_div_sqrt3 = 0.57735026919;
float _2_div_3 = 0.66667;
float _sqrt_3_2 = 0.8660254;
float _sqrt_3_3 = 0.5773503;   //_2_div_3 * _sqrt_3_2



ClarkeTransformationStruct abc2alphabeta(float VoltageA, float VoltageB, float VoltageC){
  ClarkeTransformationStruct output;

  output.Alpha = _2_div_3  * ( VoltageA  - 0.5 * (VoltageB + VoltageC) );
  output.Beta  = _sqrt_3_3 * ( VoltageB - VoltageC );
  return output;
}


/*
ClarkeTransformationStruct abc2alphabeta(float VoltageA, float VoltageB, float VoltageC){
	ClarkeTransformationStruct output;

	output.Alpha = (VoltageA * _2_div_3 ) - VoltageB/3 - VoltageC/3 ;
	output.Beta = _1_div_sqrt3 * (VoltageB - VoltageC );
	return output;
}
*/

