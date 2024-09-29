#include "ClarkeParkTransformation.h"


float _1_div_sqrt3 = 0.57735026919;
float _2_div_3 = 0.6666666;

ClarkeTransformationStruct abc2alphabeta(float VoltageA, float VoltageB, float VoltageC){
	ClarkeTransformationStruct output;

	output.Alpha = (VoltageA * _2_div_3 ) - VoltageB/3 - VoltageC/3 ;
	output.Beta = _1_div_sqrt3 * (VoltageB - VoltageC );
	return output;
}
