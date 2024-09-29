/*
 * ClarkeParkTransformation.h
 *
 *  Created on: Oct 15, 2023
 *      Author: pedro
 */

#ifndef INC_CLARKEPARKTRANSFORMATION_H_
#define INC_CLARKEPARKTRANSFORMATION_H_


typedef struct{
   float Alpha;
   float Beta;
}ClarkeTransformationStruct;




ClarkeTransformationStruct abc2alphabeta(float VoltageA, float VoltageB, float VoltageC);

#endif /* INC_CLARKEPARKTRANSFORMATION_H_ */
