/*
 * controller.h
 *
 *  Created on: Oct 22, 2023
 *      Author: pedro
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_



typedef struct {
	//Controller gains
	float Kp;
	float Ki;

	//output limits
	float limMin;
	float limMax;

	//sample time (in seconds)
	float T;

	float prevError;

	float Phase;

	float Frequency;
	float prevFrequency;
	//controller output

}PIController;

typedef struct {
	//Data Input
	float cutOffFrequency;
	float SampleRate;

	//Calculated Data
	float alpha;
	float prevOutput;
	float output;
}Filter;

typedef struct {
    // Coefficients for the filter
    double a1; // Coefficient x[n]
    double a2; // Coefficient x[n-1]
    double a3; // Coefficient x[n-2]
    double b1; // Coefficient y[n-1]
    double b2; // Coefficient y[n-2]

    double x1; // Previous input value     x[n-1]
    double x2; // Input value before last  x[n-2]
    double y1; // Previous output value    y[n-1]
    double y2; // Output value before last y[n-2]

    // Sampling and cutoff frequencies
    double SampleRate; // Sampling rate of the system
    double cutOffFrequency; // Cutoff frequency of the low-pass filter
} Filter2ndOrder;




//Controller functions
void PIDController_Init(PIController *pid);
float PIDController_Update(PIController *pid, float measurement);
float integrator(PIController *pid);
float AlphaBetaCalculation(float alpha, float beta, float phase);


//Auxiliary functions
float Deg2Rad(float degree);
float Sine(float phase);
double Cossine(float phase);

//Filter functions
void LowPassFilter_init(Filter *filt);
double lowPassFilter(double input, Filter *filt);

void LowPassFilter2ndOrder_init(Filter2ndOrder *filt);
double lowPassFilter2ndOrder(double input, Filter2ndOrder *filt);


#endif /* INC_CONTROLLER_H_ */
