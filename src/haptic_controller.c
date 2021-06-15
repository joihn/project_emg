/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "lib/circular_buffer.h"
#include "torque_regulator.h"
#include "math.h"
#include "stdlib.h"
//#include "../CMSIS/core/arm_math.h"

// for FFT functions
#include "stdio.h"
#include "complex.h"
typedef double complex cplx;

// for FFT functions

#define PADDLE_ID 16


#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define NOMINAL_TORQUE 0.021f

#if (PADDLE_ID == 16)
	#define DRY_FRICTION_POS 0.0007f
	#define DRY_FRICTION_NEG 0.0009f
#elif (PADDLE_ID == 14)
	#define DRY_FRICTION_POS 0.0041f/5.0f
	#define DRY_FRICTION_NEG 0.0042f/5.0f
#endif

#define VISCOUS_FRICTION 2.9e-6f /15.0f   // from our calc
#define MGL 0.048f*9.81f*0.0251f

#define D2R PI/180.f
#define WALL_ANGLE_POS 15.f
#define WALL_ANGLE_NEG -15.f

#define N_SAMPLES 128 //need to be a power of 2
#define WINDOW_SIZE 50 //Moving average of the standard deviation
#define ONE_OVER_WINDOW_SIZE 0.02f

#define A0 0.9989f
#define A1 -1.9857f
#define A2 0.9989f
#define B1 1.9854f
#define B2 -0.9975f

volatile uint32_t hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

volatile float32_t hapt_paddleAngle; // Paddle angle [deg].
volatile float32_t hapt_paddleSpeed; // Paddle speed [deg/s].
volatile float32_t hapt_paddleAccel; // Paddle acceleration [deg/s^2].
volatile float32_t hapt_paddlePrevPos; // Previous paddle angle [deg].
volatile float32_t hapt_paddleSpeedFilt; // Filtered paddle speed [deg/s].

const float32_t filterCutoffFreqSpeed = 200.0f; // Cutoff frequency for the speed filter [Hz].
volatile bool wall_on;
volatile bool hapt_compensation_on;
volatile float32_t wall_K;
volatile float32_t wall_B;

volatile bool fft_on;
volatile bool std_on;
volatile bool notch_filter;

volatile float32_t muscle_value[N_SAMPLES];
volatile float32_t muscle_value_filt;
volatile float32_t filtered_value_Prev;
volatile float32_t filtered_value_Prev_Prev;
volatile float32_t actual_value;

volatile float32_t amplifier_value;
volatile float32_t amplifier_value_Prev;
volatile float32_t amplifier_value_Prev_Prev;


volatile float32_t sum_value;
volatile float32_t std_dev_value;

volatile float32_t energy_fft;
volatile float32_t energy_std;
volatile uint32_t iteration;


void hapt_Update(void);

float32_t hapt_LowPassFilter(float32_t previousFilteredValue,
							 float32_t input, float32_t dt,
							 float32_t cutoffFreq);

float32_t hapt_HighPassFilter(float32_t previousFilteredValue, float32_t input,
							 float32_t dt, float32_t cutoffFreq);

void _fft(cplx buf[], cplx out[], uint32_t n, uint32_t step);
void fft(cplx buf[], uint32_t n);


/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{

	hapt_timestamp = 0;
	hapt_motorTorque = 0.0f;

	hapt_paddlePrevPos = 0.0f;
	hapt_paddleSpeedFilt = 0.0f;

	hapt_compensation_on = 0;
	wall_on = 0;

	wall_K = 0.005f; //go down to 0.00005 when no muscle force detected 	//0.01f;
	wall_B = 0.0f; 															//0.003f;

	fft_on = 0;
	std_on = 0;
	notch_filter = 0;

	energy_fft = 0;
	iteration = 0;
	actual_value = 0.0f;

	sum_value = 0.0f;
	std_dev_value = 0.0f;
	muscle_value_filt = 0.0f;
	filtered_value_Prev = 0.0f;
	filtered_value_Prev_Prev = 0.0f;



    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod, cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorBool("Compensation", (bool*)&hapt_compensation_on, READWRITE);
//    comm_monitorFloat("paddle_speed [deg/s]", (float32_t*)&hapt_paddleSpeed, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);
    comm_monitorFloat("paddle_pos [deg]", (float32_t*)&hapt_paddleAngle, READONLY);
    comm_monitorFloat("paddle_speed_filt [deg/s]", (float32_t*)&hapt_paddleSpeedFilt, READONLY);
//    comm_monitorFloat("paddle_accel_filt [deg/s^2]", (float32_t*)&hapt_paddleAccelFilt, READONLY);

    comm_monitorBool("Wall", (bool*)&wall_on, READWRITE);
    comm_monitorFloat("Virtual Spring", (float32_t*)&wall_K, READWRITE);
    comm_monitorFloat("Virtual Damping", (float32_t*)&wall_B, READWRITE);
    comm_monitorFloat("Cuttoff Frequnecy Speed", (float32_t*)&filterCutoffFreqSpeed, READWRITE);
    comm_monitorFloat("Muscle value", (float32_t*)&muscle_value[iteration], READONLY);
    comm_monitorUint32("iteration", (uint32_t*)&iteration, READONLY);
    comm_monitorFloat("Actual value", (float32_t*)&actual_value, READONLY);
    comm_monitorBool("FFT on", (bool*)&fft_on, READWRITE);
    comm_monitorFloat("Energy FFT", (float32_t*)&energy_fft, READONLY);
    comm_monitorBool("Notch filter on", (bool*)&notch_filter, READWRITE);

}



/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update()
{


	// Get the value of the amplifier
	amplifier_value = adc_GetChannelVoltage(ADC_CHANNEL_6);

	// Compute the dt (uncomment if you need it).
	float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

	// Increment the timestamp.
	hapt_timestamp += cbt_GetHapticControllerPeriod();

	// Get the Hall sensor voltage.
	hapt_hallVoltage = hall_GetVoltage();
	hapt_paddleAngle = enc_GetPosition() / REDUCTION_RATIO;

	// Compute the speed and acceleration from the Hall paddle angle.
	// Filter the speed and acceleration.
	hapt_paddleSpeed = (hapt_paddleAngle - hapt_paddlePrevPos) / dt;
	hapt_paddleSpeedFilt = hapt_LowPassFilter(hapt_paddleSpeedFilt,
											  hapt_paddleSpeed, dt,
											  filterCutoffFreqSpeed);

//	memmove(&muscle_value[0], &muscle_value[1], (N_SAMPLES - 1) * sizeof(muscle_value[0]));
//	muscle_value[N_SAMPLES - 1] = hapt_hallVoltage * (-25.78f) + 63.39f;


	iteration = ((iteration + 1) % N_SAMPLES);


	if (fft_on == 0){
		actual_value = amplifier_value;

	}


	if (fft_on == 1){
		float32_t mean = 0.0f;
		float32_t filtered_value = 0.0f;
		float32_t tmp = hapt_LowPassFilter(muscle_value_filt, amplifier_value,
											dt, 500);

		muscle_value_filt = hapt_HighPassFilter(muscle_value_filt, tmp,
												dt, 10);


		//implement notch filter
		if (notch_filter == 1){
			filtered_value = A0 * amplifier_value + A1 * amplifier_value_Prev + A2 * amplifier_value_Prev_Prev + \
							 B1 * filtered_value_Prev + B2 * filtered_value_Prev_Prev;

			filtered_value_Prev = filtered_value;
			filtered_value_Prev_Prev = filtered_value_Prev;

			amplifier_value_Prev = amplifier_value;
			amplifier_value_Prev_Prev = amplifier_value_Prev;

		} else {
			filtered_value = muscle_value_filt;
		}

		muscle_value[iteration] = pow(filtered_value, 2);
		actual_value = muscle_value[iteration];



		//add last value and remove value (WINDOW_SIZE + 1) before
		sum_value += muscle_value[iteration];
		sum_value -= muscle_value[(iteration - (WINDOW_SIZE + 1))% N_SAMPLES];
		mean = sum_value * ONE_OVER_WINDOW_SIZE;
		energy_fft = pow(mean, 0.5);







	}


	// Compute the motor torque compensation, and apply it.
	if (hapt_compensation_on == 1){
		hapt_motorTorque=0;
		//compensate for gravity
		hapt_motorTorque += MGL * (float32_t)sin(hapt_paddleAngle * PI/180.f ) / REDUCTION_RATIO;

		//compensate for viscous friction
		hapt_motorTorque +=  (VISCOUS_FRICTION * hapt_paddleSpeedFilt); // / REDUCTION_RATIO;

		//compensate for dry friction
		if (hapt_paddleSpeedFilt > 0){
			hapt_motorTorque +=  DRY_FRICTION_POS ;// / REDUCTION_RATIO;
		} else if (hapt_paddleSpeedFilt < 0){
			hapt_motorTorque +=  -1*DRY_FRICTION_NEG ;// / REDUCTION_RATIO;
		}
	}

	// apply a virtual wall
	if (wall_on == 1){
		if(hapt_paddleAngle > WALL_ANGLE_POS){
			hapt_motorTorque += wall_K*(WALL_ANGLE_POS - hapt_paddleAngle) - wall_B*hapt_paddleSpeedFilt;
		} else if(hapt_paddleAngle < WALL_ANGLE_NEG){
			hapt_motorTorque += wall_K*(WALL_ANGLE_NEG - hapt_paddleAngle) - wall_B*hapt_paddleSpeedFilt;
		}

	}

	// saturation
	if (hapt_motorTorque > NOMINAL_TORQUE){
		hapt_motorTorque = NOMINAL_TORQUE;
	} else if (hapt_motorTorque < -1*NOMINAL_TORQUE){
		hapt_motorTorque = -1*NOMINAL_TORQUE;
	}


	// No torque applied
	if (hapt_compensation_on == 0  && wall_on == 0){
		hapt_motorTorque = 0.0f;
	}
	hapt_motorTorque = 0.0f;

	torq_SetTorque(hapt_motorTorque);
	hapt_paddlePrevPos = hapt_paddleAngle;

}

/**
* @brief Filters a signal with a first-order low-pass filter.
* @param previousFilteredValue the previous filtered value.
* @param input the filter input (the current sample of the signal to filter).
* @param dt the time elapsed since the last call of this function [s].
* @param cutoffFreq the cutoff frequency of the filter [Hz].
* @return the new output of the filter.
*/

float32_t hapt_LowPassFilter(float32_t previousFilteredValue, float32_t input,
							 float32_t dt, float32_t cutoffFreq)
{
	float32_t tau = 1.0f / (2.0f * PI * cutoffFreq); // Rise time (time to reach 63% of the steady-state value).
	float32_t alpha = dt / (tau+dt); // Smoothing factor.
	return alpha * input + (1.0f - alpha) * previousFilteredValue;
}

float32_t hapt_HighPassFilter(float32_t previousFilteredValue, float32_t input,
							 float32_t dt, float32_t cutoffFreq)
{
	float32_t tau = 1.0f / (2.0f * PI * cutoffFreq); // Rise time (time to reach 63% of the steady-state value).
	float32_t alpha = tau / (tau+dt); // Smoothing factor.
	return alpha * input + (1.0f - alpha) * previousFilteredValue;
}



void _fft(cplx buf[], cplx out[], uint32_t n, uint32_t step)
{
	if (step < n) {
		_fft(out, buf, n, step * 2);
		_fft(out + step, buf + step, n, step * 2);

		for (int i = 0; i < n; i += 2 * step) {
			cplx t = cexp(-I * PI * i / n) * out[i + step];
			buf[i / 2]     = out[i] + t;
			buf[(i + n)/2] = out[i] - t;
		}
	}
}

void fft(cplx buf[], uint32_t n)
{
	cplx out[n];
	for (uint32_t i = 0; i < n; i++) out[i] = buf[i];

	_fft(buf, out, n, 1);

}




