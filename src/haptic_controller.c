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

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define NOMINAL_TORQUE 0.021f
#define PADDLE_ID 16

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

#define N_SAMPLES 1000 //need to be a power of 2
#define WINDOW_SIZE 1000 //Moving average of the standard deviation
//#define ONE_OVER_WINDOW_SIZE 0.02f

#define A0 0.9989f
#define A1 -1.9857f
#define A2 0.9989f
#define B1 1.9854f
#define B2 -0.9975f
#define CALIB_SAMP 6000 //N sample to take for calibration

volatile uint32_t hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

volatile float32_t hapt_paddleAngle; // Paddle angle [deg].
volatile float32_t hapt_paddleSpeed; // Paddle speed [deg/s].
volatile float32_t hapt_paddleAccel; // Paddle acceleration [deg/s^2].
volatile float32_t hapt_paddlePrevPos; // Previous paddle angle [deg].
volatile float32_t hapt_paddleSpeedFilt; // Filtered paddle speed [deg/s].

volatile float32_t hapt_paddleSetAngle;

volatile float32_t filtered_value ;

volatile uint32_t hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_paddleAngle; // Paddle angle [deg].
volatile float32_t hapt_paddleSpeed; // Paddle speed [deg/s].
volatile float32_t hapt_paddleAccel; // Paddle acceleration [deg/s^2].
volatile float32_t hapt_paddlePrevPos; // Previous paddle angle [deg].
volatile float32_t hapt_paddlePrevSpeed; // Previous paddle speed [deg/s].
volatile float32_t hapt_paddleAngleFilt; // Filtered paddle angle [deg].
volatile float32_t hapt_paddleSpeedFilt; // Filtered paddle speed [deg/s].
volatile float32_t hapt_paddleAccelFilt; // Filtered paddle acceleration [deg/s].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

volatile float32_t filterCutoffFreqAngle = 10.0f; // Cutoff frequency for the position filter [Hz].
const float32_t filterCutoffFreqSpeed = 200.0f; // Cutoff frequency for the speed filter [Hz].
const float32_t filterCutoffFreqAccel = 10.0f; // Cutoff frequency for the acceleration filter [Hz].

volatile float32_t previousTmp = 0; // Cutoff frequency for the acceleration filter [Hz].

volatile float32_t PID_kp = 0.015;	//0.01		0.022
volatile float32_t PID_ki = 0.15;	//0.003		0.015
volatile float32_t PID_kd = 0.0006; //0.0008	0.00055

volatile float32_t PID_tau_i = 0.1;

volatile float32_t PID_errorPos;
volatile float32_t PID_errorPosPrev;
volatile float32_t PID_integrator;
volatile float32_t PID_derivative;

volatile bool positionControl;

volatile bool wall_on;
volatile bool hapt_compensation_on;
volatile float32_t wall_K;
volatile float32_t wall_B;

volatile bool std_on;
volatile bool notch_filter;

volatile float32_t muscle_value[N_SAMPLES];
volatile float32_t muscle_value_filt;
volatile float32_t muscleOutput;

volatile float32_t filtered_value_Prev;
volatile float32_t filtered_value_Prev_Prev;
volatile float32_t actual_value;

volatile float32_t amplifier_value =0;
volatile float32_t muscle_value_filt_Prev;
volatile float32_t muscle_value_filt_Prev_Prev;


volatile float32_t sum_value;
volatile float32_t std_dev_value;

volatile float32_t energy_fft;


volatile float32_t energy_std;
volatile uint32_t iteration;
volatile bool calib_rest=0;
volatile bool calib_contract=0;
volatile float64_t calib_sum = 0.0f;
volatile int32_t j_calib=0;

volatile float32_t rms_rest = 0.007f; // Value for Maxime, can be calibrated using the built in feature
volatile float32_t rms_contract = 0.15f;

void hapt_Update(void);

float32_t hapt_LowPassFilter(float32_t previousFilteredValue,
							 float32_t input, float32_t dt,
							 float32_t cutoffFreq);

float32_t hapt_HighPassFilter(float32_t previousFilteredValue, float32_t input, float32_t previousInput,
							 float32_t dt, float32_t cutoffFreq);
float32_t remapF(float32_t x, float32_t in_min, float32_t in_max, float32_t out_min, float32_t out_max, bool sat);

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

	wall_K = 0.005f;
	wall_B = 0.0f;

	std_on = 0;
	notch_filter = 1;

	energy_fft = 0;

	iteration = 0;
	actual_value = 0.0f;

	sum_value = 0.0f;
	std_dev_value = 0.0f;
	muscle_value_filt = 0.0f;
	filtered_value_Prev = 0.0f;
	filtered_value_Prev_Prev = 0.0f;

    filtered_value = 0.0f;
	//PID
    hapt_motorTorque = 0.0f;
    hapt_paddlePrevPos = 0.0f;
    hapt_paddlePrevSpeed = 0.0f;
    hapt_paddleSpeedFilt = 0.0f;
    hapt_paddleAccelFilt = 0.0f;

    //hapt_PID_on = 0;
    //hapt_HallSensor =  0;
    hapt_paddleSetAngle = 0.0f;

    PID_errorPos = 0.0f;
    PID_errorPosPrev = 0.0f;
    PID_integrator = 0.0f;
    PID_derivative = 0.0f;
    positionControl = 0;
    calib_rest = 0;
    calib_contract = 0;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod, cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorBool("Compensation", (bool*)&hapt_compensation_on, READWRITE);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);
    comm_monitorFloat("paddle_pos [deg]", (float32_t*)&hapt_paddleAngle, READONLY);
    comm_monitorFloat("paddle_speed_filt [deg/s]", (float32_t*)&hapt_paddleSpeedFilt, READONLY);
    comm_monitorBool("Wall", (bool*)&wall_on, READWRITE);
    comm_monitorFloat("Virtual Spring", (float32_t*)&wall_K, READWRITE);
    comm_monitorFloat("Virtual Damping", (float32_t*)&wall_B, READWRITE);
    comm_monitorFloat("Cuttoff Frequnecy Speed", (float32_t*)&filterCutoffFreqSpeed, READWRITE);
    comm_monitorFloat("Muscle value", (float32_t*)&muscle_value[iteration], READONLY);
    comm_monitorUint32("iteration", (uint32_t*)&iteration, READONLY);
    comm_monitorFloat("Actual value", (float32_t*)&actual_value, READONLY);
    comm_monitorBool("Notch filter on", (bool*)&notch_filter, READWRITE);
    comm_monitorFloat("paddle_set_pos [deg]", (float32_t*)&hapt_paddleSetAngle, READWRITE);
    comm_monitorBool("Position control", (bool*)&positionControl, READWRITE);
    comm_monitorBool("amplifier_value", (bool*)&amplifier_value, READWRITE);
    comm_monitorBool("muscle_value_filt", (float32_t*)&muscle_value_filt, READWRITE);
    comm_monitorFloat("filtered_value", (float32_t*)&filtered_value, READONLY);
    comm_monitorFloat("Energy FFT", (float32_t*)&energy_fft, READONLY);
    comm_monitorFloat("muscleOutput", (float32_t*)&muscleOutput, READONLY);
    comm_monitorBool("calib_rest", (float32_t*)&calib_rest, READWRITE);
    comm_monitorBool("calib_contract", (float32_t*)&calib_contract, READWRITE);
    comm_monitorFloat("rms_rest", (float32_t*)&rms_rest, READONLY);
    comm_monitorFloat("rms_contract", (float32_t*)&rms_contract, READONLY);
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

	// _________RMS computation___________
    iteration = ((iteration + 1) % N_SAMPLES);
	float32_t mean = 0.0f;
    float32_t tmp = hapt_LowPassFilter(muscle_value_filt, amplifier_value,
                                        dt, 500);

    muscle_value_filt = hapt_HighPassFilter(muscle_value_filt, tmp, previousTmp,
                                            dt, 10);
    previousTmp = tmp;

    //implement notch filter
    if (notch_filter == 1){
        filtered_value = A0 * muscle_value_filt + A1 * muscle_value_filt_Prev + A2 * muscle_value_filt_Prev_Prev + \
                         B1 * filtered_value_Prev + B2 * filtered_value_Prev_Prev;

        filtered_value_Prev = filtered_value;
        filtered_value_Prev_Prev = filtered_value_Prev;

        muscle_value_filt_Prev = muscle_value_filt;
        muscle_value_filt_Prev_Prev = muscle_value_filt_Prev;

    } else {
        filtered_value = muscle_value_filt;
    }

    muscle_value[iteration] = pow(filtered_value, 2);
    actual_value = muscle_value[iteration];

    //add last value and remove value (WINDOW_SIZE + 1) before
    sum_value += muscle_value[iteration];
    sum_value -= muscle_value[(iteration - (WINDOW_SIZE + 1))% N_SAMPLES];
    mean = sum_value / WINDOW_SIZE;
    energy_fft = pow(mean, 0.5); //sqrt
    //do some more processing on it
    muscleOutput = hapt_LowPassFilter( muscleOutput,
                                       energy_fft, dt,
                                              3);

    //// calibration for each user
    if (calib_rest){
        calib_sum += muscleOutput;
        j_calib+=1;
        if (j_calib==CALIB_SAMP){
            //compute mean
            rms_rest = calib_sum/CALIB_SAMP;
            //clean
            j_calib=0;
            calib_sum=0.0f;
            calib_rest =0;
        }
    }
    if (calib_contract){
        calib_sum += muscleOutput;
        j_calib+=1;
        if (j_calib==CALIB_SAMP){
            //compute mean
            rms_contract = calib_sum/CALIB_SAMP;
            //clean
            j_calib=0;
            calib_sum=0.0f;
            calib_contract =0;
        }
    }

	//// Compute the motor torque compensation, and apply it.
	if (hapt_compensation_on ==1){
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

	if (positionControl){

        hapt_paddleSetAngle = remapF(muscleOutput, rms_rest, rms_contract, -25.0f, +25.0f, true);

	    /// filtering
        hapt_paddleAngleFilt = hapt_LowPassFilter(hapt_paddleAngleFilt,
                                                  hapt_paddleAngle, dt,
                                                  filterCutoffFreqAngle);

        hapt_paddleSpeed = (hapt_paddleAngleFilt - hapt_paddlePrevPos) / dt;
        hapt_paddleSpeedFilt = hapt_LowPassFilter(hapt_paddleSpeedFilt,
                                                  hapt_paddleSpeed, dt,
                                                  filterCutoffFreqSpeed);

        hapt_paddleAccel = (hapt_paddleSpeedFilt - hapt_paddlePrevSpeed) / dt;


        hapt_motorTorque = 0.0f ;
        //gravity
        hapt_motorTorque += MGL * (float32_t)sin(hapt_paddleAngle * PI/180.f ) / REDUCTION_RATIO;

        // compensate for viscous friction
        hapt_motorTorque +=  copysign(1.0, hapt_paddleSpeedFilt) * (VISCOUS_FRICTION * abs(hapt_paddleSpeedFilt) + DRY_FRICTION_NEG)/ REDUCTION_RATIO;


        //PID1
        PID_errorPos = (hapt_paddleAngleFilt - hapt_paddleSetAngle);

        PID_integrator = PID_integrator + dt *PID_errorPos;

        utils_SaturateF(&PID_integrator, -0.3, 0.3); // prevent the integrator from exploding

        float32_t PID_derror = PID_errorPos - PID_errorPosPrev;

        PID_errorPosPrev = PID_errorPos;

        PID_derivative = PID_derror/dt;

        float32_t hapt_motorTorqueUnsat = -1*(PID_kp * PID_errorPos + PID_ki * PID_integrator + PID_kd * PID_derivative);
        //saturation of the motor torque
        if (hapt_motorTorqueUnsat > NOMINAL_TORQUE){
            hapt_motorTorque = NOMINAL_TORQUE;
        } else if (hapt_motorTorqueUnsat < -1*NOMINAL_TORQUE){
            hapt_motorTorque = -1*NOMINAL_TORQUE;
        } else {
            hapt_motorTorque = hapt_motorTorqueUnsat;
        }

        //anti-windup for integrator
        if (PID_ki != 0) {
            PID_integrator = PID_integrator + dt/PID_ki * (hapt_motorTorque - hapt_motorTorqueUnsat);
        }
        else{
            hapt_motorTorque=0.0f;
        }
	}

	torq_SetTorque(hapt_motorTorque);
	hapt_paddlePrevPos = hapt_paddleAngle;
    hapt_paddlePrevSpeed = hapt_paddleSpeed;

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
/**
* @brief Filters a signal with a first-order low-pass filter.
* @param previousFilteredValue the previous filtered value.
* @param input the filter input (the current sample of the signal to filter).
* @param dt the time elapsed since the last call of this function [s].
* @param cutoffFreq the cutoff frequency of the filter [Hz].
* @return the new output of the filter.
*/
float32_t hapt_HighPassFilter(float32_t previousFilteredValue, float32_t input,  float32_t previousInput,
							 float32_t dt, float32_t cutoffFreq)
{
	float32_t tau = 1.0f / (2.0f * PI * cutoffFreq); // Rise time (time to reach 63% of the steady-state value).
	float32_t alpha = tau / (tau+dt); // Smoothing factor.
	return alpha * previousFilteredValue + alpha * (input - previousInput);
}

float32_t remapF(float32_t x, float32_t in_min, float32_t in_max, float32_t out_min, float32_t out_max, bool sat) {
//// remap from one range of values to another
  float32_t temp =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (sat) {
      if (temp > out_max )
          temp = out_max;

      if (temp < out_min )
          temp = out_min ;

  }
  return temp;
}


