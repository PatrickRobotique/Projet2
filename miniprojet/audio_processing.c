#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>

#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float oldtheta = 0;
static int mustsend=0;
static int speedL=0;
static int speedR=0;




#define MIN_VALUE_THRESHOLD	5000
#define SPEED	100

#define FREQ_ID	66	//1000Hz

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		150	//we don't analyze after this index to not use resources for nothing

/*
*	b
*	and to execute a motor command depending on it
*/
void direction(float theta, int quadrant){
//	if (quadrant){
//		if(theta>0){
//			speedR += SPEED;
//			speedL -= SPEED;
//		}
//		if(theta<0){
//			speedR -= SPEED;
//			speedL += SPEED;
//		}
//	}
//	else{
//		if(theta>0){
//			speedR += 2*SPEED;
//			speedL -= 2*SPEED;
//			}
//
//		if(theta<0){
//			speedR -= 2*SPEED;
//			speedL += 2*SPEED;
//		}
//	}
	if(abs(theta)>2.5){
		left_motor_set_speed(1000);
		right_motor_set_speed(1000);
		return;
	}
	if(abs(theta)<2.5){
	if(quadrant==1){
			speedR -= SPEED;
			speedL += SPEED;
	}
	if(quadrant==0){
				speedL -= SPEED;
				speedR += SPEED;
			}
	}

	if (speedL > 1000) {speedL = 1000;}
	if (speedR > 1000) {speedR = 1000;}
	if (speedL < 100) {speedL = 100;}
	if (speedR < 100) {speedR = 100;}
	if(abs(theta)<2.5){
		left_motor_set_speed(speedL);
		right_motor_set_speed(speedR);
	}
}
void sound_remote(float* mic1,float* mic2,float* mic3){
		float max_norm1 = MIN_VALUE_THRESHOLD;
		float max_norm2 = MIN_VALUE_THRESHOLD;
		float max_norm3 = MIN_VALUE_THRESHOLD;
		float L1=0;
		float L2=0;
		float L3=0;
		int quadrant;
		int16_t max_norm_index = FREQ_ID;
		//search for the highest peak
//		for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
//			if(mic1[i] > max_norm1){
//				max_norm1 = mic1[i];
//				max_norm_index = i;
//			}
//
//		}

		float theta = 0;

		max_norm1=mic1[max_norm_index];
//		max_norm2=mic2[max_norm_index];
//		max_norm3=mic3[max_norm_index];
		if(max_norm1>MIN_VALUE_THRESHOLD){
			max_norm1=((mic1[max_norm_index-1]+mic1[max_norm_index]+mic1[max_norm_index+1])/3);
			max_norm2=((mic2[max_norm_index-1]+mic2[max_norm_index]+mic2[max_norm_index+1])/3);
			max_norm3=((mic3[max_norm_index-1]+mic3[max_norm_index]+mic3[max_norm_index+1])/3);
			L1=max_norm1*max_norm1;
			L2=max_norm2*max_norm2;
			L3=max_norm3*max_norm3;

		//Angle entre robot et source dans un système polaire

		if((L1*L3)-(L2*L3)>0){
			quadrant=0;
		}
		else{
			quadrant=1;
		}
		float x = 0;
		float y = 0;
		x= (2*L1*L2)-(L2*L3)-(L1*L3);
		y= (L1*L3)-(L2*L3);
		theta = atan2f(y,x);
		//Print theta on realterm
		//chprintf((BaseSequentialStream *) &SDU1, "Theta = %f deg \n",theta);
		if(abs(theta-oldtheta)<0.3){

			float thetamoy = (oldtheta+theta)/2;
			oldtheta=theta;
			direction(thetamoy, quadrant);
		}
		else{
			oldtheta=(theta+oldtheta)/2;
		}


	}
	if(max_norm1<MIN_VALUE_THRESHOLD){
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	mustsend++;
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];
		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		nb_samples = 0;
		sound_remote(micRight_output,micLeft_output,micFront_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else{
		return NULL;
	}
}
