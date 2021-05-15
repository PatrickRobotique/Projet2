#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <direction.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
//Arrays containing the last values of amplitude at 1000 Hz
static float oldamplitude1[5]={0,0,0,0,0};
static float oldamplitude2[5]={0,0,0,0,0};
static float oldamplitude3[5]={0,0,0,0,0};
static float sum_error=0;
//Calibration values computed at each reset of the e-puck
static float cali1=0;
static float cali2=0;
static float cali3=0;
//Array containing the last values of theta
static uint8_t oldtheta[5]={0,0,0,0,0};
static uint8_t calibrated=0;


#define MIN_VALUE_THRESHOLD	20000// minimal value for the amplitude
#define FREQ_ID	64	//1000Hz in the FFT tab
#define TRUE 1
#define FALSE 0
#define DIVSUM 25
// Values for the PI controller and the speed settings
#define MAX_SUM_ERROR	2000
#define ROTATION_THRESHOLD 3
#define ROTATION_COEFF 15
#define ERROR_THRESHOLD 10
#define GOAL 500
#define HALFWAY 250
#define KP 2
#define KI 0.1



void saveolddata(void);


void audio_calibration(void);


	/*
	 * PI_regulator that uses the sound amplitude on one
	 * microphone and a theoretical "goal" amplitude.
	 */
int16_t pi_regulator(void){

	int16_t error = 0;
		int16_t speed = 0;
		uint16_t goal = GOAL;
		/*The distance is proportional to the intensity of the sound,
		 *which is in 1/r^2, so to have a linear decrement of speed
		 *while approaching the GOAL, we take the square root of intensity.
	 	*/
		uint16_t distance = (int16_t)sqrt(oldamplitude1[0]*cali1);
		error = goal-distance;

		/*If the e-puck is at the GOAL distance from the source of sound,
		*the speed will be set to zero, so it won't oscillate.
		*/
		if(abs(error) < ERROR_THRESHOLD){
			return 0;
		}

		sum_error += error-HALFWAY;

		//To avoid infinite growth of the error in some cases, we set a maximum
		if(sum_error > MAX_SUM_ERROR){
			sum_error = MAX_SUM_ERROR;
		}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
		}
		//
		speed = KP * error + KI*sum_error;
		//Speed can't be too big, due to the time between 2 updates of theta,

		if (speed > 400){
			speed = 400;
		}
		if (speed < -400){
			speed = -400;
				}
	    return (int16_t)speed;
}
	/*
	* This function uses the amplitudes at 1000 Hz from the 3 microphones
	* to compute an angle in polar coordinates between the source of
	* sound and the center of the circle formed by the three microphones.
	*/
void sound_remote(void){

	int speedR =0;
	int speedL =0;
	uint8_t max_norm_index = FREQ_ID;
	/*
	* If there is no sound, the motors are set to 0 and we leave
	* this function and this thread.
	*/
	if(micRight_output[max_norm_index]<MIN_VALUE_THRESHOLD ){
		get_speed_audio(speedL, speedR);
		chThdSleepMilliseconds(50);
		return;
	}
	/*
	* The last amplitude is stored, and if there has already been
	* a calibration, we continue. Or else we leave this function
	*/
	saveolddata();
	if(!calibrated){
		chThdSleepMilliseconds(50);
		return;
	}

	float amplitude1 = 0,amplitude2 = 0,amplitude3 = 0;
	//Normalizes the values stored by the microphone in each tab to prevent
	//big gaps between theta values.
	for(uint8_t i=0; i<5;i++){
		amplitude1+=(oldamplitude1[i]*cali1);
		amplitude2+=(oldamplitude2[i]*cali2);
		amplitude3+=(oldamplitude3[i]*cali3);
	}
	/*The intensity is directly proportionnal to the amplitude squared,
	* divided by 25 because we summed 5 amplitudes for each microphone.
	* In order to save space, the variables will keep the name "amplitudeX",
	* but they are actually intensities.
	*/
	amplitude1=amplitude1*amplitude1/DIVSUM;
	amplitude2=amplitude2*amplitude2/DIVSUM;
	amplitude3=amplitude3*amplitude3/DIVSUM;

	/*
	* Angle between robot and source in polar coordinates.
	* This angle comes from the paper:"Mutual accoustic identification
	* in the swarm of e-puck robots".
	* atan2(y,x) gives the arctangent of y/x as well as the quadrant, so
	* we have different values according to the sign of x.
	* The way intensities and y/x are put together is given in the paper.
	* Everything is explained in details in the "rapport".
	*/

	float x= (2*amplitude1*amplitude2)-(amplitude2*amplitude3)-(amplitude1*amplitude3);
	float y= (amplitude1*amplitude3)-(amplitude2*amplitude3);
	float vrai_theta = 0;
	vrai_theta = atan2f(y,x);
	vrai_theta=vrai_theta*10;
	int8_t theta = (int8_t)vrai_theta;
	_Bool quadrant=FALSE;
	// Theta negative implies that y/x is on the left quadrant
	if(theta<0){
			quadrant=TRUE;
			theta=-theta;
	}
	//The value of theta is stored, so we can normalize it with old values
	oldtheta[0]=theta;
	uint8_t sum_theta=0;
		for(uint8_t i=0; i<5;i++){
			sum_theta+=oldtheta[i];
		}
		theta=sum_theta/5;
	if(quadrant){
		theta=-theta;
	}
	// The speed is given by the pi_regulator here
	int16_t speed=pi_regulator();

	/*To prevent the oscillation of the robot, the value of theta is set to 0 below
	 * a fixed threshold and the rotation_coeff is not taken into account for the
	 * speed.
	 */
	if(abs(theta) < ROTATION_THRESHOLD){
		theta = 0;
	}
	//applies the speed from the PI regulator and the correction for the rotation
	speedR = (speed - ROTATION_COEFF * theta);
	speedL = (speed + ROTATION_COEFF * theta);
	// Stores the speed in "direction.c"
	get_speed_audio(speedL, speedR);
	chThdSleepMilliseconds(45);
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
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;


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
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		nb_samples = 0;
		sound_remote();
		if(!calibrated){
			audio_calibration();
		}
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
			return micBack_cmplx_input;
		}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}

	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
/*
 * Simple function that stores the value of each amplitude for further use
 */
void saveolddata(void){
	uint8_t i;
	//Push the old amplitude one step back, to let the first spot for the new one
	for(i=4; i>=1;i--){
		oldamplitude1[i]=oldamplitude1[i-1];
		oldamplitude2[i]=oldamplitude2[i-1];
		oldamplitude3[i]=oldamplitude3[i-1];
		oldtheta[i]=oldtheta[i-1];
	}
	// Saves the last amplitude given by the mics.
	oldamplitude1[0]=micRight_output[FREQ_ID];
	oldamplitude2[0]=micLeft_output[FREQ_ID];
	oldamplitude3[0]=micBack_output[FREQ_ID];
}
/*
 * This function is run while the source of sound is equidistant
 * from each microphone, we suppose that its amplitude at 1000 Hz for each microphone
 * is the same.
 */
void audio_calibration(void){
	// Only processes calibration if we have 5 samples for each microphone.
	if(!oldamplitude1[4]){

		chThdSleepMilliseconds(50);

		return;
	}
	float amplitude1=0;
	float amplitude2=0;
	float amplitude3=0;
	//5 samples for each microphone are summed
	for(uint8_t i=0; i<5;i++){
		amplitude1+=(oldamplitude1[i]);
		amplitude2+=(oldamplitude2[i]);
		amplitude3+=(oldamplitude3[i]);
		}
	/*
	* In order to calibrate, we calculate the mean of the sum of the three amplitudes.
	* As we know the amplitude at each microphone, we can compute the ratio between
	* each microphone amplitude and the mean.
	* Then we set cali1,2,3 to be the three calibration multipliers.
	*/
	float amplitude_moyenne= (amplitude1+amplitude2+amplitude3)/3;
	/*
	* Cali1,2 and 3 will be used for calibration until next reset
	* from the epuck2, which sets all values to 0. Thus they will be recalculated.
	*/
	cali1=amplitude_moyenne/amplitude1;
	cali2=amplitude_moyenne/amplitude2;
	cali3=amplitude_moyenne/amplitude3;
	// This function is run only once at each reset.
	calibrated=TRUE;

}
