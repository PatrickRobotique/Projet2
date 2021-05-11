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

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
static float oldintensity1[5]={0,0,0,0,0};
static float oldintensity2[5]={0,0,0,0,0};
static float oldintensity3[5]={0,0,0,0,0};
static float oldtheta[5]={0,0,0,0,0};
static float sum_error=0;
static int mustsend=0;
static float kp=0.005;
static float ki=0.005;
static float weights[5]={0.20,0.2,0.2,0.2,0.2};




#define MIN_VALUE_THRESHOLD	10000// minimal value for the amplitude, if it's lower, we leave the thread

#define FREQ_ID	65	//1000Hz in the FFT tab

// Values for the PI controller and the speed setting
#define MAX_SUM_ERROR	100000
#define ROTATION_THRESHOLD 0.05
#define ROTATION_COEFF 150
#define ERROR_THRESHOLD 2000
/*
*	b
*	and to execute a motor command depending on it
*/














void saveolddata(void);


//simple PI regulator implementation
int16_t pi_regulator(void){

	float error = 0;
	float speed = 0;
	float goal = 300000;
	float distance = oldintensity1[0];
	error = goal-distance;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error-50000;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = kp * error +ki*sum_error;
	if (speed > 1000) {speed = 1000;}
    return (int16_t)speed;
}
void sound_remote(void){

	int speedR =0;
	int speedL =0;
	saveolddata();
	uint8_t max_norm_index = FREQ_ID;
	if(micRight_output[max_norm_index]<MIN_VALUE_THRESHOLD){
		get_speed_audio(speedL, speedR);

		chThdSleepMilliseconds(100);

		return;
	}
	float intensity1 = 0,intensity2 = 0,intensity3 = 0,theta = 0;

	for(uint8_t i=0; i<5;i++){//Weight the values the mic stored in each tab to prevent big gaps in theta values
		intensity1+=(oldintensity1[i]);
		intensity2+=(oldintensity2[i]);
		intensity3+=(oldintensity3[i]);
	}

	float intensity1norm=3*intensity1/(intensity1+intensity2+intensity3);//Intensity is proportional to the amplitude squared
	float intensity2norm=3*intensity2/(intensity1+intensity2+intensity3);
	float intensity3norm=3*intensity3/(intensity1+intensity2+intensity3);
	intensity1=intensity1norm;
	intensity2=intensity2norm;
	intensity3=intensity3norm;
		//Angle between robot and source in polar coordinates

	float x = 0;
	float y = 0;
	x= (2*intensity1*intensity2)-(intensity2*intensity3)-(intensity1*intensity3);
	y= (intensity1*intensity3)-(intensity2*intensity3);
	theta = atan2f(y,x);

	mustsend++;
	if(mustsend%5==0){
		mustsend=0;
	}
	uint8_t quadrant=0;
	if(theta<0){
		quadrant=1;
		theta=-theta;
		oldtheta[0]=theta;
	}
	oldtheta[0]=theta;
	for(uint8_t i=1; i<5;i++){
	theta+=oldtheta[i];
	}
	theta=theta/5;
	if(quadrant){
		theta=-theta;
	}
	int16_t speed=pi_regulator();


		//if the line is nearly in front of the camera, don't rotate
	if(fabs(theta) < ROTATION_THRESHOLD){
		theta = 0;
	}
		//applies the speed from the PI regulator and the correction for the rotation
	speedR = (speed - ROTATION_COEFF * theta);
	speedL = (speed + ROTATION_COEFF * theta);
	get_speed_audio(speedL, speedR);
	left_motor_set_speed(speedL);
    	right_motor_set_speed(speedR);

	//chSysUnlock();
	//chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us\n",time_fft);
	chThdSleepMilliseconds(100);
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

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		nb_samples = 0;
		sound_remote();
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

void saveolddata(void){
	uint8_t i;
	for(i=4; i>=1;i--){//Push the old intensities one step back, to let the first spot for the new one
		oldintensity1[i]=oldintensity1[i-1];
		oldintensity2[i]=oldintensity2[i-1];
		oldintensity3[i]=oldintensity3[i-1];
		oldtheta[i]=oldtheta[i-1];
	}
	oldintensity1[0]=micRight_output[FREQ_ID];// Save the last intensity
	oldintensity2[0]=micLeft_output[FREQ_ID];
	oldintensity3[0]=micBack_output[FREQ_ID];
}

