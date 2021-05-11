#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <msgbus/messagebus.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <audio_processing.h>
#include <fft.h>
#include <sensorcalibrate.h>
#include <arm_math.h>

// Weights for the Braitenberg obstacle avoidance algorithm
int weightleft[8] = {-5, -5, -5, 0, 0, 5, 5, 5};
int weightright[8] = {5, 5, 5, 0, 0, -5, -5, -5};
static int Speed[4] = {0,0,0,0};
//Variables contenant les vitesses à donner aux moteurs
#define SPEEDLAUDIO 0
#define SPEEDRAUDIO 1
#define SPEEDLSENSOR 2
#define SPEEDRSENSOR 3

void movebitch(void){
int leftwheel, rightwheel;
	int sensor[8];
	double sensorMean[8];
	int numberOfSamples = 10;
	int i,n;
    			// Forward speed
    			leftwheel = 400;
    			rightwheel = 400;
    			for (i=0;i<8;i++)
    				sensorMean[i]=0;
    			//Compute an average value of each sensor on multiple samples to reduce noise
    			for (n=0;n<numberOfSamples;n++)
    			{
    				// Get sensor values
    				for (i = 0; i < 8; i++) {
    					// Use the sensorzero[i] value generated in sensor_calibrate() to zero sensorvalues
    					sensor[i] = get_calibrated_prox(i);
    					//linearize the sensor output and compute the average
    					sensorMean[i]+=12.1514*log((double)sensor[i])/(double)numberOfSamples;
    				}
    			}
			// condition sensor à mettre et calcul
			if(sensorMean[0]>80 || sensorMean[1]> 80 || sensorMean[6]>80 || sensorMean[7]>80){
    			// Add the weighted sensors values
    			for (i = 0; i < 8; i++) {
    				leftwheel += weightleft[i] * (int)sensorMean[i];
    				rightwheel += weightright[i] * (int)sensorMean[i];
    			}
    			// Speed bounds, to avoid setting to high speeds to the motor
    			if (leftwheel > 1000) {leftwheel = 1000;}
    			if (rightwheel > 1000) {rightwheel = 1000;}
    			if (leftwheel < -1000) {leftwheel = -1000;}
    			if (rightwheel < -1000) {rightwheel = -1000;}
    			Speed[SPEEDLSENSOR]= leftwheel;
			Speed[SPEEDRSENSOR]=rightwheel;
			//vitesses si les capteurs de proximité détectent des objets à éviter
    			left_motor_set_speed(Speed[SPEEDLSENSOR]);
    			right_motor_set_speed(Speed[SPEEDRSENSOR]);
			}
			else{
			//vitesses si il n'y pas d'objets à éviter, déterminé par la triangulation
			left_motor_set_speed(Speed[SPEEDLAUDIO]);
    			right_motor_set_speed(Speed[SPEEDRAUDIO]);
			}
   		}

void get_speed_audio(int speedLaudio, int speedRaudio){
	Speed[SPEEDLAUDIO] = speedLaudio;
	Speed[SPEEDRAUDIO] = speedRaudio;
}
