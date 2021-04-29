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
#include <utilit.h>
#include <arm_math.h>
int MAX = 50; // bon là va falloir faire des mesures pour que avoir une valeur cohérente
void movebitch(){
int leftwheel, rightwheel;
	int sensor[8];
	double sensorMean[8]=10;
	int numberOfSamples;
	int i,n;
	int sensorsum;

left_motor_set_speed(400);
right_motor_set_speed(400);
    while (1) {
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
    					sensor[i] = get_prox(i)-sensorzero[i];
    					//linearize the sensor output and compute the average
    					sensorMean[i]+=12.1514*log((double)sensor[i])/(double)numberOfSamples;
    				}
    			}
			for (i = 0; i < 8; i++) {
				sensorsum += sensorMean[i];
			}

			// condition sensor à mettre et calcul 
			if(sensorsum> MAX){ 
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
    			left_motor_set_speed(leftwheel);
    			right_motor_set_speed(rightwheel);}
			

			else{
			left_motor_set_speed(leftwheel);// mettre tes valeurs de moteur ici
    			right_motor_set_speed(rightwheel);
			}


    		}
}