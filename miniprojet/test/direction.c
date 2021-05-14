#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include <main.h>
#include <motors.h>
#include <msgbus/messagebus.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <arm_math.h>

// Weights for the Braitenberg obstacle avoidance algorithm
int weightleft[8] = {-5, -5, -5, 0, 0, 5, 5, 5};
int weightright[8] = {5, 5, 5, 0, 0, -5, -5, -5};
static int Speed[4] = {0,0,0,0};
#define SPEEDLAUDIO 0
#define SPEEDRAUDIO 1
#define SPEEDLSENSOR 2
#define SPEEDRSENSOR 3

void sensor_values(void){

int leftwheel, rightwheel;
	int sensor[8];
	float sensorMean[8];
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
    					sensorMean[i]+=12.1514*log((double)sensor[i]+1)/(double)numberOfSamples;
    				}
    			}

			// condition sensor à mettre et calcul
			if(sensorMean[0]>54 || sensorMean[1]> 54 || sensorMean[6]>54 || sensorMean[7]>54){
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
    			left_motor_set_speed(Speed[SPEEDLSENSOR]);
    			right_motor_set_speed(Speed[SPEEDRSENSOR]);
			}


			else{
			left_motor_set_speed(Speed[SPEEDLAUDIO]);// mettre tes valeurs de moteur ici
    		right_motor_set_speed(Speed[SPEEDRAUDIO]);
			}
   		}



void get_speed_audio(int speedLaudio, int speedRaudio){

	Speed[SPEEDLAUDIO] = speedLaudio;
	Speed[SPEEDRAUDIO] = speedRaudio;

}


static THD_WORKING_AREA(waThdSensor, 4096);
static THD_FUNCTION(ThdSensor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	sensor_values();
        /*
        *   pause the thread during 100ms
        */
    	  chThdSleepMilliseconds(50);
    }

}

void thd_sensor_start(void){
	chThdCreateStatic(waThdSensor, sizeof(waThdSensor), NORMALPRIO, ThdSensor, NULL);
}


