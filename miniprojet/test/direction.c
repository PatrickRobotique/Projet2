/*Cet partie du programme détermine la vitesse des moteurs.
Il y a aussi la gestion des capteurs infrarouges ainsi que
l'algorithme de Braitenberg, inspiré d'un exercice de Distributed Intelligent System
*/
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

// Poids des capteurs IR
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
    			// Vitesse avant modification
    			leftwheel = 400;
    			rightwheel = 400;
    			for (i=0;i<8;i++)
    				sensorMean[i]=0;
    			//On fait une moyenne des mesures des capteurs IR pour plus de précision
    			for (n=0;n<numberOfSamples;n++)
    			{
    				//Mesures capteurs IR 
    				for (i = 0; i < 8; i++) {
    					sensor[i] = get_calibrated_prox(i);
    					//linéarisation des mesures (cf rapport)
    					sensorMean[i]+=12.1514*log((double)sensor[i]+1)/(double)numberOfSamples;
    				}
    			}

			// condition des capteurs IR avants pour savoir si ils contrôlent les moteurs
			if(sensorMean[0]>54 || sensorMean[1]> 54 || sensorMean[6]>54 || sensorMean[7]>54){
    			// Ajout des poids
    			for (i = 0; i < 8; i++) {
    				leftwheel += weightleft[i] * (int)sensorMean[i];
    				rightwheel += weightright[i] * (int)sensorMean[i];
    			}

    			// Limites sur les vitesses
    			if (leftwheel > 1000) {leftwheel = 1000;}
    			if (rightwheel > 1000) {rightwheel = 1000;}
    			if (leftwheel < -1000) {leftwheel = -1000;}
    			if (rightwheel < -1000) {rightwheel = -1000;}
    			Speed[SPEEDLSENSOR]= leftwheel;
				Speed[SPEEDRSENSOR]=rightwheel;
    			left_motor_set_speed(Speed[SPEEDLSENSOR]);
    			right_motor_set_speed(Speed[SPEEDRSENSOR]);
			}
			//Sinon c'est la triangulation qui dirige les moteurs
			else{
			left_motor_set_speed(Speed[SPEEDLAUDIO]);
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
        //pause de 50 ms
    	  chThdSleepMilliseconds(50);
    }

}

void thd_sensor_start(void){
	chThdCreateStatic(waThdSensor, sizeof(waThdSensor), NORMALPRIO, ThdSensor, NULL);
}
