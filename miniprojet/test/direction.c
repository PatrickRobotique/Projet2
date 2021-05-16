/*Cette partie du programme détermine la vitesse des moteurs.
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
#define DIST_OBJET 52
#define AVT_CNTRE_GC 7
#define AVT_CNTRE_DT 0
#define AVT_GC 6
#define AVT_DT 1
#define GAUCHE 5
#define DROITE 2
#define NUM_CAPTEUR 8
#define SPD_MAX 1000
#define SPD_INI 400


void sensor_values(void){

	int leftwheel, rightwheel;
	int sensor[NUM_CAPTEUR];
	float sensorMean[NUM_CAPTEUR];
	int numberOfSamples = 10;
	int i,n;
    // Vitesse avant modification
    leftwheel = SPD_INI;
    rightwheel = SPD_INI;
    for (i=0;i<NUM_CAPTEUR;i++){
    	sensorMean[i]=0;
    }
    //On fait une moyenne des mesures des capteurs IR pour plus de précision
    for (n=0;n<numberOfSamples;n++){
    	//Mesures capteurs IR
    	for (i = 0; i < NUM_CAPTEUR; i++) {
    		sensor[i] = get_calibrated_prox(i);
    		//linéarisation des mesures
    		sensorMean[i]+=12.1514*log((double)sensor[i]+1)/(double)numberOfSamples;
    	}
    }
	// conditions des capteurs IR pour savoir si ils contrôlent les moteurs
	if(sensorMean[AVT_CNTRE_DT]>DIST_OBJET || sensorMean[AVT_CNTRE_GC]>DIST_OBJET||
		sensorMean[AVT_DT]> DIST_OBJET || sensorMean[AVT_GC]>DIST_OBJET ||
		sensorMean[DROITE]>DIST_OBJET  || sensorMean[GAUCHE]>DIST_OBJET){
		// Ajout des poids
		for (i = 0; i < NUM_CAPTEUR; i++) {
			leftwheel += weightleft[i] * (int)sensorMean[i];
			rightwheel += weightright[i] * (int)sensorMean[i];
		}
		// Limites sur les vitesses
		if (leftwheel > SPD_MAX) {leftwheel = SPD_MAX;}
		if (rightwheel > SPD_MAX) {rightwheel = SPD_MAX;}
		if (leftwheel < -SPD_MAX) {leftwheel = -SPD_MAX;}
		if (rightwheel < -SPD_MAX) {rightwheel = -SPD_MAX;}
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

static THD_WORKING_AREA(waThdSensor, 2048);
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


