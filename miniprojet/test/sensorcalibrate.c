#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "sensorcalibrate.h"
#include <sensors/proximity.h>

int sensorzero[8];
//Ici on initialise les capteurs de proximité
void sensor_calibrate() {
	int i, j;
	long sensor[8];

	for (i=0; i<8; i++) {
		sensor[i]=0;
	}

	for (j=0; j<32; j++) {
		for (i=0; i<8; i++) {
			sensor[i]+=get_prox(i);
		}
	}

	for (i=0; i<8; i++) {
		sensorzero[i]=(sensor[i]>>5);
	}
}
