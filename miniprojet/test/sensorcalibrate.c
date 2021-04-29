//******************************************************************************
//  Name:   sensorcalibrate.c 
//  Author: 
//  Date:   
//  Rev:    October 5, 2015 by Florian Maushart
//  Descr:  Take the average of 32 samples of the e-puck distance sensors for 
//	    calibration and output them to sensorzero[i]
//******************************************************************************


#include <stdio.h>
#include <string.h>

//#include "uart/e_uart_char.h"
#include "ch.h"
#include "hal.h"


#include "sensorcalibrate.h"
//#include <epuck1x/a_d/advance_ad_scan/e_prox.h>
#include <sensors/proximity.h>

int sensorzero[8];

void sensor_calibrate() {
	int i, j;
	//char buffer[80];
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
		//sprintf(buffer, "%d, ", sensorzero[i]);
		//e_send_uart1_char(buffer, strlen(buffer));
	}


	//sprintf(buffer, " calibration done\r\n");
	//e_send_uart1_char(buffer, strlen(buffer));

}
