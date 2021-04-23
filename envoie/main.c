#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <msgbus/messagebus.h>
#include <audio/microphone.h>
//#include <epuck1x/a_d/advance_ad_scan/e_prox.h>
#include <sensors/proximity.h>

#include <audio_processing.h>
#include <fft.h>
#include <sensorcalibrate.h>
#include <utilit.h>
#include <arm_math.h>
int weightleft[8] = {-5, -5, -5, 0, 0, 5, 5, 5};
int weightright[8] = {5, 5, 5, 0, 0, -5, -5, -5};
//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}




int main(void)
{

	int leftwheel, rightwheel;
	int sensor[8];
	double sensorMean[8];
	int numberOfSamples;
	int i,n;

    halInit();
    chSysInit();
    mpu_init();
    /** Inits the Inter Process Communication bus. */

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);

    //e_init_prox();
    proximity_start();
    //Calibrate sensors
    sensor_calibrate();
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    //inits the motors
    motors_init();
    //Set the original speed of the robot
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
    			right_motor_set_speed(rightwheel);

    			/* Indicate with leds on which side we are turning (leds are great for debugging)
    			if (leftwheel>rightwheel) {
    				e_set_led(1, 1);
    				e_set_led(7, 0);
    			} else {
    				e_set_led(1, 0);
    				e_set_led(7, 1);
    			}*/

    		}

/*#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    //while (1){
}
#define STACK_CHK_GUARD 0xe2dee396


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
