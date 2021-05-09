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
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <audio_processing.h>
#include <direction.h>
#include <fft.h>
#include <arm_math.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;


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


static THD_WORKING_AREA(waThdSensor, 2048);
static THD_FUNCTION(ThdSensor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	movebitch();
        /*
        *   1st case :  pause the thread during 500ms
        */
         chThdSleepMilliseconds(100);
    }

}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    motors_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    //inits the motors

    //start prox sensor
    mic_start(&processAudioData);

    proximity_start();

    //chThdSleepMilliseconds(100);
    //start microphone

    chThdCreateStatic(waThdSensor, sizeof(waThdSensor), NORMALPRIO, ThdSensor, NULL);


    /* Infinite loop. */
    while (1){
    }
}





#define STACK_CHK_GUARD 0xe2dee396


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
