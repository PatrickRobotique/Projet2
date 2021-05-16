#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <audio_processing.h>
#include <direction.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);
    //starts the USB communication
    usb_start();
    //starting motors
    motors_init();
    //start prox sensor
    proximity_start();
    //proximity.c library, calibration of all 8 IR sensors
    calibrate_ir();
    //start microphone
    mic_start(&processAudioData);
    //starts the thread for the sensors
    thd_sensor_start();
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
