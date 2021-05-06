#include "ch.h"
#include "hal.h"
#include <math.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>


//simple PI regulator implementation

// intensitemoy c'est la moy d'intensité des micro donc à calculer dans direction ou ici à voir
int16_t pi_regulator(float intensitemoy, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = intensitemoy - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	// donc on peut le garder mais faudra aviser en testant, pas indispensable
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// je propose qu'on mette un mette un MAX SUM ERROR assez bas comme ça, ça casse pas trop les couilles
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}
	// kp et ki constante globale a def 
	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}
