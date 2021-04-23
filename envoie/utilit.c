//#include "./motor_led/e_epuck_ports.h"
//#define SELECTOR0 palReadPad(GPIOC, GPIOC_SEL_0)
//#define SELECTOR1 palReadPad(GPIOC, GPIOC_SEL_1)
//#define SELECTOR2 palReadPad(GPIOC, GPIOC_SEL_2)
//#define SELECTOR3 palReadPad(GPIOD, GPIOD_SEL_3)
void waiting(long num) {
	long i;
	for(i=0;i<num;i++);
}

/*int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}*/
