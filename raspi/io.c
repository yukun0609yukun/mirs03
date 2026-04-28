#include <stdio.h>
#include <wiringPi.h>
#include "io.h"

// ピン配置
//static const int pin_sw_f  = 25;
//static const int pin_sw_l  = 21;
//static const int pin_sw_r  = 27;
//static const int pin_sw_f  = 18;
//static const int pin_sw_l  = 25;
//static const int pin_sw_r  = 26;
static const int pin_sw_f  = 4;
static const int pin_sw_l  = 7;
static const int pin_sw_r  = 8;

int io_open(){
	// GPIOデバイスのオープン
	if(wiringPiSetupGpio() != 0){
		printf("failed to open GPIO\n");
		return -1;
	}else{
		// ピン設定
		pinMode(pin_sw_f, INPUT);
		pinMode(pin_sw_l, INPUT);
		pinMode(pin_sw_r, INPUT);
		pullUpDnControl(pin_sw_f, PUD_UP);
		pullUpDnControl(pin_sw_l, PUD_UP);
		pullUpDnControl(pin_sw_r, PUD_UP);
		
		printf("GPIO opened\n");
		return 0;
	}
}

void io_get_sw(int *sw_f, int *sw_l, int *sw_r){
	*sw_f = digitalRead(pin_sw_f);
	*sw_l = digitalRead(pin_sw_l);
	*sw_r = digitalRead(pin_sw_r);
	printf("F=%d  L=%d   R=%d \n", *sw_f, *sw_l,*sw_r);
}
