#include <stdio.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <pthread.h>
#include "uss.h"

long uss_rf, uss_rb;
long uss_br, uss_bl;

// 測定の時間間隔[ms]
static const int t_uss = 50;
//static const int t_uss = 200;

// 測定範囲の下限・上限[cm]
//static const int var_min =  16;
//static const int var_max = 1000;

// センサからMIRS中心までの距離[cm]
static const int dist_center = 16;
static long uss_rf_prev, uss_rb_prev;

static int fd_rf, fd_rb;

int uss_open_rf(){
	fd_rf = wiringPiI2CSetup(0x70);
	
	// ソフトウェアリビジョンの確認
	if(wiringPiI2CReadReg8(fd_rf, 0x00) == 0x06){
		printf("USS_L opened\n");
		return 0;
	}else{
		printf("failed to open USS_L\n");
		return -1;
	}
}

int uss_open_rb(){
	fd_rb = wiringPiI2CSetup(0x71);
	
	// ソフトウェアリビジョンの確認
	if(wiringPiI2CReadReg8(fd_rb, 0x00) == 0x06){
		printf("USS_R opened\n");
		return 0;
	}else{
		printf("failed to open USS_R\n");
		return -1;
	}
}

int uss_open_br(){
	fd_rb = wiringPiI2CSetup(0x72);
	
	// ソフトウェアリビジョンの確認
	if(wiringPiI2CReadReg8(fd_rb, 0x00) == 0x06){
		printf("USS_R opened\n");
		return 0;
	}else{
		printf("failed to open USS_R\n");
		return -1;
	}
}

int uss_open_bl(){
	fd_rb = wiringPiI2CSetup(0x73);
	
	// ソフトウェアリビジョンの確認
	if(wiringPiI2CReadReg8(fd_rb, 0x00) == 0x06){
		printf("USS_R opened\n");
		return 0;
	}else{
		printf("failed to open USS_R\n");
		return -1;
	}
}

long uss_get_rf(){
	unsigned char lowbyte, highbyte;
	long val;
	
	wiringPiI2CWriteReg8(fd_rf, 0x00, 0x51);
	usleep(t_uss * 1000);
	
	lowbyte  = wiringPiI2CReadReg8(fd_rf, 0x03);
	highbyte = wiringPiI2CReadReg8(fd_rf, 0x02);
	val = lowbyte + highbyte * 256;
	//printf("highyte=%d    lowbyte =%d\n", highbyte, lowbyte);
	if( highbyte == 128){
		uss_rf = uss_rf_prev;
	} else {
		uss_rf = val + dist_center;
	}
	uss_rf_prev = uss_rf;

	return uss_rf;
}

long uss_get_rb(){
	unsigned char lowbyte, highbyte;
	long val;
	
	wiringPiI2CWriteReg8(fd_rb, 0x00, 0x51);
	usleep(t_uss * 1000);
	
	lowbyte  = wiringPiI2CReadReg8(fd_rb, 0x03);
	highbyte = wiringPiI2CReadReg8(fd_rb, 0x02);
	val = lowbyte + highbyte * 256;
	//printf("highyte=%d    lowbyte =%d\n", highbyte, lowbyte);
	if( highbyte == 128){
		uss_rb = uss_rb_prev;
	} else {
		uss_rb = val + dist_center;
	}
	
	uss_rb_prev = uss_rb;

	return uss_rb;
}

long uss_get_br(){
	unsigned char lowbyte, highbyte;
	long val;
	
	wiringPiI2CWriteReg8(fd_rb, 0x00, 0x51);
	usleep(t_uss * 1000);
	
	lowbyte  = wiringPiI2CReadReg8(fd_rb, 0x03);
	highbyte = wiringPiI2CReadReg8(fd_rb, 0x02);
	val = lowbyte + highbyte * 256;
	
	uss_br = val + dist_center;
	return uss_br;
}

long uss_get_bl(){
	unsigned char lowbyte, highbyte;
	long val;
	
	wiringPiI2CWriteReg8(fd_rb, 0x00, 0x51);
	usleep(t_uss * 1000);
	
	lowbyte  = wiringPiI2CReadReg8(fd_rb, 0x03);
	highbyte = wiringPiI2CReadReg8(fd_rb, 0x02);
	val = lowbyte + highbyte * 256;
	
	uss_bl = val + dist_center;
	return uss_bl;
}

void uss_thread(){

	uss_rf_prev = uss_get_rf();
        usleep(10 * 1000); // wait 10msec
	uss_rb_prev = uss_get_rb();
        usleep(10 * 1000); // wait 10msec

        while(1){
                uss_get_rf();
                usleep(10 * 1000); // wait 10msec
                uss_get_rb();
                usleep(10 * 1000); // wait 10msec
	/*
                uss_get_br();
                usleep(10 * 1000); // wait 10msec
                uss_get_bl();
                usleep(10 * 1000); // wait 10msec
	*/
                //printf("uss_rf = %ld    uss_rb = %ld\n", uss_rf, uss_rb); }
	}


	pthread_exit(0);

}
