#include <stdio.h>
#include <unistd.h>
#include "arduino.h"
#include "io.h"
#include "request.h"
#include "uss.h"
#include "person.h"

#define ST_STEP 10
#define P_STEP 10

int main(){
	//double volt;
	int speed = 50; //[cm]::
	int count = 0;
	int vel_l, vel_r;
	int dist;
	int dist_l=0, dist_r=0;
	int p_flag =0;
	//int p_flag_prev = 0;
	int dist_ref=0;
	
	if(io_open() != 0) return -1;
	if(arduino_open() != 0) return -1;
	//if(uss_open_l() != 0) return -1;
	//if(uss_open_r() != 0) return -1;
	//if(uss_open_rf() != 0) return -1;
	//if(uss_open_rb() != 0) return -1;

	printf("speed? [cm/s]\n");
	scanf("%d",&speed);
	printf("run distance? [cm]\n");
	scanf("%d",&dist_ref);

	printf("press enter to start\n");
	getchar();

	//なめらかスタート
	for ( count = 0; count < ST_STEP ; count++){
		vel_l = speed * (float)count / (float)ST_STEP;
		vel_r = vel_l;
		request_set_runmode(VEL, vel_l, vel_r);
		usleep(100*1000);
	}
	
	while(1){
		// 人がいるためスピードを緩める
		if( person_check() == 1 ){
			p_flag = 1;
			//vel_l = 0;
			//vel_r = 0;
			vel_l = speed *( 1.0 - area * 5.0 );
			if( vel_l < 0) vel_l = 0;
			vel_r = vel_l;
			printf("area =%f    vel_l=%d, \n", area, vel_l);
			request_set_runmode(VLT, vel_l, vel_r);
			printf("人がいるの\n");
		} else {
			p_flag = 0;
			vel_l = speed ;
			vel_r = speed ;
			//printf("vel_l =%d, vel_r=%d\n",vel_l, vel_r);
			request_set_runmode(VLT, vel_l, vel_r);
		}

		/*
		//人がいなくなったあとのなめらかスタート
		if( p_flag == 0 && p_flag_prev ==1){
			for ( count = 0; count < P_STEP ; count++){
				vel_l = speed * (float)count / (float)P_STEP;
				vel_r = vel_l;
				request_set_runmode(VLT, vel_l, vel_r);
				usleep(100*1000);
			}
		}
		p_flag_prev = p_flag;
		*/

		request_get_dist(&dist_l, &dist_r);
		dist = ( dist_l + dist_r ) /2 ;
		
		//後処理 
		if( dist >= dist_ref){
			request_set_runmode(STP, 0, 0);
			break;
		}
		//制御周期
		usleep(40 * 1000);
	}
	
	request_set_runmode(STP, 0, 0);
	arduino_close();

	return 0;
}
