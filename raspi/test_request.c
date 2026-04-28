#include <stdio.h>
#include "arduino.h"
#include "request.h"

int main(){
	int mode, speed, dist;
	double volt;
	//char buf[256];
	run_state_t state;
	int dist_l = 0 , dist_r = 0;
	int run_dist;
	
	if(arduino_open() != 0) return -1;

	while(1){
		//printf("0:stop  1:straight  2:rotate  3:line trace  4:get_mode  5:get_volt  6:quit\n");
		//printf("0:stop  1:straight  2:rotate  3:circle  4:line trace  5:get_mode  6:get_volt  7:quit\n");
		//printf("0:stop  1:straight  2:rotate  3:circle  4:line trace  5:velocity  6:get_mode  7:get_volt  8:quit\n");
		printf("0:stop  1:straight  2:rotate  3:circle  4:linetrace  5:velocity  6: linetrace(vel)  7:get_mode  8:get_volt  9:quit\n");
		scanf("%d",&mode);
		
		switch(mode){
		case 0:
			request_set_runmode(STP, 0, 0);
			break;
		case 1:
			printf("speed? [cm/s]\n");
			scanf("%d",&speed);
			printf("dist? [cm]\n");
			scanf("%d",&dist);
			
			request_set_runmode(STR, speed, dist);
			while(1){
				request_get_runmode(&state, &speed, &dist);
				printf("state=%d    speed=%d   dist=%d\n", state, speed, dist);
				if( state == STP ) break;
			}
			break;

		case 2:
			printf("speed? [deg/s]\n");
			scanf("%d",&speed);
			
			printf("angle? [deg]\n");
			scanf("%d",&dist);
			
			request_set_runmode(ROT, speed, dist);
			while(1){
				request_get_runmode(&state, &speed, &dist);
				printf("state=%d    speed=%d   dist=%d\n", state, speed, dist);
				if( state == STP ) break;
			}
			break;

		case 3:
			printf("circle radius? [cm]\n");
			scanf("%d",&speed);
			printf("circle degree ? [deg]\n");
			scanf("%d",&dist);
			
			request_set_runmode(CIR, speed, dist);
			while(1){
				request_get_runmode(&state, &speed, &dist);
				printf("state=%d    speed=%d   dist=%d\n", state, speed, dist);
				if( state == STP ) break;
			}
			break;

		case 4:
			printf("speed? [cm/s]\n");
			scanf("%d",&speed);
			printf("dist? [cm]\n");
			scanf("%d",&dist);
			
			request_set_runmode(LTC, speed, dist);
			while(1){
				request_get_runmode(&state, &speed, &dist);
				if( state == STP ) break;
				printf("state=%d    speed=%d   dist=%d\n", state, speed, dist);
			}
			break;

		case 5:
			printf("left speed? [cm/s]\n");
			scanf("%d",&speed);
			printf("right speed? [cm/s]\n");
			scanf("%d",&dist);
			printf("run distance? [cm]\n");
			scanf("%d",&run_dist);
			
			request_set_runmode(VEL, speed, dist);
			while(1){
				request_get_dist(&dist_l, &dist_r);
				dist = ( dist_l + dist_r ) /2.0;
				printf("dist=%d    dist_l=%d   dist_r=%d\n", dist, dist_l, dist_r);
				if( dist > run_dist ) break;
			}
			printf("velocity mode finish!\n");
			request_set_runmode(STP, 0, 0);
			break;

		case 6:
			printf("speed? [cm/s]\n");
			scanf("%d",&speed);
			//printf("right speed? [cm/s]\n");
			//scanf("%d",&dist);
			printf("run distance? [cm]\n");
			scanf("%d",&run_dist);

			dist = speed;
			request_set_runmode(VLT, speed, dist);
			while(1){
				request_get_dist(&dist_l, &dist_r);
				dist = ( dist_l + dist_r ) /2.0;
				printf("dist=%d    dist_l=%d   dist_r=%d\n", dist, dist_l, dist_r);
				if( dist > run_dist ) break;
			}
			printf("velocity linetrace mode finish!\n");
			request_set_runmode(STP, 0, 0);
			break;

		case 7:
			request_get_runmode(&state, &speed, &dist);
			printf("state = %s\n",((state == STR) ? "STR" : (state == ROT) ? "ROT" : "STP"));
			break;

		case 8:
			request_get_batt(&volt);
			printf("batt = %4.2lf\n", volt);
			break;
		case 9:
			return 0;
		default:
			break;
		}
	}
	
	arduino_close();
	return 0;
}
