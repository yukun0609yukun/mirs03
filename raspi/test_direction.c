#include <stdio.h>
#include "arduino.h"
#include "direction.h"
#include "uss.h"

int main(){
	int dist;
	char buf[256];
	
	//if(io_open() != 0) return -1;
	if(arduino_open() != 0) return -1;
	//if(uss_open_l() != 0) return -1;
	//if(uss_open_r() != 0) return -1;
	if(uss_open_bl() != 0) return -1;
	if(uss_open_br() != 0) return -1;
	
	printf("dist? [cm]\n");
	fgets(buf, sizeof(buf), stdin);
	sscanf(buf, "%d", &dist);
	
	if(direction_correct(dist) == 0){
		printf("finished to correct\n");
	}else{
		printf("failed to correct\n");
	}
	
	arduino_close();
	return 0;
}
