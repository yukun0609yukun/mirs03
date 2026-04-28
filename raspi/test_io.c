#include <stdio.h>
#include "io.h"

int main(){
	int sw_f, sw_l, sw_r;
	
	if(io_open() != 0) return -1;
	
	while(1){
		io_get_sw(&sw_f, &sw_l, &sw_r);
		printf("switch_f = %d, switch_l = %d, switch_r = %d\n", sw_f, sw_l, sw_r);
		usleep(100 * 1000);
	}
	
	return 0;
}
