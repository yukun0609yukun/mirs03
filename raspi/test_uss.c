#include <stdio.h>
#include <unistd.h>
#include "uss.h"

int main(){
	int flag_rf = 0, flag_rb = 0;
	
	if(uss_open_rf() != 0){
		printf(" left uss open faild !!\n");
		flag_rf = -1;
	}
	if(uss_open_rb() != 0){
		printf(" right uss open faild !!\n");
		flag_rb = -1;
	}
	if( flag_rf == -1 && flag_rb == -1) return -1;
	
	while(1){
		uss_rf = uss_get_rf();
		usleep(500 * 1000);
		
		uss_rb = uss_get_rb();
		usleep(500 * 1000);
		
		printf("uss_rf = %6ld, uss_rb = %6ld\n", uss_rf, uss_rb);
		usleep(10 * 1000);
	}
	
	return 0;
}
