#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "uss.h"

void thread_uss();
//long uss_l, uss_r;

int main(){

	pthread_t th_uss_id;
	int flag_rf = 0, flag_rb = 0;

	if(uss_open_rf() != 0){
		printf(" right forward uss open faild !!\n");
		flag_rf = -1;
	}
	if(uss_open_rb() != 0){
		printf(" right back uss open faild !!\n");
		flag_rb = -1;
	}
	if( flag_rf == -1 && flag_rb == -1) return -1;

	pthread_create(&th_uss_id, NULL, (void *)uss_thread, NULL);

	while(1){
		printf("uss_rf = %6ld, uss_rb = %6ld\n", uss_rf, uss_rb);
		usleep(1000 * 1000); // 1sec wait
	}

	pthread_join( th_uss_id, NULL);
	return 0;
}
