/*
走行コースの前提
	1. 真東に直進
	2.一点半径で90度北にカーブ
座標は基地局を原点としたもの
数値は記述がなければメートル[m]
eart, notrh はグローバル変数、基地局からの現在の座標値
*/
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
//#include "io.h"
#include "person.h"

int main(){
	
	int person_flag;

	printf("test_peson start!!\n");

	while(1){
		person_flag = person_check();
		printf("person_flag = %d  \n", person_flag);
		sleep(1);
	}

	//pthread_join(tid_A, NULL);
	//printf("test_serverrtk finished!!\n");
        return 0;

}
