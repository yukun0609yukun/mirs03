#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include "person.h"

#define MAX_LINE_LENGTH 10

float area;

//static int flag = 0;
static FILE *fp;

char person_file[256]="../person_check_py/person_file.txt";
//char person_file[256]="../../person_check_py/person_file.txt";

void person_fopen(){

	//fp = fopen(person_file,"r");
	if( (fp = fopen(person_file,"r")) == NULL){
		printf("person file open error!\n");
		exit(1);
	}

}

//int person_check(int *p_flag) {
int person_check() {

	//int i;
	char *p;
	//char str[100][1];
	char str[100];
	int flag = 0;
 
	// 受信

	fp = fopen(person_file,"r");
	if( fp == NULL){
		printf("person_fileのオープンに失敗しました\n");
		exit(1);
	}

	if (fgets(str, MAX_LINE_LENGTH, fp) != NULL) {
		p = strchr(str, '\n');  // 改行文字を探す
		if (p) *p = '\0';        // 改行文字あれば、上書きして消す
	}

	flag = atoi(str);
        printf("flag=%d\n",flag);

	fclose(fp);

	/*
	if( flag == 1) {
		*p_flag = 1;
        	printf("p_flag=%d\n",*p_flag);

	}
	*/

	return flag;
}
