#include <stdio.h>
#include <unistd.h>
#include <wiringSerial.h>
#include "arduino.h"

static int fd;

int arduino_open(){
	//fd = serialOpen("/dev/ttyACM0", 9600); //USB
	fd = serialOpen("/dev/ttyACM0", 115200); //USB
	//fd = serialOpen("/dev/ttyACM1", 115200); //USB
	//fd = serialOpen("/dev/ttyACM2", 115200); //USB
	//fd = serialOpen("/dev/ttyAMA0", 9600); //GPIO
	
	if(fd < 0){
		printf("failed to open Arduino\n");
		return -1;
	}else{
		// バッファのクリア
		while(serialDataAvail(fd) > 0){
			serialGetchar(fd);
		}
		
		printf("Arduino opened\n");
		return 0;
	}
}

void arduino_close(){
	serialClose(fd);
	printf("Arduino closed\n");
}

void arduino_clear(){
	while(serialDataAvail(fd) > 0){
		serialGetchar(fd);
	}
}

int arduino_read(serial_data_t *serial_data) {
	int i;
	
	//printf("number of serial data= %d\n",serialDataAvail(fd));
	while(serialDataAvail(fd) >= 7){
		serial_data->val[0] = serialGetchar(fd);
		
		if(serial_data->val[0] >= 0x80){
			for(i = 1; i < 7; i++){
				serial_data->val[i] = serialGetchar(fd);
			}
			return 0;
		}
	}
	
	return -1;
}

void arduino_write(serial_data_t serial_data) {
	int i;

	for(i = 0; i < 7; i++){
		serialPutchar(fd, serial_data.val[i]);
	}
}

int arduino_receive(command_data_t *command_data) {
	serial_data_t serial_data;
	
	if(arduino_read(&serial_data) == 0){
		*command_data = arduino_decode2(arduino_decode1(serial_data));
		return 0;
	}else{
		return -1;
	}
}

void arduino_send(command_data_t command_data){
	arduino_write(arduino_encode1(arduino_encode2(command_data)));
}

middle_data_t arduino_decode1(serial_data_t serial_data) {
	middle_data_t middle_data;
	
	middle_data.val[0] = ((serial_data.val[0] << 1) & 0xfe) | ((serial_data.val[1] >> 6) & 0x01);
	middle_data.val[1] = ((serial_data.val[1] << 2) & 0xfc) | ((serial_data.val[2] >> 5) & 0x03);
	middle_data.val[2] = ((serial_data.val[2] << 3) & 0xf8) | ((serial_data.val[3] >> 4) & 0x07);
	middle_data.val[3] = ((serial_data.val[3] << 4) & 0xf0) | ((serial_data.val[4] >> 3) & 0x0f);
	middle_data.val[4] = ((serial_data.val[4] << 5) & 0xe0) | ((serial_data.val[5] >> 2) & 0x1f);
	middle_data.val[5] = ((serial_data.val[5] << 6) & 0xc0) | ((serial_data.val[6] >> 1) & 0x3f);
	
	return middle_data;
}

command_data_t arduino_decode2(middle_data_t middle_data) {
	command_data_t command_data;
	
	command_data.val[0] = ((signed short)middle_data.val[0] << 8) | middle_data.val[1];
	command_data.val[1] = ((signed short)middle_data.val[2] << 8) | middle_data.val[3];
	command_data.val[2] = ((signed short)middle_data.val[4] << 8) | middle_data.val[5];
	
	return command_data;
}

serial_data_t arduino_encode1(middle_data_t middle_data) {
	serial_data_t serial_data;
	
	serial_data.val[0] =                              0x80  | ((middle_data.val[0] >> 1) & 0x7f);
	serial_data.val[1] = ((middle_data.val[0] << 6) & 0x40) | ((middle_data.val[1] >> 2) & 0x3f);
	serial_data.val[2] = ((middle_data.val[1] << 5) & 0x60) | ((middle_data.val[2] >> 3) & 0x1f);
	serial_data.val[3] = ((middle_data.val[2] << 4) & 0x70) | ((middle_data.val[3] >> 4) & 0x0f);
	serial_data.val[4] = ((middle_data.val[3] << 3) & 0x78) | ((middle_data.val[4] >> 5) & 0x07);
	serial_data.val[5] = ((middle_data.val[4] << 2) & 0x7c) | ((middle_data.val[5] >> 6) & 0x03);
	serial_data.val[6] = ((middle_data.val[5] << 1) & 0x7e);
	
	return serial_data;
}

middle_data_t arduino_encode2(command_data_t command_data) {
	middle_data_t middle_data;
	
	middle_data.val[0] = (command_data.val[0] >> 8) & 0x00ff;
	middle_data.val[1] =  command_data.val[0]       & 0x00ff;
	middle_data.val[2] = (command_data.val[1] >> 8) & 0x00ff;
	middle_data.val[3] =  command_data.val[1]       & 0x00ff;
	middle_data.val[4] = (command_data.val[2] >> 8) & 0x00ff;
	middle_data.val[5] =  command_data.val[2]       & 0x00ff;
	
	return middle_data;
}
