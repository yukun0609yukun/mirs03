#ifndef __ARDUINO__
#define __ARDUINO__

typedef struct{
	unsigned char val[7];
} serial_data_t;

typedef struct{
	unsigned char val[6];
} middle_data_t;

typedef struct{
	signed short val[3];
} command_data_t;

extern int  arduino_open();
extern void arduino_close();
extern void arduino_clear();

extern int  arduino_read(serial_data_t *serial_data);
extern void arduino_write(serial_data_t serial_data);

extern int  arduino_receive(command_data_t *command_data);
extern void arduino_send(command_data_t command_data);

extern middle_data_t  arduino_decode1(serial_data_t serial_data);
extern command_data_t arduino_decode2(middle_data_t middle_data);
extern serial_data_t  arduino_encode1(middle_data_t middle_data);
extern middle_data_t  arduino_encode2(command_data_t command_data);

#endif
