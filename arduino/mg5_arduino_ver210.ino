/*
  Ver1.0からの変更点　　　RaspberryPi と２バイトデータのシリアル通信をできるようにした 
  Ver2.0からの変更点　　　RaspberryPi とのシリアル通信の速度を9600bps から 115200bpsに変更した
 */

#include "define.h"

//いじる部分
//直進移動にかかる時間[cm]
int repeat_x[4] = {500, 500, 500, 500};
//回転にかかる時間[deg]
int repeat_rot = 90;

//繰り返し回数（ここはいじらない）
int repeatstate = 0;

void setup() {
  //OMAJINAI
  io_open();
  encoder_open();
  motor_open();
  raspi_open();
}

void loop() {
  for (repeatstate = 0; repeatstate < 4; repeatstate++) {
    //run_ctrl_set(ROT, 150, 90);
    //run_ctrl_execute();
    //vel_ctrl_execute();
    //delay(3000);
    run_ctrl_set(STR, 100, 100);
    run_ctrl_execute();
    vel_ctrl_execute();
    delay(3000);
  }
}
