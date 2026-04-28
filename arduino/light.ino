static float Kp = 0.2; //0.025; // 0.025
static float Kd = 1.0; //0.10; //0.10

static float err = 0;
static float err_prev = 0;

static int gray = 460 ; //720; //625; // 400; //830;

//int light_mod(int gray){
int light_mod(){
  int light_val, vel_mod;
  light_val = analogRead(PIN_LS);
  //Serial.print("light_val=");
  //Serial.println(light_val);
  err = (float)(light_val - gray); //anti-clockwise rotation
  //err = -(float)(light_val - gray); //clockwise rotation
  vel_mod = Kp * err + Kd * (err - err_prev);
  err_prev = err;
  return vel_mod;
} 

void light_active(){
  digitalWrite(PIN_LIGHT, HIGH);
}

void light_inactive(){
  digitalWrite(PIN_LIGHT, LOW);
}
