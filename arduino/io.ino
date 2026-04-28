void io_open() {
  pinMode(PIN_LED, OUTPUT);
//  pinMode(PIN_SW, INPUT);
  pinMode(PIN_BATT, INPUT);
  pinMode(PIN_LIGHT, OUTPUT);
  digitalWrite(PIN_LED, LOW);
//  digitalWrite(PIN_SW, HIGH);
  digitalWrite(PIN_BATT, LOW);
  digitalWrite(PIN_LIGHT, LOW);
}


//int io_get_sw() {
//  return digitalRead(PIN_SW);
//}

double io_get_batt() {
  return analogRead(PIN_BATT) * 5.0 / 1024.0 / V_RATIO;
}
