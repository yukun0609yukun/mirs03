static run_state_t run_state = STP;
static double speed_ref  = 0.0;
static double speed_curr = 0.0;
static double speed_cir = 20.0; 
static double dist_ref   = 10.0;
static double dist_curr  = 0.0;
static double er  = 0.0 , er_d = 0.0 ;
static double er_sum  = 0.0;
static double er_prev  = 0.0;
//static int  gray = 400 ; //750;
static int speed_l, speed_r;

void run_ctrl_execute() {
  // 直進制御において減速を開始する距離 [cm]
  const double dist_vel_down = 0.0;

  // 回転制御において減速を開始する角度 [deg]
  const double angle_vel_down = 0.0;

  // 直進制御における左右のタイヤの距離差の補正ゲイン
  const double Ks_p = 1.0;//10.0 ; // 40.0
  const double Ks_i =  0.0 ; // 0.2  1.0  
  const double Ks_d = 5.0; //100.0 ; // 50.0  20.0  5.0/

    // 回転制御における左右のタイヤの距離差の補正ゲイン
  const double Kr = 0.0;

  // 円弧走行制御における左右のタイヤの距離差の補正ゲイン
  const double Kc_p = 1.0 ; // 40.0
  const double Kc_i =  0.0 ; // 0.2  1.0
  const double Kc_d = 10.0 ; // 50.0  20.0  5.0

  float th_curr, th_ref; //円弧走行角 [rad]
  float d_l_ref, d_r_ref; //左右のレファレンス距離
  float er_l, er_r ; //左右の実際の走行距離とレファレンス距離の差
  float er_l_sum =0, er_r_sum = 0;
  float er_l_d = 0, er_r_d = 0;
  float vel_ref_l , vel_ref_r;
  float vel_mod_l , vel_mod_r;
  float R ; // 円弧走行半径


  int sign;
  double d_l, d_r, v_l, v_r, ratio, vel_ref, vel_mod;

  // 負の指令値に対応
  if (dist_ref >= 0.0) {
    sign = 1;
  } 
  else {
    sign = -1;
  }

  distance_get(&d_l, &d_r);
  vel_ctrl_get(&v_l, &v_r);

  switch (run_state) {

  case STP:
    vel_ctrl_set(0.0, 0.0);
    light_inactive();
    break;

  case STR:
    //Serial.println("STR !!");

    // 直進距離
    dist_curr  = (d_l + d_r) / 2.0;
    speed_curr = (v_l + v_r) / 2.0 * sign;

    /*
    if(dist_curr >= dist_ref) {
      run_state = STP;
      return;
    }
    */
    
    // 減速率
    ratio = sign * (dist_ref - dist_curr) / dist_vel_down;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    if (speed_ref == 0.0 || dist_ref == 0.0 || ratio == 0.0) {
      run_state = STP;
      vel_ctrl_set(0.0, 0.0);
    } 
    else {
      // 減速の実行
      vel_ref = sign * speed_ref * ratio;

      // 左右のタイヤの距離差の補正
      er = d_l - d_r;
      er_sum += er;
      er_d = er - er_prev;
      vel_mod = Ks_p * er + Ks_i * er_sum  + Ks_d  * er_d ;
      er_prev = er;

      vel_ctrl_set((vel_ref - vel_mod), (vel_ref + vel_mod));
    }

    break;

  case ROT:
    // 回転角度
    dist_curr  = -(d_l - d_r) / D_TIRE * 180.0 / PI;
    speed_curr = -(v_l - v_r) / D_TIRE * 180.0 / PI * sign;

    /*
    if (dist_curr >= dist_ref) {
      run_state = STP;
      return;
    } 
    */

    // 減速率
    ratio = sign * (dist_ref - dist_curr) / angle_vel_down;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    if (speed_ref == 0.0 || dist_ref == 0.0 || ratio == 0.0) {
      run_state = STP;
      vel_ctrl_set(0.0, 0.0);
    } 
    else {
      // 減速の実行＆角速度指令値→速度指令値に変換
      vel_ref = sign * speed_ref * ratio * D_TIRE / 2.0 * PI / 180.0;

      // 左右のタイヤの距離差の補正
      vel_mod = -(d_l + d_r) * Kr;

      vel_ctrl_set(-(vel_ref - vel_mod), (vel_ref + vel_mod));
    }

    break;

  case CIR:
  
    // 円弧走行
    dist_curr  = (d_l + d_r) / 2.0;
    speed_curr = (v_l + v_r) / 2.0 * sign;

    //PI = 3.1415;
    R = speed_ref;
    th_ref = dist_ref;
    vel_ref = speed_cir; // [cm/s]

    th_curr  = (d_l + d_r) / 2. / R * 180.0 / PI; //現在の円弧角度
    //speed_curr = -(v_l - v_r) / D_TIRE * 180.0 / PI * sign;

    //Serial.print("   th_curr=");
    //Serial.println(th_curr);

    //vel_ctrl_set(10,10);

    if( th_ref > 0 ){

      vel_ref_l = vel_ref * ( R - D_TIRE/2.) / R;
      vel_ref_r = vel_ref * ( R + D_TIRE/2.) /R;

      //Serial.print("vel_ref_l =");
      //Serial.println(vel_ref_l);
      //Serial.print("vel_ref_r =");
      //Serial.println(vel_ref_r);

      d_l_ref = th_curr * PI /180. * ( R - D_TIRE/2.);
      d_r_ref = th_curr * PI /180. * ( R + D_TIRE/2.);
      er_l = d_l - d_l_ref;
      er_r = d_r - d_r_ref;
      //Serial.print("er_l =");
      //Serial.print(er_l);
      //Serial.print("  er_r =");
      //Serial.println(er_r);

      vel_mod_l = Kc_p * er_l + Kc_i * er_l_sum  + Kc_d  * er_l_d ;
      vel_mod_r = Kc_p * er_r + Kc_i * er_r_sum  + Kc_d  * er_r_d ;
      //vel_mod_l = 0;
      //vel_mod_r = 0;

      vel_ctrl_set((vel_ref_l - vel_mod_l), (vel_ref_r - vel_mod_r));

      if ( th_curr >= th_ref ) {
        run_state = STP;
        //vel_ctrl_set(0.0, 0.0);
      }
    }

    if( th_ref < 0 ){

      vel_ref_l = vel_ref * ( R + D_TIRE/2.) / R;
      vel_ref_r = vel_ref * ( R - D_TIRE/2.) /R;

      //Serial.print("vel_ref_l =");
      //Serial.println(vel_ref_l);
      //Serial.print("vel_ref_r =");
      //Serial.println(vel_ref_r);

      d_l_ref = th_curr * PI /180. * ( R + D_TIRE/2.);
      d_r_ref = th_curr * PI /180. * ( R - D_TIRE/2.);
      er_l = d_l - d_l_ref;
      er_r = d_r - d_r_ref;
      //Serial.print("er_l =");
      //Serial.print(er_l);
      //Serial.print("  er_r =");
      //Serial.println(er_r);

      vel_mod_l = Kc_p * er_l + Kc_i * er_l_sum  + Kc_d  * er_l_d ;
      vel_mod_r = Kc_p * er_r + Kc_i * er_r_sum  + Kc_d  * er_r_d ;
      //vel_mod_l = 0;
      //vel_mod_r = 0;

      vel_ctrl_set((vel_ref_l - vel_mod_l), (vel_ref_r - vel_mod_r));

      if ( th_curr >= -th_ref ) {
        run_state = STP;
        //vel_ctrl_set(0.0, 0.0);
      }
    }
  
    break;

  case LTC:
    //Serial.println("LTC !!");
    light_active();

    // 直進距離
    dist_curr  = (d_l + d_r) / 2.0;
    speed_curr = (v_l + v_r) / 2.0 * sign;

    /*
    if(dist_curr >= dist_ref) {
      run_state = STP;
      return;
    }
    */
    
    // 減速率
    ratio = sign * (dist_ref - dist_curr) / dist_vel_down;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    if (speed_ref == 0.0 || dist_ref == 0.0 || ratio == 0.0) {
      run_state = STP;
      vel_ctrl_set(0.0, 0.0);
      light_inactive();
    } 
    else {
      // 減速の実行
      vel_ref = sign * speed_ref * ratio;

      // 左右のタイヤの距離差の補正
      /*
      er = d_l - d_r;
      er_sum += er;
      er_d = er - er_prev;
      vel_mod = Ks_p * er + Ks_i * er_sum  + Ks_d  * er_d ;
      er_prev = er;
      */
      //vel_ref = speed_ref;  
      vel_mod = light_mod();

      vel_ctrl_set((vel_ref - vel_mod), (vel_ref + vel_mod));
    }

    break;

  case VEL:
    vel_ctrl_set( speed_l , speed_r);
    break;
    
  case VLT:
    light_active();
    vel_ref = speed_l;
    vel_mod = light_mod();
    vel_ctrl_set((vel_ref - vel_mod), (vel_ref + vel_mod));
    break;

  }

}

void run_ctrl_set(run_state_t state, double speed, double dist) {

  run_state = state;  
  //vel_ctrl_reset();
  
  if( run_state == STP) vel_ctrl_reset();
  
  if( run_state == STR || run_state == ROT || run_state == CIR || run_state == LTC ){
    speed_ref = abs(speed);
    dist_ref = dist;
    vel_ctrl_reset();
  }
  if( run_state == VEL || state == VLT ){
    speed_l = speed;
    speed_r = dist;
    //Serial.print("speed=");
    //Serial.println(speed);
  }
  
  er = 0;
  er_prev = 0;
  er_sum = 0;
}

void run_ctrl_get(run_state_t *state, double *speed, double *dist) {
  *state = run_state;
  *speed = speed_curr;
  *dist = dist_curr;
}
