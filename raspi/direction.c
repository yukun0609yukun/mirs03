#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "direction.h"
#include "request.h"
#include "uss.h"

int direction_correct(int dist){
	// 共通パラメータ
	const int count_max = 15; // 試行回数
	const int dist_uss  = 17; // センサ間距離[cm]
	const int speed_str = 15; // 直進速度[cm/s]
	const int speed_rot =  5; // 回転速度[deg/s]
	
	// 距離補正用パラメータ
	const int param1    = 25; // 角度の修正を優先するセンサ測定値の差の閾値[cm]
	const int param2    =  2; // 距離補正成功とする距離の誤差の閾値[cm]
	const int dist_max  = 50; // 距離補正時の直進距離の上限[cm]
	
	// 角度補正用パラメータ
	const int param3    =  1; // 角度補正成功とするセンサ測定値の差の閾値[cm]
	const int angle_max =  5; // 角度補正時の回転角度の上限[deg]
	
	int count = 0;
	int uss_l, uss_r, uss_diff, dist_curr, dist_diff, angle_curr;
	int run_speed, run_dist;
	run_state_t run_state;
	correct_state_t correct_state;
	
	// 指定距離が0以下の場合は距離補正をスキップする
	if(dist > 0){
		#ifdef __DIRECTION_DEBUG__
		printf("start to correct distance[%d]\n", dist);
		#endif
		
		correct_state = DIST;
	}else{
		#ifdef __DIRECTION_DEBUG__
		printf("start to correct direction\n");
		#endif
		
		correct_state = DIR;
	}
	
	while(1){
		// 超音波測定
		//uss_l = uss_get_l();
		//uss_r = uss_get_r();
		uss_l = uss_get_bl();
		uss_r = uss_get_br();
		
		// 超音波測定値が規定範囲外の場合-1に変更
		//if(uss_l > 300) uss_l = -1;
		//if(uss_r > 300) uss_r = -1;
		
		// 測定値から距離と角度を計算
		uss_diff   = uss_l - uss_r;
		dist_curr  = (uss_l + uss_r) / 2.0;
		dist_diff  = dist - dist_curr;
		angle_curr = atan2(uss_diff, dist_uss) * 180.0 / M_PI;
		
		#ifdef __DIRECTION_DEBUG__
		if(uss_l != -1 && uss_r != -1){
			printf("count[%2d] : dist = %3d, angle = %3d (l = %3d, r = %3d)\n", count, dist_curr, angle_curr, uss_l, uss_r);
		}else{
			printf("count[%2d] : dist = ???, angle = ??? (l = %3d, r = %3d)\n", count, uss_l, uss_r);
		}
		#endif
		
		switch(correct_state){
		case DIST:
			// 両センサが無効→前進
			if(uss_l == -1 && uss_r == -1){
				request_set_runmode(STR, speed_str, 20);
			}
			
			// 右センサのみ無効、または右に対して左の測定値が大きすぎる→左回転
			else if((uss_l != -1 && uss_r == -1) || (uss_diff > param1)){
				request_set_runmode(ROT, speed_rot, 10);
			}
			
			// 左センサのみ無効、または右に対して左の測定値が大きすぎる→右回転
			else if((uss_l == -1 && uss_r != -1) || (uss_diff < -param1)){
				request_set_runmode(ROT, speed_rot, -10);
			}
			
			// それ以外の場合→距離補正
			else{
				// 目標距離との誤差が閾値以内なら角度補正に移行
				if(abs(dist_diff) <= param2){
					#ifdef __DIRECTION_DEBUG__
					printf("finished to correct distance\n");
					printf("start to correct direction\n");
					#endif
					
					correct_state = DIR;
					usleep(1000);
				}else{
					if(dist_diff >  dist_max) dist_diff =  dist_max;
					if(dist_diff < -dist_max) dist_diff = -dist_max;
					request_set_runmode(STR, speed_str, -dist_diff);
				}
			}
			break;
			
		case DIR:
			// 両センサが有効の場合のみ角度補正を行う
			if(uss_l != -1 && uss_r != -1){
				// 車体角度が閾値以内なら正対補正終了
				if(abs(uss_diff) <= param3){
					#ifdef __DIRECTION_DEBUG__
					printf("finished to correct direction\n");
					#endif
					
					return 0;
				}else{
					if(angle_curr >  angle_max) angle_curr =  angle_max;
					if(angle_curr < -angle_max) angle_curr = -angle_max;
					request_set_runmode(ROT, speed_rot, -angle_curr);
				}
			}
			break;
		}
		
		usleep(1000);
		
		// 機体が停止するまで待機
		while(1){
			request_get_runmode(&run_state, &run_speed, &run_dist);
			if(run_state == STP) break;
			usleep(1000);
		}
		
		// 規定回数で達成できなければ失敗
		count++;
		if(count > count_max){
			#ifdef __DIRECTION_DEBUG__
			printf("failed to correct\n");
			#endif
			
			return -1;
		}
		
		usleep(1000);
	}
}
