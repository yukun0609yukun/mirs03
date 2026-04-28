#include <stdio.h>
#include <math.h>
#include "position.h"

// マップ右下を原点とするして、上にx軸、左にy軸をとる

// フィールドの大きさ[pixel]
static const int xl = 400;
static const int yl = 400;

// 座標データ[cm]と角度データ[deg]
static double x   = 0.0;
static double y   = 0.0;
static double dir = 0.0;

void position_set_coord(double x_s, double y_s, double dir_s){
	x = x_s;
	y = y_s;
	dir = dir_s;
}

void position_straight(double dist){
	x += dist * cos(dir * M_PI / 180.0);
	y += dist * sin(dir * M_PI / 180.0);
}

void position_rotate(double angle){
	dir += angle; 
	
	// dir の値が 0～359に入るように修正
	if(dir >= 360.0) dir -= 360.0;
	if(dir <    0.0) dir += 360.0;
}

void position_correct(double dist){
	// 上
	if(dir >= 315.0 || dir <  45.0){
		x = xl - dist;
		dir = 0.0;
	}
	
	// 左
	if(dir >=  45.0 && dir < 135.0){
		y = yl - dist;
		dir = 90.0;
	}
	
	// 下
	if(dir >= 135.0 && dir < 225.0){
		x = dist;
		dir = 180.0;
	}
	
	// 右
	if(dir >= 225.0 && dir < 315.0){
		y = dist;
		dir = 270.0;
	}
}

void position_get_coord(double *x_g, double *y_g, double *dir_g){
	*x_g = x;
	*y_g = y;
	*dir_g = dir;
}

char position_get_area(){
	// エリアの定義をここに書く
	if(x <= xl/2 && y <= yl/2) return 'A';
	if(x >  xl/2 && y <= yl/2) return 'B';
	if(x >  xl/2 && y >  yl/2) return 'C';
	if(x <= xl/2 && y >  yl/2) return 'D';
	
	return '?';
}
