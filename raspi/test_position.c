#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

int main(){
	int dist[]     = {300, 300, 300, 300};
	int rot[]      = {90, 90, 90, 90};
	int cor_data[] = {50, 50, 50, 50};
	
	int i;
	double x, y, dir;
	char area;
	
	// 初期位置
	position_set_coord(45, 45, 0);
	
	// 巡回動作中
	while(1){
		for(i = 0; i < 4; i++){
			// 直進走行
			position_straight(dist[i]);
			position_get_coord(&x, &y, &dir);
			area = position_get_area();
			printf("x = %4d, y = %4d, dir = %3d, area = %c\n", (int)x, (int)y, (int)dir, area);
			
			// 正体補正
			position_correct(cor_data[i]);
			position_get_coord(&x, &y, &dir);
			area = position_get_area();
			printf("x = %4d, y = %4d, dir = %3d, area = %c\n", (int)x, (int)y, (int)dir, area);
			
			// 回転走行
			position_rotate(rot[i]);
			position_get_coord(&x, &y, &dir);
			area = position_get_area();
			printf("x = %4d, y = %4d, dir = %3d, area = %c\n", (int)x, (int)y, (int)dir, area);
			
			usleep(500 * 1000);
		}
	}
	
	return 0;
}
