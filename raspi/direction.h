#ifndef __DIRECTION__
#define __DIRECTION__

#define __DIRECTION_DEBUG__

typedef enum{
	DIST = 0,
	DIR
} correct_state_t;

extern int direction_correct(int dist);

#endif
