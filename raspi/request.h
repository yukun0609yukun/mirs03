#ifndef __REQUEST__
#define __REQUEST__

typedef enum{
	STP = 0,
	STR,
	ROT,
	CIR,
	LTC,
	VEL,
	VLT
} run_state_t;

extern void request_set_runmode(run_state_t state, int speed, int dist);
extern int  request_get_runmode(run_state_t *state, int *speed, int *dist);
extern int  request_get_dist(int *dist_l, int *dist_r);
extern int  request_get_batt(double *volt);

#endif
