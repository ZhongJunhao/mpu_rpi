#ifndef _CALC_ANGLE_H_
#define _CALC_ANGLE_H_

#include <linux/types.h>
#include <linux/jiffies.h>
#include "fifo.h"

#define NUM_SIZE  125


//fixed-point scal of atan  = 180/pi * (1<<15)
#define ATAN_SCAL   1877468u

#define LOOP_N   100

#define LAG_NUM 	0xffff

#define Q_ANGLE		1024

#define Q_BIAS		2000

#define R_MEASURE	512

#define ATAN_TO_GYO(x)		((x)>>0)


#define ANGLE_0		(0)
#define ANGLE_90	(90*(1<<15))
#define ANGLE_180	(180*(1<<15))
#define ANGLE_270	(270*(1<<15))
#define ANGLE_360	(360*(1<<15))

struct b_num{
	uint16_t d[NUM_SIZE];
	uint8_t n;
};	

struct pos_stat{
	//x0y平面内G投影与x轴的夹角
	int32_t x_y;
	//y0z平面内G投影与y轴的夹角
	int32_t y_z;
	//x0z平面内G投影与z轴的夹角
	int32_t z_x;
};

struct kalman_operator{
	struct pos_stat angle;
	struct pos_stat bias;
	int32_t p[2][2];//error covariance matrix 
//	uint32_t pre_time; // use for calculate delta time 
//	int32_t pre_rate; //last rate of gyro measure 
//	int32_t pre_y;
};

#define MAX(a,b)  ((a)>(b) ? (a) : (b))

int init_calc_angle(void);

void exit_calc_angle(void );

extern void calc_angle_by_acc(struct acc_stat * acc ,struct pos_stat *re);

extern uint32_t calc_angle_acc(int32_t x,int32_t y);

extern void init_kalman(struct kalman_operator * op , struct acc_stat *acc);

extern void flush_kalman(struct kalman_operator *op , struct acc_stat  * acc ,struct gyro_stat * gyro);
#endif
