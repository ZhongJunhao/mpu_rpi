#ifndef _ZJH_FIFO_
#define _ZJH_FIFO_

#include <linux/types.h>

#define _ZJH_FIFO_MAX_ 20

struct acc_stat{
	int16_t x,y,z;
};

struct gyro_stat{
	int16_t x,y,z;
} ;


struct mpu_stat{
	struct acc_stat acc_stat;
	int16_t temperature ; 
	struct gyro_stat gyro_stat;
	uint32_t  jiff;
};

#define MPU_DATA_SIZE 	(sizeof(struct acc_stat) + sizeof(struct gyro_stat) + 2)

#define STAT_SIZE   (sizeof(struct acc_stat) + sizeof(struct gyro_stat) + sizeof(int16_t))

struct que{
	uint8_t tail,front;
	struct mpu_stat  queue[_ZJH_FIFO_MAX_];
};

typedef uint8_t op_fun_t(uint8_t * );
#define FIFO_RT_OK   0
#define FIFO_RT_FAIL -1
	
extern void zjh_fifo_init(struct que* que);
extern uint8_t zjh_fifo_put(struct que*que , op_fun_t op);
extern uint8_t zjh_fifo_get(struct que*que , op_fun_t op);


#endif
