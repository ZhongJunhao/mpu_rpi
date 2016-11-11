#include "fifo.h"

void zjh_fifo_init(struct que* que)
{
	que->front = 0;
	que->tail = 0;
}

inline uint8_t zjh_fifo_full(struct que* que)
{
	return ((que->tail+1)%_ZJH_FIFO_MAX_) == que->front;
}

inline uint8_t zjh_fifo_empty(struct que*que)
{
	return que->front== que->tail;
}

uint8_t zjh_fifo_put(struct que* que ,op_fun_t op)
{
	if( zjh_fifo_full(que) )
	{
		return -1;
	}
	
	if(op((uint8_t *)&que->queue[que->tail]) == FIFO_RT_OK)
	{
		que->tail = (que->tail+1)%_ZJH_FIFO_MAX_;
		return 0;
	}
	return -1;
}

uint8_t zjh_fifo_get(struct que* que , op_fun_t op)
{
	if(zjh_fifo_empty( que))
	{
		return -1;
	}
	if(op((uint8_t *)&que->queue[que->front]) == FIFO_RT_OK )
	{
		que->front = (que->front+1)%_ZJH_FIFO_MAX_;
		return 0;
	}
	return -1;
}

