#include "test.h"

int main(void)
{
	struct pos_stat  angle ;

	int fd ;

	int ret = 0;
	
	fd = open("/dev/mpu",O_RDWR);

	if(fd<0)
	{
		printf("open err \n");
		exit(0);
	}

	ret = ioctl(fd,MPU_IOC_GET_ANGLE,&angle);

	if(ret != 0)
	{
		printf("ioctl err\n");
		exit(0);
	}

	printf("%f ,%f,%f\n", angle.x_y/((double)(1<<15)) ,\
					    angle.y_z/((double)(1<<15)) ,\
					    angle.z_x/((double)(1<<15)) );
	
	
	return 0;
	
}
