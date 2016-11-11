#include "test.h"

void send_angle(int sockfd,int mpu_fd )
{
	struct prot * buff = (struct prot *)malloc(sizeof(uint32_t )+sizeof(struct pos_stat));
	int ret = 0;
	struct pos_stat angle ;
	
	ioctl(mpu_fd,MPU_IOC_GET_ANGLE,&angle);

	buff->types = PROT_RET_ANGLE;
	memcpy(buff->data,&angle , sizeof(struct pos_stat ));

	ret = send(sockfd,(void *)buff,sizeof(uint32_t )+sizeof(struct pos_stat) ,0);
	
	if(ret <= 0)
	{	
		printf("send err\n");
	}
	
	
}


int main(int argc,char ** argv)
{
	int sockfd ; 
	int sockclient ;
	int mpu_fd;
	int ret = 0;
	int temp_size = 0;
	int recv_buff[BUFF_MAX]; 
	struct sockaddr_in uper_addr;
	struct sockaddr_in  host_addr;
	struct prot * prt =NULL;
	
	if(argc < 1)
	{
		printf("need arg \n");
		exit(0);
	}
	
	if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) <0)
	{
		printf("sockfd err\n");
		exit(0);
	}

	
	memset(&host_addr , 0 , sizeof(struct sockaddr_in));
	host_addr.sin_family = AF_INET;
	host_addr.sin_port = htons(HOST_PORT);
	host_addr.sin_addr.s_addr = INADDR_ANY;
	
	if(bind(sockfd,(struct sockaddr *)&host_addr,sizeof(host_addr))<0)
	{
		printf("bind err \n");
		exit(0);
	}

	if(listen(sockfd,5)<0)
	{
		printf("listen err\n");
		exit(0);
	}

	sockclient = accept(sockfd ,(struct sockaddr *) &uper_addr ,	(socklen_t *) &temp_size );

	if(sockclient < 0)
	{
		printf("accept err\n");
		exit(0);
	}
	
	mpu_fd = open(argv[1],O_RDWR);
	if(mpu_fd < 0)
	{
		printf("open %s err",argv[1]);
		exit(0);
	}

	
	
	while(1)
	{		
		memset(recv_buff,0,BUFF_MAX);
		if(recv(sockclient , recv_buff , BUFF_MAX , 0)<=0)
		{
			printf("recv <0 \n");
			break;
		}
		
		prt = (struct prot *)recv_buff;
		switch(prt->types)
		{
			case PROT_GET_ANGLE :
				send_angle(sockclient , mpu_fd);
				printf("send angle \n");
				break;
			default:
				printf("unkwon prot types\n");
		}
		
	}
	close(mpu_fd);
	close(sockfd);
	close(sockclient);
	printf("main end\n");
	return 0;
}

