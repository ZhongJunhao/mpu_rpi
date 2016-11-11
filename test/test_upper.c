#include "test.h"


int send_cmd_get_angle(int sockfd)
{
	struct prot * buff = (struct prot*) malloc(sizeof(uint32_t));
	buff->types = PROT_GET_ANGLE ;
	return send(sockfd , (void *)buff , sizeof(uint32_t) , 0 );	
}

int main(int argc , char ** argv)
{
	int sockfd ;
	struct sockaddr_in rpi_addr;
	char recv_buff[BUFF_MAX];
	int ret = 0;
	int size=0;
	if(argc < 2)
	{
		printf("less argument \n");
		exit(0);
	}

	if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) <0)
	{
		printf("sockfd err\n");
		exit(0);
	}

	
	if(strcmp(argv[1] , "get") == 0)
	{
		
		memset(&rpi_addr , 0 , sizeof(struct sockaddr_in));
		rpi_addr.sin_family = AF_INET;
		rpi_addr.sin_port = htons(HOST_PORT);
		inet_pton(AF_INET , argv[2] , &rpi_addr.sin_addr);
		if(connect(sockfd,(struct sockaddr *)&rpi_addr,sizeof(struct sockaddr)) < 0)
		{
			printf("connect err\n");
		}
		ret = send_cmd_get_angle(sockfd);
		if(ret <=0)
		{
			printf("send err \n");
			exit(0);
		}
		ret = recv(sockfd , recv_buff , BUFF_MAX , 0  );

		if(ret <=0)
		{
			printf("recv err\n");
			exit(0);
		}
		printf("angle : %d\n",*(int32_t *)(((struct prot * )recv_buff)->data));
		
	}
	else 
	{
		printf("unkown operation \n");
		exit(0);
	}
	close(sockfd);
	return 0;
}
