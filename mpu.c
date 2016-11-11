#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <mach/platform.h>
#include "mpu6050.h"
#include "fifo.h"
#include "calc_angle.h"

/* BSC register offsets */
#define BSC_C			0x00
#define BSC_S			0x04
#define BSC_DLEN		0x08
#define BSC_A			0x0c
#define BSC_FIFO		       0x10
#define BSC_DIV			0x14
#define BSC_DEL			0x18
#define BSC_CLKT		       0x1c

/* Bitfields in BSC_C */
#define BSC_C_I2CEN		(1<<15)
#define BSC_C_INTR		(1<<10)
#define BSC_C_INTT		(1<<9)
#define BSC_C_INTD		(1<<8)
#define BSC_C_ST		(1<<7)
#define BSC_C_CLEAR_1		(1<<5)
#define BSC_C_CLEAR_2		(1<<4)
#define BSC_C_READ		(1)

/* Bitfields in BSC_S */
#define BSC_S_CLKT		(1<<9)
#define BSC_S_ERR		(1<<8)
#define BSC_S_FAIL		(BSC_S_CLKT | BSC_S_ERR) 
#define BSC_S_RXF		(1<<7)
#define BSC_S_TXE		(1<<6)
#define BSC_S_RXD		(1<<5)
#define BSC_S_TXD		(1<<4)
#define BSC_S_RXR		(1<<3)
#define BSC_S_TXW		(1<<2)
#define BSC_S_DONE		(1<<1)
#define BSC_S_TA		(1<<0)
#define CLK_IN_ID		"bcm2708_i2c.1"

#define I2C_SPEED      400000
#define DEV_NAME      "zjh_mpu"
#define WAIT_TIME      150
#define DELTA_TIME     (msecs_to_jiffies(90))

#define MPU_IOC_MAGIC 			'M'
#define MPU_IOC_GET_ANGLE  	_IOR(MPU_IOC_MAGIC,1,int32_t)
#define MPU_IOC_MAXNR  			3

#ifdef _ZJH_DBG_ON_
#define ZJH_DBG(fmt,x...)	printk(KERN_DEBUG "zjh:" fmt , ##x)
#else
#define ZJH_DBG(fmt,x...)
#endif

struct i2c_pack{
	bool  succ;
	uint8_t  sla;
	uint8_t  flag;  
	uint8_t  sub_addr;
	uint8_t  d_len;
	uint8_t  pos;
	uint8_t *data;
};

#define I2C_FLG_NONE		         0x00  // NULL
#define I2C_FLG_WRITE			  0x01  // write 
#define I2C_FLG_READ			  0x02  //read directely after restar
#define I2C_FLG_ND_RESTART		  0x04   //send register address 

static struct i2c_pack default_pack = {
			 .succ = false,
			 .sla = MPU_SLAVER_ADDR,
			 .flag = I2C_FLG_ND_RESTART,
			 .sub_addr = MPU_DATA_ADDR,
			 .d_len = sizeof(struct mpu_stat),
			 .pos = 0,
			 .data = NULL,
};

#define I2C_RESET_DEF_PACK(pack) do{\
			 pack.succ = false;\
			 pack.sla = MPU_SLAVER_ADDR;\
			 pack.flag = I2C_FLG_ND_RESTART;\
			 pack.sub_addr = MPU_DATA_ADDR;\
			 pack.d_len = MPU_DATA_SIZE;\
			 pack.pos = 0;\
			 pack.data = NULL;\
			 }while(0)



struct zjh_dev{

	dev_t dev_no;

	struct cdev  cdev;
	
	struct clk * clk_in;
	
	void * __iomem base_addr;

	struct completion comp ; 
	
	struct i2c_pack *pack;

	struct timer_list timer;

	struct que * deal_que;

	struct tasklet_struct tasklet ;

	struct kalman_operator * kalman;

	uint8_t buff[16];
	uint8_t i;
	uint8_t wait_tasklet;
};

atomic_t dev_avli = ATOMIC_INIT(1);
static struct zjh_dev * zjh_dev;

//file operations ..
static int mpu_open(struct inode *nd,struct file * fp);
static int mpu_release(struct inode *nd,struct file * fp);
static long mpu_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = mpu_open,
	.release = mpu_release,
	.unlocked_ioctl = mpu_ioctl,
};

static inline void take_buff(struct mpu_stat * stat , uint8_t *buff)
{
	stat->acc_stat.x = ((int16_t)buff[0])<<8;
	stat->acc_stat.x |= buff[1];
	stat->acc_stat.y = ((int16_t)buff[2])<<8;
	stat->acc_stat.y |= buff[3];
	stat->acc_stat.z = ((int16_t)buff[4])<<8;
	stat->acc_stat.z |= buff[5];

	stat->temperature = ((int16_t)buff[6])<<8;
	stat->temperature |= buff[7];

	stat->gyro_stat.x = ((int16_t)buff[8])<<8;
	stat->gyro_stat.x |= buff[9];
	stat->gyro_stat.y = ((int16_t)buff[10])<<8;
	stat->gyro_stat.y |= buff[11];
	stat->gyro_stat.z = ((int16_t)buff[12])<<8;
	stat->gyro_stat.z |= buff[13];
}

static uint8_t op_put_to_fifo(uint8_t * buff)
{
//	uint8_t i = 0;
//	void * base_addr = zjh_dev->base_addr;
	struct mpu_stat * stat = (struct mpu_stat *)buff;
/*	while( i < MPU_DATA_SIZE)
	{
		buff[i] = (zjh_dev->buff[i]);
		i++;
	}
	*/
	take_buff(stat,zjh_dev->buff);
	
	stat->jiff = jiffies ; 
	return FIFO_RT_OK;
}

static uint8_t op_deal_fifo(uint8_t *buff)
{
	struct mpu_stat * stat = (struct mpu_stat *)buff;
	
/*	ZJH_DBG("%d %d \n" , stat->acc_stat.x , stat->acc_stat.y);


	ZJH_DBG("gyo: %d , %d ,%d\n" , stat->gyro_stat.x ,\
				stat->gyro_stat.y,stat->gyro_stat.z);

	flush_kalman(zjh_dev->kalman ,stat->acc_stat.x ,\
			stat->acc_stat.y,stat->gyro_stat.z,stat->jiff);
*/
	flush_kalman(zjh_dev->kalman, &stat->acc_stat , &stat->gyro_stat);
//	ZJH_DBG("angle :%d \n", zjh_dev->kalman->angle);
	
	return FIFO_RT_OK;
}

static inline void zjh_i2c_init_pin(void)
{
	void * gpio = ioremap(GPIO_BASE,4);
	uint32_t temp ; 
	temp = readl(gpio);
	temp &= ~(0x3f<<6); // clean bit[6:11] FSEL2|FSEL3
	temp |= (0x24<<6); //set bit[6:11] 100100 
	writel(temp , gpio);
	iounmap(gpio);
}

static inline void zjh_i2c_init_clk(struct zjh_dev* dev ,uint32_t speed)
{
	uint32_t in ;
	uint32_t div;

	in = clk_get_rate(dev->clk_in);
	div =  in / speed;

	writel( div , dev->base_addr + BSC_DIV );
	
}

static inline void zjh_i2c_reset(void * __iomem base_addr)
{
	writel( (BSC_C_CLEAR_1) , base_addr + BSC_C );
	writel( 0 ,base_addr + BSC_C);
	writel( (BSC_S_CLKT|BSC_S_DONE|BSC_S_ERR) , base_addr + BSC_S );	
	
	
}

static inline void zjh_i2c_put_buff(void * __iomem base_addr ,struct  i2c_pack* pack)
{
//	ZJH_DBG("put : pos :%d\n",pack->pos);
	while( (readl(base_addr + BSC_S) & BSC_S_TXD)  
		    && pack->pos < pack->d_len)
	{
		if(pack->pos == 0)
		{
			writel(pack->sub_addr,base_addr + BSC_FIFO);
		}
		else
		{
			writel(pack->data[pack->pos-1] ,base_addr + BSC_FIFO);
		}
		pack->pos ++;
	}
}

static inline void zjh_i2c_get_buff(void * __iomem base_addr,struct  i2c_pack * pack)
{
//	uint16_t time_out = 0x7fff;
//	while( (readl(base_addr + BSC_S) & BSC_S_TA)  && (time_out--) );
	if(pack->data)
	{
		while( (readl(base_addr + BSC_S) & BSC_S_RXD) 
			   && pack->pos < pack->d_len)
		{
//			ZJH_DBG("get_buff:geting ..  pack->%d  \n",pack->pos);
			pack->data[pack->pos] = readl( base_addr + BSC_FIFO );
			pack->pos ++;
		}
	}
	else
	{
	//	zjh_fifo_put(zjh_dev->deal_que , op_put_to_fifo);
		while( (readl(base_addr + BSC_S) & BSC_S_RXD) 
			   && zjh_dev->i < pack->d_len)
		{
			
//			ZJH_DBG("get_buff2:geting ..  pack->%d  \n",zjh_dev->i  );
			zjh_dev->buff[zjh_dev->i] = readl( base_addr + BSC_FIFO );
			zjh_dev->i  ++;
		}
	}
}

static inline bool zjh_i2c_send_reg_addr(void *__iomem base_addr ,struct i2c_pack * pack)
{

	if( !(readl(base_addr + BSC_S) & BSC_S_TXD) 
	    || pack->flag != I2C_FLG_ND_RESTART)
	{
		return false;
	}
	writel(pack->sub_addr , base_addr + BSC_FIFO );
	return true;
}

static void zjh_i2c_start(void *__iomem base_addr , struct i2c_pack * pack)
{	
	//default c : wrtie model
	uint32_t  c =  BSC_C_I2CEN | BSC_C_INTD | BSC_C_ST |  BSC_C_INTT;
	uint8_t    sla = (pack->sla)>>1;
	uint8_t    d_len = pack->d_len;
//	ZJH_DBG("START \n");
	if(pack->flag == I2C_FLG_READ)
	{
		//read c : 
		c = BSC_C_I2CEN | BSC_C_INTD | BSC_C_ST |BSC_C_READ | BSC_C_INTR;
	}
	else if(pack->flag == I2C_FLG_ND_RESTART )
	{
		writel(pack->sub_addr , base_addr + BSC_FIFO);
		d_len = 1;
	}
//	ZJH_DBG("start:sla:0x%x , d_len: %d\n" , sla,d_len);
	writel( sla , base_addr + BSC_A );
	writel( d_len , base_addr + BSC_DLEN );
	writel( c , base_addr + BSC_C );
}

static inline void  finish_trans(struct i2c_pack *pack,bool succ)
{
	//normal transfer
	if(pack->data)
	{
//		ZJH_DBG("finish data\n");
		complete(&zjh_dev->comp);
	}
	else //calc transfer
	{
//		ZJH_DBG("finish data == NULL\n");
		zjh_fifo_put(zjh_dev->deal_que , op_put_to_fifo);
		tasklet_schedule(&zjh_dev->tasklet);
	}
	pack->succ = succ;
	zjh_dev->pack = NULL;
	zjh_i2c_reset(zjh_dev->base_addr);
}

static int mpu_opp(uint8_t reg_addr,uint8_t flag,uint8_t *buff,uint8_t count)
{
	struct i2c_pack *pack = NULL;
	int ret ; 
	pack = (struct i2c_pack*)kmalloc(sizeof(struct i2c_pack) , GFP_ATOMIC);
	if(pack == NULL)
	{
		ZJH_DBG("MPU send pack ..kmalloc fail \n");
		return -1;
	}
	zjh_i2c_reset(zjh_dev->base_addr);
	pack->succ = false;
	pack->sla = MPU_SLAVER_ADDR;
	pack->flag = flag;
	pack->sub_addr = reg_addr;
	pack->d_len = count;
	pack->data = buff;
	pack->pos = 0;
	zjh_dev->pack = pack;
	zjh_i2c_start(zjh_dev->base_addr , pack);
	
	ret = wait_for_completion_timeout( &zjh_dev->comp,\
					msecs_to_jiffies(WAIT_TIME) );
	if(ret == 0)
	{
		ZJH_DBG("wait i2c to send time out\n");
		kfree(pack);
		zjh_dev->pack=NULL;
		return -1;
	}

	if(pack->succ)
	{
//		ZJH_DBG("pack tran succ\n");
		ret = 0;
	}
	else
	{
		ZJH_DBG("pack tran err\n");
		ret = -1;
	}
	kfree(pack);
	zjh_dev->pack=NULL;
	return ret ;
	
}



#define MPU_SEND(reg,buff,count)     mpu_opp((reg),I2C_FLG_WRITE,(buff),(count)+1)
#define MPU_READ(reg,buff,count)	    mpu_opp((reg),I2C_FLG_ND_RESTART,(buff),(count))

static int mpu_init(void)
{
	static uint8_t init_cmd1[] = {0x07 ,      0x1A   ,    0x10      , 0x10    };
					            //div  |    config  |gyo_config | acc_conf
	static uint8_t init_cmd_pwr_on = 0;
								
	if(MPU_SEND(PWR_MGMT_1,&init_cmd_pwr_on,1))
	{
		ZJH_DBG("open : send pwr on err\n");
		atomic_inc(&dev_avli);
		return -1;
	}
	
	if(MPU_SEND(CMD_ADDR_1,init_cmd1,sizeof(init_cmd1)))
	{
		ZJH_DBG("open : send init cmd 1 err\n");
		atomic_inc(&dev_avli);
		return -1;
	}
	return 0;
}

static irqreturn_t zjh_i2c_irq(int irq , void * dev_id)
{
	struct zjh_dev * dev = zjh_dev;
	struct i2c_pack * pack = dev->pack;
	void * __iomem base_addr = dev->base_addr;
	uint32_t s = readl(dev->base_addr + BSC_S);
	bool handle = true;
//	ZJH_DBG("base_addr: 0x%x  stat:  0x%x\n" ,(unsigned int )base_addr,(unsigned int) s);

	if(pack == NULL) 
	{
		ZJH_DBG("irq:NULL pack\n");
		handle = true;
	}
	else if( s & BSC_S_FAIL )
	{
		finish_trans(pack, false);
		handle = true;
	}
	else if( s & BSC_S_DONE)
	{	
		//clear done bit 
		writel(BSC_S_DONE,base_addr+BSC_S);
		 if( pack->flag == I2C_FLG_READ ) //read done
		{
//			ZJH_DBG("irq:d_read\n");
			zjh_i2c_get_buff( base_addr , pack );
			finish_trans(pack,true);
		}		
		else if(pack->flag == I2C_FLG_ND_RESTART)
			//register addr write done restart to read 
		{
//			ZJH_DBG("irq:d_restart\n");
			pack->flag = I2C_FLG_READ;
			zjh_i2c_start( base_addr, pack);
		}
		else //write done 
		{
//			ZJH_DBG("irq:d_write\n");
			finish_trans(pack,true);
		}
		
		
	}
	else if( s & BSC_S_TXW )
	{
		if(pack->flag == I2C_FLG_ND_RESTART)
		{
//			ZJH_DBG("irq:TXW restart\n");
		}
		else 
		{
			zjh_i2c_put_buff(base_addr ,  pack);
		}
		writel(BSC_S_TXW,base_addr+BSC_S);
	}
	else if( s & BSC_S_RXR)
	{
		zjh_i2c_get_buff(base_addr , pack);
	}
	else
	{
		ZJH_DBG("base_addr: 0x%x  stat:  0x%x\n" ,(unsigned int )base_addr,(unsigned int) s);
		ZJH_DBG("not handle!\n");
		handle = true;
	}
	return handle ? IRQ_HANDLED : IRQ_NONE ;
}


static void mpu_timer(unsigned long arg)
{
	//int timeout = 0xff;
	void *__iomem base_addr = zjh_dev->base_addr;
	//uint32_t  c =  BSC_C_I2CEN | BSC_C_INTD | BSC_C_ST |  BSC_C_INTT;
	arg = arg;
//	ZJH_DBG("timer \n");
	if(zjh_dev->pack!=NULL)
	{
		ZJH_DBG("breaking some i2c !!\n");
		zjh_dev->pack = NULL;
		return ;
	}
	zjh_i2c_reset(base_addr);
	I2C_RESET_DEF_PACK(default_pack);
	zjh_dev->pack = &default_pack;
	zjh_dev->i = 0;
	zjh_i2c_start(base_addr , &default_pack);
	zjh_dev->timer.expires = jiffies + DELTA_TIME ;
	add_timer(&zjh_dev->timer);
	//pending bsc free
	//while((readl(BSC_S) &BSC_S_TA ) && timeout --) ;

}

static void mpu_tasklet(unsigned long arg)
{
	while(!zjh_fifo_get(zjh_dev->deal_que , op_deal_fifo));
	if(zjh_dev->wait_tasklet)
	{
		zjh_dev->wait_tasklet = 0;
		complete(&zjh_dev->comp);
	}
}

static int mpu_open(struct inode *nd,struct file *file)
{
	
//	int ret = 0;
//	static uint8_t gyo[6] = {0};

	ZJH_DBG("open\n");
	if(!atomic_dec_and_test(&dev_avli))
	{
		atomic_inc(&dev_avli);
		return -EBUSY;
	}	
	
	return 0;
	
	return -EBUSY;
}

static int mpu_release(struct inode * inode, struct file * fp)
{
	atomic_inc(&dev_avli);
	return 0;
}

static long mpu_ioctl(struct file *fp, unsigned int  cmd, unsigned long arg)
{	
	int err  = 0;;
	
	
	if(_IOC_TYPE(cmd) != MPU_IOC_MAGIC)
	{
		ZJH_DBG("ioctl err cmd\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > MPU_IOC_MAXNR)
	{
		ZJH_DBG("ioctl err cmd\n");
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ )
	{
		err = !access_ok(VERIFY_READ , (void * __user) arg , _IOC_SIZE(cmd));
	}
	if(err) 
	{
		ZJH_DBG("ioctl arg err\n");
		return -EFAULT;
	}

	switch(cmd)
	{
		case MPU_IOC_GET_ANGLE :
			zjh_dev->wait_tasklet = 1;
			tasklet_schedule(&zjh_dev->tasklet);
			err = wait_for_completion_timeout( &zjh_dev->comp,\
					msecs_to_jiffies(WAIT_TIME) );
			if(err == 0)
			{
				ZJH_DBG("wait for tasklet timeout\n");
			}
			err = copy_to_user((struct pos_stat * __user)arg,&zjh_dev->kalman->angle,sizeof(struct pos_stat));
		break;
		default:
			return -ENOTTY;
	}
	return err;
}

static int __init mpu_module_init(void)
{
	int ret = 0;
	uint8_t who = 0;
	
	struct mpu_stat stat;
	
	zjh_dev = (struct zjh_dev *)kmalloc(sizeof(struct zjh_dev),GFP_ATOMIC);
	if(zjh_dev == NULL)
	{
		ZJH_DBG("malloc\n");
		return -1;
	}
	zjh_dev->deal_que = kmalloc(sizeof(struct que),GFP_ATOMIC);
	zjh_fifo_init(zjh_dev->deal_que);
	
	if(zjh_dev->deal_que == NULL)
	{
		ZJH_DBG("mallo que\n");
		goto malloc_err;
	}
	ret = alloc_chrdev_region(&zjh_dev->dev_no , 0 , 1 , DEV_NAME);
	if(ret < 0)
	{
		ZJH_DBG("alloc char dev fail \n");
		goto alloc_dev;
	}
	ZJH_DBG("marjor : %d minor: %d \n",MAJOR(zjh_dev->dev_no) ,MINOR(zjh_dev->dev_no));

	cdev_init(&zjh_dev->cdev , &fops);
	zjh_dev->cdev.owner = THIS_MODULE;
	zjh_dev->cdev.ops = &fops;

	ret = cdev_add(&zjh_dev->cdev , zjh_dev->dev_no , 1 );
	if(ret)
	{
		ZJH_DBG("add cdev fail \n" );
		goto add_dev_fail;
	}
	
	
	if(!request_mem_region(BSC1_BASE , SZ_256-1 , DEV_NAME))
	{
		ZJH_DBG("init:request memery \n");
		goto mem_fail;
	}
	zjh_dev->base_addr = ioremap(BSC1_BASE,SZ_256-1);
	if(zjh_dev->base_addr == NULL)
	{
		ZJH_DBG("init:ioremap \n");
		goto io_map_fail;
	}
	ZJH_DBG("base addr: 0x%x \n",(unsigned int)zjh_dev->base_addr );
	ret = request_irq( INTERRUPT_I2C , zjh_i2c_irq, 0, 
					DEV_NAME,zjh_dev);
	if(ret)
	{
		ZJH_DBG("init:irq request \n");
		goto irq_request_fail;
	}

	init_completion(&zjh_dev->comp);
	tasklet_init(&zjh_dev->tasklet , mpu_tasklet , 0);
	setup_timer(&zjh_dev->timer , mpu_timer , 0);
	
	
	zjh_dev->clk_in = clk_get_sys(CLK_IN_ID,NULL);

	zjh_i2c_init_clk(zjh_dev, I2C_SPEED);
	zjh_i2c_init_pin();

	
	ret = mpu_init();

	if(ret)
	{
		ZJH_DBG("init mpu err\n");
		goto init_calc_fail;
	}
	
//	mpu_poll_read(zjh_dev->base_addr,WHO_AM_I,&who,1);
	MPU_READ(WHO_AM_I , &who , 1);
	ZJH_DBG("who am i:0x%x\n",who);
	if(init_calc_angle() == 0)
	{
		ZJH_DBG("malloc calc err\n");
		goto init_calc_fail;
	}
	zjh_dev->kalman = (struct kalman_operator *)kmalloc (sizeof(struct kalman_operator ) ,
													GFP_ATOMIC);
	if(zjh_dev->kalman == NULL)
	{
		goto mpu_init_fail;
	}
	MPU_READ(ACCEL_XOUT_H , zjh_dev->buff , MPU_DATA_SIZE);

	take_buff(&stat , zjh_dev->buff);

	ZJH_DBG("init angle\n");
	init_kalman(zjh_dev->kalman , &stat.acc_stat);
	
	ZJH_DBG("MPU_STAT size:%d",MPU_DATA_SIZE);
	ZJH_DBG("jiffies:%d , DELTA_TIME:%d\n",(int32_t)jiffies , (int32_t)DELTA_TIME);
	zjh_dev->timer.expires = jiffies + DELTA_TIME ;
	add_timer(&zjh_dev->timer);
	
	return 0;
mpu_init_fail:
	exit_calc_angle();
init_calc_fail:
	
	free_irq(INTERRUPT_I2C,zjh_dev);
irq_request_fail:
	iounmap(zjh_dev->base_addr);
	
io_map_fail:
	release_mem_region(BSC1_BASE, SZ_256-1);
	
mem_fail:
	cdev_del(&zjh_dev->cdev);
add_dev_fail:
	unregister_chrdev_region(zjh_dev->dev_no , 1);
alloc_dev:
	kfree(zjh_dev->deal_que);
malloc_err:
	kfree(zjh_dev);
	return -1;

}

static void __exit mpu_module_exit(void)
{
	del_timer(&zjh_dev->timer);
	free_irq(INTERRUPT_I2C,zjh_dev);
	iounmap(zjh_dev->base_addr);
	release_mem_region(BSC1_BASE , SZ_256-1);
	cdev_del(&zjh_dev->cdev);
	unregister_chrdev_region(zjh_dev->dev_no , 1);
	kfree(zjh_dev->deal_que);
	kfree(zjh_dev);
	exit_calc_angle();	
}

module_init(mpu_module_init);
module_exit(mpu_module_exit);
MODULE_AUTHOR("Zhong Jun hao ");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("zjh's mpu driver");


