#include <linux/slab.h>
#include "calc_angle.h"

#define EXCHANGE(a,b)  do{(a)^= (b); (b)^=(a); (a)^=(b);}while(0)

#define BN_MALLOC()	   ((struct b_num *)kmalloc(sizeof(struct b_num) , GFP_ATOMIC))

#define BN_FREE(x) 	   (kfree((x)))

static struct b_num *mod ,*tmp ;
static struct b_num *c ;
static struct b_num *a,*b;
static struct b_num *n_mul,*n_div;

int init_calc_angle(void)
{
	mod = BN_MALLOC();
	tmp = BN_MALLOC();
	c = BN_MALLOC();
	a = BN_MALLOC();
	b = BN_MALLOC();
	n_mul = BN_MALLOC();
	n_div = BN_MALLOC();
	return (mod!=NULL )&&( tmp!=NULL) &&( c!=NULL) && (a!=NULL)\
			&& (b!=NULL )&& (n_mul!=NULL) && (n_div!=NULL);
}

void exit_calc_angle(void )
{
	BN_FREE(mod);
	BN_FREE(tmp);
	BN_FREE(c);
	BN_FREE(a);
	BN_FREE(b);
	BN_FREE(n_mul);
	BN_FREE(n_div);
}

void  bn_init_by_int(struct b_num *num , uint32_t a)
{
	int i =0;
	num->d[0] = a&0xffff;
	num->d[1] = (a&(0xffff<<16))>>16;
	i=2;
	while(i>0 && num->d[i-1] ==0) 
	{
		i--;
	}
	num->n = i;
}

uint32_t bn_to_int(struct b_num * num)
{
	int i = (num->n > 2)? 2 : num->n;
	uint32_t ret = 0;

	while(i>0)
	{
		ret |= (((uint32_t)num->d[i-1])<<(16*(i-1)));
		i--;
	}
	
	return ret;
}

uint8_t bn_count_d(struct b_num * num)
{
	return num->n;
}

void bn_cp(struct b_num *dst,struct b_num *src)
{
	int i ;
	for(i=0 ; i<src->n ; i++)
	{
		dst->d[i] = src->d[i];
	}
	dst->n = src->n;
}

/*
void bn_shift_left(struct b_num * num ,int n, struct b_num * to)
{
	int i=0;
	int count = bn_count_d(num);
	bn_init_by_int(to,0);

	to->n = (n+count >=NUM_SIZE) ? NUM_SIZE-1 : n+count;
	
	for(i=count-1; i>=0 ; i-- )
	{
		if(i+n >= NUM_SIZE)
		{
			continue;
		}
		to->d[i+n] = num->d[i] ;
	}
}
*/

void bn_shift_left(struct b_num * num)
{
	int i;
	num->n = (num->n == NUM_SIZE) ? NUM_SIZE : num->n+1;
	i=num->n;
	if(i==NUM_SIZE)
	{
		i--;
	}
	for( ; i>0 ; i--)
	{
		num->d[i] = num->d[i-1];
	}
	num->d[i] = 0;
}

void bn_add_short(struct b_num *a , uint16_t b)
{
	uint32_t tmp = b;
	int i = 0;
	for(i=0 ; tmp !=0 && i < (a->n +1) ; i++)
	{
		if(i >= a->n)
		{
			a->d[i] = 0;
			a->n ++;
		}
		tmp = a->d[i] + tmp  ;
		a->d[i] = tmp & 0xffff;
		tmp = tmp>>16;
	}
	
	
}

#define BN_CMP_LESS     -1     //a<b
#define BN_CMP_EQ	      0     //a==b
#define BN_CMP_OVER      1     //a>b

int bn_cmp(struct b_num *a ,struct b_num *b)
{
	int acount = bn_count_d(a);
	int bcount = bn_count_d(b);

	if(acount != bcount)
	{
		return acount>bcount ? 1 : -1;
	}
	acount --;
	while(acount>=0)
	{
		if(a->d[acount] > b->d[acount])
		{
			return 1;

		}

		else if(a->d[acount] < b->d[acount])
		{
			return -1;
		}
		acount --;
	}
	return 0;
}


void bn_sub(struct b_num *a , struct b_num *b , struct b_num *c)
{
	int i =0;
	uint32_t temp = 0; 
	uint32_t carry  = 0;
	uint8_t acount = 0;
	acount = bn_count_d(a);
	
	for(i=0 ; i<acount ; i++)
	{
		if(i<b->n)
		{
			temp = a->d[i] - b->d[i] - carry;
		}
		else
		{
			temp = a->d[i] - carry;
		}
		c->d[i] = temp & 0xffff;
		carry = temp >>31; //signed bit
	}
	c->n = acount;
	while(c->n >0 && c->d[c->n-1] == 0 ) 
	{ 
		c->n--;
	}
}


void bn_mul(struct b_num *a , struct b_num *b , struct b_num *c)
{
	int i,j;
	int in,jn;
	//uint32_t temp = 0;
	uint32_t carry = 0;
	uint8_t acount = bn_count_d(a);
	uint8_t bcount = bn_count_d(b);
	bn_init_by_int(c,0);
	
	c->n = 0;
	in = acount;
	jn = bcount;
	
	
	for(i=0 ; i<in ; i++)
	{
		for(j=carry=0 ; j<jn  || carry != 0; j++,carry >>= 16)
		{
			if(i+j >= NUM_SIZE) 	
			{	
				break;
			}
			if(j<b->n) 
			{
				carry += a->d[i] * b->d[j];
			}
			
			if(i+j >= c->n )
			{
				c->d[ c->n++ ] = carry & 0xffff;
			}
			else
			{
				carry += c->d[i+j]; 
				c->d[i+j] = carry &0xffff;
			}
		}
	}
	while(c->n >0 && c->d[c->n-1] == 0 ) 
	{ 
		c->n--;
	}
	
}

void bn_mul_short(struct b_num * a , uint16_t b , struct  b_num  * c)
{
	uint32_t tmp = 0;
	uint32_t carry = 0;

	int  i ;
	c->n = a->n;
	for(i = 0 ; i< a->n || carry != 0 ; i++)
	{
		if(i >= a->n)
		{
			a->d[i] = 0;
			c->n ++;
		}
		tmp = a->d[i] *b + carry;
		c->d[i] = tmp & 0xffff;
		carry = (tmp )>>16;
	}	
}


#define BN_DIV_ZERO  	-1 
//dive by zero

#define BN_DIV_OK 		0
//ok

void bn_div(struct b_num *a , struct b_num * b ,struct b_num *c ,int  *err)
{
	
	int32_t i,j,k,mid;
	uint8_t acount , bcount;


	
	acount = bn_count_d(a);
	bcount = bn_count_d(b);

	bn_init_by_int(mod , 0);

	*err = BN_DIV_OK;
	
	
	if(bcount == 0)
	{
		*err = BN_DIV_ZERO;
		bn_init_by_int(c,-1);
		return ;
	}

	if(bn_cmp(a,b) == BN_CMP_LESS )
	{
		bn_init_by_int(c,0);
//		printk("div less");
		return ;
	}

	for(i=acount-1 ; i>=0 ; i--)
	{

		bn_shift_left(mod);
		bn_add_short(mod,a->d[i]);
		j = 0;
		k = (1<<16)-1;
	
		while(j<k)
		{
			mid = (j+k+1)/2;
			bn_mul_short(b,(uint16_t)mid,tmp);
			//b*mid > mod
			if(bn_cmp(tmp,mod) == BN_CMP_OVER)
			{
				k = mid -1;
			}
			else
			{
				j = mid;
			}
			
		}
	//	printk("div: j: %d\n",j);
		c->d[i] = j ;
		bn_mul_short(b,(uint16_t)j,tmp);
		bn_sub(mod , tmp , mod);

	}
	c->n =acount;

	while(c->n >0 && c->d[c->n-1] == 0 ) 
	{ 
		c->n--;
	}
//	printk("div c->n : %d",c->n);

}

/*
void bn_div(struct b_num *a , struct b_num *b , struct b_num *c , int *err)
{

	struct b_num tema , temb;
	int acount  = bn_count_d(a);
	int bcount = bn_count_d(b);
	int ccount = acount - bcount;
	uint32_t intb = 0;
	uint32_t y = 0;
	uint32_t tem = 0;
	*err = BN_DIV_OK;
	if(bcount == 0)
	{
		*err = BN_DIV_ZERO;
		bn_init_by_int(c , -1);
		return ;
	}

	tema = *a;
	bn_init_by_int(c,0);
	c->n = ccount +1;
	if(bcount<=2)
	{
		intb = bn_to_int(b);
		if(intb == 0)
		{
			printk("err div 0 \n");
			return ;
		}
		while(ccount>=0)
		{
			tem = (y<<16)+a->d[acount-1];
			c->d[ccount] = tem/intb;
			y = tem%intb;
			ccount --;
			acount --;
		}
	}
	else
	{
		while(ccount >= 0)
		{
			bn_shift_left(b , ccount ,& temb);
			while(bn_cmp(&tema , &temb) != BN_CMP_LESS)
			{
				bn_sub(&tema , &temb , &tema);
				c->d[ccount] ++;
			}
			ccount --;
		}
	}
	
}

*/

uint32_t bn_div_to_int(struct b_num * a,struct b_num *b)
{

	int err;
	uint32_t ret ; 
	if(c == NULL)
	{
		return -1;
	}
	
	bn_div(a,b,c , &err);

	if(err == BN_DIV_OK)
	{
		ret =  bn_to_int(c);
		return ret;
	}
	return -1;
}

void bn_mul_intset(uint32_t * set,int n , struct b_num *res)
{
	
	int i = 0;
	
	
	bn_cp(a,res);
	for(i=0 ; i<n ; i++)
	{
		bn_init_by_int(b , set[i] );
		bn_mul(a , b , res);
		bn_cp(a,res);
	}
	
}

void bn_mul_32_to_64(uint32_t a,uint32_t b,uint32_t * hc,uint32_t * lc)
{
	uint32_t a2 = a&0xffff;
	uint32_t a1 = a>>16;
	uint32_t b2 = b&0xffff;
	uint32_t b1 = b>>16;

	*lc = (a2*b2) + ((a1*b2)&0xffff) + ((a2*b1)&0xffff);
	*hc = (a1*b1) + ((a1*b2)>>16) + ((a2*b1)>>16);
}

void bn_64_div_32(uint32_t ah,uint32_t al,uint32_t b ,uint32_t *ch,uint32_t *cl)
{
	uint32_t mod;
	uint32_t div;
	uint32_t res = 0;
	bool bhbit = 0;
	int i =0;
	
	if(b == 0)
	{
		*ch = -1;
		*cl =  -1;
		return ;
	}

	if(ah == 0)
	{
		*ch = 0;
		*cl = al/b;
		return ;
	}
	
	*ch = ah/b;
	mod = ah%b;
	if(mod==0)
	{
		*cl = al/b;
		return;
	}
	if(b&0x80000000)
	{
		bhbit = (mod>=0x80000000);
		div = b&0x7fffffff;
			
		for(i=0 ; i<32 ; i++)
		{
			res <<=1;
			mod = (mod<<1) | ((al>>(32-i))&1);
			
			if(bhbit)
			{
				res ++ ;
				mod ^= 0x80000000;
				mod -= div;
				bhbit = (mod >= 0x80000000);
			}
			else if(mod >= div)
			{
				res ++;
				mod -= div;
			}
			else
			{
				bhbit = (mod >= 0x80000000);
			}
		}
	}
	else
	{
		for(i=0 ; i<32 ; i++ )
		{
			res >>=1;
			mod = (mod<<1) | ((al>>(31-i))&1);
			if(mod>=b)
			{
				res ++;
				mod -= b;
			}
		}
	}
	*cl = res;
	
}


int32_t bn_a_mul_b_div_c(int32_t a,int32_t b,int32_t c)
{
	int sym = 0;
	int32_t res = 0;
	uint32_t temh,teml,resh,resl;
	
	if(a<0 )
	{
		a = -a;
		sym ++;
	}
	if( b<0 )
	{
		b = -b;
		sym ++;
	}
	if(c<0)
	{
		c = -c;
		sym ++;
	}

	bn_mul_32_to_64(a,b,&temh,&teml);
	bn_64_div_32(temh,teml,c,&resh,&resl);
	
	res = resl;
	if(sym&1)
	{
		res = -res;
	}
	return res;
	
}


//acrtan(x/y)
uint32_t my_atan2(uint32_t x,uint32_t y)
{
	uint32_t n = 0;
	uint32_t x2 = x*x;
	uint32_t y2 = y*y;
	uint32_t x2py2 = x2+y2;
	uint32_t tem = 0;
	uint32_t ans = 0;

	uint32_t div[5] = {x2py2};
	uint32_t d_n = 1;
	uint32_t mul[5] = {x,y,ATAN_SCAL};
	uint32_t m_n = 3;


	if(x == 0)
	{
		return 0;
	}
//	printk("atan2(%d,%d)\n",x,y);


	bn_init_by_int(n_mul , 1);
	bn_init_by_int(n_div , 1);
	
//	unsigned int err = 0;

//	tem = bn_mul_div( mul , m_n , div , d_n );
//	printk("a\n");
	
	bn_mul_intset(mul,m_n,n_mul);
	bn_mul_intset(div,d_n,n_div);

//	printk("b\n");
	
	ans = bn_div_to_int(n_mul,n_div);
	tem = ans;
//	printk("c\n");
//	printk("%d\n",tem);
	
	for(n=1; n<LOOP_N && tem > 10 ; n++)
	{
	//	tem = 2*tem*n*x2 / ((2*n+1)*(y2+x2));
		mul[0] = 2*n;
		mul[1] = x2 ;
	//	mul[3] = tem;
		m_n = 2;
		div[0] = (2*n+1);
		div[1] = x2py2 ;
		d_n = 2;
	//	tem = bn_mul_div(mul , m_n , div , d_n );
//		printk("d\n");
		bn_mul_intset(mul,m_n,n_mul);
		bn_mul_intset(div,d_n,n_div);
		tem = bn_div_to_int(n_mul,n_div);	
		
		ans += tem;
//		printk("%d\n",tem);
	}
	//decimal compensation
	ans += n/2;
	//printk("e\n");

	return ans;
}


//calc angle g'0x in plane x0y 
 uint32_t calc_angle_acc(int32_t x,int32_t y)
{
//	const uint32_t angle_shift[4] = {ANGLE_0 , ANGLE_180 , ANGLE_270 , ANGLE_360};
	uint8_t shift = 0; //quadrant
	uint32_t change = 0; //have been change x y?
	uint32_t res ; 

//	printk("calc_angle(%d,%d)\n",x,y);
	
	//quadrant
	if(x<0)
	{
		x = -x;
		shift |= 1;
	}
	if(y<0)
	{
		y = -y;
		shift |= 2;
	}
	if(x<5)
	{
		res = ANGLE_90 ; 
	}
	else
	{
		if(y /x > 5)
		{
			EXCHANGE(x,y);
			change = 1;
		}
		res = my_atan2((uint32_t)y,(uint32_t)x);
		if(change )
		{
			//atan(x/y) = 90 - atan(y/x)
			res = ANGLE_90 - res;
		}
	}

	
	
	switch(shift)
	{
		case 0: break;  //first quadrant
		case 1: res = ANGLE_180 - res; //second quadrant
			     break;
		case 2: res = ANGLE_360 - res; // fourth quadrant
			     break;
		case 3: res = ANGLE_180 + res; // third quadrant
			     break;
	}
//	printk("angle res:%u\n",res);
	return res;
}

void init_kalman(struct kalman_operator * op , struct acc_stat *acc)
{
//	int i =0;
	op->angle.x_y= calc_angle_acc(acc->x,acc->y);
	op->bias.x_y = 0;
	op->angle.y_z= calc_angle_acc(acc->y,acc->z);
	op->bias.y_z = 0;
	op->angle.z_x = calc_angle_acc(acc->z,acc->x);
	op->bias.z_x = 0;
//	op->pre_time = time;
	//op->pre_rate = gyo_z;
//	op->pre_y = R_MEASURE;
//	printk("init_kalman:%d,%d\n",op->angle,op->pre_rate);
	
	op->p[0][0] = 0;
	op->p[1][1] = 0;
	op->p[0][1] = 0;
	op->p[1][0] = 0;
}

//calculate a + b 
inline int32_t angle_add(int32_t a,int32_t b)
{
	int32_t tmp = a+b;
	if(tmp<0)
	{
		tmp = ANGLE_360 + tmp;
	}
	else
	if(tmp>=ANGLE_360)
	{
		tmp = tmp-ANGLE_360;
	}
	return tmp; 
}


//calculate |a-b|
inline int32_t angle_sub(int32_t a,int32_t  b)
{
	int32_t ret ; 
	uint8_t change = 0;
	if(a<=b)
	{
		EXCHANGE(a,b);
		change ++;
	}	
	ret = a-b;
	if(ret > ANGLE_180)
	{
		b+= ANGLE_360;
		ret = b-a;
		change ++;
	}
	if(change & 1)
	{
		ret = -ret;
	}
	return ret ;
}

void flush_kalman(struct kalman_operator *op ,struct acc_stat  * acc ,struct gyro_stat * gyro)
{
	//delta time
	uint32_t dt = 90;//jiffies_to_msecs(time - op->pre_time);
	//innovation 
	int32_t ino_xy,ino_yz,ino_zx;
	//innovation covariances
	int32_t s	;
	//average rate
	
//	printk("flush_kalman(%d,%d,%d,%d)\n",acc_x,acc_y,gyo_z,30);
	//mark the time & rate
//	op->pre_time = time;
	

//	printk("rate:%d ,%d,%d\n",op->pre_rate,gyo_z,rate);

	//the a priori estimate of angle 
	op->angle.x_y = angle_add(op->angle.x_y , dt*(gyro->z - op->bias.x_y));
	op->angle.y_z = angle_add(op->angle.y_z , dt*(gyro->x - op->bias.y_z));
	op->angle.z_x = angle_add(op->angle.z_x , dt*(gyro->y - op->bias.z_x));
//	printk("angle1: %d\nbias:%d\n",op->angle,op->bias);
	
	// estimation error covariance 
	op->p[0][0] += 1*(1*(op->p[1][1]) - (op->p[0][1]) +  Q_ANGLE);
	op->p[0][1] -= 1*(op->p[1][1]);
	op->p[1][0] -= 1*(op->p[1][1]);
	op->p[1][1] += 1*Q_BIAS;
	
/*	printk("p1:\n%d , %d\n%d,%d\n",op->p[0][0], op->p[0][1] ,\
								op->p[1][0] , op->p[1][1]);
*/
	//innovation covariance
	s = (op->p[0][0]) +R_MEASURE;
	
//	printk("s:%d\n" , s);
	//innovation
	ino_xy = angle_sub(calc_angle_acc(acc->x , acc->y) , (op->angle.x_y));
	ino_yz = angle_sub(calc_angle_acc(acc->y , acc->z) , (op->angle.y_z));
	ino_zx = angle_sub(calc_angle_acc(acc->z , acc->x) , (op->angle.z_x));
//	printk("y:%d\n",y);
	//kalman gain
	//k[0] = p00 /s;
	//k[1] = p10 /s;

	//Update estimate with measurement
	//angle += k[0] * y;
	//bias += k[1] *y;
	op->angle.x_y= angle_add(op->angle.x_y, bn_a_mul_b_div_c(op->p[0][0],ino_xy,s));
	op->angle.y_z= angle_add(op->angle.y_z, bn_a_mul_b_div_c(op->p[0][0],ino_yz,s));
	op->angle.z_x= angle_add(op->angle.z_x, bn_a_mul_b_div_c(op->p[0][0],ino_zx,s));

	op->bias.x_y = (bn_a_mul_b_div_c(op->p[1][0],ino_xy,s)/90);
	op->bias.y_z = (bn_a_mul_b_div_c(op->p[1][0],ino_yz,s)/90);
	op->bias.z_x = (bn_a_mul_b_div_c(op->p[1][0],ino_zx,s)/90);
//	printk("angle2: %d\n",op->angle);
	// Calculate estimation error covariance 
	//p10 -= k[1] * p00
	//p11 -= k[1] * p01
	//p01 -= k[0] * p01
	//p00 -= k[0] * p00
	op->p[1][0]  -= bn_a_mul_b_div_c(op->p[1][0],op->p[0][1] , s);
	op->p[1][0] /=(1<<2);
	op->p[1][1]  -= bn_a_mul_b_div_c(op->p[1][0],op->p[0][0] , s);
	op->p[1][1] /=(1<<2);
	op->p[0][1]  -= bn_a_mul_b_div_c(op->p[0][0],op->p[0][1] , s);
	op->p[0][1] /=(1<<2);
	op->p[0][0]  -= bn_a_mul_b_div_c(op->p[0][0],op->p[0][0] , s);
	op->p[0][0] /=(1<<2);

/*	printk("p:\n%d , %d\n%d,%d\n",op->p[0][0], op->p[0][1] ,\
								op->p[1][0] , op->p[1][1]);
*/
}



