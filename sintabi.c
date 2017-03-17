/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 500                                           //定义查表的点数							 用int 数组，抽样数可以翻倍
#define PI 3.1415926535897932384626433832795028841971       //定义圆周率值
int  sin_tb[TAB_N+1];                                 			//初始化1/4的sin表(256点)      不同于sinTab , 该数据计算 STD_CCR/2 *sin(i)

//unsigned int max_sampels;												//  samples 按设计最大抽样数确定的，比如0.5Hz分辨率，载波频率5K，
																									//	载波周期不变时，基波0.5Hz时抽样数为50*100/0.5=10000;
unsigned int steps;																//  =samples/TAB_N  ,程序实际用这个值控制；
unsigned int t_samples;
unsigned int h_samples;
unsigned int q_samples;


/*
*				msamples:   一个周期的最大抽样数需求，也就是最低频率时一个周期的抽样数；
				f:				需要加入计算的STD_CCR值/2
*										
*/

void init_sinTabCCR(unsigned int msamples,unsigned int f)
{
	int i;
	
	for ( i=0;i<=TAB_N;i++)
		sin_tb[i] = f*sin(i*PI/2/TAB_N)+0.50;				// 4舍5入
	t_samples = msamples;
	h_samples = t_samples>>1;
	q_samples = h_samples>>1;
	steps = q_samples/TAB_N;								//   必须整数，这由程序员人工控制，切记； 
}

/*********************************************************************************
函数原型：int sinTabCCR(int i)
函数功能：采用查表加线性模拟的方法计算一个点 i 的正弦值*STD_CCR/2 的值
输入参数：i变化值为 【0--samples】，samples应设计为TAB_N的4的倍数，相当于 0 - 2*PI
输出参数：对应点的正弦值
**********************************************************************************/
int  sinTabCCR(unsigned int i)
{
    int n,j;
    int a;

    if(i>=h_samples)                //  PI - 2*PI
    {
    	i = i-h_samples;
    	if ( i >=q_samples )
    		i = h_samples-i;
    	n = i/steps;
    	j = i%steps;
      a=-sin_tb[n]-((j*2*(sin_tb[n+1]-sin_tb[n]))/steps+1)/2;
    }
    else {
    	if(i>=q_samples )       // PI/2 ~ PI
    		i = h_samples -i;
    	n = i/steps;
    	j = i%steps;
			a=sin_tb[n]+((j*2*(sin_tb[n+1]-sin_tb[n]))/steps+1)/2;
    }
    
    return a;
}