/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 250                                           //定义实际查表的点数
#define PI 3.1415926535897932384626433832795028841971       //定义圆周率值
float  sin_tab[TAB_N+1];                                 //初始化1/4的sin表(256点)

//unsigned int max_sampels;												//  samples 按设计最大抽样数确定的，比如0.5Hz分辨率，载波频率5K，
																								//	载波周期不变时，基波0.5Hz时抽样数为50*100/0.5=10000;
unsigned int steps;															//  =samples/TAB_N  ,程序实际用这个值控制；
unsigned int total_samples;
unsigned int h_samples;
unsigned int q_samples;
/*
*				msamples:   一个周期的抽样数；
*/

void init_sinTab(unsigned int msamples)					//  msamples == 500 000  时支持0.1hz/5k  够了，
{
			int i;

			for (i=0;i<=TAB_N;i++)
			{
				sin_tab[i]=sin(i*PI/2/TAB_N);
			}
			total_samples = msamples;
			h_samples = total_samples>>1;
			q_samples = h_samples>>1;
			steps = q_samples/TAB_N;								//   !!!必须整数，这由程序员人工控制，切记；
}


/*********************************************************************************
函数原型：float sinTab(unsigned int i)
函数功能：采用查表加线性模拟的方法计算一个点 i 的正弦值,
输入参数：i变化值为 【0--samples】，samples应设计为TAB_N*4的整数倍，相当于 0 - 2*PI
输出参数：对应点的正弦值
**********************************************************************************/
float sinTab(unsigned int i)
{
    int n;
    float a,j;

    if( i >= h_samples)                // 0 ~ PI/2
    {
        i = i-h_samples;
        if ( i>= q_samples ) 
        		i=h_samples-i;
        n = i/steps;
        j = i%steps;	
        a=-sin_tab[n]-j/steps *(sin_tab[n+1]-sin_tab[n]);
		}
		else {
				if (i >= q_samples )
					i = h_samples - i;
				n = i/steps;
				j = i%steps;
				a = sin_tab[n]-j/steps *(sin_tab[n+1]-sin_tab[n]);
		}	
    return a;
}