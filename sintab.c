/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 250                                           //定义查表的点数
#define PI 3.1415926535897932384626433832795028841971       //定义圆周率值
float  sin_tab[TAB_N+1];                                 //初始化1/4的sin表(256点)

//unsigned int max_sampels;												//  samples 按设计最大抽样数确定的，比如0.5Hz分辨率，载波频率5K，
																								//	载波周期不变时，基波0.5Hz时抽样数为50*100/0.5=10000;
unsigned int steps;															//  =samples/TAB_N  ,程序实际用这个值控制；


/*
*				msamples:   一个周期的抽样数；
*/

void init_sinTab(int msamples)
{
			int i;

			for (i=0;i<=TAB_N;i++)
			{
				sin_tab[i]=sin(i*PI/2/TAB_N);
			}
			steps = msamples/TAB_N/4;								//   必须整数，这由程序员人工控制，切记；
}


/*********************************************************************************
函数原型：float sin_tab(int i)
函数功能：采用查表加线性模拟的方法计算一个点 i 的正弦值,
输入参数：i变化值为 【0--samples】，samples应设计为TAB_N的整数倍，相当于 0 - 2*PI
输出参数：对应点的正弦值
**********************************************************************************/
float sinTab(int i)
{
    int n,j;
    float a;

    n=i/steps;             // sin_tab[i] = sin(2*PI*i/TAB_N);
    j=i%steps;
    if(n>=0&&n<TAB_N)                // 0 ~ PI/2
    {
        a=sin_tab[n]+(float)j/steps *(sin_tab[n+1]-sin_tab[n]);
    }
    else if(n>=TAB_N && n<TAB_N*2)       // PI/2 ~ PI
    {
        n = TAB_N*2 - n;
        a=sin_tab[n]+(float)j/steps * (sin_tab[n-1]-sin_tab[n]);
    }
    else if(n>=TAB_N*2 && n<TAB_N*3)    // PI ~ 3/4*PI
    {
        n-=TAB_N*2;
        a=-sin_tab[n]-(float)j/steps *(sin_tab[n+1]-sin_tab[n]);
    }
    else if(n>=3*TAB_N && n<TAB_N*4)      // 3/4*PI ~ 2*PI
    {
        n=TAB_N*4-n;
        a=-sin_tab[n]-(float)j/steps * (sin_tab[n-1]-sin_tab[n]);
    }
    return a;
}