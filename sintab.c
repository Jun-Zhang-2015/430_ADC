/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 250                                           //������ĵ���
#define PI 3.1415926535897932384626433832795028841971       //����Բ����ֵ
float  sin_tab[TAB_N+1];                                 //��ʼ��1/4��sin��(256��)

//unsigned int max_sampels;												//  samples �������������ȷ���ģ�����0.5Hz�ֱ��ʣ��ز�Ƶ��5K��
																								//	�ز����ڲ���ʱ������0.5Hzʱ������Ϊ50*100/0.5=10000;
unsigned int steps;															//  =samples/TAB_N  ,����ʵ�������ֵ���ƣ�


/*
*				msamples:   һ�����ڵĳ�������
*/

void init_sinTab(int msamples)
{
			int i;

			for (i=0;i<=TAB_N;i++)
			{
				sin_tab[i]=sin(i*PI/2/TAB_N);
			}
			steps = msamples/TAB_N/4;								//   �������������ɳ���Ա�˹����ƣ��мǣ�
}


/*********************************************************************************
����ԭ�ͣ�float sin_tab(int i)
�������ܣ����ò�������ģ��ķ�������һ���� i ������ֵ,
���������i�仯ֵΪ ��0--samples����samplesӦ���ΪTAB_N�����������൱�� 0 - 2*PI
�����������Ӧ�������ֵ
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