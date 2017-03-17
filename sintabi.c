/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 500                                           //������ĵ���							 ��int ���飬���������Է���
#define PI 3.1415926535897932384626433832795028841971       //����Բ����ֵ
int  sin_tb[TAB_N+1];                                 			//��ʼ��1/4��sin��(256��)      ��ͬ��sinTab , �����ݼ��� STD_CCR/2 *sin(i)

//unsigned int max_sampels;												//  samples �������������ȷ���ģ�����0.5Hz�ֱ��ʣ��ز�Ƶ��5K��
																									//	�ز����ڲ���ʱ������0.5Hzʱ������Ϊ50*100/0.5=10000;
unsigned int steps;																//  =samples/TAB_N  ,����ʵ�������ֵ���ƣ�
unsigned int t_samples;
unsigned int h_samples;
unsigned int q_samples;


/*
*				msamples:   һ�����ڵ�������������Ҳ�������Ƶ��ʱһ�����ڵĳ�������
				f:				��Ҫ��������STD_CCRֵ/2
*										
*/

void init_sinTabCCR(unsigned int msamples,unsigned int f)
{
	int i;
	
	for ( i=0;i<=TAB_N;i++)
		sin_tb[i] = f*sin(i*PI/2/TAB_N)+0.50;				// 4��5��
	t_samples = msamples;
	h_samples = t_samples>>1;
	q_samples = h_samples>>1;
	steps = q_samples/TAB_N;								//   �������������ɳ���Ա�˹����ƣ��мǣ� 
}

/*********************************************************************************
����ԭ�ͣ�int sinTabCCR(int i)
�������ܣ����ò�������ģ��ķ�������һ���� i ������ֵ*STD_CCR/2 ��ֵ
���������i�仯ֵΪ ��0--samples����samplesӦ���ΪTAB_N��4�ı������൱�� 0 - 2*PI
�����������Ӧ�������ֵ
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