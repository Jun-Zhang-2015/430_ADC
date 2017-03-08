/**************************************************
 * 19.496 Project
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include <adc.h>
#include <math.h>

#define  abs(x)					((x)>0? x : -(x))
#define  PI  						3.1415926535897932384626
 		
#define  MIN_RATE    		5														  //  ���޸�
#define  MAX_RATE    		50														//  ���޸�

#define  MIN_SAMPLES		100														//  50Hz����ʱһ���ڳ���256��
#define  Multi_N				(MAX_RATE/MIN_RATE)						//  ����������   ����ΪSTD_CCR  
#define  STD_CCR				1600													//	3200��һ����     UP/DOWN ģʽ�±Ƚ�CCR���3200/2
#define  MAX_SAMPLES    (MIN_SAMPLES*Multi_N)					//  һ������������
#define  MAX_MA 				0.95

unsigned int sin_tab[MIN_SAMPLES/4 *Multi_N+1];				//  ��СƵ��(5Hz)ʱ��������  һ������ֻ��Ҫ1/4�������ݼ����������	
unsigned int step=Multi_N;							// 10 ȱʡ 50Hz ʱ�������������㷨�õ���
float ma=MAX_MA;								//  modulation index

unsigned int samples =  MAX_SAMPLES;									// ȱʡһ���ڳ�������������ťת���仯��
unsigned int hsamples = MAX_SAMPLES>>1;								// PI--�����ڳ�����
unsigned int qsamples = MAX_SAMPLES>>2;								// PI/2 --1/4 ���ڳ�����
		
unsigned int min_adc = 0;														  // �����˳�ʼֵ���������н��е���			Ԥ��Ϊ 0,1023������ȥ����������
unsigned int max_adc = 1023;													// �����˳�ʼֵ���������н��е���		
unsigned int max_adcv =1020;														//  ��λ����adc 1021-1023 ��Ч
unsigned int gap_adc = (1023-0)/Multi_N;		// 102  gap = ÿ�μ����ֵ�� gap_adc = (max_adc - min_adc)/10;     
unsigned int adcvalue;																//  ���ڴ�Ŷ�������adcֵ
unsigned int curr_ccr=STD_CCR*2;												//  ��ǰ ������  �������ӵ� CCR��׼


unsigned int idx=0;											// 	��һ������λ��������״̬������ ,  idx���ֵΪsamples/2 
unsigned int state=0;																	//  ������idx��SIN������ż����ʾ���ϰ����ڣ�������ʾλ���°�����
unsigned int cflag=0;																	//  �����޸ĵ�Ƶ������־



void PWM_setUp_upDownMode()
{
 //TA0.1 TA0.2
   TA0CTL |= TASSEL_2 + TACLR + MC_3;		//+TAIE; Up/Down mode     //SMCLK, : Timer counts up to TAxCCR0
   TA0CCR0=  STD_CCR;                       
   
   TA0CCTL1 |= OUTMOD_6;								//+CCIE;       	// CCR1 toggle/set                   
   TA0CCTL2 |= OUTMOD_2;								//+CCIE;       	// CCR2 toggle/reset
   TA0CCTL0 |= OUTMOD_6+CCIE;          	//Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
   
   P1DIR |= BIT7+BIT6;                          
   P1SEL0 |= BIT7+BIT6; 
}

void ADC_setup()
{
   	ADC_init(ADC_BASE,
            ADC_SAMPLEHOLDSOURCE_SC,
            ADC_CLOCKSOURCE_ADCOSC,
            ADC_CLOCKDIVIDER_1);
   	ADC_enable(ADC_BASE); 
   	ADC_enableInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
   	ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
   	ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);
   	ADC_configureMemory(ADC_BASE,
            ADC_INPUT_A5,
            ADC_VREFPOS_AVCC,
            ADC_VREFNEG_AVSS);
}

void ADC_setup1()
{
  // Configure ADC A5 pin
  SYSCFG2 |= ADCPCTL5;
  
  ADCCTL0 |= ADCSHT_6 + ADCON + ADCMSC;   //  ADCON      
  ADCCTL1 |= ADCSHS_2 + ADCSSEL_2 + ADCDIV_7 + ADCCONSEQ_2; //repeat single channel; TA1.1 trig sample start �����ǲ�֪��Ϊɶ��1.1��
  ADCCTL2 |= ADCPDIV_2 + ADCRES;          // 10-bit conversion results
  ADCMCTL0 |= ADCINCH_5;          //ADC Input Channel A5 P1.5
  ADCIE |= ADCIE0;                // Enable ADC conv complete interrupt
  ADCCTL0 |= ADCENC;              // ADC enable

  // ADC conversion trigger signal --TimerA1.1  Ϊɶ??����������Ǻ�����һ���� ���ǲ���֪����ԭ����ѡ�������ġ���
  TA1CTL |= TASSEL_1 + TACLR;      // 01 ACLK 
  TA1CCR0 = 1200;           //PWM period
  TA1CCR1 = 1000;            // TA1.  ADC trigger     
  TA1CCTL1 |= OUTMOD_4;            // toogle
  TA1CTL |= MC_1;           //UP mode
  
  P4DIR |= BIT0;     //��������������û��
  P4SEL0 |= BIT0;

}



int main(void)
{
    unsigned int v = 9999,i=0,j,cr;
    
    WDTCTL = WDTPW | WDTHOLD;                          // Stop watchdog timer

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);  
    __bic_SR_register(SCG0);                           // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked
    
    CSCTL4 |= SELMS__DCOCLKDIV | SELA__REFOCLK;        // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                       // default DCOCLKDIV as MCLK and SMCLK source

    P1DIR |= BIT0 | BIT4;                              // set MCLK and LED pin as output
    P1SEL0 |= BIT4;                                    // set MCLK pin as second function
    P8DIR |= BIT0 | BIT1;                              // set ACLK and SMCLK pin as output
    P8SEL0 |= BIT0 | BIT1;                             // set ACLK and SMCLK pin as second function    
    
        //GPIO 8.1
  //	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    
    PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
//  PMM_unlockLPM5();                                  // to activate previously configured port settings
   
//  PWM_setUp();   // suqare wave frequency: 1M--450 ,8M--3.4K, 16M--6.9kHz 
		PWM_setUp_upDownMode();														//  ע���޸�
		ADC_setup();

    
//   sin_tab[]  ���㣬ֻ��Ҫ�ķ�֮һ���ڵı�񣬼������һ�����ڵ�SIN��
//	 sin(x+PI)=-sin(x);    sin(x)=sin(PI-x)
//
		for ( i =0;i<=MAX_SAMPLES/4;i++)
 			sin_tab[i]= sin(i*PI*2/MAX_SAMPLES)/2;		// 

    __enable_interrupt(); 

		while(1)
    {
     	
     	do{
     	  j=0;	
				for ( i=0;i<8;i++)
        	{
     			ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
					__delay_cycles(8000000/50);				//	��ʱ10ms	 ; 8000000*(1/MCLK)=0.5s 
     			j+=adcvalue;
          }
        j>>=3; 											//  ����8 ȡƽ����
     				
        if ( abs(j-v)>40 ) 				  //  
           break;
      }while(1);	   //����ť�������޷�������
			//  ����
		
    	v=j;
			if ( v < min_adc )										//  ���
       	min_adc = v;
     	if ( v >max_adc)
       	max_adc = v;
      gap_adc= (max_adc-min_adc)/Multi_N;
      max_adcv = gap_adc*Multi_N;

			i = v>max_adcv ? max_adcv : v;			//       ���� max_adcv �Ͱ�max_adcv ��		
			ma = (MAX_MA/max_adcv)*i;									

  		i = i-min_adc;
  		j= (i+gap_adc-1)/gap_adc;     //       i = min_adc ʱ step = 0 , ��Ҫ����
  		if ( j <=0 ) 															//    �޳��쳣ֵ��
  			step = 1;
  		else if( j >=Multi_N )
  			step = Multi_N;
  		else  step = j;															

  		cr = STD_CCR*(float)gap_adc*step/(i);
      if ( cr != curr_ccr )
      	{
  		//  �޸� TA0CCR0��������Ҫ��һ��ֹͣʲô����Ȼ��ıȽϺ��ʣ�����������Ԥ��ȵȣ�USERGUIDE����������		
      	TA0CCR0 = cr;
      	curr_ccr=cr;
      	}
    }

}

// ADC interrupt service routine
#pragma vector=ADC_VECTOR           				// these two lines are used by IAR and CCC
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              				// conversion complete
        {      
    	     //  adcvalue = ADCMEM0;
          adcvalue = ADCMEM0;
        break;
        }          
  }
  
  ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}

 //On the compare of TA0CCTL0
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMERA0_ISR0(void) //Flag cleared automatically
{
		static unsigned int step1,c0,c1,c2;
		static int n,r,fan;			// ����unsigned  ��Ϊr ����ȡ��ֵ

		TA0CCR1 = c1;
		TA0CCR2 = c2;
		
		n = idx;
		if ( n >=hsamples )
				{
		    n -=hsamples;
		    r = -1;
		    }
		else r = 1;

		if ( n >= qsamples )
			n =hsamples-n;

    c1 = (curr_ccr>>1) - r*curr_ccr*ma*sin_tab[n];				//	�����ó�����	//  ��������Ҫ�� �������TA0CCR1�����ܻ�С�� 1 �� ����ͨ��ma ���ֵȡֵ��������
    c2 = c1>1 ? c1-1 : 0 ;    								  //  �ٴο��ƣ���ʵ����Ҫ��
	  idx +=step1;	
	  if ( idx >= samples )		// �Ƿ񵽴�����
			idx -=samples;			//  idx %=samples;
		step1 = step; 
  
//		unsigned int n,r,fan;
//                
//		if ( idx <= qsamples )
//			r = ma*sin_tab[idx];
//		else if ( idx > qsamples && idx <=hsamples )
//			r = ma*sin_tab[hsamples-idx];      //sin(x)=sin(PI-x)
//    else if ( idx >hsamples && idx <= (hsamples + qsamples) )
//    	r = -ma*sin_tab[idx-hsamples];
//    else if ( idx > (hsamples + qsamples) )
//        r = -ma*sin_tab[samples-idx];
//    
//    TA0CCR1 = STD_CCR>>1-r;        // 1/2Tc - r
//    
//    fan = TA0CCR1 -1;
//    if(fan<=0)
//    {
//     fan = 0;
//    }
//    TA0CCR2= fan;   	
//
//		//		TA0CCR2 = sin_tab[idx>qsamples ? hsamples-idx : idx ]*ma;			
//   
//	  idx +=step;	
//	  if ( idx >= hsamples )		// �Ƿ񵽴������
//			{
//			idx -=hsamples;			//  idx %=hsamples;   idx = idx-hsamples?
//			}					
}

//void PWM_setUp()
//{
// //TA0.1 TA0.2
//   TA0CTL |= TASSEL_2+MC_1+TACLR;//+TAIE;             //SMCLK, Up mode: Timer counts up to TAxCCR0
//   TA0CCR0=  STD_CCR;                       
//   
//   TA0CCTL1 |= OUTMOD_7;//+CCIE;                      //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
//   TA0CCTL2 |= OUTMOD_7;//+CCIE;  
//   TA0CCTL0 |= OUTMOD_7+CCIE;
//   
//   P1DIR |= BIT7+BIT6;                          
//   P1SEL0 |= BIT7+BIT6; 
//}
//

