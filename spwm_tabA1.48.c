/**************************************************
 * 19.496 Project
 * 1.48
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include <adc.h>
#include <math.h>

extern int sinTabCCR(unsigned int i);
extern void init_sinTabCCR(unsigned int samples,unsigned int ccr);							

#define  abs(x)					((x)>0? x : -(x))
#define  PI  						3.1415926535897932384626
 		
#define  MIN_RATE    		0.1													  //  ���޸�
#define  MAX_RATE    		50														//  ���޸�

#define  MIN_SAMPLES		100														//  50Hz����ʱһ���ڳ���100��
#define  Multi_N				500														// 	10��0.5Hz,500��0.1Hz (MAX_RATE/MIN_RATE)			����������   ����ΪSTD_CCR  
#define  STD_CCR				1600													//	3200��һ����     UP/DOWN ģʽ�±Ƚ�CCR���3200/2
#define  MAX_SAMPLES    ((unsigned int)MIN_SAMPLES*Multi_N)					//  һ������������
#define  MAX_MA 				0.9
#define  MIN_MA					0.225

// float sin_tab[MIN_SAMPLES/4 *Multi_N+1];						1.4�治��Ҫ���� 	//  ��СƵ��(5Hz)ʱ��������  һ������ֻ��Ҫ1/4�������ݼ����������	
unsigned int step=Multi_N;														// 10 ȱʡ 50Hz ʱ�������������㷨�õ���
float ma=MAX_MA;																			//  modulation index

		/*  1.4����Щ���趨��
		unsigned int samples =  MAX_SAMPLES;									// ȱʡһ���ڳ�������������ťת���仯��
		unsigned int hsamples = MAX_SAMPLES>>1;								// PI--�����ڳ�����
		unsigned int qsamples = MAX_SAMPLES>>2;								// PI/2 --1/4 ���ڳ�����
		*/		
		
unsigned int min_adc = 0;														  // �����˳�ʼֵ���������н��е���			Ԥ��Ϊ 0,1023������ȥ����������
unsigned int max_adc = 1023;													// �����˳�ʼֵ���������н��е���		
unsigned int max_adcv =1020;														//  ��λ����adc 1021-1023 ��Ч
unsigned int gap_adc = (1023-0)/Multi_N;		// 102  gap = ÿ�μ����ֵ�� gap_adc = (max_adc - min_adc)/10;     
unsigned int adcvalue;																//  ���ڴ�Ŷ�������adcֵ
//unsigned int curr_ccr=STD_CCR;											1.4���ز�Ƶ�ʲ��䣬���趨��	//  ��ǰ ������  �������ӵ� CCR��׼


unsigned int idx=0;											// 	��һ������λ��������״̬������ ,  idx���ֵΪsamples/2 
//unsigned int state=0;									1.4�棬���趨��	//  ������idx��SIN������ż����ʾ���ϰ����ڣ�������ʾλ���°�����
//unsigned int cflag=0;									1.4�棬���趨��								//  �����޸ĵ�Ƶ������־



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
    unsigned int v = 9999,i=0,j;
    
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
    
        //GPIO 1.5
  	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    
    PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
//  PMM_unlockLPM5();                                  // to activate previously configured port settings
   
//  PWM_setUp();   // suqare wave frequency: 1M--450 ,8M--3.4K, 16M--6.9kHz 
		PWM_setUp_upDownMode();														//  ע���޸�
		ADC_setup();

    
//   sin_tab[]  ���㣬ֻ��Ҫ�ķ�֮һ���ڵı�񣬼������һ�����ڵ�SIN��
//	 sin(x+PI)=-sin(x);    sin(x)=sin(PI-x)
//

		init_sinTabCCR(MAX_SAMPLES,STD_CCR/2);

                  
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

			i = v>max_adcv ? max_adcv : v;								//    ���� max_adcv �Ͱ�max_adcv ��		
			i = i-min_adc;
			ma = MIN_MA+ ((MAX_MA-MIN_MA)/max_adcv)*i;	 	//  	��� MIN_MA  ���MAX_MA							

  		j= (i+gap_adc-1)/gap_adc;     								//    i = min_adc ʱ step = 0 , ��Ҫ����
  		if ( j >=Multi_N )
  			step = Multi_N;
  		else  step = j;																//    step���ܵ���0���൱����λ���� 0Hz

      	
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
		static unsigned int step1=0,c1=STD_CCR,c2=STD_CCR-1;
		

		TA0CCR1 = c1;
		TA0CCR2 = c2;
		
   	c1 = (STD_CCR>>1)- ma*sinTabCCR(idx);				//	�����ó�����	//  ��������Ҫ�� �������TA0CCR1�����ܻ�С�� 1 �� ����ͨ��ma ���ֵȡֵ��������
    c2 = c1>1 ? c1-1 : 0 ;    								  //  �ٴο��ƣ���ʵ����Ҫ��
	  idx +=step1;	
	  if ( idx >= MAX_SAMPLES )		// �Ƿ񵽴�����
			idx -=MAX_SAMPLES;						//  idx %=samples;
		step1 = step; 
}