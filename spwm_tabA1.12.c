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

#define  MIN_SAMPLES		256														//  50Hz����ʱһ���ڳ���256��
#define  Multi_N				(MAX_RATE/MIN_RATE)						//  ����������   ����ΪSTD_CCR  
#define  STD_CCR				1250													//	1250��һ����     UP/DOWN ģʽ�±Ƚ�CCR���1250/2
#define  MAX_SAMPLES    (MIN_SAMPLES*Multi_N)					//  һ������������

unsigned int sin_tab[MIN_SAMPLES/4 *Multi_N+1];				//  ��СƵ��(1Hz)ʱ��������  һ������ֻ��Ҫ1/4�������ݼ����������	
unsigned int step=Multi_N;														//  ȱʡ 50Hz ʱ�������������㷨δ�õ���
float ma=1.000000;														//  ���ȵ���ϵ��

unsigned int samples = MIN_SAMPLES;										// ȱʡһ���ڳ�������������ťת���仯��
unsigned int hsamples = MIN_SAMPLES>>1;								// PI--�����ڳ�����
unsigned int qsamples = MIN_SAMPLES>>2;								// PI/2 --1/4 ���ڳ�����
		
unsigned int min_adc = 0;														  // �����˳�ʼֵ���������н��е���			Ԥ��Ϊ 0,1023������ȥ����������
unsigned int max_adc = 1023;													// �����˳�ʼֵ���������н��е���		
unsigned int adcvalue;																//  ���ڴ�Ŷ�������adcֵ
unsigned int curr_ccr=STD_CCR;												//  ��ǰ ������  �������ӵ� CCR��׼


unsigned int idx=0;																		// 	��һ������λ��������״̬������ ,  idx���ֵΪsamples/2 
unsigned int state=0;																	//  ������idx��SIN������ż����ʾ���ϰ����ڣ�������ʾλ���°�����
unsigned int cflag=0;																	//  �����޸ĵ�Ƶ������־

void PWM_setUp()
{
 //TA0.1 TA0.2
   TA0CTL |= TASSEL_2+MC_1+TACLR;//+TAIE;             //SMCLK, Up mode: Timer counts up to TAxCCR0
   TA0CCR0=  STD_CCR;                       
   
   TA0CCTL1 |= OUTMOD_7;//+CCIE;                      //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
   TA0CCTL2 |= OUTMOD_7;//+CCIE;  
   TA0CCTL0 |= OUTMOD_7+CCIE;
   
   P1DIR |= BIT7+BIT6;                          
   P1SEL0 |= BIT7+BIT6; 
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

    //GPIO 8.1
  	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
//        SYSCFG2 |= ADCPCTL9;  
    
    
    PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
//  PMM_unlockLPM5();                                  // to activate previously configured port settings
   
    PWM_setUp();   // suqare wave frequency: 1M--450 ,8M--3.4K, 16M--6.9kHz 

   	ADC_init(ADC_BASE,
            ADC_SAMPLEHOLDSOURCE_SC,
            ADC_CLOCKSOURCE_ADCOSC,
            ADC_CLOCKDIVIDER_1);
   	ADC_enable(ADC_BASE); 
   	ADC_enableInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
   	ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
   	ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_4_CYCLES, ADC_MULTIPLESAMPLESDISABLE);
   	ADC_configureMemory(ADC_BASE,
            ADC_INPUT_A9,
            ADC_VREFPOS_AVCC,
            ADC_VREFNEG_AVSS);
    
//   sin_tab[]  ���㣬ֻ��Ҫ�ķ�֮һ���ڵı�񣬼������һ�����ڵ�SIN��
//	 sin(x+PI)=-sin(x);    sin(x)=sin(PI-x)
//
		for ( i =0;i<=MAX_SAMPLES/4;i++)
 			sin_tab[i]= sin(i*PI*2/MAX_SAMPLES)*STD_CCR;		// sin_tab[i] = sin(i*PI*2/MAX_SAMPLES)*STD_CCR;			

    __enable_interrupt(); 

    while(1)
    {
     	
     	do{
     		ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
		 		if ( abs(adcvalue-v)<2 ) 
		 		{
		 			__delay_cycles(8000000/20);				//	��ʱ25ms	 ; 8000000*(1/MCLK)=0.5s 
		 			continue;													//  ��ťû��������������û��
		 		}
		 		break;
		 	}while(1);	
			//  ����
 			do{
				v=adcvalue;
     		ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
				__delay_cycles(8000000/20);								//  ��25ms
				if (adcvalue< min_adc )										//  ���
        	min_adc = adcvalue;
      	if (adcvalue>max_adc)
        	max_adc = adcvalue;
			}while( abs(adcvalue-v)>=2);							//  �б䣬��������ת�����ܣ�������ADC ������2��������ͬ

			//  Ӧ����ͣ�����ˣ����¼��㲽���������һ���ڳ�����
			//  &&&
			v=adcvalue;												//   
			
			i = hsamples;
			j=(v-min_adc)*Multi_N/(max_adc-min_adc);			//  ����㷨�����Ż�һ��
			cflag++;
			step = j==0 ? 1 : j;
			hsamples = (MAX_SAMPLES/step)>>1;							//  1/2�����ڵĳ�����,����ȡ2�ı���
			idx = hsamples*((float)idx/i)+0.500; 					//  ������λ
			cflag=0;
			qsamples = hsamples>>1;
			samples = hsamples<<1;												//  һ�����ڵĳ�������Ϊ4�ı�����

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
		if( cflag)							//  Ƶ�ʵ�����������
				return;
		if( (state & 0x0001) == 0 ) 															//  �ϰ�����
			{
				TA0CCR2 = 0;
				TA0CCR1 = sin_tab[idx>qsamples? hsamples-idx:idx]*ma;
    	}
    else {
    		TA0CCR1 = 0;
 				TA0CCR2 = sin_tab[idx>qsamples ? hsamples-idx : idx ]*ma;			
    	}
    
	  idx +=step;	
	  if ( idx >= hsamples )		// �Ƿ񵽴������
			{
			idx = idx-hsamples;			//  idx %=hsamples;
			state++;								//  ��ת  ż��/����  -- �ϰ�����/�°�����
			}					


}

