/**************************************************
 * 19.496 Project
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include "adc.h"

#define  PI  						3.1415926535897932384626
 		
#define  MIN_RATE    		1																				//  ���޸�
#define  MAX_RATE    		50																			//  ���޸�

#define  MIN_SAMPLES		256																			//  50Hz����ʱһ���ڳ���256��
#define  Multi_N				16																			//  ��������  = MIN_SAMPLES * Multi_N  1 2 4 8 16 32...
#define  STD_CCR				888																			//	888��һ����     UP/DOWN ģʽ�±Ƚ�CCR���444 
#define  MAX_SAMPLES    (MIN_SAMPLES/4*Multi_N)

unsigned int sin_tab[MIN_SAMPLES/4 *Multi_N+1];									//  1 Hz����ʱ ��������  һ������ֻҪ1/4���ڼ���	
unsigned int step=1;																			//

unsigned int samples = MIN_SAMPLES;															// ��������������ťת���仯��
		
unsigned int min_adc = 256;																			// �����˳�ʼֵ���������н��е���
unsigned int max_adc = 960;																			// �����˳�ʼֵ���������н��е���
unsigned int adcvalue ;																					//  ���ڴ�Ŷ�������adcֵ
unsigned int curr_ccr=888;																					//  ��ǰ TA0CCR0
float fact=1;																										//  �Ŵ����ӣ����ڷŴ�sin_tab[i] ����ȷֵ


void PWM_setUp()
{
 //TA0.1 TA0.2
   TA0CTL |= TASSEL_2+MC_1+TACLR;//+TAIE;              //SMCLK, Up mode: Timer counts up to TAxCCR0
   TA0CCR0=  curr_ccr;                       
   
   TA0CCTL1 |= OUTMOD_7;//+CCIE;                          //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
   TA0CCTL2 |= OUTMOD_7;//+CCIE;  
   TA0CCTL0 |= OUTMOD_7+CCIE;
   
   P1DIR |= BIT7+BIT6;                          
   P1SEL0 |= BIT7+BIT6; 
}

int main(void)
{
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
    
		//   һ�³�ʼ�� sin_tab ,���ֵ��  SPWM Ϊ50Hz��������ΪMIN_SAMPLES����� CCR0��ֵΪ����
		//	 
		//		

	  int i;
	  
	  for ( i =0;i<=MAX_SAMPLES;i++)
	  		sin_tab[i] = sin(i*PI/2/MAX_SAMPLES)*STD_CCR;					// ��ʼ��sin_tab ���  ��STD_CCR ���㣬������ʱ����

    __enable_interrupt();
 
//   //ADC P8.1 A9 
//   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
  
    while(1)
    {
//        P1OUT ^= BIT0;                                 	// Toggle P1.0 using exclusive-OR

      
     	ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);

			unsigned int v = 0;

		 	if ( v == adcvalue ) 
		 		{
		 		__delay_cycles(80000/5);                       			// Delay for 80000*(1/MCLK)=0.5s  ԭ����8000000����100ms
		 		continue;												//  ��λ��û�ж�
		 	}
			v = adcvalue;
			if (adcvalue< min_adc )
        	min_adc = adcvalue;
      if (adcvalue>max_adc)
        	max_adc = adcvalue;
				
			adcv = adcvalue-min_adc;
			mv = (max_adc-min_adc)/Multi_N;					//   ��Сһ��
			samples = MIN_SAMPLES*Multi_N;					//		һ��������������
			for ( step=1;step<=Multi_N;step<<1)
				{
				 	if ( adcv <= mv ) 
					 		 break;
				  mv>>1;
				  samples>>1;
				}
			
			//  �Ѿ��õ��˲��� step �� samples ,��Ҫ���� ��ǰCCRֵ���ڸ���TA0CCR0
			//	
			fact = (float)(mv-adcv)/(float)mv;
      curr_ccr = STD_CCR+ STD_CCR* ( (float)(mv-adcv)/(float)mv );
      TA0CCR0 = curr_ccr;																							//  ֱ�Ӹ�ֵ����Ҫ����ʲô�������У���
      																																//   ��������PWM_SETUP
        
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
		static unsigned int i=0;								// 	�������� ,  i==samplesʱ�� ��һ������  
		unsigned ss,ii;													//  ss �� samples ��һ�룬�������
		
		
		ss = samples>>1;
		if(i < ss) 															//  �ϰ�����
			{
				
				if ( i*2<ss)
					TA0CCR1 = sin_tab[step*i]*(1+fact);
				esle
				  TA0CCR1 = sin_tab[ss-step*i]*(1+fact);
		    TA0CCR2 = curr_ccr;				
    	}
    else {
    		TA0CCR1 = curr_ccr;
    		ii = i-ss;
    		if ( ii*2<<ss )
    			TA1CCR1 = sin_tab[step*i]*(1+fact);
    		else
    			TA1CCR1 = sin_tab[ss-step*i]*(1+fact);
    	}
    
    if( ++i >= samples)
     i = 0;
}