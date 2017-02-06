/**************************************************
 * 19.496 Project
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include "adc.h"

#define  MIN_RATE    		1           //0.5?
#define  MAX_RATE    		50
#define  MIN_SAMPLES		360																			//  50Hz波形时一周期抽样360次
#define  MAX_SAMPLES		18000					//  1 Hz波形时      抽样360×50次
#define  MAX_CCR				444																			//	888是一周期     UP/DOWN 模式下比较CCR最大444 
#define  PI                     3.14159265357



unsigned int samples = MIN_SAMPLES;				// 抽样数，随着旋钮转动变化，
unsigned int rate = MAX_RATE;		                        // 频率， 没用
unsigned int min_adc = 256;								// 设置了初始值，在运行中进行调整
unsigned int max_adc = 512;								// 设置了初始值，在运行中进行调整
unsigned int adcvalue ;										//  用于存放读进来的adc值


void PWM_setUp()
{
 //TA0.1 TA0.2
   TA0CTL |= TASSEL_2+MC_3+TACLR;//+TAIE;              //SMCLK, Up/Down mode: Timer counts up to TAxCCR0
   TA0CCR0=  444;                       
   
   TA0CCTL1 |= OUTMOD_6;//+CCIE;                          //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
   TA0CCTL2 |= OUTMOD_6;//+CCIE;  
   TA0CCTL0 |= OUTMOD_6+CCIE;
   
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
    
    __enable_interrupt();
 
//   //ADC P8.1 A9 
//   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
  
    while(1)
    {
//        P1OUT ^= BIT0;                                 // Toggle P1.0 using exclusive-OR
//        __delay_cycles(8000000);                       // Delay for 80000*(1/MCLK)=0.5s
      
      ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
      
    }
    
     return 0;
}

// ADC interrupt service routine
#pragma vector=ADC_VECTOR           // these two lines are used by IAR and CCC
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              // conversion complete
        {      
    		adcvalue = ADCMEM0;
        if (adcvalue< min_adc )
        	min_adc = adcvalue;
        if (adcvalue>max_adc)
        	max_adc = adcvalue;
        samples = 18000*((float)(adcvalue-min_adc))/((float)(max_adc-min_adc));   //default 360
        if ( samples <= MIN_SAMPLES )                              // 1下面有个死区？
        	samples = MIN_SAMPLES;																		//  最小不能低于此
        break;
        }          
  }
  
  ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}

 //On the compare of TA0CCTL0           若是up/down模式 如何使计数在888 时发生中断？？？
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMERA0_ISR0(void) //Flag cleared automatically
{
		static int setccr = MAX_CCR;			// 当前用于设置的CCR值,   0表示占空比100%， 等于MAX_CCR时，占空比为0%																	
		static unsigned int i=0;			// 	抽样计数 ,  i==samples时， 满一个周期  
		
		if(i*2 < samples) 			        //  上半周期
			{
				TA0CCR1 = setccr;
		    TA0CCR2 = MAX_CCR;				
    	}
    else {
    	TA0CCR1 = MAX_CCR;
    	TA0CCR2 = setccr;
    	}
    
    if(++i >= samples)
     i = 0;
 		setccr =MAX_CCR- MAX_CCR* abs(sin( i*2*PI/samples ));			//		算好了下次中断时用  

}

