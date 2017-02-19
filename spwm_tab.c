/**************************************************
 * 19.496 Project
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include "adc.h"

#define  PI  						3.1415926535897932384626
 		
#define  MIN_RATE    		1																				//  可修改
#define  MAX_RATE    		50																			//  可修改

#define  MIN_SAMPLES		256																			//  50Hz波形时一周期抽样256次
#define  Multi_N				16		       //表的倍数 N为分段次数																//  最大抽样数  = MIN_SAMPLES * Multi_N  1 2 4 8 16 32...
#define  STD_CCR				888*360/256            //抽样数为min_samples时 SPWM达到50Hz的CCR0数值																			//	888是一周期     UP/DOWN 模式下比较CCR最大444 
#define  MAX_SAMPLES    (MIN_SAMPLES/4*Multi_N)

unsigned int sin_tab[MIN_SAMPLES/4 *Multi_N+1];				//  1 Hz波形时 抽样数，  一个周期只要1/4周期即可	
unsigned int step=1;																			//

unsigned int samples = MIN_SAMPLES;															// 抽样数，随着旋钮转动变化，
		
unsigned int min_adc = 256;																			// 设置了初始值，在运行中进行调整
unsigned int max_adc = 960;																			// 设置了初始值，在运行中进行调整
unsigned int adcvalue ;																					//  用于存放读进来的adc值
unsigned int curr_ccr=888;																					//  当前 TA0CCR0
float fact=1;		   //倍数																								//  放大因子，用于放大sin_tab[i] 到正确值


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
    
		//   一下初始化 sin_tab ,表的值以  SPWM 为50Hz，抽样数为MIN_SAMPLES情况下 CCR0的值为蓝本
		//	 
		//		

	  int i;
	  
	  for ( i =0;i<=MAX_SAMPLES;i++)
	  	sin_tab[i] = sin(i*PI/2/MAX_SAMPLES)*STD_CCR;			// 初始化sin_tab 表格  用STD_CCR 来算，具体用时修正

    __enable_interrupt();
  
    while(1)
    { 
     	ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);

			unsigned int v = 0;

		 	if ( v == adcvalue ) 
		 		{
		 		__delay_cycles(80000/5);                       	// Delay for 80000*(1/MCLK)=0.5s  原来是8000000，现100ms
		 		continue;							//  电位器没有动
		 	}
			v = adcvalue;
			if (adcvalue< min_adc )
        	min_adc = adcvalue;
      if (adcvalue>max_adc)
        	max_adc = adcvalue;
				
			adcv = adcvalue-min_adc;
			mv = (max_adc-min_adc)/Multi_N;					//   最小一段
			samples = MIN_SAMPLES*Multi_N;					//		一个周期最多抽样数
			for ( step=1;step<=Multi_N;step<<1)
				{
				 	if ( adcv <= mv ) 
					 		 break;
				  mv>>1;
				  samples>>1;
				}
			
			//  已经得到了步长 step 和 samples ,需要计算 当前CCR值用于赋予TA0CCR0
			//	
			fact = (float)(mv-adcv)/(float)mv;
      curr_ccr = STD_CCR+ STD_CCR* ( (float)(mv-adcv)/(float)mv );
      TA0CCR0 = curr_ccr;																							//  直接赋值还是要先做什么操作才行？？
      																																//   或者运行PWM_SETUP
        
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
		static unsigned int i=0;			// 	抽样计数 ,  i==samples时， 满一个周期  
		unsigned ss,ii;					//  ss 是 samples 的一半，半个周期
		
		
		ss = samples>>1;       //二进制 右移一位 相当于除二                      
		if(i < ss) 															//  上半周期
		{
		    if ( i*2<ss)
		       TA0CCR1 = sin_tab[step*i]*(1+fact);
		    esle
		       TA0CCR1 = sin_tab[ss-step*i]*(1+fact);    //sinx = sin(PI-x）         
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
