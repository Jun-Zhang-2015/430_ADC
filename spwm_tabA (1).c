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
 		
#define  MIN_RATE    		5														  //  可修改
#define  MAX_RATE    		50														//  可修改

#define  MIN_SAMPLES		256														//  50Hz波形时一周期抽样256次
#define  Multi_N				(MAX_RATE/MIN_RATE)						//  最大抽样倍数   基数为STD_CCR  
#define  STD_CCR				1250													//	1250是一周期     UP/DOWN 模式下比较CCR最大1250/2
#define  MAX_SAMPLES    (MIN_SAMPLES*Multi_N)					//  一周期最大抽样数

unsigned int sin_tab[MIN_SAMPLES/4 *Multi_N+1];				//  最小频率(1Hz)时抽样数，  一个周期只需要1/4周期数据即可满足计算	
unsigned int step=Multi_N;														//  缺省 50Hz 时步长【本程序算法未用到】

unsigned int samples = MIN_SAMPLES;										// 缺省一周期抽样数，随着旋钮转动变化，
unsigned int hsamples = MIN_SAMPLES>>1;								// PI--半周期抽样数
unsigned int qsamples = MIN_SAMPLES>>2;								// PI/2 --1/4 周期抽样数
		
unsigned int min_adc = 256;														// 设置了初始值，在运行中进行调整
unsigned int max_adc = 960;														// 设置了初始值，在运行中进行调整
unsigned int adcvalue;																//  用于存放读进来的adc值
unsigned int curr_ccr=STD_CCR;												//  当前 TA0CCR0  本算法未变动 TA0CCR0

unsigned int idx=0;																		// 	下一个抽样位置索引，状态保持着 ,  idx最大值为samples/2 
unsigned int state=0;																	//  抽样点idx在SIN波区域，偶数表示在上半周期，奇数表示位于下半周期
unsigned int cflag=0;																	//  正在修改调频参数标志

void PWM_setUp()
{
 //TA0.1 TA0.2
   TA0CTL |= TASSEL_2+MC_1+TACLR;//+TAIE;             //SMCLK, Up mode: Timer counts up to TAxCCR0
   TA0CCR0=  curr_ccr;                       
   
   TA0CCTL1 |= OUTMOD_7;//+CCIE;                      //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
   TA0CCTL2 |= OUTMOD_7;//+CCIE;  
   TA0CCTL0 |= OUTMOD_7+CCIE;
   
   P1DIR |= BIT7+BIT6;                          
   P1SEL0 |= BIT7+BIT6; 
}

int main(void)
{
    unsigned int v = 0,i=0;
    
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
	  for ( i =0;i<=MAX_SAMPLES/4;i++)
	  		sin_tab[i] = sin(i*PI*2/MAX_SAMPLES)*STD_CCR;			// 初始化sin_tab 表格  用STD_CCR 来算，具体用时修正

      __enable_interrupt(); 

    while(1)
    {
     	
     	do{
     		ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
		 		if ( abs(adcvalue-v)<2 ) 
		 		{
		 			__delay_cycles(8000000/10);				//	延时50ms	 ;Delay for 8000000*(1/MCLK)=0.5s 
		 			continue;													//  旋钮没动，继续测试又没动
		 		}
		 		break;
		 	}while(1);	
			//  动了
 			do{
				v=adcvalue;
     		ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);
				__delay_cycles(8000000/10);
				if (adcvalue< min_adc )
        	min_adc = adcvalue;
      	if (adcvalue>max_adc)
        	max_adc = adcvalue;
			}while( abs(adcvalue-v)>=2);							//  有变，是连续在转，不管，继续读ADC 差异在2以内算相同

			//  应该是停下来了，重新计算步长；计算好一周期抽样数
			//  &&&
			cflag++;
			idx=0;														//  频率要变化，总是从0重新开始；
			state &=0xfffe;										//  从上半周期开始			
			step = (v-min_adc)*Multi_N/(max_adc-min_adc) +1;
      if(step > Multi_N)
      	step = Multi_N;
			qsamples =  (MAX_SAMPLES/step)>>2;						//  1/4个周期的抽样数,我们取4的倍数
			hsamples = qsamples<<1;
			samples = hsamples<<1;												//  一个周期的抽样数，为4的倍数；
			for ( i=0;i<=qsamples;i++)
				{
				sin_tab[i]=sin(i*PI/2/qsamples)*STD_CCR;		//   重新计算计算sin表
				}
			cflag--;
			//  
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
		if( cflag)							//  频率调整，锁定；
			{
				//  TA0CCR0=STD_CCR;
				return;
			}

		if(  state & 0x0001 == 0 ) 															//  上半周期
			{
				TA0CCR2 = 0;
				TA0CCR1 = sin_tab[idx>qsamples? hsamples-idx:idx];
    	}
    else {
    		TA0CCR1 = 0;
 				TA0CCR2 = sin_tab[idx>qsamples ? hsamples-idx : idx ];			
    	}
    
	  ++idx;	
	  if ( idx >= hsamples )		// 是否到达半周期
			{
			idx = idx-hsamples;
			state++;								//  反转  偶数/奇数  -- 上半周期/下半周期
			}					


		//需要清除中断吗？&&&							
}