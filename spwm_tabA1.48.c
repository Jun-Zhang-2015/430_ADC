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
 		
#define  MIN_RATE    		0.1													  //  可修改
#define  MAX_RATE    		50														//  可修改

#define  MIN_SAMPLES		100														//  50Hz波形时一周期抽样100次
#define  Multi_N				500														// 	10倍0.5Hz,500倍0.1Hz (MAX_RATE/MIN_RATE)			最大抽样倍数   基数为STD_CCR  
#define  STD_CCR				1600													//	3200是一周期     UP/DOWN 模式下比较CCR最大3200/2
#define  MAX_SAMPLES    ((unsigned int)MIN_SAMPLES*Multi_N)					//  一周期最大抽样数
#define  MAX_MA 				0.9
#define  MIN_MA					0.225

// float sin_tab[MIN_SAMPLES/4 *Multi_N+1];						1.4版不需要定义 	//  最小频率(5Hz)时抽样数，  一个周期只需要1/4周期数据即可满足计算	
unsigned int step=Multi_N;														// 10 缺省 50Hz 时步长【本程序算法用到】
float ma=MAX_MA;																			//  modulation index

		/*  1.4版这些无需定义
		unsigned int samples =  MAX_SAMPLES;									// 缺省一周期抽样数，随着旋钮转动变化，
		unsigned int hsamples = MAX_SAMPLES>>1;								// PI--半周期抽样数
		unsigned int qsamples = MAX_SAMPLES>>2;								// PI/2 --1/4 周期抽样数
		*/		
		
unsigned int min_adc = 0;														  // 设置了初始值，在运行中进行调整			预设为 0,1023，等于去除了其作用
unsigned int max_adc = 1023;													// 设置了初始值，在运行中进行调整		
unsigned int max_adcv =1020;														//  电位器在adc 1021-1023 无效
unsigned int gap_adc = (1023-0)/Multi_N;		// 102  gap = 每段间隔数值； gap_adc = (max_adc - min_adc)/10;     
unsigned int adcvalue;																//  用于存放读进来的adc值
//unsigned int curr_ccr=STD_CCR;											1.4版载波频率不变，无需定义	//  当前 考虑了  幅度因子的 CCR基准


unsigned int idx=0;											// 	下一个抽样位置索引，状态保持着 ,  idx最大值为samples/2 
//unsigned int state=0;									1.4版，无需定义	//  抽样点idx在SIN波区域，偶数表示在上半周期，奇数表示位于下半周期
//unsigned int cflag=0;									1.4版，无需定义								//  正在修改调频参数标志



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
  ADCCTL1 |= ADCSHS_2 + ADCSSEL_2 + ADCDIV_7 + ADCCONSEQ_2; //repeat single channel; TA1.1 trig sample start （但是不知道为啥是1.1）
  ADCCTL2 |= ADCPDIV_2 + ADCRES;          // 10-bit conversion results
  ADCMCTL0 |= ADCINCH_5;          //ADC Input Channel A5 P1.5
  ADCIE |= ADCIE0;                // Enable ADC conv complete interrupt
  ADCCTL0 |= ADCENC;              // ADC enable

  // ADC conversion trigger signal --TimerA1.1  为啥??（这个设置是和例程一样的 但是并不知道和原来的选择差别在哪。。
  TA1CTL |= TASSEL_1 + TACLR;      // 01 ACLK 
  TA1CCR0 = 1200;           //PWM period
  TA1CCR1 = 1000;            // TA1.  ADC trigger     
  TA1CCTL1 |= OUTMOD_4;            // toogle
  TA1CTL |= MC_1;           //UP mode
  
  P4DIR |= BIT0;     //这两句例程里面没有
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
		PWM_setUp_upDownMode();														//  注意修改
		ADC_setup();

    
//   sin_tab[]  计算，只需要四分之一周期的表格，即可算出一个周期的SIN表
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
					__delay_cycles(8000000/50);				//	延时10ms	 ; 8000000*(1/MCLK)=0.5s 
     			j+=adcvalue;
          }
        j>>=3; 											//  除以8 取平均；
     				
        if ( abs(j-v)>40 ) 				  //  
           break;
      }while(1);	   //若旋钮不变则无法跳出？
			//  动了
		
    	v=j;
			if ( v < min_adc )										//  如果
       	min_adc = v;
     	if ( v >max_adc)
       	max_adc = v;
      gap_adc= (max_adc-min_adc)/Multi_N;
      max_adcv = gap_adc*Multi_N;

			i = v>max_adcv ? max_adcv : v;								//    高于 max_adcv 就按max_adcv 算		
			i = i-min_adc;
			ma = MIN_MA+ ((MAX_MA-MIN_MA)/max_adcv)*i;	 	//  	最低 MIN_MA  最高MAX_MA							

  		j= (i+gap_adc-1)/gap_adc;     								//    i = min_adc 时 step = 0 , 需要处理
  		if ( j >=Multi_N )
  			step = Multi_N;
  		else  step = j;																//    step可能等于0，相当于相位不变 0Hz

      	
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
		
   	c1 = (STD_CCR>>1)- ma*sinTabCCR(idx);				//	必须用常数？	//  ！！！重要： 算出来的TA0CCR1不可能会小于 1 ， 这是通过ma 最大值取值上来控制
    c2 = c1>1 ? c1-1 : 0 ;    								  //  再次控制，其实不需要了
	  idx +=step1;	
	  if ( idx >= MAX_SAMPLES )		// 是否到达周期
			idx -=MAX_SAMPLES;						//  idx %=samples;
		step1 = step; 
}