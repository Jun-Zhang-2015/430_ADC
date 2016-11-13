/**************************************************
 * EE312 Lab 4
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include "adc.h"

int SW1_interruptFlag_=1;
int SW2_interruptFlag_ ;
float rate=1 ;
int sample;

#define  Cp 360
/*----------系统频率--载波频率 调制波频率--最大脉宽--最小脉宽--*/
unsigned int sysHz=10, triHz=9, sinHz=50,  HzMax=540,   HzMin=15, HzM=555,N2,k=0;
/*--调制系数--MHz------KHz--------Hz-----------------微秒---*/
/*----------载波比----PI/N--PI/4--PI*3/4-Tc/4-Tc*m/4-sine[j]/q14--*/
unsigned int N_ts=180,PI_N,PI_25,PI_75,Tc_2,Tc_2m,Sine_Q14;
float  m=0.9;
//unsigned int s[1204];
int sine[361]={0,286,572,857,1143,1428,           //SIN值表,Q14格式有符号数
             1713,1997,2280,2563,2845,
             3126,3406,3686,3964,4240,
             4516, 4790,5063,5334,5604,
             5872,6138,6402,6664, 6924,
             7182,7438,7692,7943,8192,
             8438,8682,8923,9162,9397,
             9630,9860,10087,10311,10531,
             10749,10963,11174,11381,11585,
             11786,11982,12176,12365,12551,
             12733,12911,13085,13255,13421,
             13583,13741,13894,14044,14189,
             14330,14466,14598,14726,14849,
             14968,15082,15191,15296,15396,
             15491,15582,15668,15749,15826,
             15897,15964,16026,16083,16135,
             16182,16225,16262,16294,16322,
             16344,16362,16374, 16382,16384,            //;90
             16382,16374,16362,16344,16322,
             16294,16262,16225,16182,16135,
             16083,16026,15964,15897,15826,
             15749,15668,15582,15491,15396,
             15296,15191,15082,14968,14849,
             14726,14598,14466,14330,14189,
             14044,13894,13741,13583,13421,
             13255,13085,12911,12733,12551,
             12365,12176,11982,11786,11585,
             11381,11174,10963,10749,10531,
             10311,10087,9860,9630,9397,
             9162,8923,8682,8438,8192,
             7943,7692,7438,7182,6924,
             6664,6402,6138,5872,5604,
             5334,5063,4790,4516,4240,
             3964,3686,3406,3126,2845,
             2563,2280,1997,1713,1428,
             1143,857,572,286, 0,                      //;180
             286,572,857,1143,1428,
             1713,1997,2280,2563,2845,
             3126,3406,3686,3964,4240,
             4516, 4790,5063,5334,5604,
             5872,6138,6402,6664, 6924,
             7182,7438,7692,7943,8192,
             8438,8682,8923,9162,9397,
             9630,9860,10087,10311,10531,
             10749,10963,11174,11381,11585,
             11786,11982,12176,12365,12551,
             12733,12911,13085,13255,13421,
             13583,13741,13894,14044,14189,
             14330,14466,14598,14726,14849,
             14968,15082,15191,15296,15396,
             15491,15582,15668,15749,15826,
             15897,15964,16026,16083,16135,
             16182,16225,16262,16294,16322,
             16344,16362,16374, 16382,16384,            //;90

             16382,16374,16362,16344,16322,
             16294,16262,16225,16182,16135,
             16083,16026,15964,15897,15826,
             15749,15668,15582,15491,15396,
             15296,15191,15082,14968,14849,
             14726,14598,14466,14330,14189,
             14044,13894,13741,13583,13421,
             13255,13085,12911,12733,12551,
             12365,12176,11982,11786,11585,
             11381,11174,10963,10749,10531,
             10311,10087,9860,9630,9397,
             9162,8923,8682,8438,8192,
             7943,7692,7438,7182,6924,
             6664,6402,6138,5872,5604,
             5334,5063,4790,4516,4240,
             3964,3686,3406,3126,2845,
             2563,2280,1997,1713,1428,
             1143,857,572,286, 0};

509  ,517  ,526  ,535  ,544  ,552  ,561  ,570  ,578  ,587  ,
595  ,604  ,612  ,621  ,629  ,638  ,646  ,655  ,663  ,671  ,
679  ,687  ,695  ,703  ,711  ,719  ,727  ,735  ,742  ,750  ,
758  ,765  ,772  ,780  ,787  ,794  ,801  ,808  ,815  ,821  ,
828  ,835  ,841  ,847  ,854  ,860  ,866  ,872  ,877  ,883  ,
889  ,894  ,899  ,905  ,910  ,915  ,919  ,924  ,929  ,933  ,
937  ,941  ,946  ,949  ,953  ,957  ,960  ,964  ,967  ,970  ,
973  ,976  ,978  ,981  ,983  ,985  ,987  ,989  ,991  ,992  ,
994  ,995  ,996  ,997  ,998  ,999  ,999  ,1000  ,1000  ,1000  ,
1000  ,1000  ,999  ,999  ,998  ,997  ,996  ,995  ,994  ,992  ,
991  ,989  ,987  ,985  ,983  ,981  ,978  ,976  ,973  ,970  ,
967  ,964  ,960  ,957  ,953  ,949  ,946  ,941  ,937  ,933  ,
929  ,924  ,919  ,915  ,910  ,905  ,899  ,894  ,889  ,883  ,
877  ,872  ,866  ,860  ,854  ,847  ,841  ,835  ,828  ,821  ,
815  ,808  ,801  ,794  ,787  ,780  ,772  ,765  ,758  ,750  ,
742  ,735  ,727  ,719  ,711  ,703  ,695  ,687  ,679  ,671  ,
663  ,655  ,646  ,638  ,629  ,621  ,612  ,604  ,595  ,587  ,
578  ,570  ,561  ,552  ,544  ,535  ,526  ,517  ,509  ,500  ,
491  ,483  ,474  ,465  ,456  ,448  ,439  ,430  ,422  ,413  ,
405  ,396  ,388  ,379  ,371  ,362  ,354  ,345  ,337  ,329  ,
321  ,313  ,305  ,297  ,289  ,281  ,273  ,265  ,258  ,250  ,
242  ,235  ,228  ,220  ,213  ,206  ,199  ,192  ,185  ,179  ,
172  ,165  ,159  ,153  ,146  ,140  ,134  ,128  ,123  ,117  ,
111  ,106  ,101  ,95  ,90  ,85  ,81  ,76  ,71  ,67  ,63  ,
59  ,54  ,51  ,47  ,43  ,40  ,36  ,33  ,30  ,27  ,
24  ,22  ,19  ,17  ,15  ,13  ,11  ,9  ,8  ,6  ,
5  ,4  ,3  ,2  ,1  ,1  ,0  ,0  ,0  ,0  ,
0  ,1  ,1  ,2  ,3  ,4  ,5  ,6  ,8  ,9  ,
11  ,13  ,15  ,17  ,19  ,22  ,24  ,27  ,30  ,33  ,
36  ,40  ,43  ,47  ,51  ,54  ,59  ,63  ,67  ,71  ,
76  ,81  ,85  ,90  ,95  ,101  ,106  ,111  ,117  ,
123  ,128  ,134  ,140  ,146  ,153  ,159  ,165  ,172  ,179  ,
185  ,192  ,199  ,206  ,213  ,220  ,228  ,235  ,242  ,250  ,
258  ,265  ,273  ,281  ,289  ,297  ,305  ,313  ,321  ,329  ,
337  ,345  ,354  ,362  ,371  ,379  ,388  ,396  ,405  ,413  ,
422  ,430  ,439  ,448  ,456  ,465  ,474  ,483  ,491  ,500  ,


// ADC interrupt service routine
#pragma vector=ADC_VECTOR           // these two lines are used by IAR and CCC
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              // conversion complete
        {
         sample = ADCMEM0;
         break;
        }
  }

  ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}

//void PWM_setUp()
//{
//   P1DIR |= BIT7+BIT6;  //set pin as output
//   P1SEL0 |= BIT7+BIT6; //P1.7 TA0.1, P1.6 TA0.2, Timer TA0 CCR1 capture: CCI1A input, compare: Out1 outputs PWM1
//
////   TA0CCR0 = 0x0800;   //2048  8hz
////   TA0CCR1 = 0x0700;   // duty period=1/16?
//   TA0CCTL1= OUTMOD_3;  //TA0.1 set (TACCR1) / reset (TACCR0)
//   TA0CCTL2= OUTMOD_3;  //TA0.2
//   TA0CCR0= 20;  //PWM period
//   TA0CCR1= 5;   //PWM Duty Cycle
////   TA0CCR0= PWMprd;     //period of PWM = PWMprd/ACLK =(max/frequency)
////   TA0CCR1= PWMduty;    //high level output time of PWM1 = Duty Cycle * Period; duty cycle= CCR1/CCR0
////   TA0CCR2= PWMduty;    //                          PWM2

////   TA0CTL=TASSEL_2+MC_1;//SMCLK (maximum operating frequency 16MHz), up-mode
//   TA0CTL=TASSEL_1+MC_1;//ACLK (approximately 32 kHz), Up mode: Timer counts up to TAxCCR0
//   TA0CTL|=ID_0;       //00b = /1, 01b = /2, 10b = /4, 11b = /8
//
//   TA0CTL |= TACLR;    //Timer_A clear The TACLR bit is automatically reset and always reads as zero
//   TA0CTL |= TAIE;      //Timer_A interrupt enable.
//   TA0CCTL0 |= CCIE;
//   TA0CCTL1 |= CCIE;  //Capture/compare interrupt enable. This bit enables the interrupt request of the corresponding CCIFG flag.
//}

void ADC_Init();
void SwitchInit();
void CalInit();

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
  ADC_Init();
  //PWM_setUp()
  CalInit();
  //Default MCLK = 1MHz

  unsigned int i = 0;
  unsigned char dialValue = 0x01;

  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings
  PMM_unlockLPM5();

  __enable_interrupt();

  for (;;)
  {
    _BIS_SR(CPUOFF);                    // CPU off
    _NOP();                             // Required only for C-spy
  }

  while(1)
  {
    ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);

    rate=0.5 + sample/256;  //sample between 0~1023
    //Refresh display (10 times for each position)
    for(int i = 0; i < 10*rate; i++)      //100ms
      refreshLedDial();
  }
}

void CalInit()
{
  PI_N=1;                        //PI_N为PI与载波比的比值180/180
  PI_25=120;                     //PI*2/3
  Tc_2=275;                      //TC/(2*2)=110us/4,其计数周期为27.5us/0.1us=275
  Tc_2m=247;                     //Tc*m/4=Tc_4*m=247
  N2=N_ts*2;                      
}

#pragma vector = PORT1_VECTOR
__interrupt void P1_ISR(void)
{
   switch(__even_in_range(P1IV,P1IV_P1IFG7))
{
     case P1IV_P1IFG3: //It is SW1
      	SW1_interruptFlag_ = -SW1_interruptFlag_;
    	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
      break;

      case P1IV_P1IFG4:  //It is SW2
    	SW2_interruptFlag_ = 1;
    	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN4);
      break;
}
}
// -------------TimerB interrupt service routine-----------
#pragma vector=TIMERB1_VECTOR
__interrupt void Timer_B(void)
{
   TBCTL&=~TBIFG;
   unsigned int j;
   int T_1,T_3,T_5;
   unsigned long Tc2mSQ;
   //-------------------------T_1--------------------------
   j=k*PI_N;
   if((j<360)&&(j>=180))
   {
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_1=Tc_2-Tc2mSQ;
   }
   else
   {
     if(j>=Cp)
       j=j-Cp;
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_1=Tc_2+Tc2mSQ;
   }
   //-------------------------T_2--------------------------
   j=j+120;
   if((j<360)&&(j>=180))
   {
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_3=Tc_2-Tc2mSQ;
   }
   else
   {
     if(j>=Cp)
       j=j-Cp;
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_3=Tc_2+Tc2mSQ;
   }
   //-------------------------T_3--------------------------
   j=j+120;
   if((j<360)&&(j>=180))
   {
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_5=Tc_2-Tc2mSQ;
   }
   else
   {
     if(j>=Cp)
       j=j-Cp;
     Tc2mSQ=((long)Tc_2m*sine[j]);
     Tc2mSQ=Tc2mSQ>>14;
     T_5=Tc_2+Tc2mSQ;
   }
   if(T_1<HzMin)
     T_1=0;
   if(T_1>HzMax)
     T_1=HzM;
   if(T_3<HzMin)
     T_3=0;
   if(T_3>HzMax)
     T_3=HzM;
   if(T_5<HzMin)
     T_5=0;
   if(T_5>HzMax)
     T_5=HzM;
   TBCCR1=T_1;       //s[i];
   TBCCR3=T_3;       //s[i+n];
   TBCCR5=T_5;       //s[i+N2];
   k++;
   if(k>N2)
     k=0;
//   TBCTL |= TBCLR;
//   TBCTLTBSSEL_2+TBIE;
}

void ADC_Init(void)
{
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

  //ADC P8.1 A9
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
}

void SwitchInit(void)
{
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);

   GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);
}
