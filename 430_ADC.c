/**************************************************
 * EE312 Lab 4
 *
 * Copyright 2015 University of Strathclyde
 **************************************************/
#include <msp430.h>
#include <driverlib.h>
#include "LedDriver.h"
#include "adc.h"


int SW1_interruptFlag_=1;
int SW2_interruptFlag_ ;
float rate=1 ;
int sample;


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
//
//}

int main(void)
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

  //Default MCLK = 1MHz

  unsigned int i = 0;

  unsigned char dialValue = 0x01;

  WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

  initialiseLedDial();
  //PWM_setUp()

  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings
  PMM_unlockLPM5();

  //Switch 1
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN3, GPIO_LOW_TO_HIGH_TRANSITION);
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);

   GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);

  __enable_interrupt();

  //ADC P8.1 A9
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);

  while(1)
  {
    ADC_startConversion(ADC_BASE, ADC_REPEATED_SINGLECHANNEL);

    if (SW1_interruptFlag_==1)
    {
 
    }
    //if(SW1_interruptFlag_==-1)
    else
    {

    }

    rate=0.5 + sample/256;

    //Set value
    setLedDial(dialValue);

    //Refresh display (10 times for each position)
    for(int i = 0; i < 10*rate; i++)      //100ms
      refreshLedDial();
  }
}
