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
      dialValue = dialValue * 0x02;

      if(0x00 == dialValue)
         dialValue = 0x01;
    }


    //if(SW1_interruptFlag_==-1)
    else
    {
      dialValue = dialValue / 0x02;

      if(0x00 == dialValue)
         dialValue = 0x80;
    }

    rate=0.5 + sample/256;

    //Set value
    setLedDial(dialValue);

    //Refresh display (10 times for each position)
    for(int i = 0; i < 10*rate; i++)      //100ms
      refreshLedDial();
  }
}
