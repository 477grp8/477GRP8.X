/* 
 * File:   main.c
 * Author: team8
 *
 * Created on February 9, 2013, 3:40 PM
 */


// Master header file for all peripheral library includes
#include <plib.h>
#include <p32xxxx.h>
#include <stdio.h>


#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1
#pragma config POSCMOD = XT, FNOSC = PRIPLL
#pragma config FWDTEN = OFF

#if defined (__32MX460F512L__) || defined (__32MX360F512L__) || defined (__32MX795F512H__)
#define SYS_FREQ (80000000L)
#elif defined (__32MX220F032D__) || defined (__32MX250F128D__)
#define SYS_FREQ (40000000L)
#endif

unsigned int channel4;	// conversion result as read from result buffer
unsigned int channel5;	// conversion result as read from result buffer
unsigned int offset;	// buffer offset to point to the base of the idle buffer

int main(void)
{


        //Debug LED port
       	PORTSetPinsDigitalOut(IOPORT_B, BIT_9);
	//PORTSetBits(IOPORT_B, BIT_9);

	UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
	UARTSetFifoMode(UART2, 0);
	UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
	UARTSetDataRate(UART2, 10000000l, 19200);
	UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_TX | UART_RX ));

	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states and
	// enable prefetch cache but will not change the PBDIV. The PBDIV value
	// is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

	// configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
	#define PARAM1  ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
	int PARAM1_2	 = ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON;

	// define setup parameters for OpenADC10
	//ADC ref external    | disable offset test    | disable scan mode | perform 2 samples | use dual buffers | use alternate mode
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON

	// define setup parameters for OpenADC10
	//		  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15


	// define setup parameters for OpenADC10
	//           set AN4 and AN5 as analog inputs
	#define PARAM4	ENABLE_AN4_ANA | ENABLE_AN5_ANA


	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL

	// use ground as neg ref for A | use AN4 for input A      |  use ground as neg ref for A | use AN5 for input B

	// configure to sample AN4 & AN5
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 |  ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5); // configure to sample AN4 & AN5

        #if defined (__32MX460F512L__) || defined (__32MX360F512L__) || defined (__32MX795F512H__)
        OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above
        #elif defined (__32MX220F032D__) || defined (__32MX250F128D__)
        AD1CHS = 0x05040000;
        AD1CON1 = 0xE4;
        AD1CON2 = 0x7;
        AD1CON3 = 0x8F00;
        ANSELBbits.ANSB3 = 1; //Set AN5 to analog
        ANSELBbits.ANSB2 = 1; //Set AN4 to analog
        #endif

	EnableADC10(); // Enable the ADC

	while (1)
	{
		while ( ! mAD1GetIntFlag() )
		{
                    // wait for the first conversion to complete so there will be vaild data in ADC result registers
                }

		// the results of the conversions are available in channel4 and channel5
		offset = 8 * ((~ReadActiveBufferADC10() & 0x01));  // determine which buffer is idle and create an offset

		channel4 = ReadADC10(offset);  		// read the result of channel 4 conversion from the idle buffer
		channel5 = ReadADC10(offset + 1);  	// read the result of channel 5 conversion from the idle buffer
                if ((channel5 > 100) && (channel4 < 100)){
                    PORTSetBits(IOPORT_B, BIT_9);
                }
                else{
                    PORTClearBits(IOPORT_B, BIT_9);
                }


		printf("Channel 4: %d\r\n", channel4);
                printf("Channel 5: %d\r\n", channel5);

        #if defined (__32MX460F512L__) || defined (__32MX360F512L__) || defined (__32MX795F512H__)
        mAD1ClearIntFlag();
        #elif defined (__32MX220F032D__) || defined (__32MX250F128D__)
        IFS0bits.AD1IF =0;
        #endif


	}

	return 0;
}









////include all necessary libraries here
//#include <stdio.h>
//#include <stdlib.h>
//#include <plib.h>
//
//// Configuration Bit settings
//// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
//// PBCLK = 40 MHz
//// Primary Osc w/PLL (XT+,HS+,EC+PLL)
//// WDT OFF
//// Other options are don't care
////
//#pragma config FPLLMUL = MUL_20
//#pragma FPLLIDIV = DIV_2
//#pragma FPLLODIV = DIV_1
//#pragma FWDTEN = OFF
//#pragma config POSCMOD = HS
//#pragma FNOSC = PRIPLL
//#pragma FPBDIV = DIV_8
//
//#define SYS_FREQ 			(80000000L)
//
//int main(void)
//{
////
////    int i;
////
//////~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
////    // Configure the device for maximum performance but do not change the PBDIV
////    // Given the options, this function will change the flash wait states, RAM
////    // wait state and enable prefetch cache but will not change the PBDIV.
////    // The PBDIV value is already set via the pragma FPBDIV option above..
////    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
////
////    // Explorer16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
////    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);
////
////    // Make all lower 8-bits of PORTA as output. Turn them off before changing
////    // direction so that we don't have unexpected flashes
////    mPORTAClearBits(BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
////                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
////
////    mPORTASetPinsDigitalOut( BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
////                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
////
////    // Now blink all LEDs ON/OFF forever.
////    while(1)
////    {
////            mPORTAToggleBits(BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
////                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
////
////            // Insert some delay
////            i = 1024*1024;
////            while(i--);
////    }
//
//
//
//
//
//}