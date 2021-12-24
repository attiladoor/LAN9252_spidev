//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                *
// http://www.bausano.net                                                                    *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                 *
//                                                                                           *  
//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, in the hope that it could be useful,          *
// WITHOUT ANY WARRANTY, even the implied warranty of FITNESS FOR A PARTICULAR PURPOSE       *
//                                                                                           *
//******************************************************************************************** 


//----- EasyCAT HAT application basic example for Raspberry ----------------------------------
//----- Derived from the example project TestEasyCAT.ino for the AB&T EasyCAT Arduino shield

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

//---- AB&T EasyCAT shield custom application example  ---------------------------------------  


// This is the legacy "TestEasyCAT" example but the variables have been 
// customized using the Easy Configuration tool. 
// To understand how to do this please see the Easy Configurator user manual.
//
//
// The input variables used in this example are:
//
//		uint16_t    Analog_0                The first analog input         
//		uint16_t    Analog_1                The second analog input
//		uint8_t     DipSwitches             The four dip switches
//		uint8_t     Bit8_FallingTestRamp    A falling test ramp
//		uint16_t    Bit16_RisingTestRamp    A rising test ramp
//
// And the output:
//
//		uint8_t     Leds;                   The four leds




//*********************************************************************************************

#include "EasyCAT.h"									 // EasyCAT library to interface     

EasyCAT EASYCAT;                    // EasyCAT istantiation

                                    // The constructor allow us to choose the pin used for the EasyCAT HAT chip select 
                                    // Without any parameter pin 24 (CE0) will be used 
                                                                      
                                    // example:                                  
//EasyCAT EASYCAT(RPI_GPIO_P1_26);  // pin 26 (CE1) will be used as SPI chip select


                                    // Remember that the board must be modified to match the pin chosen


uint16_t ContaUp;									 // used for sawthoot test generation
uint8_t ContaDown;								 //

unsigned short OutCount = 0;

int main()
{
	 char cValue;										 // used to read the output buffer 

     ContaUp = 0x0000;                                   //
     ContaDown = 0x0000;                                 //
	
	 //---- initialize the EasyCAT board -----

     if (EASYCAT.Init() == true)						 // initialization
     {
       printf("inizialized\n");							 // succesfully completed
     }
     else											 	 // initialization failed   
     {							
       printf("inizialization failed\n");				 // the EasyCAT board was not recognized
	   return -1;
     }			
														 // In the main loop we must call ciclically the 
                                                         // EasyCAT task and our application
                                                         //
                                                         // This allows the bidirectional exachange of the data
                                                         // between the EtherCAT master and our application
                                                         //
                                                         // The EasyCAT cycle and the Master cycle are asynchronous
                                                         //     
		                                                 // The delay allows us to set the EasyCAT cycle time  
                                                         // according to the needs of our application
                                                         //
                                                         // For user interface applications a cycle time of 100mS,
                                                         // or even more, is appropriate, but, for data processing 
                                                         // applications, a faster cycle time may be required
                                                         //
                                                         // In this case we can also completely eliminate this
                                                         // delay in order to obtain the fastest possible response

	while (1)
	{  
	 	  EASYCAT.MainTask();					// execute the EasyCAT task

		  // --- test sawtooth generation --- 

		  ContaUp++;						// we increment the variable ContaUp  
		  ContaDown--;						// and decrement ContaDown

		  EASYCAT.BufferIn.Cust.Analog_0 = 1234;
		  EASYCAT.BufferIn.Cust.Analog_1 = 5678; 
		  EASYCAT.BufferIn.Cust.DipSwitches = 5;		
   		  EASYCAT.BufferIn.Cust.Bit16_RisingTestRamp = ContaUp;   // medium speed rising slope
    		  EASYCAT.BufferIn.Cust.Bit8_FallingTestRamp = ContaDown; // medium speed falling slope    

		  
		  if (OutCount > 20)
		  {
			// Print every 2 seconds on consolle the values of byte[0], byte[1], byte[2] and byte[3] received from the Master
				
			OutCount = 0;

		  	printf("Leds = %d\n", EASYCAT.BufferOut.Cust.Leds);	
		  }
		  
		  OutCount ++;

		  usleep(100000);					// delay of 100mS
	}
}