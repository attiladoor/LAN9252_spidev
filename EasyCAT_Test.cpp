//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy * http://www.bausano.net *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm *
//                                                                                           *
//********************************************************************************************
//                                                                                           *
// This software is distributed as an example, in the hope that it could be
// useful,          * WITHOUT ANY WARRANTY, even the implied warranty of FITNESS
// FOR A PARTICULAR PURPOSE       *
//                                                                                           *
//********************************************************************************************

//----- EasyCAT HAT application basic example for Raspberry
//----------------------------------
//----- Derived from the example project TestEasyCAT.ino for the AB&T EasyCAT
// Arduino shield

#include <stdio.h>
#include <unistd.h>

#include "EasyCAT.h" // EasyCAT library to interface

#define LOBYTE(x) ((unsigned char)((x)&0xff))
#define HIBYTE(x) ((unsigned char)((x) >> 8 & 0xff))

EasyCAT EASYCAT; // EasyCAT istantiation

// The constructor allow us to choose the pin used for the EasyCAT HAT chip
// select Without any parameter pin 24 (CE0) will be used

// example:
// EasyCAT EASYCAT(RPI_GPIO_P1_26);  // pin 26 (CE1) will be used as SPI chip
// select

// Remember that the board must be modified to match the pin chosen

unsigned short ContaUp;   // used for sawthoot test generation
unsigned short ContaDown; //

unsigned short OutCount = 0;

int main() {
  char cValue; // used to read the output buffer

  ContaUp = 0x0000;   //
  ContaDown = 0x0000; //

  //---- initialize the EasyCAT board -----

  if (EASYCAT.Connect() == 0) // initialization
  {
    printf("inizialized\n"); // succesfully completed
  } else                     // initialization failed
  {
    printf("inizialization failed\n"); // the EasyCAT board was not recognized
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

  while (1) {
    EASYCAT.MainTask(); // execute the EasyCAT task

    // --- test sawtooth generation ---

    ContaUp++;   // we increment the variable ContaUp
    ContaDown--; // and decrement ContaDown

    // we use these variables to create sawtooth,
    // with different slopes and periods, for
    // test pourpose, in input Bytes 2,3,4,5,30,31

    EASYCAT.BufferIn.Byte[2] = LOBYTE(ContaUp); // slow rising slope
    EASYCAT.BufferIn.Byte[3] = HIBYTE(ContaUp); // extremly slow rising slope

    EASYCAT.BufferIn.Byte[4] = LOBYTE(ContaDown); // slow falling slope
    EASYCAT.BufferIn.Byte[5] = HIBYTE(ContaDown); // extremly slow falling slope
    EASYCAT.BufferIn.Byte[6] = 123;

    EASYCAT.BufferIn.Byte[30] = LOBYTE(ContaUp)
                                << 2; // medium speed rising slope
    EASYCAT.BufferIn.Byte[31] = LOBYTE(ContaDown)
                                << 2; // medium speed falling slope

    // --- eight bits management ---

    cValue = EASYCAT.BufferOut.Byte[0]; // we read the input bit status reading
                                        // the first byte from output buffer

    //if (OutCount > 20) {
      // Print every 2 seconds on consolle the values of byte[0], byte[1],
      // byte[2] and byte[3] received from the Master

      //OutCount = 0;

      printf("Byte[0] = %d\n", EASYCAT.BufferOut.Byte[0]);
      printf("Byte[1] = %d\n", EASYCAT.BufferOut.Byte[1]);
      printf("Byte[2] = %d\n", EASYCAT.BufferOut.Byte[2]);
      printf("Byte[3] = %d\n\n", EASYCAT.BufferOut.Byte[3]);
      printf("BytIn[30] = %d\n\n", EASYCAT.BufferIn.Byte[30]);
    //}

    //OutCount++;

    usleep(100000); // delay of 100mS
  }
}
