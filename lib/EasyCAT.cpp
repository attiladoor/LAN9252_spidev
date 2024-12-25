// NOLINTBEGIN
#include <cstring>
#include <fcntl.h> //Needed for SPI port
#include <iostream>
#include <linux/spi/spidev.h> //Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h> //Needed for SPI port
#include <unistd.h>    //Needed for SPI port
#include <unistd.h>

#include "EasyCAT.h"

// source:
// https://raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface

// #define DEB                                                     // enable
//  debug prints

//--------------------------------------------------------------------------------------------------

void EasyCAT::SPI_BuffTransfer(
    char *Buff,
    uint32_t Len) // static function for the SPI transfer
{                 //
  struct spi_ioc_transfer spi;
  memset(&spi, 0, sizeof(spi));
  int retVal = -1;
  int spi_cs_fd;

  if (spi_device) {
    spi_cs_fd = spi_cs1_fd;
  } else {
    spi_cs_fd = spi_cs0_fd;
  }

  spi.tx_buf = reinterpret_cast<unsigned long>(Buff); // transmit from "data"
  spi.rx_buf = reinterpret_cast<unsigned long>(Buff); // receive into "data"
  spi.len = Len;
  spi.delay_usecs = 0;
  spi.speed_hz = spi_speed;
  spi.bits_per_word = spi_bitsPerWord;
  spi.cs_change = 0; // 0=Set CS high after a transfer, 1=leave CS set low

  retVal = ioctl(spi_cs_fd, SPI_IOC_MESSAGE(1), &spi);

  if (retVal < 0) {
    std::cerr << "Error - Problem transmitting spi data..ioct retval: "
              << retVal << std::endl;
    exit(1);
  }
}

int EasyCAT::Disconnect() {
  int status_value = -1;
  int *spi_cs_fd;

  if (spi_device)
    spi_cs_fd = &spi_cs1_fd;
  else
    spi_cs_fd = &spi_cs0_fd;

  status_value = close(*spi_cs_fd);
  if (status_value < 0) {
    std::cerr << "Error - Could not close SPI device" << std::endl;
    exit(1);
  }
  return (status_value);
}

//---- EasyCAT board initialization
//---------------------------------------------------------------

int EasyCAT::Connect() {
  int status_value = -1;
  int *spi_cs_fd;

  //----- SET SPI MODE -----
  // SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in
  // on rising edge, output data (change) on falling edge SPI_MODE_1 (0,1)
  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge,
  // output data (change) on rising edge SPI_MODE_2 (1,0) 	CPOL = 1, CPHA
  // = 0, Clock idle high, data is clocked in on falling edge, output data
  // (change) on rising edge SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle
  // high, data is clocked in on rising, edge output data (change) on falling
  // edge
  spi_mode = SPI_MODE_0;

  //----- SET BITS PER WORD -----
  spi_bitsPerWord = 8;

  //----- SET SPI BUS SPEED -----
  spi_speed = 1000000; // 1000000 = 1MHz (1uS per bit)

  std::string spi_device_name;

  if (spi_device == 0) {
    spi_cs_fd = &spi_cs0_fd;
    spi_device_name = "/dev/spidev0.0";
  } else if (spi_device == 1) {
    spi_cs_fd = &spi_cs1_fd;
    spi_device_name = "/dev/spidev0.1";
  } else {
    std::cerr << "Error - Invalid spi device number: " << spi_device
              << std::endl;
    exit(1);
  }

  std::cout << "Choose: " << spi_device_name << std::endl;
  *spi_cs_fd = open(spi_device_name.c_str(), O_RDWR);

  if (*spi_cs_fd < 0) {
    std::cerr << "Error - Could not open SPI device" << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (status_value < 0) {
    std::cerr << "Could not set SPIMode (WR)...ioctl fail: " << status_value
              << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (status_value < 0) {
    std::cerr << "Could not set SPIMode (RD)...ioctl fail: " << status_value
              << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
  if (status_value < 0) {
    std::cerr << "Could not set SPI bitsPerWord (WR)...ioctl fail: "
              << status_value << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
  if (status_value < 0) {
    std::cerr << "Could not set SPI bitsPerWord(RD)...ioctl fail: "
              << status_value << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (status_value < 0) {
    std::cerr << "Could not set SPI speed (WR)...ioctl fail: " << status_value
              << std::endl;
    exit(1);
  }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (status_value < 0) {
    std::cerr << "Could not set SPI speed (RD)...ioctl fail: " << status_value
              << std::endl;
    exit(1);
  }

  std::cout << "Connected successfully" << std::endl;
  return status_value;
}

//---- EtherCAT task
//------------------------------------------------------------------------------

unsigned char
EasyCAT::MainTask() // must be called cyclically by the application

{
  bool WatchDog = true;
  bool Operational = false;
  unsigned char i;
  ULONG TempLong;
  unsigned char Status;

  TempLong.Long =
      SPIReadRegisterIndirect(WDOG_STATUS, 1); // read watchdog status
  if ((TempLong.Byte[0] & 0x01) == 0x01) {
    WatchDog = false; // set/reset the corrisponding flag
  } else {
    WatchDog = true;
  }

  TempLong.Long = SPIReadRegisterIndirect(
      AL_STATUS, 1);                // read the EtherCAT State Machine status
  Status = TempLong.Byte[0] & 0x0F; //
  if (Status == ESM_OP)             // to see if we are in operational state
    Operational = true;             //
  else                              // set/reset the corrisponding flag
    Operational = false;            //

  //--- process data transfer ----------
  //
  if (!Operational)                        // if watchdog is active or we are
  {                                        // not in operational state, reset
    for (i = 0; i < TOT_BYTE_NUM_OUT; i++) // the output buffer
      BufferOut.Byte[i] = 0;               //

#ifdef DEB // only if debug is enabled
    if (!Operational) {
      printf("Not operational\n");
    }
    if (WatchDog) {
      printf("WatchDog\n");
    }
#endif
  }

  else {
    SPIReadProcRamFifo(); // otherwise transfer process data from
  }                       // the EtherCAT core to the output buffer

  SPIWriteProcRamFifo(); // we always transfer process data from
                         // the input buffer to the EtherCAT core

  if (WatchDog)     // return the status of the State Machine
  {                 // and of the watchdog
    Status |= 0x80; //
  }                 //
  return Status;    //
}

//---- read a directly addressable registers
//-----------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterDirect(unsigned short Address)

// Address = register to read
{
  ULONG Result;
  UWORD Addr;
  Addr.Word = Address;
  char LocalBuff[7];

  LocalBuff[0] = COMM_SPI_READ; // SPI read command
  LocalBuff[1] = Addr.Byte[1];  // address of the register to read
  LocalBuff[2] = Addr.Byte[0];  // MsByte first

  SPI_BuffTransfer(&LocalBuff[0], 7); // SPI transfer

  Result.Byte[0] = LocalBuff[3]; // read the result
  Result.Byte[1] = LocalBuff[4]; //
  Result.Byte[2] = LocalBuff[5]; //
  Result.Byte[3] = LocalBuff[6]; //

  return Result.Long; // return the result
}

//---- write a directly addressable registers
//----------------------------------------------------

void EasyCAT::SPIWriteRegisterDirect(unsigned short Address,
                                     unsigned long DataOut)

// Address = register to write
// DataOut = data to write
{
  ULONG Data;
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;
  char LocalBuff[7];

  LocalBuff[0] = COMM_SPI_WRITE; // SPI write command
  LocalBuff[1] = Addr.Byte[1];   // address of the register to write
  LocalBuff[2] = Addr.Byte[0];   // MsByte first

  LocalBuff[3] = Data.Byte[0]; // data to write
  LocalBuff[4] = Data.Byte[1]; // LsByte first
  LocalBuff[5] = Data.Byte[2]; //
  LocalBuff[6] = Data.Byte[3]; //

  SPI_BuffTransfer(&LocalBuff[0], 7); // SPI transfer
}

//---- read an indirectly addressable registers
//--------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterIndirect(unsigned short Address,
                                               unsigned char Len)

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// a long is returned but only the requested bytes
// are meaningful, starting from LsByte
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;
  // compose the command
  //
  TempLong.Byte[0] = Addr.Byte[0]; // address of the register to read
  TempLong.Byte[1] = Addr.Byte[1]; // LsByte first
  TempLong.Byte[2] = Len;          // number of bytes to read
  TempLong.Byte[3] = ESC_READ;     // ESC read

  SPIWriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long); // write the command

  do { // wait for command execution
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD); //
  }                                                      //
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);              //

  TempLong.Long =
      SPIReadRegisterDirect(ECAT_CSR_DATA); // read the requested register
  return TempLong.Long;                     //
}

//---- write an indirectly addressable registers
//-------------------------------------------------

void EasyCAT::SPIWriteRegisterIndirect(unsigned long DataOut,
                                       unsigned short Address,
                                       unsigned char Len)

// Address = register to write
// DataOut = data to write
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;

  SPIWriteRegisterDirect(ECAT_CSR_DATA, DataOut); // write the data

  // compose the command
  //
  TempLong.Byte[0] = Addr.Byte[0]; // address of the register to write
  TempLong.Byte[1] = Addr.Byte[1]; // LsByte first
  TempLong.Byte[2] = Len;          // number of bytes to write
  TempLong.Byte[3] = ESC_WRITE;    // ESC write

  SPIWriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long); // write the command

  do // wait for command execution
  {  //
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD); //
  }                                                      //
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);              //
}

//---- read from process ram fifo
//----------------------------------------------------------------

void EasyCAT::SPIReadProcRamFifo() // read data from the output process ram,
                                   // through the fifo
                                   //
                                   // these are the bytes received from the
                                   // EtherCAT master and that will be use by
                                   // our application to write the outputs
{
  ULONG TempLong;
  char LocalBuff[64 + 3];

#if TOT_BYTE_NUM_OUT > 0

  SPIWriteRegisterDirect(ECAT_PRAM_RD_CMD,
                         PRAM_ABORT); // abort any possible pending transfer

  SPIWriteRegisterDirect(
      ECAT_PRAM_RD_ADDR_LEN,
      (0x00001000 | (static_cast<uint32_t>(TOT_BYTE_NUM_OUT) << 16)));
  // the high word is the num of bytes
  // to read 0xTOT_BYTE_NUM_OUT----
  // the low word is the output process
  // ram offset 0x----1000

  SPIWriteRegisterDirect(ECAT_PRAM_RD_CMD, 0x80000000); // start command

  //------- one round is enough if we have -----------
  //------- to transfer up to 64 bytes ---------------

  do // wait for the data to be
  {  // transferred from the output
    TempLong.Long =
        SPIReadRegisterDirect(ECAT_PRAM_RD_CMD); // process ram to the read fifo
  }                                              //
  while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT / 4)); //

  LocalBuff[0] = COMM_SPI_READ; // SPI read command
  LocalBuff[1] = 0x00;          // address of the read FIFO
  LocalBuff[2] = 0x00;          // MsByte first

  SPI_BuffTransfer(&LocalBuff[0], 3 + FST_BYTE_NUM_ROUND_OUT); // SPI transfer

  memcpy(reinterpret_cast<char *>(&BufferOut.Byte[0]), &LocalBuff[3],
         FST_BYTE_NUM_ROUND_OUT);
#endif

#if SEC_BYTE_NUM_OUT > 0 //-- if we have to transfer more then 64 bytes -----
                         //-- we must do another round ----------------------
                         //-- to transfer the remainig bytes ----------------

  do // wait for the data to be
  {  // transferred from the output
    TempLong.Long =
        SPIReadRegisterDirect(ECAT_PRAM_RD_CMD); // process ram to the read fifo
  }                                              //
  while (TempLong.Byte[1] != SEC_BYTE_NUM_ROUND_OUT / 4); //

  LocalBuff[0] = COMM_SPI_READ; // SPI read command
  LocalBuff[1] = 0x00;          // address of the read FIFO
  LocalBuff[2] = 0x00;          // MsByte first

  SPI_BuffTransfer(&LocalBuff[0], 3 + SEC_BYTE_NUM_ROUND_OUT); // SPI transfer

  memcpy((char *)&BufferOut.Byte[64], &LocalBuff[3], SEC_BYTE_NUM_ROUND_OUT);
#endif
}

//---- write to the process ram fifo
//--------------------------------------------------------------

void EasyCAT::SPIWriteProcRamFifo() // write data to the input process ram,
                                    // through the fifo
                                    //
                                    // these are the bytes that we have read
                                    // from the inputs of our application and
                                    // that will be sent to the EtherCAT master
{
  ULONG TempLong;
  char LocalBuff[64 + 3];

#if TOT_BYTE_NUM_IN > 0

  SPIWriteRegisterDirect(ECAT_PRAM_WR_CMD,
                         PRAM_ABORT); // abort any possible pending transfer

  SPIWriteRegisterDirect(
      ECAT_PRAM_WR_ADDR_LEN,
      (0x00001200 | (static_cast<uint32_t>(TOT_BYTE_NUM_IN) << 16)));
  // the high word is the num of bytes
  // to write 0xTOT_BYTE_NUM_IN----
  // the low word is the input process
  // ram offset  0x----1200

  SPIWriteRegisterDirect(ECAT_PRAM_WR_CMD, 0x80000000); // start command

  //------- one round is enough if we have -----------
  //------- to transfer up to 64 bytes ---------------

  do // check that the fifo has
  {  // enough free space
    TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_WR_CMD); //
  }                                                          //
  while (TempLong.Byte[1] < (FST_BYTE_NUM_ROUND_IN / 4));    //

  LocalBuff[0] = COMM_SPI_WRITE; // SPI write command
  LocalBuff[1] = 0x00;           // address of the write fifo
  LocalBuff[2] = 0x20;           // MsByte first

  memcpy(&LocalBuff[3], reinterpret_cast<char *>(&BufferIn.Byte[0]),
         FST_BYTE_NUM_ROUND_IN);

  SPI_BuffTransfer(&LocalBuff[0], 3 + FST_BYTE_NUM_ROUND_IN); // SPI transfer
#endif

#if SEC_BYTE_NUM_IN > 0 //-- if we have to transfer more then 64 bytes -----
                        //-- we must do another round ----------------------
                        //-- to transfer the remainig bytes ----------------

  do // check that the fifo has
  {  // enough free space
    TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_WR_CMD); //
  }                                                          //
  while (TempLong.Byte[1] < (SEC_BYTE_NUM_ROUND_IN / 4));    //

  LocalBuff[0] = COMM_SPI_WRITE; // SPI write command
  LocalBuff[1] = 0x00;           // address of the write fifo
  LocalBuff[2] = 0x20;           // MsByte first

  memcpy(&LocalBuff[3], (char *)&BufferIn.Byte[64], SEC_BYTE_NUM_ROUND_IN);

  SPI_BuffTransfer(&LocalBuff[0], 3 + SEC_BYTE_NUM_ROUND_IN); // SPI transfer
#endif
}
// NOLINTEND
