
#ifndef EasyCAT_H
#define EasyCAT_H

#include <stdint.h>

#if (defined CUSTOM)
#include "TestEasyCAT_Custom.h" // This file has been created by the Easy Configurator
// and must be located in the Arduino project folder
//
// There are two others files created by the Easy Configurator:
// TestEasyCAT_Custom.bin that must be loaded into the EEPROM.
// TestEasyCAT_Custom.xml that must be used by the EtherCAT master.
#endif

//*************************************************************************************

#if (!defined BYTE_NUM &&                                                      \
     !defined CUSTOM) // if BYTE_NUM is not declared in the .ino file
#define BYTE_NUM 32   // set it to the 32+32 byte default
#endif                // this is for compatibility with old applications

//-- the preprocessor calculates the parameters necessary to transfer out data
//---

// define TOT_BYTE_NUM_OUT as the
// total number of byte we have to
// transfer in output
// this must match the ESI XML
//
#ifdef BYTE_NUM // in Standard Mode
#define TOT_BYTE_NUM_OUT                                                       \
  BYTE_NUM                                 // 16, 32, 64 or 128
                                           //
#else                                      // in Custom Mode
#define TOT_BYTE_NUM_OUT CUST_BYTE_NUM_OUT // any number between 0 and 128
#endif                                     //

#if TOT_BYTE_NUM_OUT > 64 // if we have more then  64 bytes
                          // we have to split the transfer in two

#define SEC_BYTE_NUM_OUT                                                       \
  (TOT_BYTE_NUM_OUT - 64) // number of bytes of the second transfer

#if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00) // number of bytes of the second
                                        // transfer rounded to long
#define SEC_BYTE_NUM_ROUND_OUT ((SEC_BYTE_NUM_OUT | 0x03) + 1)
#else                                           //
#define SEC_BYTE_NUM_ROUND_OUT SEC_BYTE_NUM_OUT //
#endif                                          //

#define FST_BYTE_NUM_OUT 64 // number of bytes of the first transfer
#define FST_BYTE_NUM_ROUND_OUT                                                 \
  64 // number of bytes of the first transfer
     // rounded to 4 (long)

#else // if we have max 64 bytes we transfer
      // them in just one round

#define FST_BYTE_NUM_OUT                                                       \
  TOT_BYTE_NUM_OUT // number of bytes of the first and only transfer

#if ((FST_BYTE_NUM_OUT & 0x03) != 0x00) // number of bytes of the first and only
                                        // transfer rounded to 4 (long)
#define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
#else                                           //
#define FST_BYTE_NUM_ROUND_OUT FST_BYTE_NUM_OUT //
#endif                                          //

#define SEC_BYTE_NUM_OUT 0       // we don't use the second round
#define SEC_BYTE_NUM_ROUND_OUT 0 //

#endif

//-- the preprocessor calculates the parameters necessary to transfer in data
//---

// define TOT_BYTE_NUM_IN as the
// total number of byte we have to
// transfer in input
// this must match the ESI XML
//
#ifdef BYTE_NUM // in Standard Mode
#define TOT_BYTE_NUM_IN                                                        \
  BYTE_NUM                               // 16, 32, 64 or 128
                                         //
#else                                    // in Custom Mode
#define TOT_BYTE_NUM_IN CUST_BYTE_NUM_IN // any number between 0 and 128
#endif                                   //

#if TOT_BYTE_NUM_IN > 64 // if we have more then  64 bytes
                         // we have to split the transfer in two

#define SEC_BYTE_NUM_IN                                                        \
  (TOT_BYTE_NUM_IN - 64) // number of bytes of the second transfer

#if ((SEC_BYTE_NUM_IN & 0x03) != 0x00) // number of bytes of the second transfer
                                       // rounded to 4 (long)
#define SEC_BYTE_NUM_ROUND_IN ((SEC_BYTE_NUM_IN | 0x03) + 1)
#else                                         //
#define SEC_BYTE_NUM_ROUND_IN SEC_BYTE_NUM_IN //
#endif                                        //

#define FST_BYTE_NUM_IN 64 // number of bytes of the first transfer
#define FST_BYTE_NUM_ROUND_IN                                                  \
  64 // number of bytes of the first transfer
     // rounded to 4 (long)

#else // if we have max 64 bytes we transfer
      // them in just one round

#define FST_BYTE_NUM_IN                                                        \
  TOT_BYTE_NUM_IN // number of bytes of the first and only transfer

#if ((FST_BYTE_NUM_IN & 0x03) != 0x00) // number of bytes of the first and only
                                       // transfer rounded to 4 (long)
#define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
#else                                         //
#define FST_BYTE_NUM_ROUND_IN FST_BYTE_NUM_IN //
#endif                                        //

#define SEC_BYTE_NUM_IN 0       // we don't use the second round
#define SEC_BYTE_NUM_ROUND_IN 0 //

#endif

//---------------------------------------------------------------------------------

//----------------- sanity check
//-------------------------------------------------------

#ifdef BYTE_NUM // STANDARD MODE and CUSTOM MODE
                // cannot be defined at the same time
#ifdef CUST_BYTE_NUM_OUT
#error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
#error "define them correctly in file EasyCAT.h"
#endif

#ifdef CUST_BYTE_NUM_IN
#error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
#error "define them correctly in file EasyCAT.h"
#endif
#endif

#ifdef BYTE_NUM //--- for BYTE_NUM we accept only 16  32  64  128 --

#if ((BYTE_NUM != 16) && (BYTE_NUM != 32) && (BYTE_NUM != 64) &&               \
     (BYTE_NUM != 128))
#error                                                                         \
    "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
#endif

#else
//--- CUST_BYTE_NUM_OUT and CUST_BYTE_NUM_IN --------
//    must be max 128
#if (CUST_BYTE_NUM_OUT > 128)
#error                                                                         \
    "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
#endif

#if (CUST_BYTE_NUM_IN > 128)
#error                                                                         \
    "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
#endif

#endif

//*************************************************************************************************

//---- LAN9252 registers
//--------------------------------------------------------------------------

//---- access to EtherCAT registers -------------------

#define ECAT_CSR_DATA 0x0300 // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD 0x0304  // EtherCAT CSR Interface Command Register

//---- access to EtherCAT process RAM -----------------

#define ECAT_PRAM_RD_ADDR_LEN                                                  \
  0x0308 // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD 0x030C // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN                                                  \
  0x0310 // EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD 0x0314 // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA 0x0000 // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA 0x0020 // EtherCAT Process RAM Write Data FIFO

//---- EtherCAT registers -----------------------------

#define AL_STATUS 0x0130     // AL status
#define WDOG_STATUS 0x0440   // watch dog status
#define AL_EVENT_MASK 0x0204 // AL event interrupt mask

//---- LAN9252 registers ------------------------------

#define HW_CFG 0x0074    // hardware configuration register
#define BYTE_TEST 0x0064 // byte order test register
#define RESET_CTL 0x01F8 // reset register
#define ID_REV 0x0050    // chip ID and revision
#define IRQ_CFG 0x0054   // interrupt configuration
#define INT_EN 0x005C    // interrupt enable

//---- LAN9252 flags
//------------------------------------------------------------------------------

#define ECAT_CSR_BUSY 0x80

#define PRAM_ABORT 0x40000000

#define PRAM_BUSY 0x80

#define PRAM_AVAIL 0x01

#define READY 0x08

#define DIGITAL_RST 0x00000001

//---- EtherCAT flags
//-----------------------------------------------------------------------------

// EtherCAT state machine

#define ESM_INIT 0x01   // init
#define ESM_PREOP 0x02  // pre-operational
#define ESM_BOOT 0x03   // bootstrap
#define ESM_SAFEOP 0x04 // safe-operational
#define ESM_OP 0x08     // operational

//--- ESC commands
//--------------------------------------------------------------------------------

#define ESC_WRITE 0x80
#define ESC_READ 0xC0

//---- SPI
//----------------------------------------------------------------------------------------

#define COMM_SPI_READ 0x03
#define COMM_SPI_WRITE 0x02

//---- typedef
//------------------------------------------------------------------------------------

typedef union {
  unsigned short Word;
  unsigned char Byte[2];
} UWORD;

typedef union {
  unsigned long Long;
  unsigned short Word[2];
  unsigned char Byte[4];
} ULONG;

#include "EasyCAT_interface.h"

typedef enum { ASYNC, DC_SYNC, SM_SYNC } SyncMode;

//-------------------------------------------------------------------------------------------------

class EasyCAT : public EasyCAT_interface {
public:
  EasyCAT() = default; // default constructor
  ~EasyCAT() { Disconnect(); }

  unsigned char
  MainTask() override; // EtherCAT main task
                       // must be called cyclically by the application

  const PROCBUFFER_OUT &getBufferOut() override { return BufferOut; }
  PROCBUFFER_IN &getBufferIn() override { return BufferIn; }

  int Connect() override; // EasyCAT board initialization
  int Disconnect() override;

private:
  void SPIWriteRegisterDirect(unsigned short Address, unsigned long DataOut);
  unsigned long SPIReadRegisterDirect(unsigned short Address);

  void SPIWriteRegisterIndirect(unsigned long DataOut, unsigned short Address,
                                unsigned char Len);
  unsigned long SPIReadRegisterIndirect(unsigned short Address,
                                        unsigned char Len);

  void SPIReadProcRamFifo();
  void SPIWriteProcRamFifo();

  void SPI_BuffTransfer(char *Buff, uint32_t Len);

  int spi_cs0_fd; // file descriptor for the SPI device
  int spi_cs1_fd; // file descriptor for the SPI device
  unsigned char spi_mode;
  unsigned char spi_bitsPerWord;
  unsigned int spi_speed;
  int spi_device = 0; // select SPI device

  PROCBUFFER_OUT BufferOut; // output process data buffer
  PROCBUFFER_IN BufferIn;   // input process data buffer
};

#endif
