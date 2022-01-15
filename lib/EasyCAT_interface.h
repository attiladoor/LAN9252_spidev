#ifndef EASYCAT_INTERFACE_HPP
#define EASYCAT_INTERFACE_HPP

#include <stdint.h>

#ifndef BYTE_NUM
#define BYTE_NUM 32
#endif

typedef struct            //-- output buffer -----------------
{                         //
  uint8_t Byte[BYTE_NUM]; //
} PROCBUFFER_OUT;         //

typedef struct            //-- input buffer ------------------
{                         //
  uint8_t Byte[BYTE_NUM]; //
} PROCBUFFER_IN;          //

class EasyCAT_interface {
public:
  virtual unsigned char MainTask() = 0;

  virtual const PROCBUFFER_OUT &getBufferOut() = 0;
  virtual PROCBUFFER_IN &getBufferIn() = 0;

  virtual int Connect() = 0;
  virtual int Disconnect() = 0;

  virtual ~EasyCAT_interface() = default;
};

#endif