#ifndef EasyCAT_stub_H
#define EasyCAT_stub_H

#include "EasyCAT_interface.h"

class EasyCAT_stub : public EasyCAT_interface {
public:
  EasyCAT_stub() = default; // default constructor
  ~EasyCAT_stub() = default;

  unsigned char MainTask() override { return 0; }; // EtherCAT main task
  // must be called cyclically by the application

  const PROCBUFFER_OUT &getBufferOut() override { return BufferOut; }
  PROCBUFFER_IN &getBufferIn() override { return BufferIn; }

  int Connect() override { return 0; }
  int Disconnect() override { return 0; }

private:
  PROCBUFFER_OUT BufferOut; // output process data buffer
  PROCBUFFER_IN BufferIn;   // input process data buffer
};

#endif
