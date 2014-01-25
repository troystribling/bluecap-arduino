#ifndef _BYTE_SWAP_H_
#define _BYTE_SWAP_H_

#include <stdint.h>

extern "C" {

  uint16_t int16BigToHost(uint16_t bigVal);
  uint16_t int16LittleToHost(uint16_t littleVal);

  uint16_t int16HostToBig(uint16_t littleVal);
  uint16_t int16HostToLittle(uint16_t littleVal);

}

#endif