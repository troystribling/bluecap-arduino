#ifndef _BYTE_SWAP_H_
#define _BYTE_SWAP_H_

#include <stdint.h>

extern "C" {

  uint16_t uint16BigToHost(uint16_t bigVal);
  uint16_t uint16LittleToHost(uint16_t littleVal);

}

#endif