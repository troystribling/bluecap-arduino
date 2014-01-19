#include "byte_swap.h"

uint16_t uint16BigToHost(uint16_t bigVal) {
  return (bigVal << 8) | (bigVal >> 8);
}

uint16_t uint16LittleToHost(uint16_t littleVal) {
  return littleVal;
}
