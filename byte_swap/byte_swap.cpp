#include "byte_swap.h"

uint16_t int16BigToHost(uint16_t bigVal) {
  return (bigVal << 8) | (bigVal >> 8);
}

uint16_t int16LittleToHost(uint16_t littleVal) {
  return littleVal;
}

uint16_t int16HostToBig(uint16_t hostVal) {
  return (hostVal << 8) | (hostVal >> 8);
}

uint16_t int16HostToLittle(uint16_t hostVal) {
  return hostVal;
}
