#include "byte_swap.h"

uint16_t uint16BigToLittleEndian(uint16_t bigVal) {
  return (bigVal << 8) | (bigVal >> 8);
}