/* Copyright 2020-2021, Edwin Chiu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "checksum.h"
#include <stdint.h>

uint16_t ChecksumFletcher16(const char *data, uint8_t count,
                            uint16_t state /*=0*/) {
  uint8_t s1 = static_cast<uint8_t>(state & 0xff);
  uint8_t s2 = static_cast<uint8_t>((state >> 8) & 0xff);
  for (uint8_t index = 0; index < count; ++index) {
    s1 = static_cast<uint8_t>(
        (uint16_t{s1} + static_cast<uint16_t>(data[index])) % 255);
    s2 = static_cast<uint8_t>((uint16_t{s2} + uint16_t{s1}) % 255);
  }
  return static_cast<uint16_t>((uint16_t{s2} << 8) | s1);
}

// The polynomial 0x741B8CD7 has Hamming distance 6 up to 16360 bits
// and Hamming distance 4 up to 114663 bits.
//[Philip Koopman, 32-Bit Cyclic Redundancy Codes for Internet Applications
// 2002.] https://users.ece.cmu.edu/~koopman/crc/
// Table generated using
// http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
uint32_t CRC32Single(uint32_t crc, uint8_t data) {
  static constexpr uint32_t CRCTable[16] = {
      // Nibble lookup table for 0x741B8CD7 polynomial
      0x00000000, 0x741B8CD7, 0xE83719AE, 0x9C2C9579, 0xA475BF8B, 0xD06E335C,
      0x4C42A625, 0x38592AF2, 0x3CF0F3C1, 0x48EB7F16, 0xD4C7EA6F, 0xA0DC66B8,
      0x98854C4A, 0xEC9EC09D, 0x70B255E4, 0x04A9D933};

  crc = crc ^ static_cast<uint32_t>(data); // Apply all 32-bits

  // Process 32-bits, 4 at a time, or 8 rounds

  crc = (crc << 4) ^
        CRCTable[crc >> 28]; // Assumes 32-bit reg, masking index to 4-bits
  crc =
      (crc << 4) ^ CRCTable[crc >> 28]; //  0x04C11DB7 Polynomial used in STM32
  crc = (crc << 4) ^ CRCTable[crc >> 28];
  crc = (crc << 4) ^ CRCTable[crc >> 28];
  crc = (crc << 4) ^ CRCTable[crc >> 28];
  crc = (crc << 4) ^ CRCTable[crc >> 28];
  crc = (crc << 4) ^ CRCTable[crc >> 28];
  crc = (crc << 4) ^ CRCTable[crc >> 28];

  return crc;
}

uint32_t SoftCRC32(const uint8_t *data, uint32_t count) {
  if (0 == count) {
    return 0;
  }

  uint32_t crc = 0xFFFFFFFF;
  while (count--) {
    crc = CRC32Single(crc, *data++);
  }
  return crc;
}
