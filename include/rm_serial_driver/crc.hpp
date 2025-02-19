#pragma once

#include <cstddef>
#include <cstdint>

#define CRC16_INIT 0xFFFF

namespace crc16 {

uint16_t CRC16_Calc(const uint8_t *buf, size_t len, uint16_t crc);

bool CRC16_Verify(const uint8_t *buf, size_t len);

} // namespace crc16