#pragma once

#include <cstddef>
#include <stdint.h>

namespace BoardConfig {

// Signal Output GPIO Assignments (ESP32-S3 DevKitC-1-N16R8)
// North-South Axis
constexpr int PIN_NS_RED = 4;
constexpr int PIN_NS_YELLOW = 5;
constexpr int PIN_NS_GREEN = 6;

// East-West Axis
constexpr int PIN_EW_RED = 7;
constexpr int PIN_EW_YELLOW = 8;
constexpr int PIN_EW_GREEN = 9;

// UART0 Hardware Configuration
constexpr uint32_t UART_BAUD_RATE = 115200;
constexpr int PIN_UART0_TX = 43;
constexpr int PIN_UART0_RX = 44;

// Serial Ingestion Buffer and Processing Budget
constexpr size_t SERIAL_BUF_SIZE = 512;
constexpr size_t MAX_BYTES_PER_LOOP = 256;

} // namespace BoardConfig
