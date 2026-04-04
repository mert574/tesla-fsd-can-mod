#pragma once

// Shared sketch configuration for Arduino IDE and PlatformIO.
// Arduino IDE users should open RP2040CAN/RP2040CAN.ino.
// PlatformIO users should edit this file from the repository root.

// ── BOARD SELECTION ──────────────────────────────────────────────
// Uncomment ONE of the following lines to match your board:
#define DRIVER_MCP2515   // ESP32-C3 Super Mini + MCP2515 module via SPI
// #define DRIVER_SAME51    // Adafruit Feather M4 CAN Express (native ATSAME51 CAN)
// #define DRIVER_TWAI      // ESP32 boards with built-in TWAI (CAN) peripheral

// ── LILYGO T-CAN485 PIN OVERRIDE ────────────────────────────────
#define TWAI_TX_PIN GPIO_NUM_27
#define TWAI_RX_PIN GPIO_NUM_26

// ── VEHICLE HARDWARE SELECTION ───────────────────────────────────
// Uncomment ONE of the following lines to match your vehicle:
// #define LEGACY  // HW3-retrofit
#define HW3     // HW3
// #define HW4     // HW4

// ── BEHAVIOUR OPTIONS ────────────────────────────────────────────
// Uncomment any of the following lines:
// #define ISA_SPEED_CHIME_SUPPRESS      // Suppress ISA speed chime; speed limit sign will be empty while driving
// #define EMERGENCY_VEHICLE_DETECTION   // Enable emergency vehicle detection
// #define BYPASS_TLSSC_REQUIREMENT      // Always enable FSD without requiring "Traffic Light and Stop Sign Control" toggle
// #define NAG_KILLER                    // Suppress Autosteer "hands on wheel" nag (CAN 880 counter+1 echo, X179 pin 2/3)

// ── ESP32-C3 Super Mini board info ───────────────────────────────
// Board name: Nologo ESP32C3 Super Mini
// FQBN: esp32:esp32:nologo_esp32c3_super_mini
// Port: /dev/ttyACM1 (Linux) — verify with: arduino-cli board list
//
// ── ESP32-C3 Super Mini pin definitions ──────────────────────────
#define PIN_CAN_CS        1   // SPI CS
#define PIN_CAN_INTERRUPT 10  // MCP2515 INT pin
#define PIN_LED           8   // onboard blue LED (active LOW)

// Initialize SPI with ESP32-C3 Super Mini pins before driver setup
#define BOARD_SETUP_HOOK() do { \
    pinMode(PIN_LED, OUTPUT); \
    digitalWrite(PIN_LED, HIGH); \
    blinkLED(2); \
    SPI.begin(4, 3, 2, 1); \
} while(0)
