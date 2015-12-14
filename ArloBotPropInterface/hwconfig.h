#ifndef HWCONFIG_H
#define HWCONFIG_H

#include "const.h"

//---------------
// Hardware
//---------------

// A to D converter (used for Analog IR sensors)
// MCP3208 - SPI - 3 pins (CS, CLK, DATA), Breakout Board - $17.52 (proto-vantage)
//  The advantage is that this code is already written and should work.  The disadvantage is that
//  the pins can't really be shared with other devices.
// AD7828 (RB-Gra-10) - I2C - 2 pins (SDA, SCL), Breakout Board - $20 (robotshop)
//  The advantage is that it only takes 2 wires to support multiple devices (other device is IMU) and the i2c library already exists
//  The disadvantage is I have write code for it ;-(
// Total - MCP3208 - 3/RP-Gra-10 - 2

//--------------------------------------------------------------------------
// Pin Configuration
//--------------------------------------------------------------------------

// Propeller A/D Converter Pins
#define PROP_A2D_CS (21)
#define PROP_A2D_SCL (20)
#define PROP_A2D_DO (19)
#define PROP_A2D_DI (18)

// IMU
// Supports I2C and SPI interfaces
// I2C requires 2 pins (0 if share with other devices)
// SPI requires 4 pins

// Ultrasonic Sensors
// HCSR04 - 2 pins (Trigger, Echo)
//    It would be nice to get this working on a single pin, then only 1 pin is required and only one mux.
//    Wiring harness already supports separate pins, don't wanna redo the harness
// Mux - 4 pins
// Total - 6 pins

// Digital IR Sensors
// 5 devices, each requires a single pin.
// Total - 5 pins

// Motion sensor
// 1 pin


// Total - 2 (i2c) + 4 (mux) + 2 (ultrasonic) + 5 (digital ir) + 1 (motion detector) = 14 pins


// I2C
#define I2C_SDA (0)
#define I2C_SCL (1)


// A-To-D Converter for IR Sensors
#define AD7828_SDA (I2C_SDA)
#define AD7828_SCL (I2C_SCL)

#define IMU_SDA (I2C_SDA)
#define IMU_SCL (I2C_SCL)

// Multiplexer
// Pins 2, 3, 4, 5 are used
#define MUX_ADDR_START (2)
#define MUX_ADDR_END   (5)

// Pins for Ultrasonic Sensors
#define TRIG_PIN (6)
#define ECHO_PIN (7)

// Pins for Digital IR Sensors
#define DIGITAL_IR_PIN_START (8)
#define DIGITAL_IR_PIN_END   (12)

// Motion Detector
#define MOTION_DETECT (13)

// Motors Relay
#define MOTORS_RELAY (14) // Note: one pin can drive the left and right motor relays
#define MOTORS_ON    (HIGH)
#define MOTORS_OFF   (LOW)

//--------------------------------------------------------------------------
// Sensor Configuration
//--------------------------------------------------------------------------
#define NUM_ANALOG_IR_SENSORS (8)
#define NUM_DIGITAL_IR_SENSORS (5)
#define NUM_ULTRASONIC_SENSORS (16)

#endif