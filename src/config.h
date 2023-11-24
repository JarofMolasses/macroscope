/**
 * SETTING PIN B0 AS INPUT INTERFERES WITH SPI
 * PA1 and PA0 interfere with each other because shared T2.
 * You need to put X and Y steppers on completely different timers.
*/

#ifndef CONFIG_H_
#define CONFIG_H_

//#define ADC_SYNC_AVAILABLE

#include <Arduino.h> // Type definitions 

#define SINETABLESIZE 90
/**
 * Sine lookup table with 8-bit values to 90 degrees
*/
const unsigned char sinA [] PROGMEM = 
{0, 4, 8, 13, 17, 22, 26, 31, 35, 39, 44, 48, 53, 57, 61, 65, 
70, 74, 78, 83, 87, 91, 95, 99, 103, 107, 111, 115, 119, 123, 
127, 131, 135, 138, 142, 146, 149, 153, 156, 160, 163, 167, 
170, 173, 177, 180, 183, 186, 189, 192, 195, 198, 200, 203, 
206, 208, 211, 213, 216, 218, 220, 223, 225, 227, 229, 231, 
232, 234, 236, 238, 239, 241, 242, 243, 245, 246, 247, 248, 
249, 250, 251, 251, 252, 253, 253, 254, 254, 254, 254, 254, 255};


/** 
 * UART data tx/rx format enum
*/
enum logFormat_T{
    PACKET, 
    STRING
};

enum sineModes_T{
    BRAKE,              // PWM a sine wave using run/brake 
    COAST,              // PWM a sine wave using run/coast
    SQUARE              // PWM a square wave using run/brake
};


#define ADC_DRDY PA3
#define ADC_CS PA4
#if defined(ADC_SYNC_AVAILABLE)
    #define ADC_SYNC PA2
#else
    #define ADC_SYNC 0
#endif

#define stepPinx PB5
#define enPinx PB15
#define dirPinx PB2

#define stepPiny PA1
#define enPiny PA9
#define dirPiny PB1

#define coilInA PB6
#define coilInB PB7
#define coilPWMPinAux PA8


#endif