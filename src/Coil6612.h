#ifndef TB6612_H_
#define TB6612_H_

#include <HardwareTimer.h>
#include <stdint.h>

/**
 * Simple class to excite coil using AC voltage from TB6612 H-bridge
*/
class Coil6612{
    public: 
        Coil6612(uint8_t pinA_, uint8_t pinB_, uint8_t pwmPin_, HardwareTimer* Timer_);
        HardwareTimer* Timer;
        uint8_t _pinA;
        uint8_t _pinB;
        uint8_t _pwmPin;

        
        void up();
        void down();
        void stop();
};

#endif