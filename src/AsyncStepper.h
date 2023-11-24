#ifndef ASYNC_STEPPER_H_
#define ASYNC_STEPPER_H_

#include <Arduino.h>
#include <HardwareTimer.h>

// Class for managing a simple, constant speed async stepper driver with feedback to count steps
// Run steps using STM32 hardware PWM outputs
class AsyncStepper{
    public:
        typedef enum
        {
            DIRECTION_CCW = 0,  ///< Counter-Clockwise
            DIRECTION_CW  = 1,   ///< Clockwise
            DIRECTION_STOP = 2   ///< Stationary
        } Direction;
        AsyncStepper(uint8_t pwmPin_, uint8_t dirPin_, uint8_t enPin_, HardwareTimer* Timer_, uint32_t PWMchannel_, uint32_t maxSpeed_);
        AsyncStepper(uint8_t pwmPin_, uint8_t dirPin_, uint8_t enPin_, HardwareTimer* Timer_, uint32_t PWMchannel_){
            AsyncStepper(pwmPin_, dirPin_, enPin_, Timer_, PWMchannel_, 40000);
        }
        void runSpeed();
        void runSpeedToPosition(int32_t pos);
        void stop();
        void setMaxSpeed(int32_t speed);
        void setSpeed(int32_t speed);
        void setCurrentPosition(int32_t pos);
        void enable();
        void disable();

        int32_t speed();
        int32_t currentPosition();
        int32_t targetPosition();    
        byte direction();

        void stepTimerISR();
        HardwareTimer *Timer;
        uint32_t PWMchannel;
    
        byte _pwmPin;
        byte _dirPin;
        byte _fbPin;
        byte _enPin;

        volatile byte _mode;
        int32_t _maxSpeed;
        volatile int32_t _speed;
        volatile byte _direction;

        volatile int32_t _currentPos;
        volatile int32_t _targetPos;
        
};

#endif
