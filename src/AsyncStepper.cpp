#include <AsyncStepper.h>

/**
 * STM32 specific step-direction driver 
 * 
 * @param pwmPin Step pin. Must have a distinct hardware timer module from any other instance unless you want your steppers to run at the same time
 * @param dirPin Dir pin
 * @param enPin  En pin
 * @param Timer  Pointer to HardwareTimer object for PWM configuration (you could also modify this library to use the lower level TIMx instance)
 * @param PWMchannel PWM channel, must match step pin selection
 * @param maxSpeed   Speed limit in steps/second
*/
AsyncStepper::AsyncStepper(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, HardwareTimer* Timer_, uint32_t PWMchannel_, uint32_t maxSpeed){
    _pwmPin = pwmPin;
    _dirPin = dirPin;
    _enPin = enPin;
    _maxSpeed = maxSpeed;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 1000;                  // arbitrary, some default value. 
    _direction = DIRECTION_STOP;
    Timer = Timer_;
    PWMchannel = PWMchannel_;

    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    
    Timer->setMode(PWMchannel, TIMER_OUTPUT_COMPARE_PWM1,_pwmPin);
    Timer->setOverflow(_speed,HERTZ_FORMAT);
    Timer->setCaptureCompare(PWMchannel, 20, PERCENT_COMPARE_FORMAT);
    Timer->refresh();
}

void AsyncStepper::enable()
{
    digitalWrite(_enPin, LOW);
}
void AsyncStepper::disable()
{
    digitalWrite(_enPin, HIGH);
}


// Run continuously at current speed.
// negative speed = negative rotation direction.
void AsyncStepper::runSpeed()
{
    _mode = 0;
    if(_speed >= 0) 
    {
        digitalWrite(_dirPin, _direction=DIRECTION_CW);
    }
    else 
    {
        digitalWrite(_dirPin, _direction=DIRECTION_CCW);
    }
    // STM32 hardware timer 
    Timer->resume();
}

// Move to target position at current speed. Target overrides speed. 
// if _speed is negative, but the target needs positive speed to reach,
// it will run positive speed.
void AsyncStepper::runSpeedToPosition(int32_t pos)
{
    _mode = 1;
    _targetPos = pos;
    if(pos-_currentPos > 0) 
    {
        digitalWrite(_dirPin, _direction=DIRECTION_CW);
    }
    else if(pos-_currentPos < 0)
    {
        digitalWrite(_dirPin, _direction=DIRECTION_CCW);
    }
    else{
        _direction = DIRECTION_STOP;
    }

    if(_currentPos != _targetPos)
    {
        // STM32 hardware timer 
        Timer->resume();
    }
    else{
        stop();
    }
}

void AsyncStepper::stop()
{
    _direction = DIRECTION_STOP;
    // STM32 hardware timer 
    Timer->pause();
}

void AsyncStepper::setMaxSpeed(int32_t maxSpeed_)
{
    _maxSpeed = abs(maxSpeed_);
}

void AsyncStepper::setSpeed(int32_t speed)
{
    if(speed == 0)
    {
        return;
    }
    if(abs(speed) > _maxSpeed )
    {
        _speed = (speed > 0 ? _maxSpeed : -_maxSpeed);
    }
    else
    {
        _speed = speed;
    }

    // STM32 hardware timer method
    //Timer->setPWM(PWMchannel, _pwmPin, _speed, 20);
    Timer->setMode(PWMchannel, TIMER_OUTPUT_COMPARE_PWM1,_pwmPin);
    Timer->setOverflow(_speed,HERTZ_FORMAT);
    Timer->setCaptureCompare(PWMchannel, 20, PERCENT_COMPARE_FORMAT);
    Timer->refresh();
}

void AsyncStepper::setCurrentPosition(int32_t pos)
{
    stop();
    _currentPos = pos;
}

void AsyncStepper::stepTimerISR()
{
    if(_currentPos == _targetPos && _mode==1)
    {   
        // STM32 hardware timer 
        Timer->pause();
        _direction = DIRECTION_STOP;
    }
    
    switch(_direction)
    {
        case DIRECTION_CCW:
            _currentPos--;
            break;
        case DIRECTION_CW:
            _currentPos++;
            break;
        case DIRECTION_STOP:
        default:
            break;
    }
}

int32_t AsyncStepper::speed(){
    return _speed;
}

int32_t AsyncStepper::currentPosition(){
    return _currentPos;
}

int32_t AsyncStepper::targetPosition(){
    return _targetPos;
}

byte AsyncStepper::direction(){
    return _direction;
}




