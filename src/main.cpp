/** Project: Atomic force macroscope
 * 
 * Board: Blackpill V2.0 STM32F411CEU6
 * Clock: 84MHz
 * 
 * See config.h for wiring scheme
 * 
 */

/**TIMER ALLOCATIONS:
 * TIM9 - main timer system event timer internal 
 *  CH1: UART
 *  CH2: LCD
 * 
 * TIM11 - secondary timer internal (unused)
 *  
 * TIM4 
 *  Ch1 - PB6 - coil PWMA     
 *  Ch2 - PB7 - coil PWMB
 * TIM1
 *  Ch1 - Aux PWM signal. 
 * 
 * TIM3 Ch2 - PB5 - X stepper pulse generator 
 * TIM2 Ch2 - PA1 - Y stepper pulse generator
 * What if you want a third stepper? 
 * TIM5 can be accessed as a secondary option on PA0
 * TIM4 can be freed up if we don't need the coast mode for the coil driver
 * TIM10 is hard to get at, only one pin carries it and it's not convenient
*/
#define LCD_LENGTH 20
#define ADC_BUFFER_SIZE 10

#include <HardwareTimer.h>
#include <SPI.h>
#include <Wire.h>
#include "AsyncStepper.h"
#include "ADS1256.h"
#include "config.h"
#include "liquidcrystal_i2c.h"
#include "Vrekrer_scpi_parser.h"
#include <queue>

/**
 * A silly debug LCD. 20x4 characters
*/
char lcdline0[LCD_LENGTH+1];
char lcdline1[LCD_LENGTH+1];
char lcdline2[LCD_LENGTH+1];
char lcdline3[LCD_LENGTH+1];

/* uart print buffer */
char uartBuffer[48];
char checksumBuffer[8];

/**
 * ADC components
*/
ADS1256 adc(ADC_DRDY, 0, ADC_SYNC, ADC_CS, 5.00);    
int32_t adcRawCounts = 0;
double adcVolts = 0;
double adcForce = 0;
double adcHeight = 0;
/**
 * Calibration constants
 * Force in cN
 * Distance in mm
*/
double counts_to_cN = 0.03;
double counts_offset = 0.00;
double force_to_mm = (1.0/19600)*1000;  // 19600 cN/m
/**
 * Amplitude measurement buffer
*/
std::queue<int32_t> adcBuffer;



/* Step pulse generators */
/**
 * Currently selects T3Ch2 and T2Ch2 automatically for X and Y steppers.
*/
TIM_TypeDef *Instance_x = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(stepPinx), PinMap_PWM);
uint32_t PWMchannel_x = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(stepPinx), PinMap_PWM));
HardwareTimer *Timer_x = new HardwareTimer(Instance_x);
TIM_TypeDef *Instance_y = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(stepPiny), PinMap_PWM);
uint32_t PWMchannel_y = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(stepPiny), PinMap_PWM));
HardwareTimer *Timer_y = new HardwareTimer(Instance_y);
AsyncStepper xStepper = AsyncStepper(stepPinx, dirPinx, enPinx, Timer_x, PWMchannel_x, 40000);
AsyncStepper yStepper = AsyncStepper(stepPiny, dirPiny, enPiny, Timer_y, PWMchannel_y, 40000);

/**
 * Coil driver components
 */     
HardwareTimer *coilPWMTimer = new HardwareTimer(TIM4);
HardwareTimer *auxPWMTimer = new HardwareTimer(TIM1);
uint8_t PWMchannel_coilA = 1;     // Check datasheet and pinout to find the right channel
uint8_t PWMchannel_coilB = 2;
uint8_t PWMchannel_coil = 1;
volatile uint32_t probeFrequency = 36;      
volatile uint32_t coilPWMFrequency = probeFrequency*2 * SINETABLESIZE*2;
volatile uint8_t coilSineDegrees = 0;   // Tracks the degree step through the sine table
// volatile uint8_t coilDutyCycle = 0;     // Current required duty cycle converted from sine table
volatile bool coilState = false;        // Tracks which coil input is active
volatile sineModes_T sineMode = SQUARE;  // Tracks TB6612 driving mode

/* Square/ brake mode specifics*/
volatile uint8_t auxDutyCycle = 30;      // aux (supply-side) PWM duty cycle for square wave drive
volatile uint8_t staticDutyCycle = 70;   // static duty cycle has to be higher to get appreciable tip deflection
volatile uint16_t sineTableOverflow = 0;          // Tto set peak voltage of sine wave from auxDutyCycle
volatile uint16_t sineTableDutyCycleFactor = 1;   // To set peak voltage of sine wave from auxDutyCycle


/**
 * System timers
 */           
HardwareTimer *mainTimer = new HardwareTimer(TIM9);      // HardwareTimer makes some configs convenient. low-level registers accessible through TIMx
HardwareTimer *tim11 = new HardwareTimer(TIM11);           // Convenient aux timer handle via HardwareTimer 
volatile uint16_t uartFreq = 50;
volatile uint16_t lcdFreq = 5;
volatile uint16_t uartTicks = 0;
volatile uint16_t lcdTicks = 0;
volatile bool ledOn = false;                          // UART debug LED
volatile bool executeStepFlag = false;
volatile bool lcdFlag = false;
volatile bool logOn = false;
volatile uint32_t rasterLines = 0;

SCPI_Parser parser;

/**
 * return: measured force in centinewtons
*/
double countsToCN(int32_t counts)
{
  return (counts - counts_offset)* counts_to_cN;
}
/**
 * return: measured displacement in millimeters
*/
double cNtoMillimeters(double cN)
{
  return cN*force_to_mm;
}
void adcUpdateAll()
{
  adcRawCounts = adc.readSingle();
  adcBuffer.push(adcRawCounts);
  if(adcBuffer.size() >= ADC_BUFFER_SIZE)
  {
    adcBuffer.pop();
  }
  adcVolts = adc.convertToVoltage(adcRawCounts);
  adcForce = countsToCN(adcRawCounts);
  adcHeight = cNtoMillimeters(adcForce);
}

void updateLCD(byte l1, byte l2, byte l3, byte l4)
{
  if(l1){
  snprintf(lcdline0, LCD_LENGTH+1, "ADC : %+0.8f mm",adcHeight);
  HD44780_SetCursor(0,0);
  HD44780_PrintStr(lcdline0);
  }
  if(l2){
  snprintf(lcdline1, LCD_LENGTH+1, "  X:%-6d Y:%-6d",xStepper.currentPosition(), yStepper.currentPosition());
  HD44780_SetCursor(0,1);
  HD44780_PrintStr(lcdline1);
  }
  if(l3){
  snprintf(lcdline2, LCD_LENGTH+1, "COIL: %-4d Hz",probeFrequency);
  HD44780_SetCursor(0,2);
  HD44780_PrintStr(lcdline2);
  }
  if(l4){
  snprintf(lcdline3, LCD_LENGTH+1, "UART: %9d Hz",uartFreq);
  HD44780_SetCursor(0,3);
  HD44780_PrintStr(lcdline3);
  }
}

/**
 * Main system timer ISR
*/
void timer9_1ISR()
{
  if(logOn)
  {
    ledOn = !ledOn;     
    digitalWrite(PC13, ledOn ? HIGH:LOW);
    // Byte packet version
    // uint8_t adcHighByte = (uint8_t)adcRawCounts>>16;
    // uint8_t adcMidByte = (uint8_t)adcRawCounts>>8;
    // uint8_t adcLowByte = (uint8_t)adcRawCounts;
    // uint8_t xHighByte = (uint8_t)xStepper.currentPosition()>>8;
    // uint8_t xLowByte = (uint8_t)xStepper.currentPosition();
    // uint8_t yHighByte = (uint8_t)yStepper.currentPosition()>>8;
    // uint8_t yLowByte = (uint8_t)yStepper.currentPosition();
    // uint8_t escape = 0;
    // Serial.write(0xFF);       // Start byte
    // Serial.write(adcHighByte);
    // Serial.write(adcMidByte);
    // Serial.write(adcLowByte);
    // Serial.write(xHighByte);
    // Serial.write(xLowByte);
    // Serial.write(yHighByte);
    // Serial.write(yLowByte);
    // Serial.write(escape);
    
    uint32_t checksum = sprintf(uartBuffer, "__DATA__,%d,%d,%d,", adcRawCounts, xStepper.currentPosition(), yStepper.currentPosition());
    sprintf(checksumBuffer, "%d", checksum);
    Serial.print(uartBuffer);
    Serial.println(checksum);
  }

  TIM9->CCR1 += uartTicks;
}
void timer9_2ISR()
{
  lcdFlag = true;   // Slow and unimportant, do it in the loop

  TIM9->CCR2 += lcdTicks;
}

/**
 * Secondary system timer ISR
*/
void timer11_0ISR()
{
  executeStepFlag = true;
  if(rasterLines++ == 100)
  {
        xStepper.runSpeedToPosition(xStepper.currentPosition()+100);
        rasterLines = 0;
  }
  yStepper.runSpeedToPosition(yStepper.currentPosition()+100);
  executeStepFlag = false;
}


/**
 * Coil ISR for run/brake PWM using PWM pin
*/
void coilBrakeISR()
{
  if(coilSineDegrees == (SINETABLESIZE*2-1))
  {
    coilSineDegrees = 0;
    coilState = !coilState;
    digitalWriteFast(digitalPinToPinName(coilInA), coilState? HIGH:LOW);
    digitalWriteFast(digitalPinToPinName(coilInB), coilState? LOW:HIGH);
  }

  switch(sineMode)
  {
    case BRAKE:   // Produce sine function from lookup table on PWM pin
    {
      uint16_t compactSineIndex = coilSineDegrees<=(SINETABLESIZE)?coilSineDegrees : SINETABLESIZE-coilSineDegrees%SINETABLESIZE;  // Take only the wraparound value
      
      // uint16_t overflow = auxPWMTimer->getOverflow(TICK_FORMAT);
      // uint16_t sin = (overflow/255)*pgm_read_byte(&sinA[compactSineIndex]);

      uint16_t sin = (sineTableDutyCycleFactor)*pgm_read_byte(&sinA[compactSineIndex]);
      auxPWMTimer->setCaptureCompare(PWMchannel_coil, sin, TICK_COMPARE_FORMAT); // update sine duty cycle on PWM pin
    }
    break;
    case SQUARE:   // Just use the default constant PWM duty cycle
    break;
    default:
    break;
  }
  coilSineDegrees = (coilSineDegrees+1) % (SINETABLESIZE*2);    // update coil degrees, SINETABLE is only 90 degrees. 2 for a full 180
}

/**
 * Coil ISR for run/coast PWM using IN pins
*/
void coilCoastISR()
{
  if(coilSineDegrees == (SINETABLESIZE*2)-1)
  {
    coilSineDegrees = 0;    
    coilState = !coilState;

    // temporary: for hardware debug.
    digitalWrite(PB13, coilState?HIGH:LOW);
  }
  uint16_t compactSineIndex = coilSineDegrees<=(SINETABLESIZE)?coilSineDegrees : SINETABLESIZE-coilSineDegrees%SINETABLESIZE;  // Take only the wraparound value
  
  // This is slow. You can calculate constants at the first call 
  uint16_t overflow = coilPWMTimer->getOverflow(TICK_FORMAT);
  uint16_t sin = (overflow/255)*pgm_read_byte(&sinA[compactSineIndex]);

  if(coilState)
  {
    coilPWMTimer->setCaptureCompare(PWMchannel_coilA, sin, TICK_COMPARE_FORMAT); // update sine duty cycle on PWM pin
    coilPWMTimer->setCaptureCompare(PWMchannel_coilB, 0, PERCENT_COMPARE_FORMAT);
  }
  else
  {
    coilPWMTimer->setCaptureCompare(PWMchannel_coilB, sin, TICK_COMPARE_FORMAT); 
    coilPWMTimer->setCaptureCompare(PWMchannel_coilA, 0, PERCENT_COMPARE_FORMAT);
  }
  coilSineDegrees = (coilSineDegrees+1) % (SINETABLESIZE*2);    // update coil degrees, SINETABLE is only 90 degrees. 2 for a full 180
}

void stopCoil()
{   
  coilPWMTimer->detachInterrupt();
  switch(sineMode)
  {
    case COAST:
      coilPWMTimer->setCaptureCompare(PWMchannel_coilA, 0, PERCENT_COMPARE_FORMAT);
      coilPWMTimer->setCaptureCompare(PWMchannel_coilB, 0, PERCENT_COMPARE_FORMAT);
      coilPWMTimer->refresh();
      coilPWMTimer->pause();   
    break;
    case BRAKE:
    case SQUARE:                      // Square wave and brake mode share the same ISR and setup
      auxPWMTimer->detachInterrupt();
      auxPWMTimer->setCaptureCompare(PWMchannel_coil, 0, PERCENT_COMPARE_FORMAT);
      auxPWMTimer->refresh();
      auxPWMTimer->pause();
    break;
    default:
    break;
  }      

  /* Return coil driver inputs plain GPIO outputs */
  pinMode(coilInA,OUTPUT);
  pinMode(coilInB,OUTPUT);     
  digitalWrite(coilInA, LOW);
  digitalWrite(coilInB, LOW);                                    
}

void startCoil()
{
  switch(sineMode)
  {
    case COAST:
      coilPWMTimer->setPWM(PWMchannel_coilA, coilInA, coilPWMFrequency, 0);
      coilPWMTimer->setPWM(PWMchannel_coilB, coilInB, coilPWMFrequency, 0);
      coilState = false;
      coilSineDegrees = 0;
      coilPWMTimer->attachInterrupt(coilCoastISR);
      coilPWMTimer->refresh();
      coilPWMTimer->resume(); 
      break;
    case BRAKE:
    case SQUARE:                     // Square wave and brake mode share the same ISR and setup
      pinMode(coilInA,OUTPUT);
      pinMode(coilInB,OUTPUT);
      auxPWMTimer->setPWM(PWMchannel_coil, coilPWMPinAux, coilPWMFrequency, auxDutyCycle);
      coilState = false;
      coilSineDegrees = 0;
      auxPWMTimer->attachInterrupt(coilBrakeISR);
      auxPWMTimer->refresh();
      auxPWMTimer->resume(); 

      sineTableOverflow = auxPWMTimer->getOverflow(TICK_FORMAT);      // update constants for ISR calculations
      sineTableDutyCycleFactor = (uint16_t) (auxDutyCycle/100.0 * sineTableOverflow/255);
      break;
    default:
      break;
  }
}

void liftCoil()
{
  startCoil();
  stopCoil();
  digitalWrite(coilInA, HIGH);
  digitalWrite(coilInB, LOW);
  auxPWMTimer->setCaptureCompare(PWMchannel_coil, staticDutyCycle, PERCENT_COMPARE_FORMAT);
  auxPWMTimer->refresh();
  auxPWMTimer->resume();
}

void dropCoil()
{
  startCoil();
  stopCoil();
  digitalWrite(coilInA, LOW);
  digitalWrite(coilInB, HIGH);
  auxPWMTimer->setCaptureCompare(PWMchannel_coil, staticDutyCycle, PERCENT_COMPARE_FORMAT);
  auxPWMTimer->refresh();
  auxPWMTimer->resume();
}

/**
 * Reset ADC to default settings
*/
void resetADC()
{
  adc.sendDirectCommand(RESET);
  adc.setPGA(PGA_32);
  adc.setAutoCal(ACAL_DISABLED);
  adc.setDRATE(DRATE_60SPS);
  adc.setMUX(DIFF_2_3);
}

/**
 * Entry to the stepper step update ISRs
*/
void xStepISR()
{
  xStepper.stepTimerISR();    // use this ISR to enter the real ISR
}
void yStepISR()
{
  yStepper.stepTimerISR();
}

// ===================================SCPI COMMAND INTERMEDIATE FUNCTIONS - YOU SHOULD PUT BUSINESS LOGIC IN THEIR OWN FUNCTIONS=================================//

/* Execute local callback with specified number of parameters. signed int32 params only for now.*/
void scpi_nCallInt32(SCPI_P parameters, uint8_t size, void func_ptr(int32_t*), String message)
{
   if(parameters.Size()==size)    // TODO: check if numeric
  {
    int32_t target[size];
    for(int i = 0; i < size; i++)
    {
      target[i] = ((int32_t)String(parameters[i]).toInt());
    }
    (*func_ptr)(target);
    Serial.println(message);
  }
  else{
    Serial.println("Incorrect parameter length or format");
  }
}

/* Set UART transfer rate */
void scpi_uartDrate(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  uartFreq = String(parameters[0]).toInt();
  mainTimer->setOverflow(1,HERTZ_FORMAT);
  mainTimer->setCaptureCompare(1, uartFreq, HERTZ_COMPARE_FORMAT);
  uartTicks = mainTimer->getCaptureCompare(1);                      // Calculate number of ticks to increment and store in uartTicks 
}

/* Toggle UART logging */
void scpi_uartLog(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  logOn = (bool)String(parameters[0]).toInt();
}

void xySpeed(int32_t* speed)
{
  xStepper.setSpeed(speed[0]);
  yStepper.setSpeed(speed[1]);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "SPD X%-6d Y%-6d", speed[0], speed[1]);
  HD44780_PrintStr(lcdline3);
}
void scpi_xySpeed(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters,2,xySpeed,"speed xy");
}

/* internal function x absolute*/
void xAbs(int32_t* pos)
{
  if(pos[0]>=0)
  {
  xStepper.runSpeedToPosition(pos[0]);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "A X%-6d            ", pos[0]);
  HD44780_PrintStr(lcdline3);  
  }
}

/* Run X to absolute position */
void scpi_xAbs(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters,1,&xAbs,"move X absolute");
}

void xStep(int32_t *pos)
{
  int32_t stepX = pos[0];
  xStepper.runSpeedToPosition(xStepper.currentPosition()+stepX);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "R X%-6d            ", stepX);
  HD44780_PrintStr(lcdline3);
}

void scpi_xStep(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters,1,&xStep, "move X relative");
}

/* internal function y absolute*/
void yAbs(int32_t* pos)
{
  if(pos[0]>=0)
  {
    yStepper.runSpeedToPosition(pos[0]);
    HD44780_SetCursor(0,3);
    snprintf(lcdline3, LCD_LENGTH+1, "A Y%-6d            ", pos[0]);
    HD44780_PrintStr(lcdline3);
  }
}

void scpi_yAbs(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters,1,&yAbs,"move Y absolute");
}

void yStep(int32_t *pos)
{
  int32_t stepY = pos[0];
  yStepper.runSpeedToPosition(yStepper.currentPosition()+stepY);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "R Y%-6d            ", stepY);
  HD44780_PrintStr(lcdline3);
}

void scpi_yStep(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters,1,&yStep, "move Y relative");
}
void xyStep(int32_t *pos)
{
  int32_t stepX = pos[0];
  int32_t stepY = pos[1];
  xStepper.runSpeedToPosition(xStepper.currentPosition()+stepX);
  yStepper.runSpeedToPosition(yStepper.currentPosition()+stepY);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "R X%-6d Y%-6d", stepX, stepY);
  HD44780_PrintStr(lcdline3);
}



void scpi_xyStep(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters, 2, &xyStep, "move XY relative");
}

void xyAbs(int32_t *pos)
{
  int32_t posX = pos[0];
  int32_t posY = pos[1];
  xStepper.runSpeedToPosition(posX);
  yStepper.runSpeedToPosition(posY);
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "A X%-6d Y%-6d ", posX, posY);
  HD44780_PrintStr(lcdline3);
}
void scpi_xyAbs(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters, 2, &xyAbs, "move XY absolute");
}

void scpi_xZero(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  xStepper.setCurrentPosition(0);
}
void scpi_yZero(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  yStepper.setCurrentPosition(0);
}

void setCoilFreq(int32_t *freq)
{
  probeFrequency = freq[0];
  coilPWMFrequency = probeFrequency*2 * SINETABLESIZE*2;
  stopCoil();
  startCoil();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Coil %-6d            ", freq[0]);
  HD44780_PrintStr(lcdline3);
} 
void scpi_coilFreq(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  scpi_nCallInt32(parameters, 1, setCoilFreq, "Set coil freq");
}
void scpi_coilLift(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  liftCoil();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Coil lift            ");
  HD44780_PrintStr(lcdline3);
}
void scpi_coilDrop(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  dropCoil();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Coil drop            ");
  HD44780_PrintStr(lcdline3);
}
void scpi_coilOn(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  startCoil();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Coil on            ");
  HD44780_PrintStr(lcdline3);
}
void scpi_coilOff(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  stopCoil();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Coil off            ");
  HD44780_PrintStr(lcdline3);
}

void scpi_steppersOff(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  xStepper.disable();
  yStepper.disable();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Steppers disabled  ");
  HD44780_PrintStr(lcdline3);
}
void scpi_steppersOn(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  xStepper.enable();
  yStepper.enable();
  HD44780_SetCursor(0,3);
  snprintf(lcdline3, LCD_LENGTH+1, "Steppers enabled   ");
  HD44780_PrintStr(lcdline3);
}
void serial_ready()
{
  Serial.println(">");
}

void scpi_pause(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  uint32_t delayMs = String(parameters[0]).toInt();
  //Serial.println("Pause");
  delay(String(parameters[0]).toInt());
}
/* Possibly useful: block until last move complete*/
void scpi_join(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  //Serial.println("Join");
  //Serial.println(">");
  while(xStepper.direction() != AsyncStepper::DIRECTION_STOP || yStepper.direction() != AsyncStepper::DIRECTION_STOP)
  {
    adcUpdateAll();
    updateLCD(1,1,1,0);
  } // Optional: block (includes blocking the next multiline commands) until move is finished.
}

void scpi_adcDrate(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  uint16_t rate = abs(String(parameters[0]).toInt());
  uint8_t rateCode = 0;

  // Gross. round down to next available drate 
  if(rate < 10) rateCode = DRATE_5SPS;
  else if(rate < 15) rateCode = DRATE_10SPS;
  else if(rate < 30) rateCode = DRATE_15SPS;
  else if(rate < 60) rateCode = DRATE_30SPS;
  else if(rate < 100) rateCode = DRATE_60SPS;
  else if(rate < 500) rateCode = DRATE_100SPS;
  else if(rate < 1000) rateCode = DRATE_500SPS;
  else if(rate == 1000) rateCode = DRATE_1000SPS;
  else 
    //Serial.println(F("Target data rate out of range"));

  if(rateCode != 0) 
  {
    adc.setDRATE(rateCode);
    //Serial.print(F("Set ADC DRATE: ")); Serial.println(rateCode,BIN);
  }
}

void scpi_adcReset(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  resetADC();
}

void scpi_adcOffs(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  int32_t sum = 0;
  uint16_t times = 16;
  for(int i = 0; i<times; i++)
  {
    sum += adc.readSingle();
  }
  adcRawCounts = sum/times; 
  counts_offset = adcRawCounts;
}

/**
 * Computes counts to cN calibration constant
 * 
 * return: new value of counts to cN calibration constant
*/
float adcCal(float cN)
{
  int32_t sum = 0;
  uint16_t times = 16;
  for(int i = 0; i<times; i++)
  {
    sum += adc.readSingle();
  }
  adcRawCounts = sum/times; 
  counts_to_cN = cN/(adcRawCounts - counts_offset);
  return counts_to_cN;
}

void scpi_adcCal(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  Serial.println(adcCal(String(parameters[0]).toDouble()));
}
void scpi_debug(SCPI_C commands, SCPI_P parameters, Stream& interface)
{
  parser.PrintDebugInfo();
}
void scpi_meas(SCPI_C command, SCPI_P parameters, Stream& interface)
{
    uint32_t checksum = sprintf(uartBuffer, "__DATA__,%d,%d,%d,", adcRawCounts, xStepper.currentPosition(), yStepper.currentPosition());
    sprintf(checksumBuffer, "%d", checksum);
    Serial.print(uartBuffer);
    Serial.println(checksum);
}

void setup() {
}
void loop() {
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.setClock(400000);
  Wire.begin();
  
  delay(1000);
  HD44780_Init(4);
  HD44780_Clear();
  HD44780_NoDisplay();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("This machine");
  HD44780_SetCursor(0,1);
  HD44780_PrintStr("measures microns");
  HD44780_SetCursor(0,3);
  HD44780_PrintStr("UBC MECH 423");
  HD44780_Display();

  /* Register SCPI commands */
  parser.hash_magic_number = 113;        // You may need to adjust the hash magic number 
  parser.registerCommand("Join", &scpi_join);
  parser.registerCommand("pause", &scpi_pause);
  //parser.registerCommand("meas?", &scpi_meas);
  parser.SetCommandTreeBase("sys");
    parser.registerCommand(F("drate"),&scpi_uartDrate);
    parser.registerCommand(F("log"), &scpi_uartLog);  
    parser.registerCommand(F("debug"), &scpi_debug);
  parser.SetCommandTreeBase("xy");
    parser.registerCommand(F("abs"), &scpi_xyAbs);
    parser.registerCommand(F("ENable"),&scpi_steppersOn);
    parser.registerCommand(F("DISable"),&scpi_steppersOff);
    parser.registerCommand(F("step"),&scpi_xyStep);
    parser.registerCommand(F("SPeed"),&scpi_xySpeed);
  parser.SetCommandTreeBase(F("X"));                        // That's interesting. Not allowed to call this "x" lowercase
    parser.registerCommand(F("abs"), &scpi_xAbs);
    parser.registerCommand(F("step"), &scpi_xStep);
    parser.registerCommand(F("zero"), &scpi_xZero);
  parser.SetCommandTreeBase(F("Y"));
    parser.registerCommand(F("abs"), &scpi_yAbs);
    parser.registerCommand(F("step"), &scpi_yStep);
    parser.registerCommand(F("zero"), &scpi_yZero);
  parser.SetCommandTreeBase(F("ADC"));
    parser.registerCommand(F("drate"),&scpi_adcDrate);
    parser.registerCommand(F("RESet"),&scpi_adcReset);
    parser.registerCommand(F("offs"),&scpi_adcOffs);
    parser.registerCommand(F("cal"),&scpi_adcCal);
  parser.SetCommandTreeBase(F("Coil"));                   // Also interesting. If you put coil above adc it will erroneously do a coil drop even if you don't comand it
    parser.registerCommand(F("On"),&scpi_coilOn);
    parser.registerCommand(F("F"),&scpi_coilFreq);
    parser.registerCommand(F("Off"),&scpi_coilOff);
    parser.registerCommand(F("DROP"),&scpi_coilDrop);
    parser.registerCommand(F("LIFT"),&scpi_coilLift);
  Serial.begin(115200);
  Serial.flush();

  pinMode(PC13,OUTPUT);
  pinMode(PB13,OUTPUT);    // Temporary debug

  // Initial set up step generators in x and y
  xStepper.Timer->attachInterrupt(xStepper.PWMchannel, xStepISR);
  yStepper.Timer->attachInterrupt(yStepper.PWMchannel, yStepISR);
  xStepper.setSpeed(10000);
  yStepper.setSpeed(10000);

  mainTimer->setOverflow(1,HERTZ_FORMAT);
  mainTimer->setCaptureCompare(1, uartFreq, HERTZ_COMPARE_FORMAT);
  uartTicks = mainTimer->getCaptureCompare(1);                      // Calculate number of ticks to increment and store in uartTicks 
  mainTimer->setMode(1, TIMER_DISABLED);
  mainTimer->attachInterrupt(1, timer9_1ISR);
  mainTimer->setCaptureCompare(2, lcdFreq, HERTZ_COMPARE_FORMAT);
  lcdTicks = mainTimer->getCaptureCompare(2);
  mainTimer->setMode(2, TIMER_DISABLED);
  mainTimer->attachInterrupt(2, timer9_2ISR);
  mainTimer->refresh();
  mainTimer->resume();


  SPI.begin();                     
  adc.InitializeADC();
  resetADC();

  HD44780_NoDisplay();
  HD44780_Clear(); 
  HD44780_Display();

  HD44780_SetCursor(0,0);
  HD44780_PrintStr("Calibrating");
  // Initial offset calibration.
  int32_t sum = 0;
  uint16_t times = 128;
  for(int i = 0; i<times; i++)
  {
    sum += adc.readSingle();
  }
  adcRawCounts = sum/times; 
  counts_offset = adcRawCounts;

  HD44780_NoDisplay();
  HD44780_Clear(); 
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("SCPI init OK");
  HD44780_SetCursor(0,1);
  HD44780_PrintStr("Self-tests passed");
  HD44780_Display();  
  delay(2000);
  
  // Test: set up and run coil
  sineMode = BRAKE;
  // startCoil();

  xStepper.enable();
  yStepper.enable();

  serial_ready();

  // Test: run steppers
  // tim11->setOverflow(100, HERTZ_FORMAT);
  // tim11->attachInterrupt(timer11_0ISR);
  // tim11->refresh();
  // tim11->resume();

  while(true)
  {
    char* message = parser.GetMessage(Serial,"\n");
    if(message != NULL)
    {
      // HD44780_SetCursor(0,3);
      // snprintf(lcdline3, LCD_LENGTH+1, message);
      // HD44780_PrintStr(lcdline3);                // Prints exact message to screen.
      parser.Execute(message, Serial);
      serial_ready();                               // Signal that buffer's been read, host can send more commands 
    }

    adcUpdateAll();

    if(lcdFlag)
    {
      updateLCD(1,1,1,0);
      lcdFlag = false;
    }
  }
}

