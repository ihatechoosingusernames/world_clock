#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define MOTOR_PIN PA7
#define ENCODER_PIN PA3
#define TICK_PIN PA6

#define ENCODER_TICKS_PER_REV 20  // set this to the number of 'ticks' the encoder makes per revolution
#define DUTY_CYCLE 0.5  // A decimal for the duty cycle at the motor pin

const int SECS_PER_TICK = 2;//3600 / (ENCODER_TICKS_PER_REV * 2);
const unsigned int TIMER_COMPARE = (DUTY_CYCLE * 255);

int sec_counter = SECS_PER_TICK; // Countdown until the next tick
volatile bool tick = false;  // To notify of 1 second passing

//for setting register bits with AVR code
//cbi and sbi are standard (AVR) methods for setting, or clearing, bits in PORT (and other) variables. 
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


void setup() {
  cli(); //disable global interrupts

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(TICK_PIN, OUTPUT);

  sbi(GIMSK, PCIE0); // enable external pin change interrupts 0 - 7
  PCMSK0 |= B00000100;  // enable PCINT3

  // Setting up clock 0 for PWM timing
  TCCR0A = B00100011;   // Connect pin 0C0B (PA7) in fast PWM mode
  TCCR0B = B00000001;   // No pre-scaling
  OCR0B = TIMER_COMPARE;  // Count to TIMER_COMPARE before resetting

  // Setting up clock 1 for main timing

  TCCR1A = B01000000;   // Toggle pin OC1A (TICK_PIN) on timer compare interrupt A
  TCCR1B = B00001101;   // Setting clear timer on compare with OCR1A, also scaling by 1024
  OCR1A = 31;           // Output compare register, clock counts to this number
  TIMSK1 = B00000010;   // Setting timer interrupt on compare A

  sei(); //enable global interrupts
}

void loop() {
//  if (digitalRead(ENCODER_PIN) == tick) {
////    pinMode(MOTOR_PIN, OUTPUT);
//    digitalWrite(MOTOR_PIN, HIGH);
//  } else {
////    pinMode(MOTOR_PIN, INPUT);
//    digitalWrite(MOTOR_PIN, LOW);
//  } 
}

EMPTY_INTERRUPT(PCINT0_vect);  // Triggers on the encoder pin changing state

ISR(TIM1_COMPA_vect) {  // Triggers when 32 Hz timer reaches count of 32 (1Hz)
  --sec_counter;
  if(sec_counter == 0) {
    sec_counter = SECS_PER_TICK;
    tick = !tick;
  }
}
