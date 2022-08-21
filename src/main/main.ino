#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define MOTOR_PIN PA7
#define ENCODER_PIN PA3

const uint8_t SECS_PER_TICK = 5;//3600 / ENCODER_TICKS_PER_REV;
volatile uint8_t sec_counter = 0; // Countdown until the next tick
volatile bool setState = false;
bool state = false;

//for setting register bits with AVR code
//cbi and sbi are standard (AVR) methods for setting, or clearing, bits in PORT (and other) variables. 
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup() {
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  sbi(GIMSK, PCIE0); // enable external pin change interrupts 0 - 7
  PCMSK0 = B00001000; // enable PCINT3 / PA3

  //   Setting up clock 1 for main timing
  TCCR0A = B00000010;   // Toggle pin OC0A (TICK_PIN) on timer compare interrupt A
  TCCR0B = B00000101;   // Setting clear timer on compare with OCR1A, using time scaled by 1024
  TIMSK0 = B00000010;   // Setting timer interrupt on compare A
  OCR0A = 31;          // Output compare register, clock counts to this number
  
  sei(); //enable global interrupts

  pinMode(MOTOR_PIN, OUTPUT); // This is here because otherwise clock 1 doesn't work
  digitalWrite(MOTOR_PIN, LOW);
}

void loop() {
  if (setState == !state) {
    state = setState;
    if (state)
      digitalWrite(MOTOR_PIN, HIGH);
    else
      digitalWrite(MOTOR_PIN, LOW);
  }
}

ISR(PCINT0_vect) {  // Triggers on the encoder pin changing state
  setState = !setState;
}

ISR(TIM0_COMPA_vect) {  // Triggers when 32 Hz timer reaches count of 32 (1Hz)
  ++sec_counter;
  if(sec_counter >= SECS_PER_TICK) {
    setState = !setState;
    sec_counter = 0;
  }
}
