/* ============================================================================
 * stepper_jog.c  —  bare-metal AVR stepper jogger (worksheet)
 * ============================================================================
 *
 *              ATmega328P pin reference
 *
 *              +--------\/--------+
 *  PC6 / RESET |1              28| PC5 / A5 / D19
 *  PD0 / D0    |2              27| PC4 / A4 / D18
 *  PD1 / D1    |3              26| PC3 / A3 / D17
 *  PD2 / D2    |4              25| PC2 / A2 / D16
 *  PD3 / D3    |5              24| PC1 / A1 / D15
 *  PD4 / D4    |6              23| PC0 / A0 / D14
 *  VCC         |7              22| GND
 *  GND         |8              21| AREF
 *  PB6 / XTAL1 |9              20| AVCC
 * PB7 / XTAL2  |10             19| PB5 / D13
 *  PD5 / D5    |11             18| PB4 / D12
 *  PD6 / D6    |12             17| PB3 / D11
 *  PD7 / D7    |13             16| PB2 / D10
 *  PB0 / D8    |14             15| PB1 / D9
 *              +------------------+
 *
 *            Bit operation reference:
 *
 *           // Set bit n
 *           reg |= (1 << n);
 *
 *           // Clear bit n
 *           reg &= ~(1 << n);
 *
 *           // Toggle bit n
 *           reg ^= (1 << n);
 *
 *           // Check if bit n is set
 *           if (reg & (1 << n)) { }
 *
 *           // Check if bit n is clear
 *           if (!(reg & (1 << n))) { }
 *
 * Target:   ATmega328P @ 16 MHz (Arduino Uno / Nano / bare chip w/ external
 * xtal) Driver:   A4988 / DRV8825 / TMC2208 (any STEP+DIR-style stepper driver)
 * Purpose:  Learn the same skeleton grbl uses to drive steppers. Nothing more.
 *
 * Wiring (suggested — change to match your board):
 *   MCU pin              Driver pin      Arduino label
 *   --------------------------------------------------
 *   PD2  (STEP)     ->   STEP            D2
 *   PD3  (DIR)      ->   DIR             D3
 *   PD4  (ENABLE)   ->   ~ENABLE         D4     (A4988/DRV8825 enable is active
 * LOW) GND             ->   GND             GND (stepper driver also needs its
 * own motor supply + motor coils wired)
 *
 * How stepping works at the electrical level:
 *   - DIR pin is sampled by the driver just before each STEP rising edge.
 *     Set DIR, then wait a hair (>=200ns is plenty), then pulse STEP.
 *   - STEP: every LOW->HIGH edge advances the motor one (micro)step.
 *   - Pulse width needs to be >= 1us (A4988) or >= 1.9us (DRV8825). 2us is
 * safe.
 *   - Delay between pulses controls speed. Too fast and the motor stalls.
 *
 * ============================================================================
 */

/* --- 1. F_CPU ---------------------------------------------------------------
 * util/delay.h uses F_CPU to compute loop counts for _delay_us/_delay_ms.
 * Define it BEFORE including delay.h. You can also pass -DF_CPU=16000000UL
 * on the avr-gcc command line and skip defining it here — pick one.
 *
 * Arduino Uno/Nano: 16 MHz crystal -> 16000000UL
 */
#define F_CPU 16000000UL

/* --- 2. Includes ------------------------------------------------------------
 * You need:
 *   - the I/O register map for your chip      (hint: <avr/???.h>)
 *   - busy-wait delay macros                  (hint: <util/?????.h>)
 *   - fixed-width integer types               (standard C header)
 */
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

/* --- 3. Pin assignments -----------------------------------------------------
 * Pick your pins. Any digital pin works — PORTB, PORTC, or PORTD bit numbers.
 * Below are the suggested pins from the wiring table. Fill in the bit numbers.
 *
 * Naming convention: for port D bit 2, avr/io.h gives you the macro PD2
 * which is literally just the number 2. So `(1 << PD2)` is `0b00000100`.
 */
#define STEP_BIT PD2
#define DIR_BIT PD3
#define ENABLE_BIT PD4

/* And the three registers for port D. These are just aliases for clarity —
 * you could use DDRD/PORTD/PIND directly. But naming them by role makes the
 * code easier to retarget to a different port later.
 */
#define STEPPER_DDR DDRD
#define STEPPER_PORT PORTD

/* --- 4. Tuning knobs --------------------------------------------------------
 * STEP_PULSE_US:   how long STEP stays HIGH. >= driver minimum.
 * STEP_PERIOD_US:  total time from one rising edge to the next.
 *                  Smaller = faster. Start slow (a few ms) so you can see it.
 *
 * Example starting values: STEP_PULSE_US=2, STEP_PERIOD_US=2000
 *   -> 500 pulses/sec -> 500 steps/sec
 *   -> at 200 steps/rev full-step, that's 2.5 rev/sec. At 1/16 microstepping,
 *      that's 0.156 rev/sec — nice and visible.
 */
#define STEP_PULSE_US 2
#define STEP_PERIOD_US 6000

/* --- 5. The "args" (compile-time for now) -----------------------------------
 * Change these, recompile, reflash. Later, replace with UART parsing.
 *   JOG_DIR:   0 or 1  (which way the motor spins depends on your wiring)
 *   JOG_STEPS: how many STEP pulses to emit
 */
#define JOG_DIR 1
#define JOG_STEPS 50

/* --- 6. Bit-twiddling helpers -----------------------------------------------
 * The three AVR idioms you'll use constantly. Commit these to muscle memory:
 *
 *   set bit N of R:     R |=  (1 << N);
 *   clear bit N of R:   R &= ~(1 << N);
 *   toggle bit N of R:  R ^=  (1 << N);
 *   read bit N of R:    (R >> N) & 1     // or: !!(R & (1 << N))
 *
 * `volatile` tells the compiler "this memory can change outside normal program
 * flow, don't cache it in a register." Every I/O register must be accessed
 * through a volatile pointer. avr/io.h already does this — but when we take a
 * pointer to one ourselves, we have to preserve that volatile.
 */
static inline void pin_high(volatile uint8_t *reg, uint8_t bit) {
  /* TODO: set `bit` in *reg */
  *reg |= (1 << bit);
}

static inline void pin_low(volatile uint8_t *reg, uint8_t bit) {
  /* TODO: clear `bit` in *reg */
  *reg &= ~(1 << bit);
}

/* --- 7. Configure pins as outputs -------------------------------------------
 * Set the DDR bits for STEP, DIR, (ENABLE) to 1 so the MCU drives them.
 * Then:
 *   - drive ENABLE low    (A4988/DRV8825 enable is active-low => low = enabled)
 *   - drive DIR per JOG_DIR
 *   - leave STEP low, ready for the first rising edge
 */
static void pins_init(void) {
  /* TODO: make STEP, DIR, ENABLE outputs                                    */
  /* TODO: drive ENABLE low to enable the driver                             */
  /* TODO: set DIR according to JOG_DIR                                      */
  /* TODO: ensure STEP starts low                                            */
}

/* --- 8. One step ------------------------------------------------------------
 * Emit one step pulse. Sequence:
 *     STEP high
 *     _delay_us(STEP_PULSE_US)
 *     STEP low
 *     _delay_us(STEP_PERIOD_US - STEP_PULSE_US)
 *
 * Gotcha: _delay_us() wants a *compile-time constant*. That's fine here since
 * our constants are #defines. If you ever want a runtime-variable delay,
 * either call _delay_us(1) in a loop or (better) use a hardware timer.
 */
static void step_once(void) { /* TODO */ }

/* --- 9. Jog -----------------------------------------------------------------
 * Set direction, give the driver a microsecond to latch it, then emit
 * `steps` pulses.
 */
static void jog(uint8_t dir, uint16_t steps) {
  /* TODO: set DIR pin from `dir`                                            */
  /* TODO: _delay_us(1) or so to let DIR settle before first STEP edge       */
  /* TODO: for (uint16_t i = 0; i < steps; i++) step_once();                 */
}

/* --- 10. main ---------------------------------------------------------------
 * On a microcontroller, main() must NEVER return. There's no OS to return to.
 * If it did, the compiler's startup code puts you in an infinite loop anyway,
 * but we do it explicitly for clarity.
 */
int main(void) {
  pins_init();
  /* TODO: call jog(JOG_DIR, JOG_STEPS); */

  for (;;) {
    /* idle forever. grbl would sleep the CPU here and wake on interrupt. */
  }

  return 0; /* never reached */
}

/* ============================================================================
 * STRETCH GOALS (in rough order of difficulty):
 *
 *  1. Replace the compile-time JOG_DIR / JOG_STEPS with UART input.
 *     Init USART at 115200 baud, read chars in a loop, parse "f20\n" etc.
 *     This is how grbl talks to you.
 *
 *  2. Replace _delay_us() with Timer1 CTC mode + OCR1A compare-match ISR.
 *     Set OCR1A to your desired step period, toggle STEP in the ISR.
 *     This is grbl's stepper engine (see stepper.c in the grbl source).
 *
 *  3. Add acceleration: start slow, ramp step rate up, ramp back down.
 *     Grbl uses a planner + Bresenham — but a trapezoidal ramp on a single
 *     axis is a great warm-up.
 *
 *  4. Add a second motor. Now you need Bresenham to coordinate them so they
 *     finish their step counts together. This is where grbl gets interesting.
 * ============================================================================
 */
