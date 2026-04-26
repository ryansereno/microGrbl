/* ============================================================================
 * stepper_jog.c  —  bare-metal AVR stepper jogger (worksheet)
 * ============================================================================
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
 * ----------------------------------------------------------------------------
 *               * AVR GPIO Registers (PORTx, DDRx, PINx)
 *
 * Each port (B, C, D on the 328P) has THREE one-byte registers.
 * Each bit in a register corresponds to one pin on that port.
 *
 *   DDRx  — Data Direction Register = who drives this pin? Me (1) or the
 * outside world (0) Bit = 1  -> pin is an OUTPUT Bit = 0  -> pin is an INPUT
 * (default on reset) Set once during init. Don't touch it again.
 *
 *   PORTx — Port Output Register  (dual role!) -  if I'm driving: what value?
 * (0 or 1)? if outside is driving: pull-up on(1) or off (0)? If pin is OUTPUT:
 * bit = the value driven on the pin (1=high, 0=low) If pin is INPUT:  bit = 1
 * enables internal pull-up, 0 = floating This is the register you toggle during
 * normal operation.
 *
 *   PINx  — Port Input Register -  sensor, tells me the actual voltage on the
 * wire Reads the current electrical state of the pin (regardless of dir).
 *           Quirk: writing 1 to a PINx bit TOGGLES the matching PORTx bit
 *                  (one-cycle toggle — used in grbl's step ISR).
 *
 * ----------------------------------------------------------------------------
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
 * ----------------------------------------------------------------------------
 * Target:   ATmega328P @ 16 MHz (Arduino Uno / Nano / bare chip w/ external
 * xtal) Driver:   A4988 / DRV8825 / TMC2208 (any STEP+DIR-style stepper driver)
 * Purpose:  Learn the same skeleton grbl uses to drive steppers. Nothing more.
 *
 * Wiring :
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
#define STEPPER_PIN PIND

#define BUTTON_DDR DDRD
#define BUTTON_PORT PORTD
#define BUTTON_PIN PIND

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

/* --- 6. Button input --------------------------------------------------------
 * Goal: trigger one jog every time a momentary pushbutton is pressed.
 *
 * --- Wiring -----------------------------------------------------------------
 * The lazy/standard way: one leg of the button to your chosen MCU pin, the
 * other leg to GND. No external resistor needed — we'll use the AVR's
 * built-in pull-up.
 *
 *     MCU pin  ----+
 *                  |
 *                  /  pushbutton (NO, momentary)
 *                  |
 *     GND      ----+
 *
 * Idle:    pull-up holds the pin HIGH (reads 1).
 * Pressed: button shorts pin to GND, pin goes LOW (reads 0).
 * So "pressed" is an *active-low* signal. Don't get tripped up by the
 * inversion — every hobbyist does at least once.
 *
 * --- Pin choice -------------------------------------------------------------
 * PORTD bits 2,3,4 are taken by the stepper. PD5, PD6, PD7 are all free,
 * or you can hop ports entirely. PD2 / PD3 are special (they're INT0 / INT1,
 * the external interrupt pins) — nice if you ever want hardware interrupts,
 * but here we'll just poll in main(). Any free pin is fine.
 *
 * Suggestion: PD5 (Arduino D5). Define BUTTON_BIT, BUTTON_DDR, BUTTON_PORT,
 * BUTTON_PIN below — same pattern as the stepper.
 *
 * --- The three registers, one more time ------------------------------------
 *   DDRx  bit = 0   -> pin is INPUT (this is the reset default, but be
 *                      explicit; future-you will thank you).
 *   PORTx bit = 1   -> enable internal pull-up (~20-50k to VCC).
 *   PINx  bit       -> read the live electrical state. 0 = pressed,
 *                      1 = released (because of the active-low wiring).
 *
 * --- Debouncing -------------------------------------------------------------
 * Mechanical contacts bounce. For ~1-20ms after the button moves, the signal
 * rapidly oscillates between HIGH and LOW as the metal contacts chatter. If
 * you naively poll PINx and fire on every LOW, one physical press will
 * trigger your jog 3-15 times. You will absolutely see this on real hardware.
 *
 * Two common fixes — both work, pick one:
 *
 *   (a) Delay-after-detect (simple, blocking):
 *         if (button reads pressed) {
 *             _delay_ms(20);                    // let the bounce settle
 *             if (button still reads pressed) { // confirm it's real
 *                 do_the_thing();
 *                 while (button still pressed) {} // wait for release
 *                 _delay_ms(20);                  // debounce the release too
 *             }
 *         }
 *
 *   (b) State-change detection (cleaner, still blocking _delay_ms):
 *         track previous stable state; only act on HIGH->LOW transitions;
 *         require the new state to hold for N ms before accepting it.
 *
 * Either way: the *edge* (press event) is what fires the jog, not the level
 * (held down). Otherwise holding the button = continuous jogging, which may
 * or may not be what you want — for "90 deg per click", it's not.
 *
 * --- TODOs for you ----------------------------------------------------------
 *   [ ] #define BUTTON_BIT / BUTTON_DDR / BUTTON_PORT / BUTTON_PIN
 *   [ ] In pins_init() (or a new button_init()):
 *         - clear DDR bit             (input)
 *         - set PORT bit              (pull-up on)
 *   [ ] Write a `button_pressed()` helper that returns 1 only on a fresh
 *       press edge (not while held). Hint: keep a `static uint8_t prev;`
 *       inside the function.
 *   [ ] In main()'s for(;;) loop: if button_pressed(), call jog(...).
 *   [ ] Test it. Spam the button. If you ever get a double-fire, your
 *       debounce window is too short — bump it from 20ms to 50ms.
 *
 * --- Stretch (don't do this yet) -------------------------------------------
 * The "right" answer is a pin-change interrupt (PCINT) or INT0/INT1, with a
 * timer-based debounce flag. Polling is fine for a worksheet — interrupts are
 * for when main() is busy doing something else and can't be trusted to poll
 * fast enough.
 * ============================================================================
 */
#define BUTTON_BIT PD5

static uint8_t button_pressed(void) {
  static uint8_t previous = 0;
  uint8_t now = 0;

  if (!(BUTTON_PIN & (1 << BUTTON_BIT))) {
    _delay_ms(50);
    if (!(BUTTON_PIN & (1 << BUTTON_BIT))) {
      now = 1;
    }
  }

  uint8_t edge = (now && !previous);
  previous = now;
  return edge;
}

/* --- 7. Configure pins as outputs -------------------------------------------
 * Set the DDR bits for STEP, DIR, (ENABLE) to 1 so the MCU drives them.
 * Then:
 *   - drive ENABLE low    (A4988/DRV8825 enable is active-low => low = enabled)
 *   - drive DIR per JOG_DIR
 *   - leave STEP low, ready for the first rising edge
 */
static void pins_init(void) {
  /* make STEP, DIR, ENABLE outputs*/
  STEPPER_DDR |= (1 << STEP_BIT) | (1 << DIR_BIT) | (1 << ENABLE_BIT);

  /* make BUTTON input*/
  BUTTON_DDR &= ~(1 << BUTTON_BIT);

  /* drive ENABLE low to enable the driver*/
  STEPPER_PORT |= (1 << ENABLE_BIT);

  /* set BUTTON pin to use  pull-up resistor*/
  BUTTON_PORT |= (1 << BUTTON_BIT);

  /* set DIR according to JOG_DIR*/
  if (JOG_DIR) {
    STEPPER_PORT |= (1 << DIR_BIT);
  } else {
    STEPPER_PORT &= ~(1 << DIR_BIT);
  }

  /* ensure STEP starts low*/
  STEPPER_PORT &= ~(1 << STEP_BIT);
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
static void step_once(void) {
  STEPPER_PORT |= (1 << STEP_BIT);
  _delay_us(STEP_PULSE_US);
  STEPPER_PORT &= ~(1 << STEP_BIT);
  _delay_us(STEP_PERIOD_US - STEP_PULSE_US);
}

/* --- 9. Jog -----------------------------------------------------------------
 * Set direction, give the driver a microsecond to latch it, then emit
 * `steps` pulses.
 */
typedef enum { DIR_REV = 0, DIR_FWD = 1 } dir_t;
static void jog(dir_t dir, uint16_t steps) {
  /* set DIR pin from `dir`                                            */
  if (dir) {
    STEPPER_PORT |= (1 << DIR_BIT);
  } else {
    STEPPER_PORT &= ~(1 << DIR_BIT);
  }
  /*  _delay_us(1) or so to let DIR settle before first STEP edge       */
  _delay_us(1);
  /* for (uint16_t i = 0; i < steps; i++) step_once();                 */
  for (uint16_t i = 0; i < steps; i++) {
    step_once();
  }
}

/* --- 10. main ---------------------------------------------------------------
 * On a microcontroller, main() must NEVER return. There's no OS to return to.
 * If it did, the compiler's startup code puts you in an infinite loop anyway,
 * but we do it explicitly for clarity.
 */
int main(void) {
  pins_init();
  /* call jog(JOG_DIR, JOG_STEPS); */
  for (;;) {
    /* idle forever. grbl would sleep the CPU here and wake on interrupt. */
    if (button_pressed()) {
      // 90 degrees at 1/16 microstepping = 800 steps
      jog(DIR_FWD, 800);
    }
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
