#include <stdbool.h>
#include <led.h>

/*
 * This functions are meant to be used for debugging. `ledInit` should be
 * called for them to work.
 * Use `led_blocking_assert` to verify some condition and be visually notified,
 * by starting an infinite loop of blinking leds.
 */

void time_loop() {
  int i, j;
  j = 0;
  while (j < 100) {
    i = 0;
    while (i < 100000) {
      __asm__("");
      i++;
    }
    j++;
  }
}

void set_led_GR() {
  ledSet(LED_RED_L,   0);
  ledSet(LED_RED_R,   0);
  ledSet(LED_BLUE_L,  0);
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 1);
}

void set_led_GL() {
  ledSet(LED_RED_L,   0);
  ledSet(LED_RED_R,   0);
  ledSet(LED_BLUE_L,  0);
  ledSet(LED_GREEN_L, 1);
  ledSet(LED_GREEN_R, 0);
}


void set_led_RR() {
  ledSet(LED_RED_L,   0);
  ledSet(LED_RED_R,   1);
  ledSet(LED_BLUE_L,  0);
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
}

void set_led_RL() {
  ledSet(LED_RED_L,   1);
  ledSet(LED_RED_R,   0);
  ledSet(LED_BLUE_L,  0);
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
}

void led_blocking_assert(bool condition) {
  if (!condition) {
    do {
      set_led_RR();
      time_loop();
      set_led_RL();
      time_loop();
    } while (true);
  }
}

