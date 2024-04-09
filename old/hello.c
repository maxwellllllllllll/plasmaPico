#include <stdio.h>
#include <stdbool.h>
#include <pico/stdlib.h>

#include <hardware/pio.h>

#include "hello_t.pio.h"

#define LED_BUILTIN 25;

int main() {
  stdio_init_all();
  PIO pio = pio0;
  uint state_machine_id = 0;
  uint offset = pio_add_program(pio, &hello_t_program);

  hello_t_program_init(pio, state_machine_id, offset, 25, 1);  
  
  while(1) {
    //do nothing
  }
}