#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pinsToggle.pio.h"


 uint32_t S1 = 0x00000008; // 1000
 uint32_t nextState;


int main() {
    static const uint startPin = 10;
    static const float pio_freq = 64000;

    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio, &pinsToggle_program);

    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    pinsToggle_program_init(pio, sm, offset, startPin, div);

    
    pio_sm_put(pio0, sm, S1);

    busy_wait_ms(2000);

    // Start running our PIO program in the state machine
    pwm_set_enabled(0, true);

    // Do nothing
    while (true) {
        sleep_ms(1000);
    }
}