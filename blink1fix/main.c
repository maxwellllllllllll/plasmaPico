#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blink1.pio.h"
#include "blink2.pio.h"

int main() {

    static const uint led_pin = 13;
    static const float pio_freq = 64000;

    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset1 = pio_add_program(pio, &blink1_program);
    uint offset2 = pio_add_program(pio, &blink2_program);

    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    blink1_program_init(pio, sm, offset, led_pin, div);
    blink2_program_init(pio, sm, offset, led_pin, div);

    // Start running our PIO program in the state machine
    pio_sm_set_enabled(pio, sm, true);

    // Do nothing
    while (true) {
        sleep_ms(1000);
    }
}