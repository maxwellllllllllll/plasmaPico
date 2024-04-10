#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pinsToggle.pio.h"


// pin "addresses"
uint32_t S1 = 0x00000008; // 1000
uint32_t S2 = 0x00000004; // 0100
uint32_t S3 = 0x00000002; // 0010
uint32_t S4 = 0x00000001; // 0001

// States
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;


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

    
    // Writes a state (word of data) to a state machine's TX FIFO
    pio_sm_put(pio0, sm, S1);

    busy_wait_ms(2000);

    // Start running our PIO program in the state machine
    pwm_set_enabled(0, true);

    // Writes annother state (word of data) to state machine's TX FIFO
    pio_sm_put(pio0, sm, S2);

    // Do nothing
    while (true) {
        sleep_ms(1000);
    }
}