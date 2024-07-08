#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

#include "pinsToggle.pio.h"

#include "pico/malloc.h"


// pin "addresses"
uint32_t S4 = 0x00000008; // 1000
uint32_t S3 = 0x00000004; // 0100
uint32_t S2 = 0x00000002; // 0010
uint32_t S1 = 0x00000001; // 0001

// Pulse States
uint32_t __not_in_flash("pwm") stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t __not_in_flash("pwm") freeCycle, possCycle, negCycle;
uint32_t __not_in_flash("pwm") nextState, cycleCount;

// Pulse Charicteristics
uint32_t __not_in_flash("pwm") delay;
uint32_t __not_in_flash("pwm") target;

// pio state machine
uint __not_in_flash("pwm") sm;

// Data block
uint16_t __not_in_flash("pwm") block_length_uint;

// Facilitates cpu-pwm communication
uint8_t __not_in_flash("pwm") pwm_flag;


// Device States
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define BLOCK_FOUND 3
#define GETTING_BLOCK 4


void __time_critical_func(on_pwm_wrap)() {
    pwm_clear_irq(0);

    //pio_sm_put(pio0, sm, nextState);
    pio0->txf[sm] = nextState; // Same as pio_sm_put without checking

    // // Calculates delay from data block
    //target = 52; //block[cycleCount];
    
    // Update nextState for next cycle
    if (target < 100) { // Negative pulses
        nextState = negCycle;
        delay = target * 5; // Delay in PIO cycles @ 25 MHz
    } else { // Positive pulses
        nextState = possCycle;
        delay = (target - 100) * 5; // Delay in PIO cycles @ 25 MHz
    }
 
    if (delay < 25) {nextState = freeCycle;} // Lower bound on DCP (1 us + switching time)/20 us ~7.5%
    if (delay > 450) {delay = 450;}          // Upper bound on DCP (18 us + switching time)/20 us ~92.5%
    nextState = nextState | ( delay << 8);

    // for debugging
    //next_state_block[cycleCount] = nextState;

    pwm_flag = 1;
 
    cycleCount++;
}


void init_pulse() {
    static const uint startPin = 10;

    set_sys_clock_khz(125000, true); //125000

    // state definitions
    stop2free = (S2 | S4) << 4;  // Turn on S2 and S4
    free2stop = 0;  // Turn all off
    freeCycle = ((S2 | S4) << 28) | ((S2 | S4) << 24) | ((S2 | S4) << 4) | (S2 | S4);
 
    free2poss = ((S2 | S3) << 4) | S2;  // S2 only then S2 and S3
    poss2free = ((S2 | S4) << 4) | S2;  // S2 only then S2 and S4
    possCycle = (poss2free << 24) | free2poss;
 
    free2neg = ((S1 | S4) << 4) | S4;  // S4 only then S1 and S4
    neg2free = ((S2 | S4) << 4) | S4;  // S4 only then S2 and S4
    negCycle = (neg2free << 24) | free2neg;
 
    cycleCount = 0;


    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio, &pinsToggle_program);

    // PIO clock divider
    float div = 5.f; //(float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    pinsToggle_program_init(pio, sm, offset, startPin, div);


    // PWM wrapping setup
    pwm_clear_irq(0);
    pwm_set_irq_enabled(0, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, div); // Use system clock frequency (25 MHz)
    pwm_config_set_wrap(&config, 499);   // Wrap every 20 us (0-499)
    pwm_init(0, &config, false);


    // // Setting up ADC
    // adc_init();

    // // Make sure GPIO is high-impedance, no pullups etc
    // adc_gpio_init(26);
    // // Select ADC input 0 (GPIO26)
    // adc_select_input(0);

    return;
}


void shutdown_pulse() {

    // Turn off PWM
    pwm_set_enabled(0, false);
    irq_set_enabled(PWM_IRQ_WRAP, false);
 
    // Return to off state
    pio_sm_put(pio0, sm, stop2free);
 
    //Turn off pio
    pio_sm_set_enabled(pio0, sm, false);
    
    return;
}


// Split into seperate functions
void run_pulse() {
    uint32_t setpoint;

    block_length_uint = 200;

    // Loads freewheeling state as first PWM pulse
    delay = 250;
    nextState = (stop2free << 24) | (( delay << 8) | free2stop);

    // Start PWM
    pwm_set_enabled(0, true);

    // Pulse Loop
    for (uint16_t cycle = 0; cycle <= block_length_uint; cycle++) {
        setpoint = 52;

        while (true) {
            if (pwm_flag == 1) {target = setpoint; pwm_flag = 0; break;}
        }
    }

    pwm_set_enabled(0, false);

    return;
}


/* Continually scans for input. Does not terminate until 
an input block is received*/
void scan_for_input() {
    char in;
    state = AWAIT_HEADER;

    while ( true ) {
        scanf("%c", &in);
        
        //printf("%d\n", in);

        switch (state) {
            case AWAIT_HEADER:
                if (in == 0x55) {
                    state = AWAIT_SYNC;
                    printf("Await header");
                }
                break;
            
            case AWAIT_SYNC:
                if (in == 0x3C) {
                    printf("await sync");
                    state = BLOCK_FOUND;
                    return;
                }
            default:
                break;
        }
    }
}


int main() {

    stdio_init_all();

    scan_for_input();

    sleep_ms(5000); // Needed to allow stdio init to complete without interfearing with FIFO buffers

    init_pulse();

    run_pulse();

    shutdown_pulse();


    return 0;
}