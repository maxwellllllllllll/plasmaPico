#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pinsToggle.pio.h"


// pin "addresses"
uint32_t S4 = 0x00000008; // 1000
uint32_t S3 = 0x00000004; // 0100
uint32_t S2 = 0x00000002; // 0010
uint32_t S1 = 0x00000001; // 0001

// States
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;

uint16_t delay;

uint sm;


const uint LED_PIN = 25;


void delay_test(){
    nextState = freeCycle;
    pio_sm_put(pio0, sm, nextState);

    nextState = (possCycle) | ( delay << 8 );
    pio_sm_put(pio0, sm, nextState);

}


void on_pwm_wrap() {
    pwm_clear_irq(0);
    pio_sm_put(pio0, sm, nextState);
    // // pio0->txf[sm] = nextState; // Same as pio_sm_put without checking
 
//     // Update nextState for next cycle
//     uint32_t delay = 0;
//     if (cycleCount < 100) { // Negative pulses
//         delay = (100-cycleCount)*5; // Delay in PIO cycles @ 25 MHz
//         nextState = negCycle;
//     } else { // Positive pulses
//         delay = (cycleCount-100)*5; // Delay in PIO cycles @ 25 MHz
//         nextState = possCycle;
//     }
 
//     if (delay < 25) {nextState = freeCycle;} // Lower bound on DCP (1 us + switching time)/20 us ~7.5%
//     if (delay > 450) {delay = 450;}          // Upper bound on DCP (18 us + switching time)/20 us ~92.5%
//     nextState = nextState | ( delay << 8);
 
//     cycleCount++;
//     if (cycleCount > 200) {cycleCount=0;} // Wrap cycle count for test
}


int main() {
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


    // Turn on SPA in freewheeling state and activate PWM
    delay = 250; // largest: 65536
    nextState = (stop2free << 24) | (( delay << 8) | free2stop);
    pio_sm_put(pio0, sm, nextState);
    
    busy_wait_ms(1);

    delay_test();

    busy_wait_ms(1000);


    // pwm_set_enabled(0, true);

    // // Ramp positive pulses
    // for (int i=0; i<=17; i++) {
    //     sleep_ms(1000);
    //     delay = (i+1)*25; // 1-18 us (5% - 90% DCP)
    //     nextState = (poss2free << 24) | ( delay << 8) | free2poss;
    // }
    
    // // Ramp negative pulses
    // for (int i=0; i<=17; i++) {
    //     sleep_ms(1000);
    //     delay = (i+1)*25; // 1-18 us (5% - 90% DCP)
    //     nextState = (neg2free << 24) | ( delay << 8) | free2neg;
    // }

    // Turn off PWM
    pwm_set_enabled(0, false);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    sleep_ms(1);
 
    // Return to off state
    pio_sm_put(pio0, sm, stop2free);
    sleep_ms(1000);
 
    //Turn off pio
    pio_sm_set_enabled(pio0, sm, false);


    //Use board LED as status indicator
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);

    // Do nothing
    while (true) {
        sleep_ms(1000);
    }
}