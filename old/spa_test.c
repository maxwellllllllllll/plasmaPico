#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
 
#include "spa_toggle.pio.h"
 
#define CYCLE_TIME 500

uint setup_spa(int start_pin, PIO pio) {
    // output pins
    // printf("Setup pins\n");
    for (int i=0; i<4; i++) {
        pio_gpio_init(pio, start_pin+i);
    }
 
    // load the pio program into the pio memory
    // printf("Setup PIO\n");
    uint offset = pio_add_program(pio, &spa_toggle_program);
    // make a sm config
    pio_sm_config c = spa_toggle_program_get_default_config(offset);
    sm_config_set_clkdiv(&c, 5.f); // 25 MHz
    // set the 4 output pins
    uint sm = 0;
    //pio_sm_set_consecutive_pindirs(pio, sm, start_pin, 4, true);
    // set the 'set' pins
    //sm_config_set_set_pins(&c, start_pin, 4);
    sm_config_set_out_pins(&c, start_pin, 4);
    pio_sm_set_consecutive_pindirs(pio, sm, start_pin, 4, true);
    // set shift such that bits shifted by 'out' end up in the lower 16 bits
    sm_config_set_out_shift(&c, true, false, 0);
    // init the pio sm with the config
    pio_sm_init(pio, sm, offset, &c);
    // enable the sm
    pio_sm_set_enabled(pio, sm, true);
 
    //
    return sm;
}
 
uint32_t S1 = 0x00000008; // 1000
uint32_t S2 = 0x00000004; // 0100
uint32_t S3 = 0x00000002; // 0010
uint32_t S4 = 0x00000001; // 0001
 
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;
 
uint sm0;
 
void on_pwm_wrap() {
    pwm_clear_irq(0);
    // pio_sm_put(pio0, sm0, nextState);
    pio0->txf[sm0] = nextState; // Same as pio_sm_put without checking
 
    // Update nextState for next cycle
    uint32_t delay = 0;
    if (cycleCount < 100) { // Negative pulses
        delay = (100-cycleCount)*5; // Delay in PIO cycles @ 25 MHz
        nextState = negCycle;
    } else {                // Positive pulses
        delay = (cycleCount-100)*5; // Delay in PIO cycles @ 25 MHz
        nextState = possCycle;
    }
 
    if (delay < 25) {nextState = freeCycle;} // Lower bound on DCP (1 us + switching time)/20 us ~7.5%
    if (delay > 450) {delay = 450;}          // Upper bound on DCP (18 us + switching time)/20 us ~92.5%
    nextState = nextState | ( delay << 8);
 
    cycleCount++;
    if (cycleCount > 200) {cycleCount=0;} // Wrap cycle count for test
}
 
int main() {
 
    set_sys_clock_khz(125000, true); //125000
    // stdio_init_all();
    // printf("Starting\n");
    // for (int i=0; i<10; i++) {
    //  printf("Waiting... %d\n",10-i);
    //  sleep_ms(2000);
    // }
 
    //
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
 
    // Setup PIO unit
    sm0 = setup_spa(10,pio0);
 
    // Use PWM wrapping for timing
    pwm_clear_irq(0);
    pwm_set_irq_enabled(0, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
 
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.f); // Use system clock frequency (25 MHz)
    pwm_config_set_wrap(&config, 499);   // Wrap every 20 us (0-499)
    pwm_init(0, &config, false);
 
    // Turn on SPA in freewheeling state and activate PWM
    uint32_t delay = 10*25;
    nextState = (stop2free << 24) | (( delay << 8) | free2stop);
    pio_sm_put(pio0, sm0, nextState);
    busy_wait_ms(2000);
    pwm_set_enabled(0, true);
 
    busy_wait_ms(2000);
    // Ramp positive pulses
    for (int i=0; i<=17; i++) {
        sleep_ms(1000);
        delay = (i+1)*25; // 1-18 us (5% - 90% DCP)
        nextState = (poss2free << 24) | ( delay << 8) | free2poss;
    }
 
    // Ramp negative pulses
    for (int i=0; i<=17; i++) {
        sleep_ms(1000);
        delay = (i+1)*25; // 1-18 us (5% - 90% DCP)
        nextState = (neg2free << 24) | ( delay << 8) | free2neg;
    }
 
    // Turn off PWM
    pwm_set_enabled(0, false);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    sleep_ms(1);
 
    // Return to off state
    pio_sm_put(pio0, sm0, free2stop);
    sleep_ms(1);
 
    //Turn off pio
    pio_sm_set_enabled(pio0, sm0, false);
 
    return 0;
}