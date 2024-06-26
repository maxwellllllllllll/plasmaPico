#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pinsToggle.pio.h"

#include "pico/malloc.h"


// pin "addresses"
uint32_t S4 = 0x00000008; // 1000
uint32_t S3 = 0x00000004; // 0100
uint32_t S2 = 0x00000002; // 0010
uint32_t S1 = 0x00000001; // 0001

// Pulse States
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;

uint16_t delay;

uint sm;

// Device States
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define BLOCK_FOUND 3
#define GETTING_BLOCK 4

// Data Block
unsigned char* block;


const uint LED_PIN = 25;


/*Once a block is detected by scan_for_input(), classifies and
collects the rest of the block*/
uint16_t get_block(){
    char c;
    
    char block_length1;
    char block_length2;
    uint16_t block_length_uint;
    char cs_received;
    char trailer_received;
    uint8_t cs = 0;

    char block_type;
    #define DATA 0x01
    #define TRAILER 0x55
    #define DATA_BLOCK_LENGTH 256

    state = GETTING_BLOCK;

    scanf("%c", &block_type);
        
    switch (block_type) {
        case DATA:
            //printf("\ngetting data block");
            scanf("%c", &block_length1);
            scanf("%c", &block_length2);

            int block_length = (block_length2 << 8) | block_length1;
            printf("\nbl: %d\n", block_length);
            block_length_uint = (uint16_t)block_length;
            //printf("\n %u bl:", block_length_uint);

            block = (unsigned char*)malloc(block_length_uint * sizeof(unsigned char)); // replace with specified length
            //printf(sizeof(block) / sizeof(block[0]));
            // Note: length of data for 200ms is 10,000 delays (bytes)

            // // Builds start of block (already read)
            // // not needed
            // block[0] = 0x55;                             // Header
            // block[1] = 0x3C;                             // Sync
            // block[2] = 0x01;                             // Block Type
            // block[3] = block_length;                     // Data Length

            for (uint16_t block_index = 0; block_index < block_length; block_index++){
                scanf("%c", &c);
                //printf("\n C: %d", c);

                // adds to block
                block[block_index] = c;

                // checksum
                cs += c;
                //printf("\n cs: %d", cs);
            }

            // verify checksum
            scanf("%c", &cs_received);
            printf("cs: %d, csr: %d\n", cs, cs_received);

            if (cs != cs_received) {
                // TODO Error handling
                printf("CHECKSUM ERROR");
            }
            else {
                printf("Checksum evaluated successfully");
            }

            // block[block_length + 6 - 2] = cs_received;               // Checksum

            // Trailer
            scanf("%c", &trailer_received);
            printf("\n%d", trailer_received);

            if (trailer_received != TRAILER) {
                printf("\nyou should never get here");
            }

            // block[block_length + 6 - 1] = TRAILER; // Trailer`

            break;


        default:
            // handle this better
            printf("TYPE ERROR");
            break;
    }

    
    return block_length_uint;
    
}


/* Continually scans for input. Does not terminate until 
an input block is received*/
void scan_for_input(){
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


void on_pwm_wrap() {
    pwm_clear_irq(0);
    pio_sm_put(pio0, sm, nextState);
    // pio0->txf[sm] = nextState; // Same as pio_sm_put without checking

    // Calculates delay from data block
    uint8_t elem = block[cycleCount]; //block must be of the correct length TODO: fix
    
    if (elem < 100) {
        delay = elem * 5;
    }

    else if (elem >= 100) {
        delay = (elem - 100) * 5;
    }
    
 
    // Update nextState for next cycle
    if (block[cycleCount] < 100) { // Negative pulses
        //delay = (100-cycleCount)*5; // Delay in PIO cycles @ 25 MHz
        nextState = negCycle;
    } else { // Positive pulses
        //delay = (cycleCount-100)*5; // Delay in PIO cycles @ 25 MHz
        nextState = possCycle;
    }
 
    if (delay < 25) {nextState = freeCycle;} // Lower bound on DCP (1 us + switching time)/20 us ~7.5%
    if (delay > 450) {delay = 450;}          // Upper bound on DCP (18 us + switching time)/20 us ~92.5%
    nextState = nextState | ( delay << 8);
 
    cycleCount++;
    //if (cycleCount > 200) {cycleCount=0;} // Wrap cycle count for test
}


// Split into seperate functions
void run_pulse() {
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

    pwm_set_enabled(0, true);

    busy_wait_ms(200);

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


int main() {
    uint16_t block_length;

    stdio_init_all();

    scan_for_input();

    switch (state) {
        case BLOCK_FOUND:
            block_length = get_block();
            break;
        
        default:
            break;
    }

    // for (int i = 0; i <= block_length; i++) {
    //     printf("\n%c", block[i]);
    // }

    run_pulse();

    return 0;
}