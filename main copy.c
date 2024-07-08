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
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;

uint16_t delay;

uint sm;

uint16_t target;

int flag_track = 0;

// Device States
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define BLOCK_FOUND 3
#define GETTING_BLOCK 4

// Data Block
unsigned char* block;
// Contains uint 0-200 delay values output to pio (TODO: make this more clear)
unsigned char* gpio_block;
uint8_t gpio_block_cs;

//ADC Block
unsigned int* adc_block;


// For PWM Sync
int pwm_flag = 0;


const uint LED_PIN = 25;
const uint TRIGGER_PIN = 2;


// Debug nextState block
uint32_t next_state_block[1000];


typedef struct {

    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (s)*/
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError;        // for integrator
    float differentiator;
    float prevMeasurement;  // for differentiator

    /* Controller output */
    float out;

} pid_controller;


void pid_controller_init(pid_controller *pid) {

    // Clear controller variables
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}


float pid_controller_update(pid_controller *pid, float setpoint, float measurement) {

    float error = setpoint - measurement;

    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    // TODO: Add integrator clamping

    // Derivative
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) // NOTE: understand this better
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


    // Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {pid->out = pid->limMax;}
    else if (pid->out < pid->limMin) {pid->out = pid->limMin;}


    // Store error and measurement for next cycle
    pid->prevError = error;
    pid->prevMeasurement = measurement;


    // Return controller output
    return pid->out;
}


/*Initializes all nescessary memory arrays with sizes defined at runtime*/
void mem_init(uint16_t block_length){
    //block = (unsigned char*)malloc(block_length * sizeof(unsigned char)); // replace with specified length
    //adc_block = (unsigned int*)malloc(block_length * sizeof(unsigned int)); // for use later NOTE: Make sure using int instead of char works
    //gpio_block = (unsigned char*)malloc((block_length + 7) * sizeof(unsigned char)); // for use later
}


/*Once a block is detected by scan_for_input(), classifies and
collects the rest of the block*/
// TODO: Rewrite me to use terminator stuff!!
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

            mem_init(block_length_uint);

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
                //block[block_index] = c;

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

    //printf("here?");

    //pio_sm_put(pio0, sm, nextState); // Does moving this up here fix it? NOTE: Will break first pulse

    pio0->txf[sm] = nextState; // Same as pio_sm_put without checking

    // // Calculates delay from data block
    target = 52;//block[cycleCount]; //block must be of the correct length TODO: fix
    
    if (target < 100) {
        delay = target * 5;
    }

    else if (target >= 100) {
        delay = (target - 100) * 5;
    }
    
    // Update nextState for next cycle
    if (target < 100) { // Negative pulses
        //delay = (100-cycleCount)*5; // Delay in PIO cycles @ 25 MHz
        nextState = negCycle;
    } else { // Positive pulses
        //delay = (cycleCount-100)*5; // Delay in PIO cycles @ 25 MHz
        nextState = possCycle;
    }
 
    if (delay < 25) {nextState = freeCycle;} // Lower bound on DCP (1 us + switching time)/20 us ~7.5%
    if (delay > 450) {delay = 450;}          // Upper bound on DCP (18 us + switching time)/20 us ~92.5%
    nextState = nextState | ( delay << 8);

    // for debugging
    //next_state_block[cycleCount] = nextState;

    //pwm_flag = 1;
 
    cycleCount++;
    //if (cycleCount > 200) {cycleCount=0;} // Wrap cycle count for test
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


    // Initializes trigger pin
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_IN);

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


    //Use board LED as status indicator
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);
    
    return;
}


// Split into seperate functions
void run_pulse(uint16_t block_length_uint) {
    uint8_t *buffer;
    //uint16_t scale_target;
    uint16_t result;
    const float conversion_factor = 3.3f / (1 << 12);
    float setpoint;
    float measurement;


    // // PID Definitions (TODO: Figure out if I want this here or in header)
    // #define PID_KP 1.0f
    // #define PID_KI 1.0f
    // #define PID_KD 1.0f

    // #define PID_TAU 0.02f

    // #define PID_LIM_MIN -100.0f
    // #define PID_LIM_MAX 100.0f
    
    // #define PID_LIM_MIN_INT -50.0f
    // #define PID_LIM_MAX_INT 50.0f

    // #define SAMPLE_TIME_S 0.00002 // 20 us

    // pid_controller pid = {  PID_KP, PID_KI, PID_KD,
    //                         PID_TAU,
    //                         PID_LIM_MIN, PID_LIM_MAX,
	// 		                PID_LIM_MIN_INT, PID_LIM_MAX_INT,
    //                         SAMPLE_TIME_S };

    // pid_controller_init(&pid);


    // // Initializes pwm
    // init_pulse();    


    // Turn on SPA in freewheeling state and activate PWM
    //delay = 250;
    //nextState = (stop2free << 24) | (( delay << 8) | free2stop);
    //pio_sm_put(pio0, sm, nextState);


    // Wait until trigger pin is flipped
//    while (true) {if (gpio_get(TRIGGER_PIN) == 1) {break;}}


    // Start PWM
    pwm_set_enabled(0, true);
    

    // Pulse Loop
//     for (uint16_t cycle = 0; cycle <= block_length_uint; cycle++) {
//         // Target coil voltage
//         //target = block[cycle];

// //         // ADC Sampling
// //         //adc_block[cycle + 5] = (adc_read() * conversion_factor);

// // //        setpoint = (float)target;

// // //        measurement = setpoint; //(adc_block[cycle+5]/3.3) * 100; // This needs to be changed for actual coil (part of coil tuning)

// //         // PID Control
        
// //         //printf("\nT: %u, SP: %f", target, setpoint);

// // //        pid_controller_update(&pid, setpoint, measurement);

// //         //target = (uint8_t)pid.out;
        
// // //        printf("\nADC: %f, SP: %f, Target: %u", measurement, setpoint, target);

// // //        gpio_block[cycle + 5] = target;
// //         //printf("\nCycle: %u, Block: %u", cycle, gpio_block[cycle + 5]);
// // //        gpio_block_cs += target;

// //         if (target < 100) {
// //             scale_target = target * 5;
// //         }

// //         else if (target >= 100) {
// //             scale_target = (target - 100) * 5;
// //         }
// //         //printf("\nt: %u", target);

//         setpoint = block[cycle];

//         // adc_block[cycle + 5] = (adc_read() * conversion_factor);

//         printf("\n %f", setpoint);


//         // while (true) {
//         //     if (pwm_flag == 1) {
                    
//         //         target = setpoint;

//         //         pwm_flag = 0; 

//         //         break;
//         //     }
//         // }
//     }

    busy_wait_us(20*100);

    pwm_set_enabled(0, false);

    return;
}


void build_return_block(uint16_t block_length_uint) {
    // Return GPIO Block
    // Build GPIO Block
    gpio_block[0] = 0x55;                                // Head
    gpio_block[1] = 0x3C;                                // Sync
    gpio_block[2] = 0x02;                                // Block Type (GPIO Result)
    gpio_block[3] = 0x00;                                // Data Length First Byte
    gpio_block[4] = 0x00;                                // Data Length Second Byte
    // GPIO Data
    gpio_block[block_length_uint+1] = gpio_block_cs;     // Checksum
    gpio_block[block_length_uint+2] = 0x55;              // Trailer
}


void send_block(uint16_t block_length_uint) {
    for (int i=0; i < (block_length_uint + 7); i++) {
        //printf("\nRX: %u", gpio_block[i]);
    }
    //printf("done.");
}


int main() {
    uint16_t block_length_uint;
    int length; // very temp plz

    stdio_init_all();

    init_pulse();

    while (true) {
        scan_for_input();

        switch (state) {
            case BLOCK_FOUND:
                printf("\nblock found");
                block_length_uint = get_block();
                printf("\nblock loaded");
                break;
            
            default:
                break;
        }

        // for (int i = 0; i <= block_length; i++) {
        //     printf("\n%c", block[i]);
        // }

        printf("\nrunning pulse");
        run_pulse(block_length_uint);
        printf("\npulse ran");
    }

    shutdown_pulse();

    // for (int i = 0; i < 1000; i++) {
    //     printf("\nRX: %x", next_state_block[i]);
    // }

    //build_return_block(block_length_uint);

    //busy_wait_ms(1000); //DELETE MEEEEEEE
    //send_block(block_length_uint);


    return 0;
}