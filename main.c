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
unsigned char* block; // loaded with data for pulse
uint16_t __not_in_flash("pwm") block_length_uint;

// Return Block
uint32_t* pio_block;
uint8_t pio_block_cs; //is it better to have the checksums calculated at the end as part of return_block()? It's slightly slower but maybe cleaner? TODO: figure this out

// ADC block
uint16_t* adc_block;
uint8_t adc_block_cs;

// Facilitates cpu-pwm communication
uint8_t __not_in_flash("pwm") pwm_flag;


// Device States
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define BLOCK_FOUND 3
#define GETTING_BLOCK 4

// Block Types
char block_type;
#define DATA 0x01


// PID Controller
typedef struct {

    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

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


float __time_critical_func(pid_controller_update)(pid_controller *pid, float setpoint, float measurement) {

    float error = setpoint - measurement;

    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid-> Ki * pid->T * (pid->integrator + error);

    // TODO: Add integrator clamping

    // Derivative
    pid->differentiator = error - pid->prevError;


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


void __time_critical_func(on_pwm_wrap)() {
    pwm_clear_irq(0);

    //pio_sm_put(pio0, sm, nextState);
    pio0->txf[sm] = nextState; // Same as pio_sm_put without checking

    // // Calculates delay from data block
    //target = 52; //block[cycleCount];
    
    // Update nextState for next cycle
    if (target < 100) { // Negative pulses
        nextState = negCycle;
        delay = (target) * 5; // Delay in PIO cycles @ 25 MHz
    } else { // Positive pulses
        nextState = possCycle;
        delay = (target-100) * 5; // Delay in PIO cycles @ 25 MHz
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


    // Setting up ADC
    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

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
void run_pulse(uint16_t pulseCycles) {
    uint32_t setpoint;
    float measurement; // ADC in
    const float conversion_factor = 3.3f / (1 << 12); // Converts ADC signal to voltage

    // Loads freewheeling state as first PWM pulse
    delay = 250;
    nextState = (stop2free << 24) | (( delay << 8) | free2stop);


    // Sets up PID
    // PID Definitions (TODO: Figure out if I want this here or in header)
    #define PID_KP 1.0f
    #define PID_KI 1.0f
    #define PID_KD 1.0f

    #define PID_LIM_MIN 0.0f
    #define PID_LIM_MAX 200.0f
    
    #define PID_LIM_MIN_INT 0.0f
    #define PID_LIM_MAX_INT 100.0f

    #define SAMPLE_TIME_S 0.00002 // 20 us

    pid_controller pid = {  PID_KP, PID_KI, PID_KD,
                            PID_LIM_MIN, PID_LIM_MAX,
			                PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                            SAMPLE_TIME_S };

    pid_controller_init(&pid);

    // Start PWM
    pwm_set_enabled(0, true);

    // Pulse Loop
    for (uint16_t cycle = 0; cycle <= pulseCycles; cycle++) {
        setpoint = block[cycle];

        measurement = (adc_read() * conversion_factor) * 100; // NOTE: ADC sample comes under 20us earlier than pio output (0-200)
        adc_block[cycle + 5] = measurement;

        // PID Control
        pid_controller_update(&pid, setpoint, measurement);

        // Loads PID modilated waveform to return array
        pio_block[cycle + 5] = ((uint32_t)(pid.out));

        //printf("\nSP: %u, ADC: %f, PID: %u", setpoint, measurement, pio_block[cycle+5]);

        while (true) {
            if (pwm_flag == 1) {target = pio_block[cycle + 5]; pwm_flag = 0; break;}
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

        switch (state) {
            case AWAIT_HEADER:
                if (in == 0x55) {
                    state = AWAIT_SYNC;
                }
                break;
            
            case AWAIT_SYNC:
                if (in == 0x3C) {
                    state = BLOCK_FOUND;
                    return;
                }
            default:
                break;
        }
    }
}


/*Initializes all nescessary memory arrays with sizes defined at runtime*/
void mem_init(uint16_t block_length) {
    block = (unsigned char*)malloc(block_length * sizeof(unsigned char));
    pio_block = (uint32_t*)malloc((block_length + 7) * sizeof(uint32_t)); // TODO: remember to change this once return_block() is reworked
    adc_block = (uint16_t*)malloc((block_length + 7) * sizeof(uint16_t));
}


/*Once a block is detected by scan_for_input(), classifies and
collects the rest of the block*/
// TODO: Rewrite me to use terminator stuff!!
uint16_t get_block(){
    char c_in;
    
    char block_length1;
    char block_length2;
    uint16_t block_length_uint;

    #define TRAILER 0x55
    #define DATA_BLOCK_LENGTH 256

    // checksum
    uint8_t cs = 0; // calculated checksum
    char cs_received; // recieved checksum

    char trailer_received;

    state = GETTING_BLOCK;

    // clears block TODO: make sure this works
    for (int i = 0; i < block_length_uint; i++) {
        block[i] = '\0';
    }


    scanf("%c", &block_type);
        
    switch (block_type) {
        case DATA:
            scanf("%c", &block_length1);
            scanf("%c", &block_length2);

            int block_length = (block_length2 << 8) | block_length1;
            block_length_uint = (uint16_t)block_length;

            mem_init(block_length_uint);

            // Sequentially reads all data bytes and loads them into block
            for (uint16_t block_index = 0; block_index < block_length; block_index++) { //Rewrite me to use trailer byte once that is better implimented
                scanf("%c", &c_in);

                block[block_index] = c_in;

                // checksum
                cs += c_in;
            }

            // verify checksum
            scanf("%c", &cs_received);
            if (cs != cs_received) {
                // TODO Error handling
                printf("CHECKSUM ERROR");
            }

            // trailer
            scanf("%c", &trailer_received);
            if (trailer_received != TRAILER) {
                printf("\nyou should never get here");
            }

            break;
        
        
        default:
            // handle this better
            printf("TYPE ERROR");
            break;
    }


    return block_length_uint;
}


void build_return_pio(uint16_t block_length) { // for some reason a general return func doesn't work as well TODO: figure out why

    // Return GPIO Block
    // Build GPIO Block
    pio_block[0] = 0x55;                                             // Head
    pio_block[1] = 0x3C;                                             // Sync
    pio_block[2] = 0x02;                                             // Block Type (GPIO Result)
    pio_block[3] = 0x00;                                             // Data Length First Byte
    pio_block[4] = 0x00;                                             // Data Length Second Byte
   
    // [GPIO Data]
   
    pio_block[block_length+1] = (uint32_t)pio_block_cs;              // Checksum
    pio_block[block_length+2] = 0x55;                                // Trailer
}


void send_pio(uint16_t block_length) {
    for (int i=0; i < (block_length + 7); i++) {
        printf("\nRX: %u", pio_block[i]);
    }
}


void build_return_adc(uint16_t block_length) {

    // Return GPIO Block
    // Build GPIO Block
    adc_block[0] = 0x55;                                             // Head
    adc_block[1] = 0x3C;                                             // Sync
    adc_block[2] = 0x03;                                             // Block Type (ADC Result)
    adc_block[3] = 0x00;                                             // Data Length First Byte
    adc_block[4] = 0x00;                                             // Data Length Second Byte
   
    // [GPIO Data]
   
    adc_block[block_length+1] = (uint32_t)pio_block_cs;              // Checksum
    adc_block[block_length+2] = 0x55;                                // Trailer
}


void send_adc(uint16_t block_length) {
    for (int i=0; i < (block_length + 7); i++) {
        printf("\nRX: %u", adc_block[i]);
    }
}


int main() { //TODO: Rewrite so that command pulses are possible (enable/disable pid, etc.)
uint16_t numCycles;

    stdio_init_all();

    scan_for_input();

    numCycles = get_block();

    sleep_ms(5000); // Needed to allow stdio init to complete without interfearing with FIFO buffers

    init_pulse();

    // Loops pulses for new inputs
    while (true) {
        run_pulse(numCycles);

        build_return_pio(numCycles);
        send_pio(numCycles);
        sleep_ms(10);
        build_return_adc(numCycles);
        send_adc(numCycles);

        
        scan_for_input();
        numCycles = get_block(); // put this in an if statment depending on what scan_for_input returns
        sleep_ms(5000); // NOTE: will need to figure out where to do this once scan_for_input returns implimented

        //add in break here if scan_for_input returns a different thing
    }
    

    shutdown_pulse();


    return 0;
}