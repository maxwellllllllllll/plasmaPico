#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

#include "pinsToggle.pio.h"

uint32_t S1 = 0x00000008; // 1000
uint32_t S2 = 0x00000004; // 0100
uint32_t S3 = 0x00000002; // 0010
uint32_t S4 = 0x00000001; // 0001
 
uint32_t stop2free, free2stop, free2poss, free2neg, poss2free, neg2free;
uint32_t freeCycle, possCycle, negCycle;
uint32_t nextState, cycleCount;
 
uint sm0;

int main() {
    
}