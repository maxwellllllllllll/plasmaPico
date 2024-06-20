#include <stdio.h>
#include "pico/stdlib.h"

#define LED_PIN 25
char numCompare = 255;

// State Transition Stuff
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define GETTING_BLOCK 3


uint16_t get_block(){
    char c;
    
    char block_length;
    char* block;
    char cs_received;
    char trailer_received;
    uint8_t cs;

    char block_type;
    #define DATA 0x01
    #define TRAILER 0x55

    state = GETTING_BLOCK;

    scanf("%c", &block_type);
        
    switch (block_type) {
        case DATA:
            printf("getting data block");
            scanf("%c", &block_length);

            block = (char*)malloc(block_length * sizeof(char));

            // Builds start of block (already read)
            block[0] = 0x55;                            // Header
            block[1] = 0x3C;                            // Sync
            block[2] = 0x01;                            // Block Type
            block[3] = block_length;                    // Block Length

            for (uint16_t block_index = 4; block_index <= block_length - 3; block_index++){
                scanf("%c", &c);

                // adds to block
                block[block_index] = c;

                // checksum
                cs += c;
            }

            // verify checksum
            scanf("%c", &cs_received);

            if (cs != cs_received) {
                // TODO Error handling
                printf("CHECKSUM ERROR");
            }

            block[block_length + 6 - 2] = cs;              // Checksum

            // Trailer
            scanf("%c", &trailer_received);

            if (trailer_received != TRAILER) {
                printf("you should never get here");
            }

            block[block_length + 6 - 1] = trailer_received; // Trailer

        default:
            // handle this better
            printf("TYPE ERROR");
    }
    
}


void scan_for_input(){
    char in;
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
                    get_block();
                }
            default:
                break;
        }
    }
}


uint16_t get_block_old(){
    uint16_t buffer_index;
    char num;

    int block[10];

    while ( true ) {
        scanf("%c", &num);
        sleep_ms(100);

        if (num != numCompare){
            // echo
            printf("%d\n", num);

            // append to array
            block[buffer_index] = num;
            buffer_index ++;
        }
        else{
            printf("block: \n");
            for (int i=0; i <= buffer_index; i++){
                printf("%d,%d\n", block[i], buffer_index);
                sleep_ms(100);
            }
        }        
   }

   return buffer_index;
}


int main() {

    stdio_init_all();

    uint16_t buffer_index = get_block();

    return 0;
}