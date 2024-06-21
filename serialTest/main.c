#include <stdio.h>
#include "pico/stdlib.h"

#include "pico/malloc.h"


// State Transition Stuff
int state;
#define AWAIT_HEADER 1
#define AWAIT_SYNC 2
#define BLOCK_FOUND 3
#define GETTING_BLOCK 4

unsigned char* block;


uint16_t get_block(){
    char c;
    
    char block_length;
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
            scanf("%c", &block_length);
            block_length_uint = (uint16_t)block_length;
            //printf("\n %u bl:", block_length_uint);

            block = (unsigned char*)malloc(block_length_uint * sizeof(unsigned char)); // this is way better but noooooooo we have to allocate runtime resournces :(
            //printf(sizeof(block) / sizeof(block[0]));

            // Builds start of block (already read)
            // TODO: Move most of this up above switch case
            block[0] = 0x55;                             // Header
            block[1] = 0x3C;                             // Sync
            block[2] = 0x01;                             // Block Type
            block[3] = block_length;                     // Data Length

            for (uint16_t block_index = 4; block_index < block_length + 4; block_index++){
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
            //printf("\n%d", cs_received);

            if (cs != cs_received) {
                // TODO Error handling
                printf("CHECKSUM ERROR");
            }
            else {
                //printf("Checksum evaluated successfully");
            }

            block[block_length + 6 - 2] = cs_received;               // Checksum

            // Trailer
            scanf("%c", &trailer_received);

            if (trailer_received != TRAILER) {
                printf("you should never get here");
            }

            block[block_length + 6 - 1] = TRAILER; // Trailer

            break;


        default:
            // handle this better
            printf("TYPE ERROR");
            break;
    }

    
    return block_length_uint;
    
}


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
                    //printf("Await header");
                }
                break;
            
            case AWAIT_SYNC:
                if (in == 0x3C) {
                    //printf("await sync");
                    state = BLOCK_FOUND;
                    return;
                }
            default:
                break;
        }
    }
}


// uint16_t get_block_old(){
//     uint16_t buffer_index;
//     int numCompare;
//     char num;
//     int block[10];
//     while ( true ) {
//         scanf("%c", &num);
//         sleep_ms(100);
//         if (num != numCompare){
//             // echo
//             printf("%d\n", num);
//             // append to array
//             block[buffer_index] = num;
//             buffer_index ++;
//         }
//         else{
//             printf("block: \n");
//             for (int i=0; i <= buffer_index; i++){
//                 printf("%d,%d\n", block[i], buffer_index);
//                 sleep_ms(100);
//             }
//         }        
//    }
//    return buffer_index;
// }


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

    for (int i = 0; i <= block_length + 6; i++) {
        printf("\n%c", block[i]);
    }

    return 0;
}