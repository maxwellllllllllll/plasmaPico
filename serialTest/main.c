#include <stdio.h>
#include "pico/stdlib.h"

#define LED_PIN 25
char num;
char numCompare = 255;

uint8_t block[10];

uint16_t get_block(){
    uint16_t buffer_index;

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