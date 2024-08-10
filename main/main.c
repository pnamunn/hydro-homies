#include <stdio.h>
#include <stdint.h>

void app_main(void)
{
    while(1) {
        printf("Aloha ahydro ahomies.");
        for(uint32_t i = 0; i < 4000000; i++) {}
    }
}
