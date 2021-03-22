/*
 * mbed Library program
 *      Frequency Counter Hardware relataed program
 *
 * Copyright (c) 2014,'20 Kenji Arai / JH1PJL
 *  http://www7b.biglobe.ne.jp/~kenjia/
 *  https://os.mbed.com/users/kenjiArai/
 *      Additional functions and modification
 *      started: October   18th, 2014
 *      Revised: August     4th, 2020
 */

#include "mbed.h"
#include "freq_counter.h"

//F_COUNTER fc(p30);     // for LPC1768
//F_COUNTER fc(dp14);    // for LPC1114
F_COUNTER fc(PA_0);    // for F401,F411,F446

int main() {
    uint32_t frequency = 0;
    float time_window = 1.0; //seconds
    DigitalOut led(LED1);
    printf("Measuring...\n");

    while(true) {
          led = !led;
          frequency = fc.count_pulses(time_window);  // gate time in seconds
          printf("Counts in %d ms:\t\t\t\t\t\t\t%d\n\n", int(time_window*1000), frequency);
    }
}