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

#ifndef        MBED_F_COUNTER
#define        MBED_F_COUNTER

#include "mbed.h"

/** Frequency Counter
 *
 *  CAUTION: Direct access to the CPU Timer module!!
 *           No way to change pin asｓign and timer module,
 *      mbed LPC1768  -> p30,
 *      mbed LPC1114FN28 -> dp14,
 *      Nucleo F401RE,F411RE & F446RE -> PA_0/A0
 *
 * @code
 * #include "mbed.h"
 * #include "freq_counter.h"
 *
 * //F_COUNTER fc(p30);     // for LPC1768
 * //F_COUNTER fc(dp14);    // for LPC1114
 * F_COUNTER fc(PA_0);    // for F401,F411,F446
 *
 * int main() {
 *   uint32_t frequency = 0;
 *
 *   while(true) {
 *      freqency = fc.read_frequency(1000000);  // gate time: 1 sec
 *      printf("%d [Hz]", frequency);
 *   }
 * }
 * @endcode
 */

class F_COUNTER
{
public:
    /** Configure data pin
      * @param frequency counter input pin
      */
    F_COUNTER(PinName f_in);

    /** Read measured frequency
      * @param gate time (uSec) gate = 1sec -> set 1000000
      * @return measured frequency
      */
    uint32_t read_frequency(uint32_t gate_time);
    
    /** Read counted pulses
      * @param gate time (Sec) gate
      * @return measured pulses
      */
    uint32_t count_pulses(float gate_time);

protected:
    DigitalIn _pin;
    Timer _t;

    void initialize(void);
    uint32_t rd_frq(uint32_t gate_time);

private:
    uint32_t freq;

};

#endif  //  MBED_F_COUNTER
