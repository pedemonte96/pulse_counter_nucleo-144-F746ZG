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
//------------------------------------------------------------------------------
//  Reference program No.1 (see line 122)
//------------------------------------------------------------------------------
//  5MHzOSC
//      https://os.mbed.com/users/mio/code/5MHzOSC/
//  by fuyono sakura
//      https://os.mbed.com/users/mio/

//------------------------------------------------------------------------------
//  Reference program No.2 (see line 197)
//------------------------------------------------------------------------------
//  fc114
//      https://os.mbed.com/users/rutles/code/fc1114/
//  by Tetsuya Suzuki
//      https://os.mbed.com/users/rutles/

#include "mbed.h"
#include "freq_counter.h"

F_COUNTER::F_COUNTER(PinName f_in): _pin(f_in)
{
    initialize();
}

void F_COUNTER::initialize(void)
{
#if defined(TARGET_LPC1768)
#   warning "My board is gone, so I could NOT check updated SW on this board!!"
    LPC_SC->PCONP |= 1 << 22;       // 1)Power up TimerCounter3 (bit23)
    LPC_PINCON->PINSEL0 |= 3 << 8;  // 2)Set P0[23] to CAP3[0]
    LPC_TIM2->TCR  = 2;             // 3)Counter Reset (bit1<=1,bit0<=0)
    LPC_TIM2->CTCR = 1;             // 4)Count on riging edge Cap3[0]
    LPC_TIM2->CCR  = 0;             // 5)Input Capture Disabled
    LPC_TIM2->TCR  = 1;             // 6)Counter Start (bit1<=0,bit0<=1)
#elif defined(TARGET_LPC1114)
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);  //TMR32B0 wakeup
    LPC_IOCON->PIO1_5 |= (1 << 1);  // Set dp14 (pin20) as CT32B0_CAP0
    LPC_IOCON->PIO1_5 |= (1 << 5);  // Hysteresis enable
    LPC_TMR32B0->TCR = 2;           // reset
    LPC_TMR32B0->CTCR  = 1;         // counter mode
    LPC_TMR32B0->CCR  = 0;          // Input Capture Disable)
    LPC_TMR32B0->PR  = 0;           // no prescale
    LPC_TMR32B0->TCR = 1;           // start
#elif defined(TARGET_NUCLEO_F401RE)\
    || defined(TARGET_NUCLEO_F411RE)\
    || defined(TARGET_NUCLEO_F746ZG)\
    || defined(TARGET_NUCLEO_F446RE)
    
    // TIMER 2
    // PA5 -> Counter frequency input pin as Timer2 TI1, channel 1
    GPIOA->AFR[0] &= 0xff0fffff;
    GPIOA->AFR[0] |= 0x00100000;//AF 1 (Data sheet page 77), timer 2 channel 1 of pin PA5
    GPIOA->MODER &= ~(GPIO_MODER_MODER5);
    GPIOA->MODER |= 0x00000800; //Alternate function mode (2) of pin 5, each pin is 2 bits
    // Initialize Timer2(32bit) for an external up counter mode
    RCC->APB1ENR |= ((uint32_t)0x00000001); //TIM2 Enabled
    // count_up + div by 1
    TIM2->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD));
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->PSC = 0x0000;
    TIM2->CCMR1 &= (uint16_t)~TIM_CCMR1_IC1F;   // input filter
    TIM2->CCER = TIM_CCER_CC1P;     // positive edge
    TIM2->SMCR &= (uint16_t)~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ECE);
    TIM2->SMCR |= (uint16_t)(TIM_TS_TI1FP1 | TIM_SMCR_SMS); // external mode 1
    
    // TIMER 3 Channel 1
    // PB4 -> Counter frequency input pin as Timer3 TI1, channel 1 AF2
    GPIOB->AFR[0] &= 0xfff0ffff;//Since it is pin 4 of port B, modify AFR4 of GPIOB
    GPIOB->AFR[0] |= 0x00020000;//Alternate function 2 (Data sheet page 77), timer 3 channel 1 of pin PB4
    GPIOB->MODER &= ~(GPIO_MODER_MODER4);
    GPIOB->MODER |= 0x00000200; //Alternate function mode (2) of pin 4, each pin is 2 bits
    // Initialize Timer3(32bit) for an external up counter mode
    RCC->APB1ENR |= ((uint32_t)0x00000002);//TIM3 Enabled
    // count_up + div by 1
    TIM3->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD));
    TIM3->ARR = 0xFFFFFFFF;
    TIM3->PSC = 0x0003;
    TIM3->EGR = TIM_EGR_UG;// to update the prescaler register!
    TIM3->CCMR1 &= (uint16_t)~TIM_CCMR1_IC1F;   // input filter
    TIM3->CCER = TIM_CCER_CC1P;     // positive edge
    TIM3->SMCR &= (uint16_t)~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ECE);
    TIM3->SMCR |= (uint16_t)(TIM_TS_TI1FP1 | TIM_SMCR_SMS); // external mode 1
    
    // TIMER 4 Channel 1
    // PB6 -> Counter frequency input pin as Timer4 TI1, channel 1 AF2
    GPIOB->AFR[0] &= 0xf0ffffff;//Since it is pin 6 of port B, modify AFR6 of GPIOB
    GPIOB->AFR[0] |= 0x02000000;//Alternate function 2 (Data sheet page 77), timer 3 channel 1 of pin PB4
    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= 0x00002000; //Alternate function mode (2) of pin 6, each pin is 2 bits
    // Initialize Timer3(32bit) for an external up counter mode
    RCC->APB1ENR |= ((uint32_t)0x00000004);//TIM4 Enabled
    // count_up + div by 1
    TIM4->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD));
    TIM4->ARR = 0xFFFFFFFF;
    TIM4->PSC = 0x0000;
    TIM4->CCMR1 &= (uint16_t)~TIM_CCMR1_IC1F;   // input filter
    TIM4->CCER = TIM_CCER_CC1P;     // positive edge
    TIM4->SMCR &= (uint16_t)~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ECE);
    TIM4->SMCR |= (uint16_t)(TIM_TS_TI1FP1 | TIM_SMCR_SMS); // external mode 1
    
#else
#   error "No support for this CPU"
#endif
}

uint32_t F_COUNTER::read_frequency(uint32_t gate_time)
{
    uint32_t f = rd_frq(gate_time);
    if (gate_time == 10000000){         // 10sec
        return f / 10;
    } else if (gate_time == 1000000){    // 1sec
        return f;
    } else if (gate_time == 100000){    // 100mS
        return f * 10;
    } else if (gate_time == 10000){     // 10mS
        return f * 100;
    } else {                            // Others (user defined)
        float ff = (float)f * 1e6 / (float)gate_time;
        return (uint32_t)ff;
    }
}

uint32_t F_COUNTER::count_pulses(float gate_time)
{
    uint32_t gate_time_micro = (uint32_t)(gate_time * 1e6);
    return rd_frq(gate_time_micro);
}

uint32_t F_COUNTER::rd_frq(uint32_t gate_time)
{
#if defined(TARGET_LPC1768)
    LPC_TIM2->TCR = 2;              // Reset the counter (bit1<=1,bit0<=0)
    LPC_TIM2->TCR = 1;              // UnReset counter (bit1<=0,bit0<=1)
    wait_us(gate_time);
    freq = LPC_TIM2->TC;            // read counter
#elif defined(TARGET_LPC1114)
    LPC_TMR32B0->TCR = 2;           // reset
    LPC_TMR32B0->TCR = 1;           // start
    wait_us(gate_time);
    freq = LPC_TMR32B0->TC;         // read counter
#elif defined(TARGET_NUCLEO_F401RE)\
    || defined(TARGET_NUCLEO_F411RE)\
    || defined(TARGET_NUCLEO_F746ZG)\
    || defined(TARGET_NUCLEO_F446RE)
    initialize();
    TIM2->CNT = 0;
    TIM3->CNT = 0;
    TIM4->CNT = 0;
    
    TIM2->CR1 |= TIM_CR1_CEN;   //Enable the TIM2 Counter
    TIM3->CR1 |= TIM_CR1_CEN;   //Enable the TIM3 Counter 
    TIM4->CR1 |= TIM_CR1_CEN;   //Enable the TIM4 Counter
    
    wait_us(gate_time);
    
    TIM4->CR1 &= ~TIM_CR1_CEN;   //Disable the TIM4 Counter
    TIM2->CR1 &= ~TIM_CR1_CEN;   //Disable the TIM2 Counter
    TIM3->CR1 &= ~TIM_CR1_CEN;   //Disable the TIM3 Counter
    
    
    printf("A_OUT (PA5 = CN7_10, TIM2_CH1):\t\t\t\t\t\t%u\n", TIM2->CNT);
    printf("B_OUT (PB4 = CN7_19, TIM3_CH1, prescaler=4 taken into account):\t\t%u\n", TIM3->CNT*4);
    printf("COINC_OUT (PB6 = CN10_13, TIM4_CH1):\t\t\t\t\t%u\n", TIM4->CNT);
    freq = TIM2->CNT + TIM3->CNT*4 + TIM4->CNT;               // read counter
#else
#   error "No support for this CPU"
#endif
    return freq;
}

////////////////////////// Reference ///////////////////////////////////////////
#if 0
//
//  CLOCK OUT to PWM1[6] Sample with Freq Counter using Cap2.0
//  For LPC1768-mbed
//
//  Reference:
//      5MHz Clock Out Code and Comment - http://mbed.org/forum/mbed/topic/733/
//
//  !! To Self Measurement Output Clock, Connect p21 <-> p30 with jumper wire.
//  2013.6.18 : Wrong comment about MR6 and Duty fix.
//

#include "mbed.h"

PwmOut fmclck(p21);     // for RESERVE pin21 as PWM1[6]
DigitalIn clkin(p30);   // for RESERVE pin30 as CAP2[0]

// Reset Counter and Count Start
void P30_RESET_CTR(void)
{
    LPC_TIM2->TCR = 2;  // Reset the counter (bit1<=1,bit0<=0)
    LPC_TIM2->TCR = 1;  // UnReset counter (bit1<=0,bit0<=1)
}

// Get Counter Value
int P30_GET_CTR(void)
{
    return LPC_TIM2->TC; // Read the counter value
}

// Setting p30 to Cap2.0
void P30_INIT_CTR(void)
{
    LPC_SC->PCONP |= 1 << 22;       // 1)Power up TimerCounter2 (bit22)
    LPC_PINCON->PINSEL0 |= 3 << 8;  // 2)Set P0[4] to CAP2[0]
    LPC_TIM2->TCR = 2;              // 3)Counter Reset (bit1<=1,bit0<=0)
    LPC_TIM2->CTCR = 1;             // 4)Count on riging edge Cap2[0]
    LPC_TIM2->CCR = 0;              // 5)Input Capture Disabled
    LPC_TIM2->TCR = 1;              // 6)Counter Start (bit1<=0,bit0<=1)
}

// Clock Output From pin21(PWM6)
// Set Clock Freq with div.
// if mbed is running at 96MHz, div is set 96 to Get 1MHz.
void PWM6_SETCLK(int div)
{
    // 1)Reset counter, disable PWM
    LPC_PWM1->TCR = (1 << 1);
    LPC_SC->PCLKSEL0 &= ~(0x3 << 12);
    // 2)Set peripheral clock divider to /1, i.e. system clock
    LPC_SC->PCLKSEL0 |= (1 << 12);
    // 3)Match Register 0 is shared period counter for all PWM1
    LPC_PWM1->MR0 = div - 1;
    LPC_PWM1->MR6 = (div + 1)>> 1;
    // 4)Start updating at next period start
    LPC_PWM1->LER |= 1;
    // 5)Enable counter and PWM
    LPC_PWM1->TCR = (1 << 0) || (1 << 3);
}

int main()
{
    // Outout mbed's "PWM6" pin to 96MHZ/19 = 5.052MHz (Approx)
    PWM6_SETCLK(19) ;
    // Outout mbed's "PWM6" pin to 96MHZ/96 = 1.000MHz (Approx)
    // PWM6_SETCLK(96) ;
    P30_INIT_CTR();
    while(1) {
        P30_RESET_CTR();
        wait(1.0); // Gate time for count
        printf("pin30 Freq = %d (Hz)\r\n",P30_GET_CTR());
    }
}
#endif

#if 0
// fc1114 - Frequency counter with i2c slave.
// target: LPC1114FN28

#include "mbed.h"

#define I2C_ADRS 0x70

DigitalOut led(dp28);
I2CSlave slave(dp5, dp27);
Ticker tick;

volatile uint32_t frq;

void isr_tick()
{
    frq = LPC_TMR32B0->TC;
    LPC_TMR32B0->TC = 0;

    led = !led;
}

int main()
{
    union {
        char b[4];
        uint32_t w;
    } buf;
    char dummy[4];

    led = 1;
    tick.attach(&isr_tick, 1);

    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);//TMR32B0 wakeup
    LPC_IOCON->PIO1_5 |= (1 << 1);// Set PIN14 as CT32B0_CAP0
    LPC_IOCON->PIO1_5 |= (1 << 5);// Hysteresis enable
    LPC_TMR32B0->TCR = 2; // reset
    LPC_TMR32B0->CTCR  = 1; // counter mode
    LPC_TMR32B0->CCR  = 0; // Input Capture Disable)
    LPC_TMR32B0->PR  = 0;// no prescale
    LPC_TMR32B0->TCR = 1; // start

    slave.address(I2C_ADRS << 1);

    while (1) {
        int i = slave.receive();
        switch (i) {
            case I2CSlave::ReadAddressed:
                buf.w = frq;
                slave.write(buf.b, 4);
                break;
            case I2CSlave::WriteGeneral:
                slave.read(dummy, 4);
                break;
            case I2CSlave::WriteAddressed:
                slave.read(dummy, 4);
                break;
        }
    }
}
#endif
