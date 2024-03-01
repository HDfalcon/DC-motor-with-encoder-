#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"

#define LED PICO_DEFAULT_LED_PIN
#define encA 2
#define encB 3
#define dir1 4
#define dir2 5
#define pwm 6

float targetVel = 40;

volatile uint32_t velocity, currTime, prevTime;
float Kp=3, Ki=13;
uint32_t cTime, pTime, lTime;
float rpm, preRpm, filtRpm;
float err, errIntegral;

void encoder_callback(uint gpio, uint32_t events);
void drive(uint8_t dir, uint8_t duty, uint slice_num);
uint pwm_setup(uint8_t pin);
float f_abs(float var);

critical_section_t crit_velocity;

int main(){
    stdio_init_all();
    sleep_ms(100);

    critical_section_init(&crit_velocity);

    //  LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    //  Motor Driver and Encoder Pin Configuration
    gpio_init(dir1);
    gpio_set_dir(dir1, GPIO_OUT);
    
    gpio_init(dir2);
    gpio_set_dir(dir2, GPIO_OUT);
    
    gpio_init(pwm);
    gpio_set_dir(pwm, GPIO_OUT);

    gpio_init(dir1);
    gpio_set_dir(dir1, GPIO_OUT);

    gpio_init(encA);
    gpio_set_dir(encA, GPIO_IN);

    gpio_init(encB); 
    gpio_set_dir(encB, GPIO_IN);

    //  Interrupt Definition
    gpio_set_irq_enabled_with_callback(encA, 0x04, 1, encoder_callback);
                                        // 0x04 == GPIO_IRQ_EDGE_RISE
    //  PWM SETUP
    uint sliceNum = pwm_setup(pwm);
    drive(1, 50, sliceNum);
    while(1){
        pTime = cTime;
        cTime = time_us_32();   // taking loop time in microseconds
        lTime = cTime - pTime;

        //handling velocity in critical section
        critical_section_enter_blocking(&crit_velocity);
        rpm = (velocity * 60 / (16*20));
        critical_section_exit(&crit_velocity);
        //filtering velocity (25hz cutoff frequency-low pass filter)
        filtRpm = (0.854 * filtRpm) + (0.0728 * rpm) + (0.0728 * preRpm);
        preRpm = rpm;

        err = targetVel - filtRpm;
        errIntegral = errIntegral + err*lTime/1e6;

        float u = Kp*err + Ki*errIntegral;
        float pwr = (int)f_abs(u);

        if(pwr > 100){pwr = 100;}
        if(u < 0){drive(0, pwr, sliceNum);}
        else{drive(1, pwr, sliceNum);}
        printf("%f \n", filtRpm);
        sleep_ms(1);
    }
    return 0;
}
//  CALLBACK
void encoder_callback(uint gpio, uint32_t events){
    int ENCB = gpio_get(encB);
    int8_t increment = 0;
    if(ENCB > 0){increment = +1;}
    else{increment = -1;}
    currTime = time_us_32();
    float dTime = ((float)(currTime - prevTime)/1e6);
    velocity = increment / dTime;
    prevTime = currTime;
}
//  motor driver
void drive(uint8_t dir, uint8_t duty, uint slice_num){
        uint32_t level = (12500 * duty) / 100;
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm), level);
    if(dir == 1){ 
        gpio_put(dir1, 1);
        gpio_put(dir2, 0);
    }
    else { 
        gpio_put(dir1, 0);
        gpio_put(dir2, 1);
    }
}
//  setting up pwm pin and getting slice number
uint pwm_setup(uint8_t pin){
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(pin);
        pwm_set_wrap(slice_num, 12500 - 1);
        pwm_set_enabled(slice_num, true);
        return slice_num;
    }

float f_abs(float var){
    if (var < 0.0){var *= -1;}
    return var;
}
