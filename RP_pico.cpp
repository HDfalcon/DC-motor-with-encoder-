#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define LED PICO_DEFAULT_LED_PIN
#define encA 2
#define encB 3
#define dir1 4
#define dir2 5
#define pwm 6

int32_t step = 0;

void encoder_callback(uint gpio, uint32_t events);
void drive(uint8_t dir, uint8_t duty, uint slice_num);
uint pwm_setup(uint8_t pin);

int main(){
    stdio_init_all();
    sleep_ms(100);

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

    //  PWM SETUP
    uint sliceNum = pwm_setup(pwm);
    drive(1, 50, sliceNum);
    while(1){
        gpio_put(LED, 1);
        drive(1, 80, sliceNum);
        sleep_ms(500);

        gpio_put(LED, 0);
        drive(0, 80, sliceNum);
        sleep_ms(100);
        printf("%d \n",step);
    }
    return 0;
}
//  CALLBACK
void encoder_callback(uint gpio, uint32_t events){
    int ENCB = gpio_get(encB);
    if(ENCB > 0){step++;}
    else{step--;}
}
//  motor driver
void drive(uint8_t dir, uint8_t duty, uint slice_num){
    if(dir == 1){ 
        gpio_put(dir1, 1);
        gpio_put(dir2, 0);
    }
    else { 
        gpio_put(dir1, 0);
        gpio_put(dir2, 1);
    }
        uint32_t level = (12500 * duty) / 100;
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pwm), level);
}
//  setting up pwm pin and getting slice number
uint pwm_setup(uint8_t pin){
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(pin);
        pwm_set_wrap(slice_num, 12500 - 1);
        pwm_set_enabled(slice_num, true);
        return slice_num;
    }
