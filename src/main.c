
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include <platform-abstraction/threading.h>
#include <platform-abstraction/semaphore.h>
#include <platform-abstraction/mutex.h>
#include <platform-abstraction/timestamp.h>

#include "beacon_angles.h"
#include "positioning.h"
#include "kalman.h"
#include "beacon_config.h"


void uart2_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    // uart tx pin
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
    // uart rx pin
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

    rcc_periph_clock_enable(RCC_USART2);
    usart_set_baudrate(USART2, 19200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

void exti_irq_init(void)
{
    // SYSCFG clock is needed for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG);

    // Setup for PF0
    exti_enable_request(EXTI0);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI0, GPIOF);
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    // Setup for PF1
    exti_enable_request(EXTI1);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI1, GPIOF);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    // Setup for PD2
    exti_enable_request(EXTI2);
    exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI2, GPIOD);
    nvic_enable_irq(NVIC_EXTI2_TSC_IRQ);
    /*
    // Setup for PB8 & PB9
    exti_enable_request(EXTI8);
    exti_set_trigger(EXTI8, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI8, GPIOB);
    exti_enable_request(EXTI9);
    exti_set_trigger(EXTI9, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI9, GPIOB);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    // Setup for PA10
    exti_enable_request(EXTI10);
    exti_set_trigger(EXTI10, EXTI_TRIGGER_RISING);
    exti_select_source(EXTI10, GPIOD);
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
    */

    // We'll also need to read the state of PB8 & PB9
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO9);
}


#define FPCCR (*((volatile uint32_t *)0xE000EF34))
#define CPACR (*((volatile uint32_t *)0xE000ED88))

void fpu_config(void)
{
    // Enable the Floating-point coprocessor (CP10 and CP11 to full access)
    CPACR |= (0x03<<(2*10)|(0x03<<(2*11)));

    __asm__ volatile (
        "dsb \n\t"  /* wait for store to complete */
        "isb \n\t"  /* reset pipeline, FPU is now enabled */
        :::);

    uint32_t fpccr = 0;
    // Disable automatic state preservation of FP state
    fpccr &= ~(1<<31);
    // Enable Lazy context save of FP state
    // -> whole fpu context is saved by ucos
    fpccr |= (1<<30);

    FPCCR = fpccr;
}


beacon_angles_t laser_one;
beacon_angles_t laser_two;

position_t beacon_a = BEACON_POS_A;
position_t beacon_b = BEACON_POS_B;
position_t beacon_c = BEACON_POS_C;
reference_triangle_t table;

robot_pos_t robot_one_pos;
mutex_t robot_one_pos_access;

position_t laser_one_pos;
mutex_t laser_one_pos_access;
semaphore_t laser_one_pos_ready;

os_thread_t laser_one_thread;
THREAD_STACK laser_one_stack[1024];

#define DEG(X) (X * 180 / 3.14159)

void laser_one_main(void *context)
{
    (void) context;

    printf("Laser One Thread\n");

    while (1) {

        os_semaphore_wait(&laser_one.measurement_ready);
        if(beacon_angles_calculate(&laser_one)){
            os_mutex_take(&laser_one_pos_access);
            os_mutex_take(&laser_one.access);
            if(positioning_from_angles(
                        laser_one.alpha,
                        laser_one.gamma,
                        laser_one.beta,
                        &table, &laser_one_pos)){
                gpio_toggle(GPIOB, GPIO13);
                os_semaphore_signal(&laser_one_pos_ready);
                //printf("x: %f, y: %f\n\r", laser_one_pos.x, laser_one_pos.y);
            }

            os_mutex_release(&laser_one_pos_access);
            os_mutex_release(&laser_one.access);
        }
    }
}


os_thread_t kalman_thread;
THREAD_STACK kalman_stack[1024];

void kalman_main(void *context)
{
    uint32_t timestamp;
    uint32_t timestamp_diff;
    uint32_t period;
    uint32_t wait_time_us;
    float delta_t;
    robot_pos_t init_pos;
    kalman_robot_handle_t handle;

    init_pos.x = KALMAN_INIT_POS_X;
    init_pos.y = KALMAN_INIT_POS_Y;
    init_pos.var_x = KALMAN_INIT_POS_VAR;
    init_pos.var_y = KALMAN_INIT_POS_VAR;
    init_pos.cov_xy = 0.0f;

    kalman_init(&handle, &init_pos);

    period = 1000000 / KALMAN_TRANS_FREQ;

    timestamp = os_timestamp_get();

    // update loop
    while(42){

        timestamp_diff = os_timestamp_diff_and_update(&timestamp);
        wait_time_us = period - timestamp_diff;
        if(wait_time_us > 0){
            os_thread_sleep_least_us(wait_time_us);
        }

        timestamp_diff += os_timestamp_diff_and_update(&timestamp);
        delta_t = timestamp_diff / 1000000.0f;

        os_mutex_take(&robot_one_pos_access);
        if(os_semaphore_try(&laser_one_pos_ready)){
            kalman_update(&handle, &laser_one_pos, delta_t, &robot_one_pos);
        } else{
            kalman_update(&handle, NULL, delta_t, &robot_one_pos);
        }
        os_mutex_release(&robot_one_pos_access);
    }
}


os_thread_t communication_thread;
THREAD_STACK communication_stack[1024];

void communication_main(void *context)
{
    robot_pos_t robot_one_pos_copy;
    while(42){
        os_thread_sleep_least_us(1000000 / OUTPUT_FREQ);
        os_mutex_take(&robot_one_pos_access);
        memcpy(&robot_one_pos_copy, &robot_one_pos, sizeof(robot_pos_t));
        os_mutex_release(&robot_one_pos_access);
        printf("%d %.3f x: %1.3f, y: %1.3f, var_x: %1.3f, var_y: %1.3f, cov: %1.3f\n\r",
                updates, time_passed / updates, robot_one_pos_copy.x, robot_one_pos_copy.y,
                robot_one_pos_copy.var_x, robot_one_pos_copy.var_y, robot_one_pos_copy.cov_xy);
    }

}

int main(void)
{
    rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);

    fpu_config();

    uart2_init();

    beacon_angles_init(&laser_one);
    beacon_angles_init(&laser_two);
    beacon_angles_set_minimal_period(&laser_one, 50000);
    beacon_angles_set_minimal_period(&laser_two, 50000);

    positioning_reference_triangle_from_points(&beacon_a, &beacon_b, &beacon_c, &table);


    exti_irq_init();

    // User LED
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);


    os_mutex_init(&robot_one_pos_access);

    os_mutex_init(&laser_one_pos_access);
    os_semaphore_init(&laser_one_pos_ready, 0);


    os_init();

    os_thread_create(&laser_one_thread, laser_one_main, laser_one_stack,
            sizeof(laser_one_stack), "L1", 0, NULL);
    os_thread_create(&kalman_thread, kalman_main, kalman_stack,
            sizeof(kalman_stack), "Kalman", 0, NULL);
    os_thread_create(&communication_thread, communication_main, communication_stack,
            sizeof(communication_stack), "Communication", 2, NULL);

    os_run();

    while (1);
}


// Beacon A, laser 1
// PF0 (connected to pin 6 on JP5)
void exti0_isr(void)
{
    exti_reset_request(EXTI0);
    beacon_angles_update_timestamp(&laser_one, A, os_timestamp_get());
}

// Beacon B, laser 1
// PF1 (connected to pin 10 on JP5)
void exti1_isr(void)
{
    exti_reset_request(EXTI1);
    beacon_angles_update_timestamp(&laser_one, B, os_timestamp_get());
}

// Beacon C, laser 1
// PD2 (connected to pin 12 on JP5)
void exti2_tsc_isr(void)
{
    exti_reset_request(EXTI2);
    beacon_angles_update_timestamp(&laser_one, C, os_timestamp_get());
}

// Beacon A and B, laser 2
// PB8 (connected to pin 5 on JP5) & PB9 (connected to pin 9 on JP5)
void exti9_5_isr(void)
{
    // Clear interrupt request flag
    exti_reset_request(EXTI8);
    exti_reset_request(EXTI9);

    if(gpio_get(GPIOB, GPIO8)){
        beacon_angles_update_timestamp(&laser_two, A, os_timestamp_get());
    }

    if(gpio_get(GPIOB, GPIO9)){
        beacon_angles_update_timestamp(&laser_two, B, os_timestamp_get());
    }
}

// Beacon C, laser 2
// PA10 (connected to pin 11 on JP5)
void exti15_10_isr(void)
{
    exti_reset_request(EXTI10);
    beacon_angles_update_timestamp(&laser_two, C, os_timestamp_get());
}

