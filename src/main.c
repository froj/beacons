
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <stdint.h>
#include <stdio.h>


#include <platform-abstraction/threading.h>
#include <platform-abstraction/semaphore.h>
#include <platform-abstraction/timestamp.h>

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

    // EXTI Line 7
    exti_enable_request(EXTI7);
    // Trigger on falling edge
    exti_set_trigger(EXTI7, EXTI_TRIGGER_RISING);
    // Port C
    exti_select_source(EXTI7, GPIOC);

    // Enable interrupt
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
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

semaphore_t mysem;

os_thread_t mythread;
THREAD_STACK mystack[1024];

void mythread_main(void *context)
{
    (void) context;
    uint32_t mytimestamp = 0;
    uint32_t mynewtimestamp;
    uint32_t t_diff;

    printf("My Thread\n");

    os_semaphore_init(&mysem, 0);

    while (1) {
        os_semaphore_wait(&mysem);

        mynewtimestamp = os_timestamp_get();

        if((t_diff = mynewtimestamp - mytimestamp) > 10000){
            mytimestamp = mynewtimestamp;
            // toggle user-LED
            gpio_toggle(GPIOB, GPIO13);
            printf("Time: %d\n", (int)t_diff);
        }
    }
}

int main(void)
{
    rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);

    fpu_config();

    uart2_init();
    exti_irq_init();

    // User LED
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);


    os_init();

    os_thread_create(&mythread, mythread_main, mystack, sizeof(mystack), "My Thread", 0, NULL);

    os_run();

    while (1);
}


/*
void exti0_isr(void)
{
}

void exti1_isr(void)
{

}
void exti2_tsc_isr(void)
{

}

void exti3_isr(void)
{

}

void exti4_isr(void)
{

}
*/

void exti9_5_isr(void)
{
    // Clear interrupt request flag
    exti_reset_request(EXTI7);
    os_semaphore_signal(&mysem);
}

/*
void exti15_10_isr(void)
{
}
*/
