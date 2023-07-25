#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <stm32h753xx.h>

#define FLASH_IRQ_NUM     4

static void flash_error_isr(void *arg) {
    printk("ISR triggered: %u ms\n", k_uptime_get_32());
    printk("FLASH_SR1: 0x%08x\n", FLASH->SR1);
    printk("FLASH_SR2: 0x%08x\n", FLASH->SR2);

    FLASH->CCR1 = FLASH_FLAG_ALL_BANK1;
    FLASH->CCR2 = FLASH_FLAG_ALL_BANK2;
}

static void enable_flash_error_interrupt(void) {
    FLASH->CR1 |= FLASH_CR_RDSERRIE;
    FLASH->CR1 |= FLASH_CR_RDPERRIE;
    FLASH->CR2 |= FLASH_CR_RDSERRIE;
    FLASH->CR2 |= FLASH_CR_RDPERRIE;

    IRQ_CONNECT(FLASH_IRQ_NUM, 0, flash_error_isr, NULL, 0);
    irq_enable(FLASH_IRQ_NUM);
}

void main(void) {
    enable_flash_error_interrupt();

    while (1) {
        // error doesn't occur if sleep is longer than 1 tick
        k_sleep(K_TICKS(1));
    }
}
