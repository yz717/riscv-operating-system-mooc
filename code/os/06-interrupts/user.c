#include "os.h"

#define DELAY 1000

void user_task0(void)
{
        uart_puts("Task 0: Created!\n");
        while (1) {
                uart_puts("Task 0: Running...\n");
                task_delay(DELAY);
                task_yield();
        }
}

void user_task1(void)
{
        uart_puts("Task 1: Created!\n");
        while (1) {
                uart_puts("Task 1: Running...\n");
                task_delay(DELAY);
                task_yield();
        }
}

void console_task(void)
{
        uart_puts("Console: type and press enter to echo.\n");
        while (1) {
                if (uart_rx_ready()) {
                        int ch = uart_getc();
                        if (ch == '\r' || ch == '\n') {
                                uart_putc('\r');
                                uart_putc('\n');
                        } else {
                                uart_putc((char)ch);
                        }
                } else {
                        task_yield();
                }
        }
}

/* NOTICE: DON'T LOOP INFINITELY IN main() */
void os_main(void)
{
        task_create(user_task0);
        task_create(user_task1);
        task_create(console_task);
}
