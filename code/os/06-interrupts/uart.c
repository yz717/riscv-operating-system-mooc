#include "os.h"

/*
 * The UART control registers are memory-mapped at address UART0. 
 * This macro returns the address of one of the registers.
 */
#define UART_REG(reg) ((volatile uint8_t *)(UART0 + reg))

/*
 * Reference
 * [1]: TECHNICAL DATA ON 16550, http://byterunner.com/16550.html
 */

/*
 * UART control registers map. see [1] "PROGRAMMING TABLE"
 * note some are reused by multiple functions
 * 0 (write mode): THR/DLL
 * 1 (write mode): IER/DLM
 */
#define RHR 0	// Receive Holding Register (read mode)
#define THR 0	// Transmit Holding Register (write mode)
#define DLL 0	// LSB of Divisor Latch (write mode)
#define IER 1	// Interrupt Enable Register (write mode)
#define DLM 1	// MSB of Divisor Latch (write mode)
#define FCR 2	// FIFO Control Register (write mode)
#define ISR 2	// Interrupt Status Register (read mode)
#define LCR 3	// Line Control Register
#define MCR 4	// Modem Control Register
#define LSR 5	// Line Status Register
#define MSR 6	// Modem Status Register
#define SPR 7	// ScratchPad Register

/*
 * POWER UP DEFAULTS
 * IER = 0: TX/RX holding register interrupts are both disabled
 * ISR = 1: no interrupt penting
 * LCR = 0
 * MCR = 0
 * LSR = 60 HEX
 * MSR = BITS 0-3 = 0, BITS 4-7 = inputs
 * FCR = 0
 * TX = High
 * OP1 = High
 * OP2 = High
 * RTS = High
 * DTR = High
 * RXRDY = High
 * TXRDY = Low
 * INT = Low
 */

/*
 * LINE STATUS REGISTER (LSR)
 * LSR BIT 0:
 * 0 = no data in receive holding register or FIFO.
 * 1 = data has been receive and saved in the receive holding register or FIFO.
 * ......
 * LSR BIT 5:
 * 0 = transmit holding register is full. 16550 will not accept any data for transmission.
 * 1 = transmitter hold register (or FIFO) is empty. CPU can load the next character.
 * ......
 */
#define LSR_RX_READY (1 << 0)
#define LSR_TX_IDLE  (1 << 5)


#define MCR_OUT2      (1 << 3)

#define IER_RX_INT_ENABLE (1 << 0)
#define IER_TX_INT_ENABLE (1 << 1)

#define uart_read_reg(reg) (*(UART_REG(reg)))
#define uart_write_reg(reg, v) (*(UART_REG(reg)) = (v))

/*
 * Simple TX ring buffer for interrupt driven transmission.
 */
#define UART_TX_BUF_SIZE 128
static char tx_buf[UART_TX_BUF_SIZE];
static volatile uint32_t tx_head;
static volatile uint32_t tx_tail;

static inline int tx_buf_empty(void)
{
        return tx_head == tx_tail;
}

static inline int tx_buf_full(void)
{
        return ((tx_head + 1) % UART_TX_BUF_SIZE) == tx_tail;
}

static inline reg_t uart_intr_save(void)
{
        reg_t mstatus = r_mstatus();
        w_mstatus(mstatus & ~MSTATUS_MIE);
        return mstatus;
}

static inline void uart_intr_restore(reg_t mstatus)
{
        w_mstatus(mstatus);
}

static void uart_update_tx_irq_locked(void)
{
        uint8_t ier = uart_read_reg(IER);
        if (!tx_buf_empty()) {
                uart_write_reg(IER, ier | IER_TX_INT_ENABLE);
        } else {
                uart_write_reg(IER, ier & ~IER_TX_INT_ENABLE);
        }
}

static void uart_try_fill_tx_locked(void)
{
        while (!tx_buf_empty() && (uart_read_reg(LSR) & LSR_TX_IDLE)) {
                char ch = tx_buf[tx_tail];
                tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
                uart_write_reg(THR, ch);
        }
        uart_update_tx_irq_locked();
}

static void uart_flush_locked(void)
{
        while (!tx_buf_empty()) {
                while ((uart_read_reg(LSR) & LSR_TX_IDLE) == 0)
                        ;
                uart_try_fill_tx_locked();
        }
}

void uart_init()
{
        /* disable interrupts. */
        uart_write_reg(IER, 0x00);

	/*
	 * Setting baud rate. Just a demo here if we care about the divisor,
	 * but for our purpose [QEMU-virt], this doesn't really do anything.
	 *
	 * Notice that the divisor register DLL (divisor latch least) and DLM (divisor
	 * latch most) have the same base address as the receiver/transmitter and the
	 * interrupt enable register. To change what the base address points to, we
	 * open the "divisor latch" by writing 1 into the Divisor Latch Access Bit
	 * (DLAB), which is bit index 7 of the Line Control Register (LCR).
	 *
	 * Regarding the baud rate value, see [1] "BAUD RATE GENERATOR PROGRAMMING TABLE".
	 * We use 38.4K when 1.8432 MHZ crystal, so the corresponding value is 3.
	 * And due to the divisor register is two bytes (16 bits), so we need to
	 * split the value of 3(0x0003) into two bytes, DLL stores the low byte,
	 * DLM stores the high byte.
	 */
	uint8_t lcr = uart_read_reg(LCR);
	uart_write_reg(LCR, lcr | (1 << 7));
	uart_write_reg(DLL, 0x03);
	uart_write_reg(DLM, 0x00);

	/*
	 * Continue setting the asynchronous data communication format.
	 * - number of the word length: 8 bits
	 * - number of stop bits：1 bit when word length is 8 bits
	 * - no parity
	 * - no break control
	 * - disabled baud latch
	 */
	lcr = 0;
	uart_write_reg(LCR, lcr | (3 << 0));

        /* initialize TX buffer state */
        tx_head = tx_tail = 0;
}

int uart_putc(char ch)
{
        while (1) {
                reg_t mstatus = uart_intr_save();
                if (!tx_buf_full()) {
                        tx_buf[tx_head] = ch;
                        tx_head = (tx_head + 1) % UART_TX_BUF_SIZE;
                        uart_try_fill_tx_locked();

                        if ((mstatus & MSTATUS_MIE) == 0) {
                                uart_flush_locked();
                        }

                        uart_intr_restore(mstatus);
                        return (int)ch;
                }

                uart_intr_restore(mstatus);
        }
}

void uart_puts(char *s)
{
	while (*s) {
		uart_putc(*s++);
	}
}

int uart_rx_ready(void)
{
        return (uart_read_reg(LSR) & LSR_RX_READY) != 0;
}

int uart_getc(void)
{
        while (0 == (uart_read_reg(LSR) & LSR_RX_READY))
                ;
        return uart_read_reg(RHR);
}

/*
 * handle a uart interrupt, called from trap.c.
 */
void uart_isr(void)
{
        uart_try_fill_tx_locked();
}
