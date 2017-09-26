// driver for ARM PrimeCell UART (PL011)
#include "types.h"
#include "defs.h"
#include "param.h"
#include "arm.h"
#include "memlayout.h"

//static volatile uint *uart_base;
void isr_uart (struct trapframe *tf, int idx);

// ***************************************************************************

// Loop <delay> times
inline void udelay(unsigned int count)
{
  asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
     : "=r"(count): [count]"0"(count) : "cc");
}

// need to change the code to handle kernelbase relocation
/*
void uart_init ( void )
{
  unsigned int ra;

  // enable Mini UART
  write32(AUX_ENABLES,1);
  
  // Mini UART Interrupt
  // bit0: 0/1 = Disable/Enable transmit interrupt
  // bit1: 0/1 = Disable/Enable receive interrupt
  // disable receive & transmit UART interrupt
  write32(AUX_MU_IER_REG,0);
  
  // Mini UART Extra Control
  // bit0: 0/1 = mini UART receiver Disable/Enable
  // bit1: 0/1 = mini UART transmitter Disable/Enable
  // disable UART receive & transmit
  write32(AUX_MU_CNTL_REG,0);
  
  // Mini UART Line Control
  // bit0: 0=7bit mode | 1=8bit mode # bit1: unk
  write32(AUX_MU_LCR_REG,3);

  // Mini UART Modem Control
  // bit1: 0=UART1_RTS line is high | 1=UART1_RTS line is low
  write32(AUX_MU_MCR_REG,0);
  
  // Mini UART Interrupt
  // disable receive & transmit interrupt
  write32(AUX_MU_IER_REG,0);

  // Mini UART Interrupt Identify
  // bit0:R 0/1 Interrupt pending / no interrupt pending
  // bit1-2: 2+4 = Clear receive FIFO + Clear transmit FIFO
  write32(AUX_MU_IIR_REG,0xC6);
  
  // Mini UART Baudrate
  // bit0-15: baud rate counter
  write32(AUX_MU_BAUD_REG,270);

  // Setup the GPIO pin 14 && 15
  ra=read32(GPFSEL1);
  ra&=~(7<<12); //gpio14
  ra|=2<<12;    //alt5
  ra&=~(7<<15); //gpio15
  ra|=2<<15;    //alt5
  write32(GPFSEL1,ra);
  
  // Disable pull up/down for all GPIO pins & delay for 150 cycles
  write32(GPPUD,0);
  xdelay(150);
  //for(ra=0;ra<150;ra++) dummy(ra);
  
  // Disable pull up/down for pin 14,15 & delay for 150 cycles
  write32(GPPUDCLK0,(1<<14)|(1<<15));
  xdelay(150);
  //for(ra=0;ra<150;ra++) dummy(ra);
  
  // Write 0 to GPPUDCLK0 to make it take effect.
  write32(GPPUDCLK0,0);

  // Mini UART Extra Control
  // enable UART receive & transmit
  write32(AUX_MU_CNTL_REG,3);
  
}
*/

//******************************************************************************
void uartputc ( int byte )
{
  while(1) {
    if(read32(AUX_MU_LSR_REG+KERNBASE)&0x20) break;
  }
  write32(AUX_MU_IO_REG+KERNBASE, byte);
}

void uart_puts(const char *s)
{
  while (*s) {
    if (*s == '\n')
      uartputc('\r');
    uartputc(*s++);
  }
}

int uartgetc ()
{
  //while(1) {
  //  if(read32(AUX_MU_LSR_REG+KERNBASE)&0x01) break;
  //}
  //return(read32(AUX_MU_IO_REG+KERNBASE)&0xFF);
  if(read32(AUX_MU_LSR_REG+KERNBASE)&0x01) {
    return(read32(AUX_MU_IO_REG+KERNBASE)&0xFF);
  } else {
    return -1;
  }
}

void print_hex(uint val) {
    char digit[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    char number[8] = {'0','0','0','0','0','0','0','0'};
    uint base = 16;
    int i = 7;
    uartputc('0');
    uartputc('x');

    while(val > 0) {
        number[i--] = digit[val % base];
        val /= base;

    }
    for(i=0;i<8;++i) {
        uartputc(number[i]);
    }
    uartputc('\r');
    uartputc('\n');
}

void hexstrings ( unsigned int d )
{
  unsigned int rb;
  unsigned int rc;

  rb=32;
  while(1)
  {
    rb-=4;
    rc=(d>>rb)&0xF;
    if(rc>9) rc+=0x37; else rc+=0x30;
    uartputc(rc);
    if(rb==0) break;
  }
  uartputc(0x20);
}

void hexstring ( unsigned int d )
{
  hexstrings(d);
  uartputc(0x0D);
  uartputc(0x0A);
}

//***********************************************************************

// enable the receive (interrupt) for uart (after PIC has initialized)
void uart_enable_rx ()
{
    //uart_base[UART_IMSC] = UART_RXI;
    
    // Mini UART enable_interrupts
    write32(AUX_MU_IER_REG+KERNBASE, 0x1);
    write32(IRQ_ENABLE1+KERNBASE, (1 << 29)); // enable the miniuart through Aux

    pic_enable(PIC_UART0, isr_uart);
}


void isr_uart (struct trapframe *tf, int idx)
{
    //if (uart_base[UART_MIS] & UART_RXI) {
        consoleintr(uartgetc);
    //}

    // clear the interrupt
    //uart_base[UART_ICR] = UART_RXI | UART_TXI;
}
