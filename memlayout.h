#ifndef MEMLAYOUT_H
#define MEMLAYOUT_H
// Memory layout

// Key addresses for address space layout (see kmap in vm.c for layout)
#define KERNBASE  		0x80000000         // First kernel virtual address 
						// P:0x80000000  (PHY_START) => V:0xc0000000
// HCLIN added for RAM's PA starts from non-zero 
// use this cmd to check and add PHY_START : grep P2V * -R
#define PHY_START		0x00000000

// #define EXTMEM	  		0x20000
// #define KERNLINK  		(KERNBASE+EXTMEM)  // Address where kernel is linked

// we first map 1MB low memory containing kernel code.
#define INIT_KERN_SZ	0x100000
#define INIT_KERNMAP 	(INIT_KERN_SZ + PHY_START)
// 1MB = 0x10 0000
// 0x100000

// versatile_pb.h
#define PHYSTOP         (0x08000000 + PHY_START) // define 128M
//#define PHYSTOP         (0x04000000 + PHY_START) // define 64M
//#define PHYSTOP         (0x02000000 + PHY_START) // define 32M
//#define PHYSTOP         (0x01000000 + PHY_START) // define 16M
//#define PHYSTOP         (0x00800000 + PHY_START) // define 8M
//#define PHYSTOP         (0x00400000 + PHY_START) // define 4M
//#define PHYSTOP         (0x00200000 + PHY_START) // define 2M
//#define PHYSTOP         (0x00100000 + PHY_START) // define 1M

#define DEVBASE1        0x3f000000
//#define DEVBASE2        0x02c00000
#define DEV_MEM_SZ      0x400000
#define VEC_TBL         0xFFFF0000

#ifndef __ASSEMBLER__

// certanly there are unused itens here. need a cleanup on raspi segments

//#include "types.h" // raspi

static inline uint v2p(void *a) { return ((uint) (a))  - KERNBASE; }
static inline void *p2v(uint a) { return (void *) ((a) + KERNBASE); }

// bof raspi
/*
enum
{
    // The GPIO registers base address.
    //GPIO_BASE = 0x20200000,
    GPIO_BASE = 0x3f200000,

    // The offsets for reach register.

    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = (GPIO_BASE + 0x94),

    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = (GPIO_BASE + 0x98),

    // The base address for UART.
    //UART0_BASE = 0x20201000,
    UART0_BASE = 0x3f201000,

    // The offsets for reach register for the UART.
    UART0_DR     = (UART0_BASE + 0x00),
    UART0_RSRECR = (UART0_BASE + 0x04),
    UART0_FR     = (UART0_BASE + 0x18),
    UART0_ILPR   = (UART0_BASE + 0x20),
    UART0_IBRD   = (UART0_BASE + 0x24),
    UART0_FBRD   = (UART0_BASE + 0x28),
    UART0_LCRH   = (UART0_BASE + 0x2C),
    UART0_CR     = (UART0_BASE + 0x30),
    UART0_IFLS   = (UART0_BASE + 0x34),
    UART0_IMSC   = (UART0_BASE + 0x38),
    UART0_RIS    = (UART0_BASE + 0x3C),
    UART0_MIS    = (UART0_BASE + 0x40),
    UART0_ICR    = (UART0_BASE + 0x44),
    UART0_DMACR  = (UART0_BASE + 0x48),
    UART0_ITCR   = (UART0_BASE + 0x80),
    UART0_ITIP   = (UART0_BASE + 0x84),
    UART0_ITOP   = (UART0_BASE + 0x88),
    UART0_TDR    = (UART0_BASE + 0x8C),
};

static inline void mmio_write(uint32 reg, uint32 data) {
    asm volatile("str %[data], [%[reg]]" : : [reg]"r"(reg), [data]"r"(data));
}

static inline uint32 mmio_read(uint32 reg) {
    uint32 data;
    asm volatile("ldr %[data], [%[reg]]" : [data]"=r"(data) : [reg]"r"(reg));
    return data;
}
// Loop <delay> times in a way that the compiler won't optimize away. 
static inline void delay(int count) {
    asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
         : : [count]"r"(count) : "cc");
}
*/
// eof raspi

#endif

#define V2P(a) (((uint) (a)) - KERNBASE)
#define P2V(a) (((void *) (a)) + KERNBASE)

#define V2P_WO(x) ((x) - KERNBASE)    // same as V2P, but without casts
#define P2V_WO(x) ((x) + KERNBASE)    // same as V2P, but without casts

// bof raspi
#ifndef RPI2_H
#define RPI2_H

#ifndef __ASSEMBLER__

#include "types.h"

//#define VIC_BASE (0x3F00B200)

//#define RPI_INTERRUPT_CONTROLLER_BASE   (0x3F00B200+KERNBASE)

/** @brief Bits in the Enable_Basic_IRQs register to enable various interrupts.
 *     See the BCM2835 ARM Peripherals manual, section 7.5 */
//#define RPI_BASIC_ARM_TIMER_IRQ         (0)

/*
#define RPI_BASIC_ARM_MAILBOX_IRQ       (1)
#define RPI_BASIC_ARM_DOORBELL_0_IRQ    (2)
#define RPI_BASIC_ARM_DOORBELL_1_IRQ    (3)
#define RPI_BASIC_GPU_0_HALTED_IRQ      (4)
#define RPI_BASIC_GPU_1_HALTED_IRQ      (5)
#define RPI_BASIC_ACCESS_ERROR_1_IRQ    (6)
#define RPI_BASIC_ACCESS_ERROR_0_IRQ    (7)
*/

/** @brief The interrupt controller memory mapped register set */
/*
typedef struct {
    volatile uint32 IRQ_basic_pending;
    volatile uint32 IRQ_pending_1;
    volatile uint32 IRQ_pending_2;
    volatile uint32 FIQ_control;
    volatile uint32 Enable_IRQs_1;
    volatile uint32 Enable_IRQs_2;
    volatile uint32 Enable_Basic_IRQs;
    volatile uint32 Disable_IRQs_1;
    volatile uint32 Disable_IRQs_2;
    volatile uint32 Disable_Basic_IRQs;

} rpi_irq_controller_t;

extern rpi_irq_controller_t* RPI_GetIrqController( void  );
*/

#endif

#endif
// eof raspi

// bof raspi
#ifndef RPI3_H
#define RPI3_H
#define IO_BASE 0x3F000000

// BCM2835 ARM peripherals - Page 89
#define GPFSEL1         (IO_BASE + 0x00200004) // GPIO Function Select 1
#define GPPUD           (IO_BASE + 0x00200094) // GPIO Pin Pull-up/down Enable
#define GPPUDCLK0       (IO_BASE + 0x00200098) // GPIO Pin Pull-up/down Enable Clock 0

// BCM2835 ARM peripherals - Page 8-20
#define AUX_ENABLES     (IO_BASE + 0x00215004) // Auxiliary enables
#define AUX_MU_IO_REG   (IO_BASE + 0x00215040) // Mini Uart I/O Data
#define AUX_MU_IER_REG  (IO_BASE + 0x00215044) // Mini Uart Interrupt Enable
#define AUX_MU_IIR_REG  (IO_BASE + 0x00215048) // Mini Uart Interrupt Identify 
#define AUX_MU_LCR_REG  (IO_BASE + 0x0021504C) // Mini Uart Line Control
#define AUX_MU_MCR_REG  (IO_BASE + 0x00215050) // Mini Uart Modem Control
#define AUX_MU_LSR_REG  (IO_BASE + 0x00215054) // Mini Uart Line Status
//#define AUX_MU_MSR_REG  (IO_BASE + 0x00215058) // Mini Uart Modem Status
//#define AUX_MU_SCRATCH  (IO_BASE + 0x0021505C) // Mini Uart Scratch
#define AUX_MU_CNTL_REG (IO_BASE + 0x00215060) // Mini Uart Extra Control
//#define AUX_MU_STAT_REG (IO_BASE + 0x00215064) // Mini Uart Extra Status
#define AUX_MU_BAUD_REG (IO_BASE + 0x00215068) // Mini Uart Baudrate

// memory mapped i/o access macros
#define write32(addr, v)      (*((volatile unsigned long  *)(addr)) = (unsigned long)(v))
#define read32(addr)          (*((volatile unsigned long  *)(addr)))
#endif
// eof raspi

// bof raspi
#define IRQ_PENDING_BASIC (IO_BASE + 0xB200)
#define IRQ_PENDING1      (IO_BASE + 0xB204)
#define IRQ_ENABLE1       (IO_BASE + 0xB210)

#define VIC_BASE (0x3F00B200)

#define PIC_TIMER0         0
//#define RPI_INTERRUPT_CONTROLLER_BASE   (0x3F00B200+KERNBASE)

#define PIC_UART0           29
// eof raspi

#endif

/*

#define IRQ_PENDING_BASIC (PBASE+0xB200)
#define IRQ_PENDING1      (PBASE+0xB204)

  if(read32(IRQ_PENDING1)) {
    miniuartintr();
  }
  
  if(read32(IRQ_PENDING_BASIC)) {
    icount++;
    if(icount&1) {
      SetActLEDState(1);
      //uart_putc ('-');
    } else {
      SetActLEDState(0);
      //uart_putc ('.');
    }
    write32(ARM_TIMER_CLI,0);
  }
}

*/