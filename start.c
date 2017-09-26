// initialize section
#include "types.h"
#include "param.h"
#include "arm.h"
#include "mmu.h"
#include "defs.h"
#include "memlayout.h"

// Loop <delay> times
inline void xdelay(unsigned int count)
{
  asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
     : "=r"(count): [count]"0"(count) : "cc");
}

void _uart_init ( void )
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

void _uart_putc ( unsigned int c )
{
  while(1) {
    if(read32(AUX_MU_LSR_REG)&0x20) break;
  }
  write32(AUX_MU_IO_REG,c);
}

void _puts(const char *s)
{
  while (*s) {
    if (*s == '\n')
      _uart_putc('\r');
    _uart_putc(*s++);
  }
}

//**********************


void _hexstrings ( unsigned int d )
{
  unsigned int rb;
  unsigned int rc;

  rb=32;
  while(1)
  {
    rb-=4;
    rc=(d>>rb)&0xF;
    if(rc>9) rc+=0x37; else rc+=0x30;
    _uart_putc(rc);
    if(rb==0) break;
  }
  _uart_putc(0x20);
}

void _hexstring ( unsigned int d )
{
  _hexstrings(d);
  _uart_putc(0x0D);
  _uart_putc(0x0A);
}

//**********************

// kernel page table, reserved in the kernel.ld
extern uint32 _kernel_pgtbl;
extern uint32 _user_pgtbl;

uint32 *kernel_pgtbl = &_kernel_pgtbl;
uint32 *user_pgtbl = &_user_pgtbl;

#define PDE_SHIFT 20

uint32 get_pde (uint32 virt)
{
    virt >>= PDE_SHIFT;
    return kernel_pgtbl[virt];
}

// setup the boot page table: dev_mem whether it is device memory
void set_bootpgtbl (uint32 virt, uint32 phy, uint len, int dev_mem )
{
    uint32	pde;
    int		idx;

    // convert all the parameters to indexes
    virt >>= PDE_SHIFT;
    phy  >>= PDE_SHIFT;
    len  >>= PDE_SHIFT;

    for (idx = 0; idx < len; idx++) {
        pde = (phy << PDE_SHIFT);

        if (!dev_mem) {
            // normal memory, make it kernel-only, cachable, bufferable
            pde |= (AP_KO << 10) | PE_CACHE | PE_BUF | KPDE_TYPE;
        } else {
            // device memory, make it non-cachable and non-bufferable
            pde |= (AP_KO << 10) | KPDE_TYPE;
        }

        // use different page table for user/kernel space
        if (virt < NUM_UPDE) {
            user_pgtbl[virt] = pde;
        } else {
            kernel_pgtbl[virt] = pde;
        }

        virt++;
        phy++;
    }
}

static void _flush_all (void)
{
    uint val = 0;

    // flush all TLB
    asm("MCR p15, 0, %[r], c8, c7, 0" : :[r]"r" (val):);

    // invalid entire data and instruction cache
    // asm ("MCR p15,0,%[r],c7,c5,0": :[r]"r" (val):);
    // asm ("MCR p15,0,%[r],c7,c6,0": :[r]"r" (val):);
}

void load_pgtlb (uint32* kern_pgtbl, uint32* user_pgtbl)
{
    uint	ret;
    char	arch;
    uint	val;

    // read the main id register to make sure we are running on ARMv6
    asm("MRC p15, 0, %[r], c0, c0, 0": [r]"=r" (ret)::);

    if (ret >> 24 == 0x41) {
        //_puts ("ARM-based CPU\n");
    }

    arch = (ret >> 16) & 0x0F;

    if ((arch != 7) && (arch != 0xF)) {
        _puts ("need AARM v6 or higher\n");
    }

    // we need to check the cache/tlb etc., but let's skip it for now

    // set domain access control: all domain will be checked for permission
    val = 0x55555555;
    asm("MCR p15, 0, %[v], c3, c0, 0": :[v]"r" (val):);

    // set the page table base registers. We use two page tables: TTBR0
    // for user space and TTBR1 for kernel space
    val = 32 - UADDR_BITS;
    asm("MCR p15, 0, %[v], c2, c0, 2": :[v]"r" (val):);

    // set the kernel page table
    val = (uint)kernel_pgtbl | 0x00;
    asm("MCR p15, 0, %[v], c2, c0, 1": :[v]"r" (val):);

    // set the user page table
    val = (uint)user_pgtbl | 0x00;
    asm("MCR p15, 0, %[v], c2, c0, 0": :[v]"r" (val):);

    // ok, enable paging using read/modify/write
    asm("MRC p15, 0, %[r], c1, c0, 0": [r]"=r" (val)::);

    val |= 0x80300D; // enable MMU, cache, write buffer, high vector tbl,
                     // disable subpage
    asm("MCR p15, 0, %[r], c1, c0, 0": :[r]"r" (val):);

    _flush_all();
}

extern void * edata_entry;
extern void * svc_stktop;
extern void kmain (void);
extern void jump_stack (void);

extern void * edata;
extern void * end;

// clear the BSS section for the main kernel, see kernel.ld
void clear_bss (void)
{
    memset(&edata, 0x00, (uint)&end-(uint)&edata);
}

void start (void)
{
	  uint32  vectbl;
    
    _uart_init();
    _puts("starting xv6 for ARM on RPI...\n");

    /*
    set_bootpgtbl(0, 0, INIT_KERNMAP, 0);
    set_bootpgtbl(KERNBASE, 0, INIT_KERNMAP, 0);
    //         we are mapping 0xFFF0 0000 to    0x0000 0000
    // so when we want to use 0xFFFF 0000 means 0x000F 0000
    set_bootpgtbl(VEC_TBL, 0, 1 << PDE_SHIFT, 0);
    set_bootpgtbl(KERNBASE+DEVBASE, DEVBASE, DEV_MEM_SZ, 1);  // DEVICE MAP
    */
    
    _hexstrings(PHY_START);
    _hexstrings(INIT_KERN_SZ);
    _hexstring(KERNBASE);
    _hexstring(DEVBASE1);
    
    // double map the low memory, required to enable paging
    // we do not map all the physical memory
    set_bootpgtbl(PHY_START, PHY_START, INIT_KERN_SZ, 0);
    set_bootpgtbl(KERNBASE+PHY_START, PHY_START, INIT_KERN_SZ, 0);

    // vector table is in the middle of first 1MB (0xF000)
    vectbl = P2V_WO ((VEC_TBL & PDE_MASK) + PHY_START);

    if (vectbl <= (uint)&end) {
        _puts("error: vector table overlap and cprintf() is 0x00000\n");
        cprintf ("error: vector table overlaps kernel\n");
    }
    // V, P, len, is_mem
    set_bootpgtbl(VEC_TBL, PHY_START, 1 << PDE_SHIFT, 0); // V, P, SZ, ISDEV
    set_bootpgtbl(DEVBASE1, DEVBASE1, DEV_MEM_SZ, 1); // V, P, SZ, ISDEV: add to prevent crash on _puts
    set_bootpgtbl(KERNBASE+DEVBASE1, DEVBASE1, DEV_MEM_SZ, 1); // V, P, SZ, ISDEV
    //set_bootpgtbl(KERNBASE+DEVBASE2, DEVBASE2, DEV_MEM_SZ, 1); // V, P, SZ, ISDEV

    load_pgtlb (kernel_pgtbl, user_pgtbl);
    jump_stack ();
    
    // We can now call normal kernel functions at high memory
    clear_bss ();
    
    kmain ();
}
