#include "types.h"
#include "defs.h"
#include "param.h"
#include "arm.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"

extern struct proc *proc;

static volatile uint *vic_base;

#define VIC_IRQPENDING_ARM 0
//#define VIC_IRQPENDING 0
#define VIC_IRQPENDING_GPU0 1
#define VIC_IRQPENDING_GPU1 2
#define VIC_INTENABLE  6
#define VIC_INTCLEAR   9

#define NUM_INTSRC 32
static ISR isrs[NUM_INTSRC];

static void default_isr(struct trapframe *tf, int n) {
    (void)tf;
    cprintf ("unhandled interrupt: %d\n", n);
    //uart_puts("unhandled interrupt: ");
    //print_hex(n);
}

void gic_init(void *base) {
    int i;

    vic_base = base;
    vic_base[VIC_INTCLEAR] = 0xFFFFFFFF;

    for(i=0;i<NUM_INTSRC;++i) {
        isrs[i] = default_isr;
    }
}

void pic_enable(int n, ISR isr) {
    if ((n<0) || (n>=NUM_INTSRC)) {
        panic("invalid interrupt source");
    }
    
    cprintf ("pic_enable: %d\n", n);
    
    isrs[n] = isr;
    vic_base[VIC_INTENABLE] = (1<<n);
}

void pic_disable(int n) {
    if ((n<0) || (n>=NUM_INTSRC)) {
        panic("invalid interrupt source");
    }
    vic_base[VIC_INTCLEAR] = (1<<n);
    isrs[n] = default_isr;
}

#define INT_REGS_BASE (0x3F00B200)

// dispatch the interrupt
void pic_dispatch (struct trapframe *tf) {
    //uint intstatus_arm;
    //uint intstatus_gpu0;
    //uint intstatus_gpu1;
    //int i;
    uint istimer;
    
    
    /*
#define IRQ_PENDING_BASIC (IO_BASE + 0xB200) // VIC_IRQPENDING_ARM
#define IRQ_PENDING1      (IO_BASE + 0xB204) // VIC_IRQPENDING_GPU0
#define IRQ_PENDING2      (IO_BASE + 0xB208) // VIC_IRQPENDING_GPU1
//
#define IRQ_ENABLE1       (IO_BASE + 0xB210)

#define VIC_BASE (0x3F00B200)
*/    
    
    // should print intstatus to debug
    //intstatus_arm = vic_base[VIC_IRQPENDING_ARM];
    //cprintf ("pic_dispatch ip->armpending: %d\n", intstatus_arm );
    
    //intstatus_gpu0 = vic_base[VIC_IRQPENDING_GPU0];
    //cprintf ("pic_dispatch ip->gpupending[0]: %d\n", intstatus_gpu0 );

    /*
    intstatus_gpu1 = vic_base[VIC_IRQPENDING_GPU1];
    cprintf ("pic_dispatch ip->gpupending[1]: %d\n", intstatus_gpu1 );
    */
    /*
    //while(intstatus_gpu0 || intstatus_gpu1 || intstatus_arm) {
      if((!intstatus_gpu0) & intstatus_arm) {
        cprintf ("timer %s", "\n" );  
        isrs[0](tf, 0);  
      }
    
      if(intstatus_gpu0 & (1 << 29)) {
        cprintf ("uart %s", "\n" );
        isrs[29](tf, 29);  
      }
    
    //}
    */
    
    /*
    intctrlregs *ip;
    
    ip = (intctrlregs *)INT_REGS_BASE;
    while(ip->gpupending[0] || ip->gpupending[1] || ip->armpending){
      if(ip->gpupending[0] & (1 << 3)) {
        cprintf ("timer %s", "\n" );  
        isrs[1](tf, 0);  
        //timer3intr();
      }
      if(ip->gpupending[0] & (1 << 29)) {
        cprintf ("uart %s", "\n" );
        isrs[29](tf, 29);  
        //miniuartintr();
      }
    }
    */
    
    
    //#define PBASE 0x3F000000
    //#define INT_REGS_BASE     (PBASE+0xB200)
    //#define IRQ_PENDING_BASIC (PBASE+0xB200)
    //#define IRQ_PENDING1      (PBASE+0xB204)
    
    //ARM_TIMER_CLI 0x3F00B40C
    istimer = 0;
    
    //while( vic_base[VIC_IRQPENDING_GPU0] || vic_base[VIC_IRQPENDING_GPU1] || vic_base[VIC_IRQPENDING_ARM]) {
      if(vic_base[VIC_IRQPENDING_ARM]) {
        //cprintf ("timer %s", "\n" );
        istimer = 1;  
        isrs[PIC_TIMER0](tf, PIC_TIMER0);  
      }
    
      if(vic_base[VIC_IRQPENDING_GPU0] & (1 << PIC_UART0)) {
        //cprintf ("uart %s", "\n" );
        isrs[PIC_UART0](tf, PIC_UART0);  
      }
    
    //}
    
    /*
    for(i=0;i<NUM_INTSRC;++i) {
        
        if (intstatus & (1<<i)) {
            cprintf (">: %x %x %x\n", intstatus, (1<<i), intstatus & (1<<i) );
            isrs[i](tf, i);
        }
    }
    */
    
    
  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  //
  // on xv6 x86
  //if(proc && proc->killed && (tf->cs&3) == DPL_USER)
  // 
  // on arm: (r14_svc == pc if SWI) 
  // - the proc in swi is in kernel space
  // - the proc not in swi is in user space
  // so: we need to compare tp->r14_svc with tp->pc
  // they need to be diferent to proc be in user space
  /*
  if(proc && proc->killed && (tf->r14_svc) != (tf->pc)) {
    exit();
  }
  */
  
  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(proc && proc->state == RUNNING && istimer) {
    yield();
  }
  
  // Check if the process has been killed since we yielded
  if(proc && proc->killed && (tf->r14_svc) != (tf->pc)) {
    exit();
  }
  
}

/*
 * dispatch the interrupt
 */
 /*
void pic_dispatch (struct trapframe *tp)
{
	int intid, intn;
	intid = gic_getack(); // iack 
	intn = intid - 32;
	
	isrs[intn](tp, intn);
	gic_eoi(intn);

  
}

*/