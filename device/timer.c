// ARM dual-timer module support (SP804)
#include "types.h"
#include "param.h"
#include "arm.h"
#include "mmu.h"
#include "defs.h"
#include "memlayout.h"
#include "spinlock.h"

#include "timer.h"

#define TIMER_LOAD    0
#define TIMER_CURVAL  1
#define TIMER_CONTROL 2
#define TIMER_INTCLR  3
#define TIMER_RAWIRQ  4
#define TIMER_MIS     5

void isr_timer (struct trapframe *tp, int irq_idx);

struct spinlock tickslock;
uint ticks;

//#define TIMER0 (0x3F00B400)

// acknowledge the timer, write any value to TIMER_INTCLR should do
static void ack_timer() {
    volatile uint *timer0 = P2V(TIMER0);
    timer0[TIMER_INTCLR] = 1;
    //cprintf ("clear %s", "\n" );
}

// initialize the timer: perodical and interrupt based
void timer_init() {
    volatile uint *timer0 = P2V(TIMER0);

    initlock(&tickslock, "time");

    // Setup the system timer interrupt 
    // Timer frequency = Clk/256 // 0x400 
    //timer0[TIMER_LOAD] = 0x2000;
    timer0[TIMER_LOAD] = 0x1000;
    timer0[TIMER_CONTROL] =
        RPI_ARMTIMER_CTRL_23BIT |
        RPI_ARMTIMER_CTRL_ENABLE |
        RPI_ARMTIMER_CTRL_INT_ENABLE |
        RPI_ARMTIMER_CTRL_PRESCALE_256;
    pic_enable(PIC_TIMER0, isr_timer);
    
    //icount=0; // xsi line
}

// interrupt service routine for the timer
void isr_timer (struct trapframe *tp, int irq_idx)
{
    acquire(&tickslock);
    ticks++;
    wakeup(&ticks);
    release(&tickslock);
    ack_timer();
}
