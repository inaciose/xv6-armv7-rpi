// BSP support routine
#include "types.h"
#include "defs.h"
#include "param.h"
#include "arm.h"
#include "proc.h"
#include "memlayout.h"
#include "mmu.h"

extern void* end;

struct cpu	cpus[NCPU];
struct cpu	*cpu;

#define MB (1024*1024)

void kmain (void)
{
    uint vectbl;

    cpu = &cpus[0];

    _puts("enter kmain\r\n");
    //todo later
    //uart_init (P2V(UART0));

    // interrrupt vector table is in the middle of first 1MB. We use the left
    // over for page tables
    vectbl = P2V_WO ((VEC_TBL & PDE_MASK) + PHY_START);
    
    init_vmm ();
    _puts("init_vmm kmain\r\n");
    kpt_freerange (align_up(&end, PT_SZ), vectbl);
    _puts("kpt_freerange kmain\r\n");
    kpt_freerange (vectbl + PT_SZ, P2V_WO(INIT_KERNMAP));
    _puts("kpt_freerange kmain\r\n");
    paging_init (INIT_KERNMAP, PHYSTOP);
    _puts("paging_init kmain\r\n");
    
    kmem_init ();
    _puts("kmem_init kmain\r\n");
    kmem_init2(P2V(INIT_KERNMAP), P2V(PHYSTOP));
    _puts("kmem_init2 kmain\r\n");
    
    trap_init ();				// vector table and stacks for models
    _puts("enter kmain\r\n");
    
    gic_init(P2V(VIC_BASE));    // arm v2 gic init
    //pic_init(P2V(VIC_BASE));
    _puts("gic_init kmain\r\n");
    
    uart_enable_rx ();			// interrupt for uart
    _puts("uart_enable_rx kmain\r\n");
    
    consoleinit ();				// console
    _puts("consoleinit kmain\r\n");

    pinit ();					// process (locks)
    _puts("pinit kmain\r\n");
    binit ();					// buffer cache
    _puts("enter kmain\r\n");
    fileinit ();				// file table
    _puts("fileinit kmain\r\n");
    iinit ();					// inode cache
    _puts("iinit kmain\r\n");
    ideinit ();					// ide (memory block device)
    _puts("ideinit kmain\r\n");
    //timer_init (HZ);			// the timer (ticker)
    timer_init ();      // the timer (ticker)
    _puts("timer_init kmain\r\n");
    sti ();
    _puts("sti kmain\r\n");
    userinit();					// first user process
    _puts("userinit kmain\r\n");
    scheduler();				// start running processes
}
