//
// Board specific information for the VersatilePB board
//
#ifndef VERSATILEPB
#define VERSATILEPB

/*
// the VerstatilePB board can support up to 256MB memory.
// but we assume it has 128MB instead. During boot, the lower
// 64MB memory is mapped to the flash, needs to be remapped
// the the SDRAM. We skip this for QEMU
#define PHYSTOP         (0x08000000 + PHY_START)
#define BSP_MEMREMAP    0x04000000

#define DEVBASE1        0x1c000000
#define DEVBASE2        0x2c000000
#define DEV_MEM_SZ      0x01000000
#define VEC_TBL         0xFFFF0000


#define STACK_FILL      0xdeadbeef

#define UART0           0x1c090000
#define UART_CLK        24000000    // Clock rate for UART

#define TIMER0          0x1c110000
#define TIMER1          0x1c120000
#define CLK_HZ          1000000     // the clock is 1MHZ
*/
// HCLIN reference vexpress.c:
// sysbus_create_simple("pl011", map[VE_UART0], pic[5]);
//
// a GIC is registered in vexpress.c
//     * 0x2c000000 A15MPCore private memory region (GIC) *
//    init_cpus(cpu_model, "a15mpcore_priv", 0x2c000000, pic);
// in init_cpus, the GIC accept 64 gpio input and one output to ARM_IRQ
// 
// 			dev // GIC device
//			busdev = SYS_BUS_DEVICE(dev);
//		    for (n = 0; n < 64; n++) {
//              pic[n] = qdev_get_gpio_in(dev, n);
//    		}
//			/* Connect the CPUs to the GIC */
//    		for (n = 0; n < smp_cpus; n++) {
//        		DeviceState *cpudev = DEVICE(qemu_get_cpu(n));
//        		sysbus_connect_irq(busdev, n, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
//    		}
/*
#define VIC_BASE        0x2c000000
#define PIC_TIMER01     2
#define PIC_TIMER23     3
#define PIC_UART0       5
#define PIC_GRAPHIC     19
*/
#endif
