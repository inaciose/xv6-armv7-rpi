#ifndef TYPES_H
#define TYPES_H

typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned char  uchar;
typedef unsigned int uint32;
typedef unsigned short uint16;
typedef unsigned char uint8;

#ifndef NULL
#define NULL ((void*)0)
#endif


/* ARM interrupt control registers */
/*
typedef struct intctrlregs {
        uint  armpending;
        uint  gpupending[2];
        uint  fiqctrl;
        uint  gpuenable[2];
        uint  armenable;
        uint  gpudisable[2];
        uint  armdisable;
} intctrlregs;
*/
#endif