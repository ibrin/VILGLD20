/* START LIBRARY DESCRIPTION *********************************************
default.h
    Copyright (c) 1997, Z-World.

DESCRIPTION:	Default libraries to be used for the boards. The use
					statements do not necessarily compile anything, it is just
					an indication that functions in the used libraries may be
					called and linked.

END DESCRIPTION **********************************************************/

/*** BeginHeader */

#use "dc.hh"
#use "bios.lib"
#use "math.lib"
#use "xmem.lib"
#use "string.lib"
#use "stdio.lib"
#use "sys.lib"
#use "drivers.lib"
#use "dma.lib"

#if (BOARD_TYPE==LITTLE_GIANT)
#use "lgiant.lib"
#use "ioexpand.lib"
#use "pbus_lg.lib"
#use "s0232.lib"
#use "s1232.lib"
#endif

#if (BOARD_TYPE==TINY_GIANT)
#use "tgiant.lib"
#use "pbus_tg.lib"
#endif

#if (BOARD_TYPE==SMARTBLOCK)
#use "scc232.lib"
#use "prport.lib"
#endif

#if (BOARD_TYPE==CPLC_BOARD)
#use "rg.lib"
#use "lcd2l.lib"
#use "plc_exp.lib"
#use "uiboard.lib"
#use "step.lib"
#endif

#if (BOARD_TYPE==L_STAR)
#use "lstar.lib"
#use "lcd2l.lib"
#use "plc_exp.lib"
#use "uiboard.lib"
#use "step.lib"
#endif

#if (BOARD_TYPE==Z104ISA)
#use "z104.lib"
#use "com232.lib"
#use "plc_exp.lib"
#use "uiboard.lib"
#use "step.lib"
#endif

#if (BOARD_TYPE==LittleG)
#use "littleg.lib"
#use "plc_exp.lib"
#use "uiboard.lib"
#use "step.lib"
#endif

#if (BOARD_TYPE==PLC_BOARD)
#use "plc_exp.lib"
#use "uiboard.lib"
#use "step.lib"
#endif

#if (BOARD_TYPE==MicroG) || (BOARD_TYPE==MicroG2)
#use "microg.lib"
#endif

#if (BOARD_TYPE==SCoreZ1)
#use "scorez1.lib"
#endif

#use "modem232.lib"
#use "network.lib"
#use "z0232.lib"
#use "z1232.lib"

/*** EndHeader */
