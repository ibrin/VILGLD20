/***************************************************************************\
	Function Prototypes for Dynamic C RunTime Library
		Copyright (c) 1989, Z-World.
\***************************************************************************/

/*** BeginHeader main */

#ifndef CC_VER
#define CC_VER 0
#endif

#ifndef CoData
#define CoData FuncBlk
#endif

#if CC_VER<0x270
#define xmemok
#endif

#if CC_VER >= 0x400
#nointerleave
#else
#interleave
#endif

#if CC_VER >= 0x420
#makechain _GLOBAL_INIT
#endif

// kill symbols defined in EPROM once but now defined in .lib files

#KILL intr1
#KILL intr2
#KILL intr3

#if ROM==1  /* enable nodebug mode if destination is EPROM */

#nodebug
#endif

#class static   /* default memory storage is static in Dynamic C ! */
#nouseix    /* default do not use ix register as frame pointer */

#define NULL        (void *) 0
#define EOF     -1

#define EDOM            234
#define ERANGE          235

#define FLOAT           float
#define REGISTER        auto
#define STATIC          static

#define BADTAN          (FLOAT)1.560796327
#define EXPLARGE        (FLOAT)89.80081863
#define HUGE_VAL    (FLOAT)3.00e38
#define INF             (FLOAT)3.00e38
#define IPIby180        (FLOAT)57.29577951
#define LNof10          (FLOAT)2.302585093
#define LOG2            (FLOAT)0.30102999567
#define LOGE            (FLOAT)0.43429448190
#define ONE             (FLOAT)1.0
#define ONE_HALF        (FLOAT)0.500000
#define OVERFLOW               49
#define PI              (FLOAT)3.14159265359
#define PIby180         (FLOAT)0.0174532925
#define PIbyTWO         (FLOAT)1.570796326795
#define POW10INF        (FLOAT)38.0
#define SQR10           (FLOAT)3.162277660168
#define TWObyPI         (FLOAT)0.63661977
#define ZERO            (FLOAT)0.0

typedef struct  { 
			char    *STACKP;
			char    *AUXSTP;
			char    *IX;
			char    *RETADR;
			char	RETCBR;
	}   jmp_buf[1];

struct free_list {
					 int  size;
					 struct free_list *next;
				 };

extern
unsigned        FUNC_BREAK_ADDR,    /* Pointer to RST28 Switch  */
			FUNC_BREAK_VAL,     /* Byte to Store for BREAK  */
			TRAP_VEC,       /* HD64180 Trap Vector      */
			RST38_VEC,      /* RST38 Vector         */
			NMI_VEC,        /* NMI Relay Vector     */
			INT_VEC[128];       /* Interrupt Table      */

#JUMP_VEC  NMI_VEC  NMI_VEC    /* default power fail vector to loop */


extern
int (*ERROR_EXIT)();    /* Pointer to Error Handler  */

extern
char            STORE_TABLE[256];   /* Pointer Store Table      */


#if CC_VER < 0x600
#define short int
#endif

typedef struct {
	unsigned short addr;    // address
	unsigned char base; }   // base (BBR or CBR)
	ADDR24_S;

typedef union { unsigned long l;            /* long for increment/decrement */
						struct { ADDR24_S a;    /* the address itself */
									char flags; /* flags associated */
								 } aaa;
				  } ADDR24;

struct progStruct {
	ADDR24         RCB,RCE,    // root code (Begin and End)
						XCB,XCE,    // extended code (Begin and End)
						RDB,RDE,    // root data (Begin and End)
						XDB,XDE;    // extended data (RAM) (Begin and End)
	unsigned short auxStkB,    // aux stack Begin
						auxStkE,    // end
						stkB,       // stack begin
						stkE,       // end
						freeB,  // free begin
						freeE,  // end
						heapB,  // heap begin
						heapE;  // end
	};

extern struct progStruct prog_param;

typedef union
{
	unsigned	short	Integer;
	unsigned long	Long;
	float				Float;
	ADDR24			Addr;
} WatchResult;

// same as cWatchList::cWatch::sBuf
typedef struct
{
	unsigned	char	Size;
	unsigned char	Type;
	unsigned	short	Mods;
	WatchResult		Value;
} WatchExpr;
	
struct tm
{
	char tm_sec;             // seconds 0-59
	char tm_min;             // 0-59
	char tm_hour;            // 0-59
	char tm_mday;            // 1-31
	char tm_mon;             // 1-12
	char tm_year;            // 00-150 (1900-2050)
	char tm_wday;            // 0-6 0==sunday
};

#if CC_VER>=0x270

// newxmem support functions

extern unsigned vrst30;

#asm fast nodebug
_b_all::
	exx                 ;   use alternate register
	pop hl              ;   get initial ret addr
	ld      a,(hl)      ;   CBR of destination
	inc hl              ;   next location
	ld      c,(hl)      ;   low byte of destination
	inc hl              ;   next location
	ld      b,(hl)      ;   hi byte of destination
	inc hl              ;   next location
	in0     h,(CBR)     ;   save current CBR
	push    hl              ;   on the stack
	out0    (CBR),a     ;   change CBR now
	ld      hl,_b_ret   ;   bouncer return address
	push    hl              ;   gets pushed next
	push    bc              ;   push destination address
	exx                 ;   change back to ordinary registers
	ret                 ;   this is actually a jp to dest.

_b_ret::
	pop af
	out0    (CBR),a
	ret

#endasm

#endif

#define ASM_NOIX nodebug speed nouseix root
#define ASM_IX nodebug speed useix

#define _LD_HL_ISPO(offset) db 0x21, offset & 0xff, (offset >> 8) & 0xff, 0x39, 0x7e, 0x23, 0x66, 0x6f
#define _LD_DE_ISPO(offset) db 0x21, offset & 0xff, (offset >> 8) & 0xff, 0x39, 0x5e, 0x23, 0x56

/*** EndHeader */

// default main function
root main() {}

