

void Xbee_ATDNC();
void Xbee_Send();
void Xbee_Frame(unsigned char *xbee_pkt,unsigned char *Msg,UINT8 len,unsigned char *Coor_Addr);
BOOL Check_PKT_Format(unsigned char *src,unsigned char size);
BOOL Check_RDP_Format(unsigned char *src,unsigned char size);
void Xbee_Frames(unsigned char *,unsigned char *,UINT8,unsigned char *);
char recive_byte(UART_MODULE UARTid);
#define delimeter      0x7E
#define frametype_tx   0x10
#define frameid        0x01
#define destadd_16bitMB       0xFF
#define destadd_16bitLB       0xFE
#define brodcast       0x00
#define options        0x00



#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined(DEFINE_GLOBALS)

        GLOBAL ARRAY unsigned char	dest_addr[8];
        GLOBAL ARRAY unsigned char	xbee_addr[2];
#elif defined (DEFINE_EXTERNS)

        GLOBAL ARRAY unsigned char	dest_addr[];
        GLOBAL ARRAY unsigned char	xbee_addr[];
#endif