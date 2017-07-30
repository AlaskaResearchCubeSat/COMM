#ifndef __COMM_H
#define __COMM_H

typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define CDH_RESET       BIT4 //P6.4

//P1IV definitions. These MUST match to the pins above!!!!!
#define CC1101_GDO0_IV      P1IV_P1IFG1  // interrupt on P1.0
#define CC1101_GDO2_IV      P1IV_P1IFG2  // interrupt on P1.1
#define CC2500_1_GDO0_IV    P1IV_P1IFG3  // interrupt on P1.2
#define CC2500_1_GDO2_IV    P1IV_P1IFG4  // interrupt on P1.3

//events for COMM task
extern CTL_EVENT_SET_t COMM_evt;
extern short beacon_on, beacon_flag;

  //events in COMM_evt
  enum{COMM_EVT_CC1101_RX_READ=1<<0, COMM_EVT_CC1101_TX_START=1<<1, COMM_EVT_CC1101_TX_THR=1<<2, COMM_EVT_CC1101_TX_END=1<<3, COMM_EVT_CC2500_1_RX_READ=1<<4, COMM_EVT_CC2500_1_TX_START=1<<5, COMM_EVT_CC2500_1_TX_THR=1<<6, COMM_EVT_CC2500_1_TX_END=1<<7, COMM_EVT_CC2500_2_RX_READ=1<<8, COMM_EVT_CC2500_2_TX_START=1<<9, COMM_EVT_CC2500_2_TX_THR=1<<10, COMM_EVT_CC2500_2_TX_END=1<<11, COMM_EVT_IMG_DAT=1<<12, COMM_EVT_LEDL_DAT=1<<13, COMM_EVT_STATUS_REQ=1<<14, COMM_EVT_GS_DECODE=1<<15};

 #define COMM_EVT_ALL (COMM_EVT_CC1101_RX_READ | COMM_EVT_CC1101_TX_START | COMM_EVT_CC1101_TX_THR | COMM_EVT_CC1101_TX_END | COMM_EVT_CC2500_1_RX_READ | COMM_EVT_CC2500_1_TX_START | COMM_EVT_CC2500_1_TX_THR | COMM_EVT_CC2500_1_TX_END | COMM_EVT_CC2500_2_RX_READ | COMM_EVT_CC2500_2_TX_START | COMM_EVT_CC2500_2_TX_THR | COMM_EVT_CC2500_2_TX_END | COMM_EVT_IMG_DAT | COMM_EVT_LEDL_DAT | COMM_EVT_STATUS_REQ | COMM_EVT_GS_DECODE)

  //data transmit types
  enum{TX_DATA_BUFFER=0,TX_DATA_RANDOM,TX_DATA_PATTERN};

  //command table for GS commands
  enum{COMM_RF_OFF=0x00, COMM_RF_ON=0xFF, COMM_BEACON_STATUS=0x0F, COMM_BEACON_HELLO=0xF0, COMM_RESET_CDH=0x33, COMM_GET_DATA=0xAA, COMM_SEND_DATA=0x55};


  //structure for status data from COMM
  typedef struct{
    unsigned char CC1101;	//MARCSTATE of CC1101 radio
    unsigned char CC2500_1;     //MARCSTATE of CC2500_1 radio
    unsigned char CC2500_2;     //MARCSTATE of CC2500_2 radio
    unsigned char Num_CMD;      //Number of commands received
    unsigned short ACDS_data;	//#ACDS packets in COMM SD card
    unsigned long LEDL_data;   	//#LEDL packets in COMM SD card
    unsigned short IMG_data;    //#IMG packets in COMM SD card
  }COMM_STAT;

  extern COMM_STAT status;

  extern unsigned char data_seed;

  extern short data_mode;
  //flags for STAT_PACKET

  //parse events from the bus for the subsystem
  void sub_events(void *p);

  //parse COMM specific events
   void COMM_events(void *p);
  
  // beacon timer setup
  void COMM_beacon_setup(void);

  void Radio_Interrupt_Setup(void);
  void PrintBuffer(char *dat, unsigned int len);
  void PrintBufferBitInv(char *dat, unsigned int len);

  extern unsigned char Tx1Buffer[];
  extern unsigned char RxBuffer[];
  extern unsigned int Tx1Buffer_Len, TxBufferPos, TxBytesRemaining;
  extern unsigned int RxBuffer_Len,  RxBufferPos, RxBytesRemaining;
  extern unsigned int state, small_packet, PkftLenUpper, PktLenLower, PktLen;
  extern BOOL INFINITE;
  extern char temp_countTX, temp_countRX, RxFIFOLen;
  extern int Tx_Flag; //used in RF_Send_Packet not sure why
  extern unsigned char IMG_Blk;

#endif
