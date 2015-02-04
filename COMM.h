#ifndef __COMM_H
#define __COMM_H

typedef int BOOL;
#define TRUE 1
#define FALSE 0

#define CDH_RESET       BIT4

extern short beacon_on, beacon_flag;

  //events in COMM_evt
  enum{CC1101_EV_RX_READ=1<<1,CC1101_EV_TX_START=1<<3,CC1101_EV_TX_THR=1<<4,CC1101_EV_TX_END=1<<5,COMM_EVT_IMG_DAT=1<<6,COMM_EVT_LEDL_DAT=1<<7,COMM_EVT_STATUS_REQ=1<<8,COMM_EVT_GS_DECODE=1<<9};

  #define COMM_EVT_ALL (CC1101_EV_RX_READ|CC1101_EV_TX_START|CC1101_EV_TX_THR|CC1101_EV_TX_END|COMM_EVT_IMG_DAT|COMM_EVT_LEDL_DAT|COMM_EVT_STATUS_REQ|COMM_EVT_GS_DECODE)

  //command table for GS commands
enum{COMM_RF_OFF=0x00, COMM_RF_ON=0xFF, COMM_BEACON_STATUS=0x0F, COMM_BEACON_HELLO=0xF0, COMM_RESET_CDH=0x33, COMM_GET_DATA=0xAA, COMM_SEND_DATA=0x55};

  //structure for status data from COMM
  //TODO: figure out COMM status
  typedef struct{
    unsigned char CC1101;	//MARCSTATE of CC1101 radio
    unsigned char Num_CMD;      //Number of commands received
    unsigned short ACDS_data;	//#ACDS packets in COMM SD card
    unsigned long LEDL_data;   	//#LEDL packets in COMM SD card
    unsigned short IMG_data;    //#IMG packets in COMM SD card
  }COMM_STAT;

  extern COMM_STAT status;

  //flags for STAT_PACKET

  //parse events from the bus for the subsystem
  void sub_events(void *p);

  //events for COMM task
  extern CTL_EVENT_SET_t COMM_evt;

  //parse COMM specific events
  void COMM_events(void *p);

  void COMM_Setup(void);
  void Radio_Interrupt_Setup(void);
  void PrintBuffer(char *dat, unsigned int len);
  void PrintBufferBitInv(char *dat, unsigned int len);

  extern unsigned char Tx1Buffer[];
  extern unsigned char RxBuffer[];
  extern unsigned int Tx1Buffer_Len, TxBufferPos, TxBytesRemaining;
  extern unsigned int RxBuffer_Len,  RxBufferPos, RxBytesRemaining;
  extern unsigned int state, small_packet, PktLenUpper, PktLenLower, PktLen;
  extern BOOL INFINITE;
  extern char temp_countTX, temp_countRX, RxFIFOLen;
  extern int Tx_Flag; //used in RF_Send_Packet not sure why
  extern unsigned char IMG_Blk;

#endif
