#ifndef __AX25_EncodeDecode_H
#define __AX25_EncodeDecode_H

  enum{COMM_CR=0x0D,COMM_FLAG=0x7E, COMM_TXHEADER_LEN=16};

// Predefined header for Beacon
  extern const char Tx1_Header[16];

// Predefined header for packet directed to SGS
  extern const char Tx2_Header[16];

  extern const char Hello[126];

// Predefined test packets
  extern const char Packet_NoBit[19];
  extern const char Packet_WBit[28];
  extern const char Packet_WBitshort[12];
  extern const char Packet_test[28];
  extern const char Packet_NoBitshort[3];

// 
  extern unsigned int RX_SR, dump;
  extern unsigned char RX_SR17, RX_ST, RXFLAG, RXMASK, RxBit, ones;


  void CRC_CCITT_Generator(unsigned char *dat, unsigned int *len);
  void Stuff_Transition_Scramble(unsigned char *dat, unsigned int *len);
  void Reverse_Scramble_Transition_Stuff(unsigned char *dat, unsigned int len);

#endif




