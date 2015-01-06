#include <msp430.h>
#include <ctl_api.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <SDlib.h>
#include "COMM.h"
#include "AX25_EncodeDecode.h"
#include "COMM_Events.h"

unsigned int RX_SR = 0, dump;
unsigned char RX_SR17 = 0, RX_ST, RXFLAG=0, RXMASK = 0x80, RxBit=0, ones=0;

//***********************************************
//          FUNCTIONS
//
// void CRC_CCITT_Generator(char *dat, unsigned int *len)
//
//      This function generates the 2 byte FCS for the AX.25 packet
//      The FCS is placed in dat at dat[len], dat[len+1], i.e. just after the data packet
//      len is then updated to len+2
//
// void Stuff_Transition_Scramble(char *dat, unsigned int *len)
//
//      This function performs the following tasks:
//          1. Adds appropriate "flags" to front and back of packet
//          2. Bit stuffing - searches for six consecutive 1's in the packet and stuffs a 0 after the fifth 1.
//          3. Encode transitions - bit=0 requires transistion (i.e. 0-1 or 1-0), bit=1 requires no transition
//          4. Scramble - whitens the data based on pre-defined polynomial (see wiki for details)
//      Uses Buffer as "scratch" processing space
//      Final array stored back to dat
//      len is updated to len + 12 or so depending on number of bits needing to be stuffed.
//      After this function the AX.25 packet is fully created and ready to be sent to the radio
//
//***********************************************

//***********************************************
//          TEST PACKETS
//
// Test packets are defined MSB first, however, they are processed for AX.25 LSB first.
// Therefore, need to reverse bit order when loaded the processing array, e.g.: __bit_reverse_char(Packet_NoBit[i])
// In order to understand the output of the processing need to again reverse the bit order when printing
// Everything, except potentially the last byte, should agree
//
//No bit stuffing TestPacket    C    Q   space space space space SSID  K    L    2    T    U   space SSID CNTR PID   0    1   carriage return
const char Packet_NoBit[19] = {0x86,0xA2,0x40, 0x40, 0x40, 0x40, 0x60,0x96,0x98,0x64,0xA8,0xAA,0x40, 0x61,0x03,0xF0,0x30,0x31,0x0D};
//
// FCS for this packet (printed bit reversed order) is 0x65 0xBC (len=21)
// After encoding for transitions test packet should be (len=33):
//    0x55 0x7F 0x7F 0x7F 0x7F 0x7F 0xD7 0x34 0x95 0x6A 0x95 0x6A 0x75 0x27 0xDD 0x76 0xCD 0xCC 0x6A 0x8A 
//    0xAB 0xFA 0xBA 0x45 0xAE 0x89 0x3E 0x7F 0x7F 0x7F 0x7F 0x7F 0x55
// After scrambling the test packet should be (len=35):
//    0x55 0x2F 0x20 0x23 0x0D 0xEB 0x7D 0x3C 0xA9 0x81 0xDD 0xB1 0xD3 0x7F 0x87 0xFE 0x2B 0x8E 0xDF 0x6E 
//    0xF9 0xB1 0x57 0x5D 0xD4 0x76 0xFB 0x25 0xD6 0x56 0xBE 0x37 0x52 0x4C 0x61
//
//Bit stuffing TestPacket      C    Q   space space space space SSID  K    L    2    T    U   space SSID CNTR PID   H    e    l    l    o   space  J    e    s    s    e   carriage return
const char Packet_WBit[28] = {0x86,0xA2,0x40, 0x40, 0x40, 0x40, 0x60,0x96,0x98,0x64,0xA8,0xAA,0x40, 0x61,0x03,0xF0,0x48,0x65,0x6C,0x6C,0x6F,0x20, 0x4A,0x65,0x73,0x73,0x65,0x0D};
//
// FCS for this packet (printed bit reversed order) is 0xF4 0xB3 (len = 30)
// After bit-stuffing and encoding for transitions test packet should be (len=43):
//    0x55 0x7F 0x7F 0x7F 0x7F 0x7F 0xD7 0x34 0x95 0x6A 0x95 0x6A 0x75 0x27 0xDD 0x76 0xCD 0xCC 0x6A 0x8A 
//    0xAB 0xFA 0x92 0x89 0x8E 0x8E 0x8F 0x4A 0x93 0x89 0x7B 0x84 0x89 0x51 0xF9 0x89 0x01 0x01 0x01 0x01 
//    0x01 0x55 0x01
// After scrambling the test packet should be (len=45):
//    0x55 0x2F 0x20 0x23 0x0D 0xEB 0x7D 0x3C 0xA9 0x81 0xDD 0xB1 0xD3 0x7F 0x87 0xFE 0x2B 0x8E 0xDF 0x6E 
//    0xF9 0xB1 0x7F 0x11 0x66 0xCD 0x95 0x8C 0x71 0x88 0x1F 0x6C 0x77 0xFF 0xE0 0x78 0x4E 0x17 0xE9 0xBE
//    0x3D 0xF3 0x49 0x79 0x03
//
//Bit stuffing TestPacket      C    Q   space space space space SSID  K    L    2    T    U   space SSID CNTR PID   H    e    l    l    o   space  J    e    s    s    e   carriage return
const char Packet_test[28] = {0x86,0xA2,0x40, 0x40, 0x40, 0x40, 0x60,0x96,0x98,0x64,0xA8,0xAA,0x40, 0x61,0x03,0xF0,0x00,0x01,0x02,0x03,0x04,0x20, 0x05,0x06,0x07,0x08,0x09,0x0D};
//
//***********************************************

//***********************************************
//                            C    Q   space space space space SSID  K    L    3    J    P   space SSID CNTR PID
const char Tx1_Header[16] = {0x86,0xA2,0x40, 0x40, 0x40, 0x40, 0x60,0x96,0x98,0x66,0x94,0xA0,0x40, 0x61,0x03,0xF0};

//                            W    L    7    C    X    B   SSID  K    L    3    J    P   space SSID CNTR PID
const char Tx2_Header[16] = {0xAE,0x98,0x6E,0x86,0xB0,0x84,0x60,0x96,0x98,0x66,0x94,0xA0,0x40, 0x61,0x03,0xF0};

// = {0x54,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x41,0x4c,0x41,0x53,0x4b,0x41,0x20,0x52,0x45,0x53,0x45,0x41,0x52,0x43,0x48,0x20,0x43,0x55,0x42,0x45,0x53,0x41,0x54,0x2e,0x20,0x50,0x6c,0x65,0x61,0x73,0x65,0x61,0x73,0x65,0x20,0x65,0x6d,0x61,0x69,0x6c,0x20,0x64,0x6c,0x74,0x68,0x6f,0x72,0x73,0x65,0x6e,0x40,0x61,0x6c,0x61,0x73,0x6b,0x61,0x2e,0x65,0x64,0x75,0x20,0x74,0x69,0x6D,0x65,0x2c,0x20,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x2c,0x20,0x61,0x64,0x64,0x72,0x65,0x73,0x73,0x2e,0x20,0x57,0x69,0x6c,0x6c,0x20,0x73,0x65,0x6e,0x64,0x20,0x51,0x53,0x4c,0x20,0x63,0x61,0x72,0x64,0x2e,0x20,0x54,0x48,0x41,0x4e,0x4b,0x53,0x21};
const char Hello[126]="This is ALASKA RESEARCH CUBESAT. Please email dlthorsen@alaska.edu time, location, freq, address.  Will send QSL card. THANKS!";
// Header: A1, A2, A3, A4, A5, A6, A7, A8, A0, A10, A11, A12, A13, A14
//         Destination Callsign  ,SSID, Transmitter callsign     ,SSID,Control,PID

// Destinaion Call Sign for Beacon data: CQ
// Symbol,     ASCII(Hex),  Shifted(Hex)
//  C             0x43        0x86
//  Q             0x51        0xA2
// space          0x20        0x40
// space          0x20        0x40
// space          0x20        0x40
// space          0x20        0x40

// Destination Call Sign for station data: WL7CXB
// Symbol,     ASCII(Hex),  Shifted(Hex)
//  W           0x57          0xAE
//  L           0x4C          0x98
//  7           0x37          0x6E
//  C           0x43          0x86
//  X           0x58          0xB0
//  B           0x42          0x84

// Transmitter Call Sign (i.e. Satellite): 
// Symbol,     ASCII(Hex),  Shifted(Hex)
//  K           0x4B          0x96
//  L           0x4C          0x98
//  3           0x33          0x66
//  J           0x4A          0x94
//  P           0x50          0xA0
// space        0x20          0x40

void CRC_CCITT_Generator(char *dat, unsigned int *len){
  int i,n;
  unsigned short FCS, SR=0xFFFF;
  unsigned short XORMask;
  char bytemask;

  // G=[0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 1]=0x1021 Generator polynomial
  //GFLIP=[1 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0]=0x8408 Flipped generator polynomial for XOR operation in loop
  const unsigned short GFLIP=0x8408;

//Need to loop through Tx1Buffer one bit at a time! 
  for(n=0;n<*len;n++){           //address Tx1Buffer one byte at a time
    bytemask=0x80;              //mask to select correct bit in Tx1Buffer byte
    for(i=0;i<8;i++){           //loop through each bit in Tx1Buffer byte
      if((dat[n] & bytemask)!=0){
        if((SR & BIT0)!=0){
          SR=SR>>1;
          SR=SR^0x0000;
        }
        else{
          SR=SR>>1;
          SR=SR^GFLIP;
        }
      }
      else{
        if((SR & BIT0)!=0){
          SR=SR>>1;
          SR=SR^GFLIP;
        }
        else{
          SR=SR>>1;
          SR=SR^0x0000;
        }
      }       
      bytemask=bytemask>>1;
    }
  }
  FCS = __bit_reverse_short(~SR);
  dat[*len]=FCS>>8;
  dat[*len+1]=FCS;
  *len=*len+2;
}

void Stuff_Transition_Scramble(char *dat, unsigned int *len){
  unsigned char *scratch;
  unsigned int j, k, SR;
  char m, n, ones, oldbit;
  unsigned char datmask, scratchmask;
  char SR12, SR17;
  const char flagT=0xFE, flagN=0x01;

  scratch=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
  if (scratch==NULL){
  printf("unable to use bus buffer \r\n");
  return;
  }

  // Initialize scratch to zeros.  Not sure if this is necessary, but I'm paranoid.
  for(k=0;k<600;k++){ 
    scratch[k] = 0x00;
  }

  //Process
  //1. Add COMM_FLAG=0x7E to front
  //2. Search through packet bitwise (after flag) for consecutive 1's.  Need to stuff a 0 bit in packet AFTER the fifth consecutive 1.
  //3. Search through packet bitwise (including flag) and encode for transitions: 0=transition (either 0-1 or 1-0) 1=no transitions.  
  //4. Steps 2 and 3 can be done simultaneously
  //5. After bit stuffing and encoding for transitions, need to scramble data
  //
  //For bit stuffing and encoding for transitions data moves from dat to scratch.
  //For scrambling, data moves from scratch back to dat
  //Lastly need to update the length of actual data in dat.
  //
  //Where to start?
  //

  //set-up first six bytes alreaded encoded for transitions
  //Unencoded bytes were 0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E 
  //The five flags (0x7E) is an arbitrary number.  Need at least one.
  scratch[0]=0xAA;
  scratch[1]=flagT;
  scratch[2]=flagT;
  scratch[3]=flagT;
  scratch[4]=flagT;
  scratch[5]=flagT;

  k=6;                    //counter for scratch bytes
  n=0;                    //counter for bits in scratch byte
  ones = 0;               //counter for consecutive ones
  scratchmask = 0x80;     //initialize scratchmask to MSB
  oldbit = 0;             //initialize state of previous bit for transition encoding.

  //This loop will stuff bits AND encode for transitions
  for(j=0;j<*len;j++){                                    // counter for dat bytes - search through entire packet
    datmask = 0x80;                                       // initialize datmask to MSB
    for(m=0;m<8;m++){                                     // shift through every bit in dat[j]
      if(((dat[j] & datmask) !=0 ) && (ones == 5)){       // dat bit = 1, 5 consecutive one's need to stuff a zero
        scratch[k] = scratch[k] | (scratchmask & ~oldbit);// forced zero bit requires transition (i.e. ~oldbit) at position of scratch mask
        oldbit = ~oldbit;                                 // update oldbit
        if(n < 7){                                        // if not last bit in scratch[k]
          scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
          n=n+1;                                          // increment bit counter for scratch[k]
        } else {                                          // just processed last bit in scratch[k]
          scratchmask = 0x80;                             // reset scratchmask to point to MSB
          n=0;                                            // reset bit pointer for scratch[k]
          k=k+1;                                          // increment to next byte in scratch
        }
        ones = 0;                                         // reset ones counter
      }
      if(((dat[j] & datmask) != 0) && (ones < 5)){        // dat bit = 1, count consecutive one's
        scratch[k] = scratch[k] | (scratchmask & oldbit); // dat bit = 1 requires no transition (i.e. oldbit) at position of scratch mask
        if(n < 7){                                        // if not last bit in scratch[k]
          scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
          n=n+1;                                          // increment bit counter for scratch[k]
        } else {                                          // just processed last bit in scratch[k]
          scratchmask = 0x80;                             // reset scratchmask to point to MSB
          n=0;                                            // reset bit pointer for scratch[k]
          k=k+1;                                          // increment to next byte in scratch
        }
        ones = ones+1;                                    // increment ones counter
      } else {                                            // dat bit = 0
        scratch[k] = scratch[k] | (scratchmask & ~oldbit);// dat bit = 0 requires transition (i.e. ~oldbit) at position of scratch mask
        oldbit = ~oldbit;                                 // update oldbit
        if(n < 7){                                        // if not last bit in scratch[k]
          scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
          n=n+1;                                          // increment bit counter for scratch[k]
        } else {                                          // just processed last bit in scratch[k]
          scratchmask = 0x80;                             // reset scratchmask to point to MSB
          n=0;                                            // reset bit pointer for scratch[k]
          k=k+1;                                          // increment to next byte in scratch
        }
        ones = 0;                                         // reset ones counter
      }
      datmask = datmask>>1;                               // shift to look at next bit
    }
  }
  //Now shift in 5 flags and fill out last byte with zero's
  if(oldbit == 0){
    for(j=0;j<5;j++){
       datmask = 0x80;
       for(m=0;m<8;m++){
          if((datmask & flagT) != 0){                     //put in one
             scratch[k] = scratch[k] | scratchmask;
          }
          if(n < 7){                                       // if not last bit in scratch[k]
             scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
             n=n+1;                                          // increment bit counter for scratch[k]
          } else {                                          // just processed last bit in scratch[k]
             scratchmask = 0x80;                             // reset scratchmask to point to MSB
             n=0;                                            // reset bit pointer for scratch[k]
             k=k+1;                                          // increment to next byte in scratch
          }
          datmask = datmask>>1;
       }
     }
       datmask = 0x80;
       for(m=0;m<8;m++){
          if((datmask & 0xAA) != 0){                     //put in one
             scratch[k] = scratch[k] | scratchmask;
          }
          if(n < 7){                                       // if not last bit in scratch[k]
             scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
             n=n+1;                                          // increment bit counter for scratch[k]
          } else {                                          // just processed last bit in scratch[k]
             scratchmask = 0x80;                             // reset scratchmask to point to MSB
             n=0;                                            // reset bit pointer for scratch[k]
             k=k+1;                                          // increment to next byte in scratch
          }
          datmask = datmask>>1;
       }
   } else {

    for(j=0;j<5;j++){
       datmask = 0x80;
       for(m=0;m<8;m++){
          if((datmask & flagN) != 0){                     //put in one
             scratch[k] = scratch[k] | scratchmask;
          }
          if(n < 7){                                       // if not last bit in scratch[k]
             scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
             n=n+1;                                          // increment bit counter for scratch[k]
          } else {                                          // just processed last bit in scratch[k]
             scratchmask = 0x80;                             // reset scratchmask to point to MSB
             n=0;                                            // reset bit pointer for scratch[k]
             k=k+1;                                          // increment to next byte in scratch
          }
          datmask = datmask>>1;
       }
     }
     datmask = 0x80;
       for(m=0;m<8;m++){
          if((datmask & 0x55) != 0){                     //put in one
             scratch[k] = scratch[k] | scratchmask;
          }
          if(n < 7){                                       // if not last bit in scratch[k]
             scratchmask = scratchmask>>1;                   // shift to next bit in scratch[k]
             n=n+1;                                          // increment bit counter for scratch[k]
          } else {                                          // just processed last bit in scratch[k]
             scratchmask = 0x80;                             // reset scratchmask to point to MSB
             n=0;                                            // reset bit pointer for scratch[k]
             k=k+1;                                          // increment to next byte in scratch
          }
          datmask = datmask>>1;
       }
   }
   if(n != 0) k=k+1;

  //clear dat
  for(j=0;j<sizeof(dat);j++){
    dat[j]=0;
  }

  //Now on to scrambling! 
  //Scramble scratch put back in dat
  SR17=0;
  SR=0;
  for(j=0;j<k+2;j++){                 //For each byte in scratch
    scratchmask = 0x80;                       //Initialize scratchmask to MSB
    for(n=0;n<8;n++){                         //For each bit in scratch[j]
      if( (scratch[j] & scratchmask) == 0){   //Grab the current bit in scratch[j]
        oldbit = 0;
      } else {
        oldbit = 1;
      }
      if( (SR & 0x0010) == 0){                //Grab the twelfth bit in SR
        SR12 = 0;
      } else {
        SR12 = 1;
      }
      oldbit = oldbit ^ (SR12 ^ SR17);        //xor(scratch[j bit n], SR[bit 12], SR[bit 17])
      if( (SR & 0x0001) == 0){                //This is 17'th bit for next loop
        SR17 = 0;
      } else {
        SR17 = 1;
      }
      SR=SR>>1;                               //Shift SR to the right
      if(oldbit != 0){                        //oldbit shifts back in at MSB
        SR = SR | 0x8000;           
      }
      if(oldbit == 0){                        //scratch[j bit n] = 0
        dat[j] = dat[j] & ~scratchmask;
      } else {                                //scratch[j bit n] = 1
        dat[j] = dat[j] | scratchmask;
      }
      scratchmask = scratchmask>>1;           //shift to next bit
    }
  }
  
  *len = k+2;
  BUS_free_buffer();

}

void Reverse_Scramble_Transition_Stuff(char *indat, unsigned int inlen){
// This function will read from the RX_FIFO into RXBuffer in RxBufferThr byte chuncks and perform the following steps
// Unscramble the data bit wise.
// Reverse transitions
// Seek out the 7E flag indicating start of AX.25 packet.  This will identify byte boundary
// RX_SR, RX_SR17 holds shift register data beyond function 

  unsigned int j, k;
  int resp;
  unsigned char SR12, bit, newbit;
  unsigned char datmask, firsteight, firstbyte = 0x69;
  printf("Reverse_Scramble_Transition\r\n");

//unscramble (in place) one byte at a time 
  for(k=0;k<inlen;k++){
    datmask = 0x80;

    for(j=0;j<8;j++){
      if((indat[k] & datmask) == 0){	//Grab the current bit in dat[k]
        bit = 0;
      } else {
        bit = 1;
      }
      if((RX_SR & 0x0800) == 0){	//Grab the twelfth bit in RX_SR
        SR12 = 0;
      } else {
        SR12 = 1;
      }
      newbit = bit ^ (SR12 ^ RX_SR17);
      if(newbit == 0){
        indat[k] = indat[k] & ~datmask;	//put back in the unscrambled bit
      } else {
        indat[k] = indat[k] | datmask;
      }
      if((RX_SR & 0x8000) == 0){  	//set up RX_SR17 for next loop;
        RX_SR17 = 0;
      } else {
        RX_SR17 = 1;
      }
      
      datmask = datmask>>1;		//shift to next bit in dat[k]
      
      RX_SR = RX_SR<<1;			//shift left
      if(bit != 0){
        RX_SR = RX_SR | 0x0001;		//shift in bit (at LSB)
      } 
    }    
  }

//reverse transitions (in place) (can combine with unscramble?)
  for(k=0;k<inlen;k++){
    datmask = 0x80;
    for(j=0;j<8;j++){
      if((indat[k] & datmask) == 0){
        bit = 0;
      } else {
        bit = 1;
      }
      if(RX_ST == bit){
        indat[k] = indat[k] | datmask;  //put in 1
      } else {
        indat[k] = indat[k] & ~datmask; // put in 0
      }
      RX_ST = bit;
      datmask = datmask>>1;
    }  
  }

//  PrintBufferBitInv(indat, inlen);


//search for flag 0111 1110, align on byte boundaries, first search for zero, then six ones, then zero.
  for(k=0;k<inlen;k++)
  {
    datmask = 0x80;
//    printf("indat[%d] = 0x%02x, RXFLAG = %d, RxBufferPos=%d, RxMask=0x%02x, RxBit=%d\r\n",k,indat[k],RXFLAG,RxBufferPos,RXMASK,RxBit);
    for(j=0;j<8;j++)
    {
      switch(RXFLAG)
      {
        case 0:     //Start of packet.  Search for first 0
            RxBufferPos = 0;
	    RXMASK = 0x80;
	    RxBit = 0;
            firsteight=0;
            dump = 0;
            //RxBuffer_Len = 0;
            ones = 0;
            if((indat[k] & datmask) == 0){    //Bit = 0, POTENTIAL FLAG, first zero
              RXFLAG = 1;
            } else {                          //Bit = 1, looking for first zero
              RXFLAG = 0;
            }
            break;
        case 1:     //Found first 0.  Find next six 1's (case 1-6)
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            if((indat[k] & datmask) == 0){  //Bit = 0, still at first zero
              RXFLAG = 1;
            } else {
              RXFLAG = RXFLAG + 1;          //Bit = 1, counting six one's
            }
            break;
        case 7:     //Found 0111 111, looking for last 0
            if((indat[k] & datmask) == 0){   //Bit = 0, FOUND FLAG BYTE
              RXFLAG = 8;
            } else {                        //Bit = 1, Start over from beginning
              RXFLAG = 0;
            }
            break;
        case 8:     //Found FLAG BYTE load next bytes to RxBuffer unless it is another FLAG BYTE   ****Need to make sure everything has been initialized correctly ******
        if(firsteight == 0) {
        //  printf("case 8 found FLAG BYTE\r\n");
          firsteight = 1;
        }
          if((indat[k] & datmask) != 0){                     //put in one
             RxBuffer[RxBufferPos] = RxBuffer[RxBufferPos] | RXMASK;
             ones = ones+1;
          } else {                                          // put in zero
             RxBuffer[RxBufferPos] = RxBuffer[RxBufferPos] & ~RXMASK;
             ones = 0;
          }
          if(RxBit < 7){                                     // if not last bit in RxBuffer[k]
            RXMASK = RXMASK>>1;                              // shift to next bit in RxBuffer[k]
            RxBit=RxBit+1;                                  // increment bit counter for RxBuffer[k]
          } else {                                           // just processed last bit in RxBuffer[k]
             RXMASK = 0x80;                                  // reset outdatmask to point to MSB
             RxBit=0;                                        // reset bit pointer for RxBuffer[k]
             if(RxBuffer[RxBufferPos] != 0x7E){              // If didn't receive another FLAG byte, save else dump byte
               if(RxBuffer[RxBufferPos] != firstbyte){       // Not what I expect for first byte of packet!  Start over!
          //      printf("Not Good RxBuffer[%d] = 0x%02x\r\n",RxBufferPos,RxBuffer[RxBufferPos]);
                firsteight=0;
                RXFLAG = 0;
               } else {
          //     printf("Good RxBuffer[%d] = 0x%02x\r\n",RxBufferPos,__bit_reverse_char(RxBuffer[RxBufferPos]));
                RxBufferPos=RxBufferPos+1;                   // increment to next byte in RxBuffer
                ones = 0;
                RXFLAG = 9;
                firsteight=0;
               }
             }
          }
          break;
        case 9:     //Load bytes to RxBuffer until end of packet FLAG BYTE found
		//dump bit stuffed zero if there would have been six ones in a row, i.e. 1111101 the zero is removed, 1111100 the zero is not removed
        if(firsteight == 0) {
        //  printf("case 9 Load RxBuffer\r\n");
          firsteight = 1;
        }
          if((indat[k] & datmask) != 0){                     //put in one
             RxBuffer[RxBufferPos] = RxBuffer[RxBufferPos] | RXMASK;
             ones = ones+1;
	         dump = 0;
          } else {    // put in zero
         
	      if(dump){							//this means I should have incremented pointer (i.e. shouldn't have dumped)
		    if(RxBit<7){
		    RXMASK = RXMASK>>1;
		    RxBit=RxBit+1;
		    }
		    else{
		    RXMASK = 0x80;
		    RxBit=0;
		    }
		  }
               
	      RxBuffer[RxBufferPos] = RxBuffer[RxBufferPos] & ~RXMASK;
             if(ones != 5) {                                // If not five ones then save the zero otherwise dump
		       dump = 0;
             } else {                                       // If after five ones dump the zero only if the next bit is a one!
               dump = 1;
             //  printf("dump=1, RXBufferPos = %d, RxBit= %d, datmask = 0x%02x\r\n", RxBufferPos,RxBit,datmask);
             }
             ones = 0;			//where should I reset this?
          }
          if(RxBit < 7){                                     // if not last bit in RxBuffer[k]
             if(!dump){
               RXMASK = RXMASK>>1;                             // shift to next bit in RxBuffer[k]
               RxBit=RxBit+1;                                  // increment bit counter for RxBuffer[k]
	     }
          } else {                                           // just processed last bit in RxBuffer[k]
	     if(!dump){
               RXMASK = 0x80;                                  // reset RXMASK to point to MSB
               RxBit=0;                                        // reset bit pointer for RxBuffer[k]
               if(RxBuffer[RxBufferPos] != 0x7E){              //If didn't receive another FLAG byte, save else dump byte
               // printf("Good RxBuffer[%d] = 0x%02x\r\n",RxBufferPos,__bit_reverse_char(RxBuffer[RxBufferPos]));
                RxBufferPos=RxBufferPos+1;                   // increment to next byte in RxBuffer
               } else {                                        // Just received END FLAG BYTE
                RXFLAG = 10;
               }
	     }
          }
          break;
        case 10:    //END OF PACKET now process packet
        printf("case 10 end of packet RxBuffer[0] = 0x%02x\r\n",RxBuffer[0]);
//Ground commands are to (destination) the satellite (KL3JP) from the (source) ground station (WL7CXB)

          if(RxBuffer[0] == firstbyte){          //If first byte is 0x96 reversed potential ground station command
            RxBuffer_Len = RxBufferPos;
            resp=comm_evt_gs_decode();		//Call ground station command decode event
	    if(resp!=RET_SUCCESS){
		printf("%s\r\n",COMM_error_str(resp));
	    }
            //RX_SR = 0;
            //RX_SR17 = 0;
            //RX_ST = 0;
            RXFLAG = 0;
            ones = 0;
          }
          break;
      }
      datmask = datmask>>1;
    }
  }

}
