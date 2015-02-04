// ALL FUNCTIONS USED TO OPERATE THE CC1100 and CC250 Radios
#include <msp430.h>
#include <stdio.h>
#include "Radio_functions.h"

char paTable_CC1101[] = {0x84};  //corresponds to +5dBm
//char paTable_CC1101[] = {0xC8};  //corresponds to +7dBm
//char paTable_CC1101[] = {0xC0};  //corresponds to +10dBm
char paTableLen=1;
int Tx_Flag;

//Function to read a single byte from the radio registers
char Radio_Read_Registers(char addr, char radio)
{
  char x;
  
  switch (radio){
  case CC1101:
     P5OUT &= ~CS_1101;                           // CS enable CC1101
     break;
  case CC2500:
     P5OUT &= ~CS_2500;                           // CS enable CC2500
     break;
  default:
    return -1;
  }


  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_SINGLE);  // Adding 0x80 to address tells the radio to read a single byte
  while (!(UC1IFG & UCB1TXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = 0;                               // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                   // Wait for TX to complete
  x = UCB1RXBUF;                               // Read data
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 

  return x;
}

//Function to read a multiple bytes from the radio registers
void Radio_Read_Burst_Registers(char addr, unsigned char *buffer, int count, char radio)
{
  char i;

  switch (radio){
  case CC1101:
     P5OUT &= ~CS_1101;                           // CS enable CC1101
     break;
  case CC2500:
     P5OUT &= ~CS_2500;                           // CS enable CC2500
     break;
  default:
    break;
  }

  while (!(UC1IFG & UCB1TXIFG));              // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Adding 0xC0 to address tells the radio to read multiple bytes
  while (UCB1STAT & UCBUSY);                  // Wait for TX to complete
  UCB1TXBUF = 0;                              // Dummy write to read 1st data byte
                                              // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  UC1IFG &= ~UCB1RXIFG;                       // Clear flag
  while (!(UC1IFG & UCB1RXIFG));              // Wait for end of 1st data byte TX
                                              // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB1TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = UCB1RXBUF;                    // Store data from last data RX
    while (!(UC1IFG & UCB1RXIFG));            // Wait for RX to finish
  }
  buffer[count-1] = UCB1RXBUF;                // Store last RX byte in buffer
  
  P5OUT |= CS_2500;                           // CS disable C2500 
  P5OUT |= CS_1101;
}

char Radio_Read_Status(char addr, char radio)
{
  char status;

  switch (radio){
  case CC1101:
     P5OUT &= ~CS_1101;                           // CS enable CC1101
     break;
  case CC2500:
     P5OUT &= ~CS_2500;                           // CS enable CC2500
     break;
  }

  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);     // Send address
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = 0;                                 // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                     // Wait for TX to complete
  status = UCB1RXBUF;                            // Read data
  P5OUT |= CS_1101;                              // CS disable C1101
  P5OUT |= CS_2500;                              // CS disable C2500 

  return status;
}

//Function that sends single address to radio initiating a state or mode change 
//(e.g. sending addr 0x34 writes to SRX register initiating radio in RX mode
void Radio_Strobe(char strobe, char radio)
{
  switch (radio){
  case CC1101:
     P5OUT &= ~CS_1101;                           // CS enable CC1101
     break;
  case CC2500:
     P5OUT &= ~CS_2500;                           // CS enable CC2500
     break;
  }

  while (!(UC1IFG & UCB1TXIFG));                  // Wait for TXBUF ready
  UCB1TXBUF = strobe;                             // Send strobe
                                                  // Strobe addr is now being TX'ed
  while (UCB1STAT & UCBUSY);                      // Wait for TX to complete

  P5OUT |= CS_2500;                               // CS disable C2500 
  P5OUT |= CS_1101;                               // CS disable C1101
}

//Function to write a single byte to the radio registers
void Radio_Write_Registers(char addr, char value, char radio)
{
  switch (radio){
  case CC1101:
    P5OUT &= ~CS_1101;                          // CS enable CC1101
    break;
  case CC2500:
    P5OUT &= ~CS_2500;                          // CS enable CC2500
    break;
  }

  while (!(UC1IFG & UCB1TXIFG));            // Wait for TXBUF ready
  UCB1TXBUF = addr;                         // Send address
  while (!(UC1IFG & UCB1TXIFG));            // Wait for TXBUF ready
  UCB1TXBUF = value;                        // Send data
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete

  P5OUT |= CS_1101;                         // CS disable C1101
  P5OUT |= CS_2500;                         // CS disable C2500 
  
}

//Function to write multiple bytes to the radio registers
void Radio_Write_Burst_Registers(char addr, unsigned char *buffer, int count, char radio)
{
  int i;

  switch(radio){
  case CC1101:
    P5OUT &= ~CS_1101;                            // CS enable CC1101
    break;
  case CC2500:
    P5OUT &= ~CS_2500;                           // CS enable CC2500
    break;
  }
  
  while (!(UC1IFG & UCB1TXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = addr | TI_CCxxx0_WRITE_BURST;      // Adding 0x40 to address tells radio to perform burst write rather than single byte write
  for (i = 0; i < count; i++)
  {
    while (!(UC1IFG & UCB1TXIFG));              // Wait for TXBUF ready
    UCB1TXBUF = buffer[i];                      // Send data
  }
  while (UCB1STAT & UCBUSY);                    // Wait for TX to complete
  
  P5OUT |= CS_1101;                            // CS disable C1101
  P5OUT |= CS_2500;                            // CS disable C2500 
}

void Reset_Radio(char radio)
{
  switch (radio){
  case CC1101:
    P5OUT |= CS_1101;               //Toggle CS with delays to power up radio
    TI_CC_Wait(30);
    P5OUT &= ~CS_1101;
    TI_CC_Wait(30);
    P5OUT |= CS_1101;
    TI_CC_Wait(45);

    P5OUT &= ~CS_1101;              // CS enable
    while (!(UC1IFG & UCB1TXIFG));  // Wait for TXBUF ready
    UCB1TXBUF = TI_CCxxx0_SRES;     // Send strobe
                                    // Strobe addr is now being TX'ed
    while (UCB1STAT & UCBUSY);      // Wait for TX to complete
    P5OUT |= CS_1101;               // CS disable
    break;
  case CC2500:
    P5OUT |= CS_2500;               //Toggle CS with delays to power up radio
    TI_CC_Wait(30);
    P5OUT &= ~CS_2500;
    TI_CC_Wait(30);
    P5OUT |= CS_2500;
    TI_CC_Wait(45);

    P5OUT &= ~CS_2500;              // CS enable
    while (!(UC1IFG & UCB1TXIFG));  // Wait for TXBUF ready
    UCB1TXBUF = TI_CCxxx0_SRES;     // Send strobe
                                    // Strobe addr is now being TX'ed
    while (UCB1STAT & UCBUSY);      // Wait for TX to complete
    P5OUT |= CS_2500;               // CS disable
    break;
  }

  P2IFG = 0;                        // Clear flags that were set (WHAT IS THIS?)
}

void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

void RF_Send_Packet(unsigned char *TxBuffer, int size, char radio)
{
  Radio_Write_Registers(TI_CCxxx0_PKTLEN, size, CC1101);
  Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer, size, radio); // Write TX data
  Radio_Strobe(TI_CCxxx0_STX, radio);                                   // Change state to TX, initiate data transfer
  Tx_Flag = 1;
}

void Write_RF_Settings(char radio)
{
switch (radio){
case CC1101:
// Register Values obtained via Smart Studio for CC1101
// Settings:
//  Carrier Freq: 437.565 MHz (437.564972)
//  Xtal Freq: 26 MHz
//  Data rate: 9.6 kBaud (9.59587)
//  Deviation: 4.8 kHz (4.760742)
//  Modulation format: 2-FSK
//  Channel spacing: 199.951172 kHz (default)
//  Rx filter BW: 42 kHz (58.035714 kHz)
//  Tx power 5 dBm
//  Using GDO0 and GDO2 to manage transmit/receive buffers
Radio_Write_Registers(TI_CCxxx0_IOCFG0,   0x00, CC1101);   // Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold
Radio_Write_Registers(TI_CCxxx0_IOCFG2,   0x02, CC1101);   // Associated to the TX FIFO: Asserts when TX FIFO is filled at or above the TX FIFO threshold. De-asserts when TX FIFO is drained below the same threshold.
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,  0x47, CC1101);   // FIFO Threshold [3:0] 7 = Bytes in TX FIFO = 33, Bytes in RX FIFO = 32
                                                           // Bit 6 = 1: Test1=0x35 and Test2=0x81 when waking from SLEEP.  ADC_RETENTION bit should be set to 1 before going into SLEEP mode if settings with an RX filter bandwidth below 325 kHz are wanted at time of wake-up.FIFO Threshold
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,  0x00, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,  0x0C, CC1101);
Radio_Write_Registers(TI_CCxxx0_FREQ2,    0x10, CC1101);
Radio_Write_Registers(TI_CCxxx0_FREQ1,    0xD4, CC1101);   // 10, A7, 62 = 433 MHz;  10, C4, EC = 436 MHz; 10, BB, 13 = 435 MHz: 10, D4, 55 (6E adjusted for measured offset) = 437.565 MHz
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0x66, CC1101);
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,  0xF8, CC1101);   // F5 = 1200 baud, F8 = 9600 baud
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,  0x83, CC1101);
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,  0x04, CC1101);   // High byte: 0000 is 2-FSK and 0001 is GFSK; Low byte: 0100 no preamble/sync+carrier sense
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,  0x02, CC1101);   // High byte: 0000 is 2 bytes of preamble, 0100 is 8 bytes of preamble
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,  0xF8, CC1101);
Radio_Write_Registers(TI_CCxxx0_CHANNR,   0x00, CC1101);
Radio_Write_Registers(TI_CCxxx0_DEVIATN,  0x14, CC1101);
Radio_Write_Registers(TI_CCxxx0_FREND1,   0xB6, CC1101);
Radio_Write_Registers(TI_CCxxx0_FREND0,   0x10, CC1101);
Radio_Write_Registers(TI_CCxxx0_MCSM1,    0x0F, CC1101);  // [3:2] RXOFF_MODE state after packet received: (00) IDLE, (11) RX.  
                                                          // [1:0] TXOFF_MODE state after packet transmitted: (00) IDLE, (11) RX
Radio_Write_Registers(TI_CCxxx0_MCSM0,    0x18, CC1101);
Radio_Write_Registers(TI_CCxxx0_FOCCFG,   0x16, CC1101);  //SmartStudio 0x1D, can't figure out why one or the other
Radio_Write_Registers(TI_CCxxx0_BSCFG,    0x1C, CC1101);  //
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2, 0xC7, CC1101);
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1, 0x30, CC1101);  //enable relative carrier sense
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0, 0xB0, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSCAL3,   0xE9, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSCAL2,   0x2A, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSCAL1,   0x00, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSCAL0,   0x1F, CC1101);
Radio_Write_Registers(TI_CCxxx0_FSTEST,   0x59, CC1101);
Radio_Write_Registers(TI_CCxxx0_TEST2,    0x81, CC1101);
Radio_Write_Registers(TI_CCxxx0_TEST1,    0x35, CC1101);
Radio_Write_Registers(TI_CCxxx0_TEST0,    0x09, CC1101);
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x00, CC1101);     
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, CC1101);      // Infinite packet length mode set
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, CC1101);      // Packet length set for fixed packet length mode
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x00, CC1101);
Radio_Write_Registers(TI_CCxxx0_SYNC1,    0x7E, CC1101);      // SYNC Word High Byte (SYNC0 sync word low byte)
Radio_Write_Registers(TI_CCxxx0_PATABLE,  0x84, CC1101);     //Set PA to 5dBm.
break;

/*
case CC2500:
// Write CC2500 register settings
Radio_Write_Registers(TI_CCxxx0_IOCFG0,   0x06, CC2500);  // GDO0 output pin config.
Radio_Write_Registers(TI_CCxxx0_IOCFG2,   0x00, CC2500);  // GDO2 output pin config.
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,  0x0F, CC2500);  // FIFO Threshold: 1 byte in TX FIFO and 63 in RX FIFO
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, CC2500);  // Packet length.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x04, CC2500);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x05, CC2500);  // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x01, CC2500);  // Device address.
Radio_Write_Registers(TI_CCxxx0_CHANNR,   0x00, CC2500);  // Channel number.
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,  0x07, CC2500);  // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,  0x00, CC2500);  // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FREQ2,    0x5D, CC2500);  // Freq control word, high byte
Radio_Write_Registers(TI_CCxxx0_FREQ1,    0x44, CC2500);  // Freq control word, mid byte.
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0xEC, CC2500);  // Freq control word, low byte.
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,  0x2D, CC2500);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,  0x3B, CC2500);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,  0x73, CC2500);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,  0x22, CC2500);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,  0xF8, CC2500);  // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_DEVIATN,  0x00, CC2500);  // Modem dev (when FSK mod en)
Radio_Write_Registers(TI_CCxxx0_MCSM1 ,   0x3F, CC2500);  // Main Radio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_MCSM0 ,   0x18, CC2500);  // Main Radio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_FOCCFG,   0x1D, CC2500);  // Freq Offset Compens. Config
Radio_Write_Registers(TI_CCxxx0_BSCFG,    0x1C, CC2500);  //  Bit synchronization config.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2, 0xC7, CC2500);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1, 0x00, CC2500);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0, 0xB2, CC2500);  // AGC control.
Radio_Write_Registers(TI_CCxxx0_FREND1,   0xB6, CC2500);  // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FREND0,   0x10, CC2500);  // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FSCAL3,   0xEA, CC2500);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL2,   0x0A, CC2500);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL1,   0x00, CC2500);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL0,   0x11, CC2500);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSTEST,   0x59, CC2500);  // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_TEST2,    0x88, CC2500);  // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST1,    0x31, CC2500);  // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST0,    0x0B, CC2500);  // Various test settings.   
break;
*/
 }
}



