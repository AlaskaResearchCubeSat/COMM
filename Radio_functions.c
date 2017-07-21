// ALL FUNCTIONS USED TO OPERATE THE CC1100 and CC250 Radios
#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "Radio_functions.h"

//char paTable_CC1101[] = {0x84};  //corresponds to +5dBm
//char paTable_CC1101[] = {0xC8};  //corresponds to +7dBm
//char paTable_CC1101[] = {0xC0};  //corresponds to +10dBm 
///char paTable_CC2500[] = {0xFF};

//char paTableLen=1;
int Tx_Flag;

//************************************************************* Radio setup**********************************************************************
// If you use UCAx for SPI look at errata USCI41. UCBUSY bit sticks. this does not occur for UCBx
//***********************************************************************************************************************************************
void radio_SPI_setup(void){
//PORT MAP THE UCA3 TO PORT 3.5,6,7 
//Code was copid from the SD lib code. SD lib code is port mapped for any communication periferial. 
//I changed the MCC to Radio and applied the Radio SPI lines to the #defines in the Radio_functions.h file
//We could try to set this up like SD card so that radios can be accessed on any of the SPI perifials and will still work. 
  //unlock registers
  PMAPKEYID=PMAPKEY;
  //allow reconfiguration
  PMAPCTL|=PMAPRECFG;
  //setup SIMO
  RADIO_PMAP_SIMO=RADIO_PM_SIMO;
  //setup SOMI
  RADIO_PMAP_SOMI=RADIO_PM_SOMI;
  //setup SIMO
  RADIO_PMAP_UCLK=RADIO_PM_UCLK;
  //lock the Port map module
  PMAPKEYID=0;

//Radio SPI on P4: P4.2=UCB1SIMO, P4.1=USB1SOMI, P4.0=UCB1CLK
  UCB1CTLW0 |= UCSWRST;                           // Put UCB1 into reset
  UCB1CTLW0  = UCCKPH|UCMSB|UCMST|UCMODE_0|UCSYNC|UCSSEL_2|UCSWRST;  // Data Read/Write on Rising Edge
                                                  // MSB first, Master mode, 3-pin SPI
                                                  // Synchronous mode
                                                  // SMCLK
  UCB1BRW = 16;                                   // Set frequency divider so SPI runs at 16/16 = 1 MHz
  UCB1CTLW0 &= ~UCSWRST;  //Bring UCB1 out of reset state

  //NOTE Max SPI clk speed is 9 MHz is this done? "CCxxxx_DN_DN503"
  
// ************************************************* PIN setup 
  //Radio CS P5.1=CC2500_CS_1 (ENABLE1), P5.2=CC2500_CS_2 (ENABLE2), 
  //Initial state for CS is High, CS pulled low to initiate SPI
  P5DIR |= CS_CC1101;                        //Set output for CC1101 CS
  P5DIR |= CS_CC2500_1;                     //Set output for CC2500_1 CS

  radio_SPI_desel(CC1101);                 // Ensure CS for CC1101 is disabled
  radio_SPI_desel(CC2500_1);                  // Ensure CS for CC2500_1 is disabled

  P4DIR |= RADIO_PIN_SIMO|RADIO_PIN_SCK;

  //Set pins for SPI usage
  P4SEL0 |= RADIO_PINS_SPI;
}

//******************************************* radio handling functions 

int radio_select; // this is a global var
int set_radio_path(char *radio){
  if (strcmp(radio,"CC1101")==0){
    radio_select=CC1101;    //CC2500_1 = 1
    return 0;
  }
  else if (strcmp(radio,"CC2500_1")==0){
    radio_select=CC2500_1;    //CC2500_2 = 2
    return 0;
  }
  else{
    return -1;
  }
}

int radio_SPI_sel (int radio_select){  // set CS lines for SPI
// NOTE add CC1101 for other radio code
  switch (radio_select){
  case CC1101:
     P5OUT &= ~CS_CC1101;                             // CS enable CC1101
     break;
  case CC2500_1:
     P5OUT &= ~CS_CC2500_1;                           // CS enable CC2500
     break;
  default:
    return -1;
  }
}   
int radio_SPI_desel(int radio_select){
// NOTE add CC1101 for other radio code
  switch (radio_select){
  case CC1101:
     P5OUT |= CS_CC1101;                           // CS enable CC1101
     break;
  case CC2500_1:
     P5OUT |= CS_CC2500_1;                           // CS enable CC2500
     break;
  default:
    return -1;
  }
}

//Function to read a SINGLE byte from the radio registers
/*SPI Accesses --> |R/W|B|A5|A4|A3|A2|A1|A0|
  The R/W bit = 1 --> read, 0 --> write
  B = 1 --> Burst R/W, 0 single R/W
 */
char Radio_Read_Registers(char addr, int radio_select){
  char x;
  
  radio_SPI_sel (radio_select); // set SPI CS
 
  while (!(UCB1IFG & UCTXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_SINGLE);  // Adding 0x80 to address tells the radio to read a single byte
  while (!(UCB1IFG & UCTXIFG));               // Wait for TXBUF ready
  UCB1TXBUF = 0;                               // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                   // Wait for TX to complete
  x = UCB1RXBUF;                               // Read data`

  radio_SPI_desel(radio_select); // de-select SPI CS

  return x;
}

//Function to read a multiple bytes from the radio registers
void Radio_Read_Burst_Registers(char addr, unsigned char *buffer, int count, int radio_select)
{
  char i;

  radio_SPI_sel (radio_select); // set SPI CS

  while (!(UCB1IFG & UCTXIFG));              // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Adding 0xC0 to address tells the radio to read multiple bytes
  while (UCB1STAT & UCBUSY);                  // Wait for TX to complete
  UCB1TXBUF = 0;                              // Dummy write to read 1st data byte
                                              // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  UCB1IFG &= ~UCRXIFG;                       // Clear flag
  while (!(UCB1IFG & UCRXIFG));              // Wait for end of 1st data byte TX
                                              // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB1TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = UCB1RXBUF;                    // Store data from last data RX
    while (!(UCB1IFG & UCRXIFG));            // Wait for RX to finish
  }
  buffer[count-1] = UCB1RXBUF;               // Store last RX byte in buffer
  
    radio_SPI_desel(radio_select);          // de-select SPI CS
}

char Radio_Read_Status(char addr, int radio_select)
{
  char status;

  radio_SPI_sel (radio_select); // set SPI CS

  while (!(UCB1IFG & UCTXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = (addr | TI_CCxxx0_READ_BURST);     // Send address
  while (!(UCB1IFG & UCTXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = 0;                                 // Dummy write so we can read data
  while (UCB1STAT & UCBUSY);                     // Wait for TX to complete
  status = UCB1RXBUF;                            // Read data

  radio_SPI_desel(radio_select);          // de-select SPI CS

  return status;
}

//Function that sends single address to radio initiating a state or mode change 
//(e.g. sending addr 0x34 writes to SRX register initiating radio in RX mode
char Radio_Strobe(char strobe, int radio_select)
{
  char status;

  radio_SPI_sel (radio_select);                 // set SPI CS

  while (!(UCB1IFG & UCTXIFG));                  // Wait for TXBUF ready
  UCB1TXBUF = strobe;                             // Send strobe
                                                  // Strobe addr is now being TX'ed
  while (UCB1STAT & UCBUSY);                      // Wait for TX to complete
  status = UCB1RXBUF;                            // Read data

  radio_SPI_desel(radio_select);                // de-select SPI CS

}

//Function to write a single byte to the radio registers
void Radio_Write_Registers(char addr, char value, int radio_select)
{

  radio_SPI_sel (radio_select);                 // set SPI CS

  while (!(UCB1IFG & UCTXIFG));            // Wait for TXBUF ready
  UCB1TXBUF = addr;                         // Send address
  while (!(UCB1IFG & UCTXIFG));            // Wait for TXBUF ready
  UCB1TXBUF = value;                        // Send data
  while (UCB1STAT & UCBUSY);                // Wait for TX to complete

  radio_SPI_desel(radio_select);                // de-select SPI CS

  
}

//Function to write multiple bytes to the radio registers
void Radio_Write_Burst_Registers(char addr, unsigned char *buffer, int count, int radio_select)
{
  int i;

  radio_SPI_sel (radio_select);                 // set SPI CS
  
  while (!(UCB1IFG & UCTXIFG));                 // Wait for TXBUF ready
  UCB1TXBUF = addr | TI_CCxxx0_WRITE_BURST;      // Adding 0x40 to address tells radio to perform burst write rather than single byte write
  for (i = 0; i < count; i++)
  {
    while (!(UCB1IFG & UCTXIFG));              // Wait for TXBUF ready
    UCB1TXBUF = buffer[i];                      // Send data
  }
  while (UCB1STAT & UCBUSY);                    // Wait for TX to complete
  
  radio_SPI_desel(radio_select);                // de-select SPI CS
}

void Reset_Radio(int radio_select)
{
  switch (radio_select){
  case CC1101:
    P5OUT |= CS_CC1101;               //Toggle CS with delays to power up radio
    TI_CC_Wait(30);
    P5OUT &= ~CS_CC1101;
    TI_CC_Wait(30);
    P5OUT |= CS_CC1101;
    TI_CC_Wait(45);

    P5OUT &= ~CS_CC1101;              // CS enable
    while (!(UCB1IFG & UCTXIFG));  // Wait for TXBUF ready
    UCB1TXBUF = TI_CCxxx0_SRES;     // Send strobe
                                    // Strobe addr is now being TX'ed
    while (UCB1STAT & UCBUSY);      // Wait for TX to complete
    P5OUT |= CS_CC1101;               // CS disable
    break;
  case CC2500_1:
    P5OUT |= CS_CC2500_1;               //Toggle CS with delays to power up radio
    TI_CC_Wait(30);
    P5OUT &= ~CS_CC2500_1;
    TI_CC_Wait(30);
    P5OUT |= CS_CC2500_1;
    TI_CC_Wait(45);

    P5OUT &= ~CS_CC2500_1;              // CS enable
    while (!(UCB1IFG & UCTXIFG));  // Wait for TXBUF ready
    UCB1TXBUF = TI_CCxxx0_SRES;     // Send strobe
                                    // Strobe addr is now being TX'ed
    while (UCB1STAT & UCBUSY);      // Wait for TX to complete
    P5OUT |= CS_CC2500_1;               // CS disable
    break;
  }

  P1IFG = 0;                        //Clear flags that were set 
}

void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

void RF_Send_Packet(unsigned char *TxBuffer, int size, int radio_select)
{
  Radio_Write_Registers(TI_CCxxx0_PKTLEN, size, radio_select);
  Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, TxBuffer, size, radio_select); // Write TX data
  Radio_Strobe(TI_CCxxx0_STX, radio_select);                                   // Change state to TX, initiate data transfer
  Tx_Flag = 1;
}

void Write_RF_Settings(int radio_select)
{
switch (radio_select){

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
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0x63, CC1101);
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
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, CC1101);      // Infinite packet length mode set and whitening is off 
Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, CC1101);      // Packet length set for fixed packet length mode
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x00, CC1101);
Radio_Write_Registers(TI_CCxxx0_SYNC1,    0x7E, CC1101);      // SYNC Word High Byte (SYNC0 sync word low byte)
Radio_Write_Registers(TI_CCxxx0_PATABLE,  0x84, CC1101);     //Set PA to 5dBm.
break;


case CC2500_1:
// Write CC2500_1 register settings
//baud : 38.4 kbs 
// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 1 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 38.085938 kHz
// Return state:  Return to RX state upon leaving either TX or RX
// Datarate = 2.39897 kBaud
// Modulation = (7) FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2433.000000 MHz
// Channel spacing = 199.950000 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock

Radio_Write_Registers(TI_CCxxx0_IOCFG2,   0x02, CC2500_1);  // GDO2 output pin config.
Radio_Write_Registers(TI_CCxxx0_IOCFG0,   0x00, CC2500_1);  // GDO0 output pin config.
Radio_Write_Registers(TI_CCxxx0_FIFOTHR,  0x07, CC2500_1);  // FIFO Threshold: 21 byte in TX FIFO and 44 in RX FIFO

Radio_Write_Registers(TI_CCxxx0_PKTLEN,   0xFF, CC2500_1); // Packet length.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL1, 0x04, CC2500_1); // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x05, CC2500_1); // Packet automation control.
Radio_Write_Registers(TI_CCxxx0_ADDR,     0x01, CC2500_1); // Device address.
Radio_Write_Registers(TI_CCxxx0_CHANNR,   0x00, CC2500_1); // Channel number.
Radio_Write_Registers(TI_CCxxx0_FSCTRL1,  0x08, CC2500_1); // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FSCTRL0,  0x00, CC2500_1); // Freq synthesizer control.
Radio_Write_Registers(TI_CCxxx0_FREQ2,    0x5C, CC2500_1); // Freq control word, high byte
Radio_Write_Registers(TI_CCxxx0_FREQ1,    0x4E, CC2500_1); // Freq control word, mid byte.
Radio_Write_Registers(TI_CCxxx0_FREQ0,    0xC3, CC2500_1); // Freq control word, low byte.
Radio_Write_Registers(TI_CCxxx0_MDMCFG4,  0x2B, CC2500_1); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG3,  0xF8, CC2500_1); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG2,  0x03, CC2500_1); // Modem configuration. FSK
Radio_Write_Registers(TI_CCxxx0_MDMCFG1,  0x22, CC2500_1); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_MDMCFG0,  0xF8, CC2500_1); // Modem configuration.
Radio_Write_Registers(TI_CCxxx0_DEVIATN,  0x50, CC2500_1); // Modem dev (when FSK mod en) for FSK(47.607 kHz Deviation)
Radio_Write_Registers(TI_CCxxx0_MCSM1 ,   0x30, CC2500_1); // MainRadio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_MCSM0 ,   0x18, CC2500_1); // MainRadio Cntrl State Machine
Radio_Write_Registers(TI_CCxxx0_FOCCFG,   0x1D, CC2500_1); // Freq Offset Compens. Config
Radio_Write_Registers(TI_CCxxx0_BSCFG,    0x1C, CC2500_1); // Bit synchronization config.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL2, 0x00, CC2500_1); // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL1, 0x58, CC2500_1); // AGC control.
Radio_Write_Registers(TI_CCxxx0_AGCCTRL0, 0x91, CC2500_1); // AGC control.
Radio_Write_Registers(TI_CCxxx0_FREND1,   0x00, CC2500_1); // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FREND0,   0x10, CC2500_1); // Front end RX configuration.
Radio_Write_Registers(TI_CCxxx0_FSCAL3,   0xA9, CC2500_1); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL2,   0x0A, CC2500_1); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL1,   0x00, CC2500_1); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSCAL0,   0x11, CC2500_1); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_FSTEST,   0x59, CC2500_1); // Frequency synthesizer cal.
Radio_Write_Registers(TI_CCxxx0_TEST2,    0x88, CC2500_1); // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST1,    0x31, CC2500_1); // Various test settings.
Radio_Write_Registers(TI_CCxxx0_TEST0,    0x0B, CC2500_1); // Various test settings.
break;
 }
}



