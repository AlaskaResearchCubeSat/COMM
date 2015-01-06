#include <msp430.h>
#include <ctl_api.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <SDlib.h>
#include "COMM.h"
#include "AX25_EncodeDecode.h"
#include "Radio_functions.h"
#include "COMM_Events.h"
#include "SD-dat.h"

COMM_STAT status;
short beacon_on=1;
CTL_EVENT_SET_t COMM_evt;
char Tx1Buffer[600];
char RxBuffer[600], RxTemp[30];
unsigned int Tx1Buffer_Len, TxBufferPos=0, TxBytesRemaining;
unsigned int RxBuffer_Len=0, RxBufferPos=0, RxBytesRemaining;

unsigned int state;

void sub_events(void *p) __toplevel{
  unsigned int e;
  int i, j, resp;
  unsigned char buf[BUS_I2C_HDR_LEN+sizeof(COMM_STAT)+BUS_I2C_CRC_LEN],*ptr;
  char TestPacket[45];  //THIS IS FOR TESTING ONLY
  unsigned int TestPacket_Len;   //THIS IS FOR TESTING ONLY
  //source and type for SPI data
  char src,type;

  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);

//******************* COMMAND TO POWER OFF??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_OFF){
      //print message
      puts("System Powering Down\r\n");
    }

//******************* COMMAND TO POWER ON??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_ON){
      //print message
      puts("System Powering Up\r\n");
    }

//******************* SEND COMM STATUS TO CDH ***************************
    if(e&SUB_EV_SEND_STAT){
      //send status
      puts("Sending status\r\n");
      //setup packet 
      ptr=BUS_cmd_init(buf,CMD_COMM_STAT);
      //fill in telemetry data
      for(i=0;i<sizeof(COMM_STAT);i++){
        ptr[i]=((unsigned char*)(&status))[i];
      }
      //send command
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(COMM_STAT),0,BUS_I2C_SEND_FOREGROUND);
      ctl_events_set_clear(&COMM_evt,COMM_EVT_STATUS_REQ,0);
      if(resp!=RET_SUCCESS){
        printf("Failed to send status %s\r\n",BUS_error_str(resp));
      }
    }

//**************** NOT SURE HOW THIS IS CALLED ************************
//    if(e&SUB_EV_TIME_CHECK){
//      printf("time ticker = %li\r\n",get_ticker_time());
//    }

// ******************* RECEIVING DATA OVER SPI *************************
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r");
      printf("status = 0X%02X \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC1101));
      //First byte contains sender address
      //Second byte contains data type
      //Both bytes are removed before the data is passed on to COMM
      src=arcBus_stat.spi_stat.rx[0];
      type=arcBus_stat.spi_stat.rx[1];
      printf("src = %d\r\n",src);
      printf("type = %d\r\n",type);

      switch(type){
      case SPI_BEACON_DAT:
        printf("trying to send beacon ON = %d\r\n",beacon_on);
        if(!beacon_on){
         BUS_free_buffer_from_event();
         break;
        }
//        Tx1Buffer_Len=COMM_TXHEADER_LEN+(arcBus_stat.spi_stat.len)+1;   //HeaderLen+(arcbusLen)+1 for carriage return
        Tx1Buffer_Len=COMM_TXHEADER_LEN+15+1;
        //Load up header
        for(i=0;i<COMM_TXHEADER_LEN;i++){                                 //Tx1_Header stored in Tx1Buffer starting at index 0
          Tx1Buffer[i]=__bit_reverse_char(Tx1_Header[i]);                 //AX.25 octets are sent LSB first
        }
//        PrintBuffer(arcBus_stat.spi_stat.rx, arcBus_stat.spi_stat.len);                   //THIS IS FOR TESTING ONLY
        //Load up data
//        for(i=0;i<arcBus_stat.spi_stat.len;i++) {      
        for(i=0;i<15;i++){                 
//          Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(arcBus_stat.spi_stat.rx[i]);  //AX.25 octets are sent LSB first
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(i);
        }
        
//        Tx1Buffer[arcBus_stat.spi_stat.len+COMM_TXHEADER_LEN]=__bit_reverse_char(COMM_CR);  //Add carriage return
        Tx1Buffer[15+COMM_TXHEADER_LEN]=__bit_reverse_char(COMM_CR);  //Add carriage return
        
        BUS_free_buffer_from_event();     //Free Buffer before call to Stuff_Transition_Scramble()

        //**** Create AX.25 packet (needs to include FCS, bit stuffed, flags) ***
        printf("Tx1Buffer_Len=%d  COMM_TXHEADER_LEN=%d\r\n", Tx1Buffer_Len,COMM_TXHEADER_LEN);
        PrintBufferBitInv(Tx1Buffer, Tx1Buffer_Len);                   //THIS IS FOR TESTING ONLY

        CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);             //Generate FCS
        Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);       //Bit stuff - Encode for transitions - Scramble data

//        PrintBuffer(Tx1Buffer, Tx1Buffer_Len);                      //Check to see if everything is correct THIS IS FOR TESTING ONLY
        ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_START,0);       //Send to Radio to transmit
/*
//************** TEST PACKETS - THIS IS FOR TESTING ONLY *****************************
        // CHOOSE WHICH TEST PACKET TO USE
//    Tx1Buffer_Len = sizeof(Packet_NoBit);                //Packet doesn't require bit-stuffing
        Tx1Buffer_Len = sizeof(Packet_WBit);                 //Packet does require bit-stuffing

        for(i=0;i<Tx1Buffer_Len;i++){                          //Fill working space
//            Tx1Buffer[i] = __bit_reverse_char(Packet_NoBit[i]);//Packet doesn't require bit-stuffing
            Tx1Buffer[i] = __bit_reverse_char(Packet_test[i]); //Packet does require bit-stuffing
        }

        CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);       //Generate FCS
//      PrintBufferBitInv(Tx1Buffer, Tx1Buffer_Len);                //Check to see if everything is correct THIS IS FOR TESTING ONLY
        Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);   //Bit stuff - Encode for transitions - Scramble data
//        PrintBufferBitInv(Tx1Buffer, Tx1Buffer_Len);                  //Check to see if everything is correct THIS IS FOR TESTING ONLY
        ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_START,0);       //Send to Radio to transmit
//        RxBytesRemaining = Tx1Buffer_Len % 15;
//        for(i=0;i<Tx1Buffer_Len/15;i++){
//          for(j=0;j<15;j++){
//             RxTemp[j] = Tx1Buffer[i*15+j];
//          }
//        Reverse_Scramble_Transition_Stuff(RxTemp, 15);
//        }
//        for(j=0;j<RxBytesRemaining;j++){
//          RxTemp[j] = Tx1Buffer[i*15+j];
//        }
//        Reverse_Scramble_Transition_Stuff(RxTemp, RxBytesRemaining);

//**************** END OF TEST PACKET TEST *******************************************
*/
        break;
      //other data, write to SD card
      default:
        //write data to SD card
        writeData(src,type,arcBus_stat.spi_stat.rx+2);
        //free buffer
        BUS_free_buffer_from_event();
        break;
      }
    }

    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
    }
  }
}


//handle COMM specific commands don't wait here.
int SUB_parseCmd(unsigned char src, unsigned char cmd, unsigned char *dat, unsigned short len){
  return ERR_UNKNOWN_CMD;
}

void COMM_events(void *p) __toplevel{
  unsigned int e, count;
  int i, resp; 
  char Rx_temp[64];

  //initialize status
    status.CC2500=0x00;     //THIS IS FOR TESTING ONLY

  Reset_Radio(CC1101);                         // Reset Radios

  __delay_cycles(800);  //Wait for radio to be ready before writing registers
// after reset need to wait until radio is ready.....
// cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
// After reset chip is in IDLE state


  Write_RF_Settings(CC1101);            // Write radios Settings
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);


  Radio_Interrupt_Setup();
  Radio_Strobe(TI_CCxxx0_SRX, CC1101);  //Initialize CC1101 in Rx mode

//  printf("Register RSSI 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_RSSI, CC1101));

/*
  printf("Register IOCFG0  (0x06) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_IOCFG0,CC1101));
  printf("Register IOCFG2  (0x00) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_IOCFG2,CC1101));
  printf("Register FIFOTHR (0x07) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FIFOTHR,CC1101));
  printf("Register FSCTRL0 (0x00) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FSCTRL0,CC1101));
  printf("Register FSCTRL1 (0x06) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FSCTRL1,CC1101));
  printf("Register FREQ2   (0x10) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FREQ2,CC1101));
  printf("Register FREQ1   (0xD4) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FREQ1,CC1101));
  printf("Register FREQ0   (0x55) 0x%02x\r\n", Radio_Read_Registers(TI_CCxxx0_FREQ0,CC1101));
  printf("Register MDMCFG4 (0xF8) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MDMCFG4, CC1101));   
  printf("Register MDMCFG3 (0x83) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MDMCFG3, CC1101));
  printf("Register MDMCFG2 (0x03) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MDMCFG2, CC1101));   
  printf("Register MDMCFG1 (0x22) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MDMCFG1, CC1101));   
  printf("Register MDMCFG0 (0xF8) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MDMCFG0, CC1101));
  printf("Register CHANNR  (0x00) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_CHANNR,  CC1101));
  printf("Register DEVIATN (0x15) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_DEVIATN, CC1101));
  printf("Register FREND1  (0x56) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FREND1,  CC1101));
  printf("Register FREND0  (0x10) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FREND0,  CC1101)); 
  printf("Register MCSM1   (0x0F) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MCSM1,   CC1101));
  printf("Register MCSM0   (0x18) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_MCSM0,   CC1101));
  printf("Register FOCCFG  (0x16) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FOCCFG,  CC1101));
  printf("Register BSCFG   (0x6C) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_BSCFG,   CC1101));
  printf("Register AGCCTRL2(0x03) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_AGCCTRL2,CC1101));
  printf("Register AGCCTRL1(0x40) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_AGCCTRL1,CC1101));
  printf("Register AGCCTRL0(0x91) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_AGCCTRL0,CC1101));
  printf("Register FSCAL3  (0xE9) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FSCAL3,  CC1101));
  printf("Register FSCAL2  (0x2A) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FSCAL2,  CC1101));
  printf("Register FSCAL1  (0x00) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FSCAL1,  CC1101));
  printf("Register FSCAL0  (0x1F) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FSCAL0,  CC1101));
  printf("Register FSTEST  (0x59) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_FSTEST,  CC1101));
  printf("Register TEST2   (0x81) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_TEST2,   CC1101));
  printf("Register TEST1   (0x35) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_TEST1,   CC1101));
  printf("Register TEST0   (0x09) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_TEST0,   CC1101));
  printf("Register PKTCTRL1(0x00) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_PKTCTRL1,CC1101));     
  printf("Register PKTCTRL0(0x00) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_PKTCTRL0,CC1101));      
  printf("Register PKTLEN  (0xFF) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_PKTLEN,  CC1101));   
  printf("Register ADDR    (0x00) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_ADDR,    CC1101));
  printf("Register SYNC1   (0x7E) 0x%02x\r\n",Radio_Read_Registers(TI_CCxxx0_SYNC1,   CC1101));
*/

  // Need to wait first for RF on command from CDH.  Nothing happens until we get that command!
  // After RF on command we can send Beacon data.
  // Would like to create some test here where we read radio status and make sure it is in the correct state.

  //init_event
  ctl_events_init(&COMM_evt,0);

  //read SD card and determine the number of bytes stored
  data_setup();

  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&COMM_evt,COMM_EVT_ALL,CTL_TIMEOUT_NONE,0);

    //update status
    if(e&COMM_EVT_STATUS_REQ){
       status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);                  
       status.CC2500++; 	//change if we manage to get the CC2500 radio working
    }

    if(e & CC1101_EV_RX_READ){ //Triggered by GDO0 interrupt     
      // Entering here indicates that the RX FIFO is more than half filed.
      // Need to read RXThrBytes into RXBuffer then move RxBufferPos by RxThrBytes
      // Then wait until interrupt received again.

        Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, Rx_temp, RxThrBytes, CC1101);
        Reverse_Scramble_Transition_Stuff(Rx_temp, RxThrBytes);
        printf("Radio State: 0x%02x \n\r", Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
    }

    if(e&CC1101_EV_TX_START){
      puts("TX Start\r\n");
      state = TX_START;
      TxBufferPos = 0;

      P2IFG &= ~CC1101_GDO2;
      P2IE |= CC1101_GDO2;                                   // Enable Port 2 GDO2
      P6OUT |= RF_SW1;   //Set T/R switch to transmit 

// Switch on the length of the initial packet.  
// If packet is > 256 then radio set up as INFINITE.
// If packet is > 64 and < 256 then radio is set up as Fixed Mode
// If packet is < 64 then radio is set up as Fixed Mode
      // Check for TX FIFO underflow, if yes then flush dat buffer
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
        Radio_Strobe(TI_CCxxx0_SRX,CC1101); // do I need this?
        __delay_cycles(16000); //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101)); 
      }

      TxBytesRemaining = Tx1Buffer_Len;

      if(TxBytesRemaining > 64)
      {
        if(TxBytesRemaining > 256)
        {
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, (Tx1Buffer_Len % 256), CC1101); // Pre-program the packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, CC1101);                // Infinite byte mode
        }
        else
        {
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, CC1101);  // Pre-program packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, CC1101);         // Fixed byte mode
        }
        Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, 64, CC1101); // Write first 64 TX data bytes to TX FIFO
        TxBytesRemaining = TxBytesRemaining - 64;
        TxBufferPos = TxBufferPos + 64;
        state = TX_RUNNING;
      }
      else
      {
        Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, CC1101);  // Pre-program packet length
        Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, CC1101);         // Fixed byte mode
        Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, Tx1Buffer_Len, CC1101); // Write TX data
        TxBytesRemaining = 0;
        TxBufferPos = TxBufferPos+Tx1Buffer_Len;
        state = TX_END;
      }

      Radio_Strobe(TI_CCxxx0_STX, CC1101);                                                  // Set radio state to Tx
   }

    if(e & CC1101_EV_TX_THR)
    {
      printf("TX THR\r\n");        
      // Entering here indicates that the TX FIFO has emptied to below TX threshold.
      // Need to write TXThrBytes (30 bytes) from TXBuffer then move TxBufferPos by TxThrBytes
      // Then wait until interrupt received again or go to TX_END.

	if(TxBytesRemaining > TxThrBytes)
        {
	   Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxThrBytes, CC1101);
           TxBufferPos += TxThrBytes;
           TxBytesRemaining = TxBytesRemaining - TxThrBytes;
           state = TX_RUNNING;
        }
        else
        {
//         Radio_Write_Registers(TI_CCxxx0_FIFOTHR, 0x4F, CC1101); // Change threshold so interrupt triggers on last byte.
           Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, CC1101); // Enter fixed length mode to transmit the last of the bytes
           Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxBytesRemaining, CC1101);
           TxBufferPos += TxBytesRemaining;
           TxBytesRemaining = 0;
           state = TX_END;
        }
    }
          
    if(e & CC1101_EV_TX_END)
    {
      printf("TX End\r\n");
      printf("TxBufferPos = %d\r\n", TxBufferPos);
      // Entering here indicates that the TX FIFO has emptied to the last byte sent
      // No more bytes to send.
      // Need to change interrupts.        

      P6OUT &= ~RF_SW1;          //Set T/R switches to receive
      P2IE &= ~CC1101_GDO2;                                   // Disable Port 2 GDO2 interrupt
      P2IFG &= ~CC1101_GDO2;                                  // Clear flag

      while (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x13){
         __delay_cycles(500);
      }

      Radio_Write_Registers(TI_CCxxx0_PKTLEN,0xFF,CC1101);        //Reset PKTLEN
      Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, CC1101);    // Reset infinite packet length mode set

      // Check for TX FIFO underflow, if yes then flush dat buffer
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
        Radio_Strobe(TI_CCxxx0_SRX,CC1101);
        __delay_cycles(16000); //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101)); 
      }

      // Toggle LED to indicate packet sent
      P7OUT ^= BIT7;
    }

    if(e&COMM_EVT_IMG_DAT)
    {
      puts("Received IMG Data");
      //Need to store IMG data in SD Card so it is ready for transmission.
      //Question as to whether or not to store data as already processed AX.25
      //or raw data ready to be processed.
      //Data will transmit on Radio 2 on command.
    }

    if(e&COMM_EVT_LEDL_DAT)
    {
      puts("Received LEDL Data");
      //Need to store LEDL data in SD Card so it is ready for transmission.
      //Question as to whether or not to store data as already processed AX.25
      //or raw data ready to be processed.
      //Data will transmit on Radio 2 on command.
    }

/*     //Received Potential Ground Command
    if(e&COMM_EVT_GS_DECODE){
      resp=comm_evt_gs_decode();
      if(resp!=RET_SUCCESS){
        printf("%s\r\n",COMM_error_str(resp));
      }
    }
    */

   }

}

void COMM_Setup(void){
//Set up peripherals for COMM MSP
//Radio SPI on P5: P5.1=UCB1SIMO, P5.2=USB1SOMI, P5.3=UCB1CLK
  UCB1CTL1 = UCSWRST;                             // Put UCB1 into reset
  UCB1CTL0 = UCCKPH|UCMSB|UCMST|UCMODE_0|UCSYNC;  // Data Read/Write on Rising Edge
                                                  // MSB first, Master mode, 3-pin SPI
                                                  // Synchronous mode
  UCB1CTL1 |= UCSSEL_2;                           // SMCLK
  UCB1BR0 = 16;                                   // Set frequency divider so SPI runs at 16/16 = 1 MHz
  UCB1BR1 = 0;

  //Radio CS P5.4=CC250_CS, P5.5=CC1101_CS, P5.6=Temp_Sensor1_CS, P5.7=TEMP_Sensor2_CS (outputs)
  //Initial state for CS is High, CS pulled low to initiate SPI
  P5OUT |= CS_1101;                     // Ensure CS for CC1101 is disabled
  P5OUT |= CS_2500;                     // Ensure CS for CC2500 is disabled
  P5OUT |= CS_TEMP1;                    // Ensure CS for temperature sensor 1 is disabled
  P5OUT |= CS_TEMP2;                    // Ensure CS for temperature sensor 2 is disabled
  
  P5DIR |= CS_1101;                     //Set output for CC1101 CS
  P5DIR |= CS_2500;                     //Set output for CC2500 CS
  P5DIR |= CS_TEMP1;
  P5DIR |= CS_TEMP2;
  P5DIR |= RADIO_PIN_SIMO|RADIO_PIN_SCK;

  //Set pins for SPI usage
  P5SEL |= RADIO_PINS_SPI;

  //Bring UCB1 out of reset state
  UCB1CTL1 &= ~UCSWRST;

  //UC1IE = UCB1TXIE|UCB1RXIE;                      //Enable transmit and receive interrupt

//Radio SW P6.0, P6.1 (outputs)
  P6OUT = 0x00;
  P6DIR = RF_SW1|RF_SW2;
  P6SEL = 0;

//SD Card SPI on P3/P5: P3.6=UCA1SIMO, P3.7=UCA2SOMI, P5.0=UCA1CLK
//THIS IS HANDELED IN JESSE'S LIBRARY

//Tx-Rx LED's P7.0-3 (inputs) CURRENTLY USED IN MAIN TO SHOW ADDR ON DEV BOARD.
}

void Radio_Interrupt_Setup(void){ // Enable RX interrupts only!  TX interrupt enabled in TX Start
  // Use GDO0 and GDO2 as interrupts to control TX/RX of radio
  P2DIR = 0;			  // Port 2 configured as inputs (i.e. GDO0 and GDO2 are inputs)
  P2IES = 0;
  P2IES |= CC1101_GDO2;           // GDO0 interrupts on rising edge = 0, GDO2 interrupts on falling edge = 1
  P2IFG = 0;                      // Clear all flags
  P2IE |= CC1101_GDO0;            // Enable GDO0 interrupt only (RX interrupt)
}



void PrintBuffer(char *dat, unsigned int len){
   int i;

   for(i=0;i<len;i++){
      printf("0X%02X ",dat[i]); //print LSB first
      if((i)%16==15){
        printf("\r\n");
      }
    }
    printf("\r\n");
    printf("\r\n");
}

void PrintBufferBitInv(char *dat, unsigned int len){
   int i;

   for(i=0;i<len;i++){
      printf("0X%02X ",__bit_reverse_char(dat[i])); //print MSB first so it is understandable
      if((i)%16==15){
        printf("\r\n");
      }
    }
    printf("\r\n");
    printf("\r\n");
}

