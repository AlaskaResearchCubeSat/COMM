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
short beacon_on=0, beacon_flag=0;
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
  //source and type for SPI data
  char src,type;

  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);

//******************* COMMAND TO POWER OFF??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_OFF){
      puts("System Powering Down\r\n");                                             //print message
      beacon_on = 0;
    }

//******************* COMMAND TO POWER ON??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_ON){
      puts("System Powering Up\r\n");                                               //print message
      beacon_on = 1;
    }

//******************* SEND COMM STATUS TO CDH ***************************
    if(e&SUB_EV_SEND_STAT){
      puts("Sending status\r\n");                                                     //send status
      ptr=BUS_cmd_init(buf,CMD_COMM_STAT);                                            //setup packet
      for(i=0;i<sizeof(COMM_STAT);i++){                                               //fill in telemetry data
        ptr[i]=((unsigned char*)(&status))[i];
      }
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(COMM_STAT),0,BUS_I2C_SEND_FOREGROUND);  //send command
      ctl_events_set_clear(&COMM_evt,COMM_EVT_STATUS_REQ,0);                          //call status update
      if(resp!=RET_SUCCESS){
        printf("Failed to send status %s\r\n",BUS_error_str(resp));
      }
    }

// ******************* RECEIVING DATA OVER SPI *************************
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r");
      status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC1101);
      printf("status.CC1101 = 0X%02X \r\n",status.CC1101);
      //First byte contains sender address
      //Second byte contains data type
      //Both bytes are removed before the data is passed on to COMM
      src=arcBus_stat.spi_stat.rx[0];
      type=arcBus_stat.spi_stat.rx[1];
      printf("src = %d\r\n",src);
      printf("type = %d\r\n",type);

      switch(type){
      case SPI_BEACON_DAT:
        printf("Trying to send beacon ON = %d FLAG = %d\r\n",beacon_on, beacon_flag);
        if(!beacon_on){
         BUS_free_buffer_from_event();
         break;
        }
        for(i=0;i<COMM_TXHEADER_LEN;i++){                                           //LOAD UP HEADER
          Tx1Buffer[i]=__bit_reverse_char(Tx1_Header[i]);                             //AX.25 octets are sent LSB first
        }

        if(!beacon_flag){                                                            //SEND HELLO MESSAGE
          Tx1Buffer_Len=COMM_TXHEADER_LEN+sizeof(Hello)+1;                            //Set length of message
          for(i=0;i<sizeof(Hello);i++){                 
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(Hello[i]);              //load message after header
          }
        } else {                                                                    //SEND STATUS MESSAGE
          Tx1Buffer_Len=COMM_TXHEADER_LEN+(arcBus_stat.spi_stat.len)+1;               //Set length of message: HeaderLen+(arcbusLen)+1 for carriage return
          Tx1Buffer[COMM_TXHEADER_LEN]=__bit_reverse_char(0x42);                      //TALK TO JESSE ABOUT WHY I CANT DEFINE SPI_BEACON_DAT = 0x42!
          for(i=1;i<arcBus_stat.spi_stat.len;i++) {                                   //load message after header
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(arcBus_stat.spi_stat.rx[i]);
          }
        }
        Tx1Buffer[Tx1Buffer_Len-1]=__bit_reverse_char(COMM_CR);                     //Add carriage return
        
        BUS_free_buffer_from_event();                                               //Free Buffer before call to Stuff_Transition_Scramble()

        //**** Create AX.25 packet (needs to include FCS, bit stuffed, flags) ***
        printf("Tx1Buffer_Len=%d  COMM_TXHEADER_LEN=%d\r\n", Tx1Buffer_Len,COMM_TXHEADER_LEN);
        PrintBufferBitInv(Tx1Buffer, Tx1Buffer_Len);                              //THIS IS FOR TESTING ONLY

        CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);                           //Generate FCS
        Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);                     //Bit stuff - Encode for transitions - Scramble data
        ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_START,0);                     //Send to Radio to transmit
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
  int i;
  
  switch(cmd){
    case CMD_BEACON_ON:
     beacon_flag = 1;
     return RET_SUCCESS;
  }
  return ERR_UNKNOWN_CMD;
}

void COMM_events(void *p) __toplevel{
  unsigned int e, count;
  int i, resp; 

  //initialize status
    status.CC2500=0x00;                          //THIS IS FOR TESTING ONLY

    Reset_Radio(CC1101);                         // Reset Radios

  __delay_cycles(800);                          //Wait for radio to be ready before writing registers
                                                // after reset need to wait until radio is ready.....
                                                // cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
                                                // After reset chip is in IDLE state
  Write_RF_Settings(CC1101);                    // Write radios Settings
  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);
  Radio_Interrupt_Setup();
  Radio_Strobe(TI_CCxxx0_SRX, CC1101);          //Initialize CC1101 in Rx mode

  // Need to wait first for RF on command from CDH.  Nothing happens until we get that command!
  // After RF on command we can send Beacon data.
  // Would like to create some test here where we read radio status and make sure it is in the correct state.


  //read SD card and determine the number of bytes stored
  data_setup();

  ctl_events_init(&COMM_evt,0);                 //Initialize Event

  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&COMM_evt,COMM_EVT_ALL,CTL_TIMEOUT_NONE,0);

    //update status
    if(e&COMM_EVT_STATUS_REQ){
       status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);                  
    }

    if(e & CC1101_EV_RX_READ){                  //READ RX FIFO
      // Triggered by GDO0 interrupt     
      // Entering here indicates that the RX FIFO is more than half filed.
      // Need to read RXThrBytes into RXBuffer then move RxBufferPos by RxThrBytes
      // Then wait until interrupt received again.
        Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxTemp, RxThrBytes, CC1101);
        Reverse_Scramble_Transition_Stuff(RxTemp, RxThrBytes);
        status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);
        printf("Radio State: 0x%02x \n\r", status.CC1101);
    }

    if(e&CC1101_EV_TX_START){                 //INITIALIZE TX START
      puts("TX Start\r\n");
      state = TX_START;
      TxBufferPos = 0;

      P2IFG &= ~CC1101_GDO2;
      P2IE |= CC1101_GDO2;                    // Enable Port 2 GDO2
      P6OUT |= RF_SW1;                        //Set T/R switch to transmit 

// Switch on the length of the initial packet.  
// If packet is > 256 then radio set up as INFINITE.
// If packet is > 64 and < 256 then radio is set up as Fixed Mode
// If packet is < 64 then radio is set up as Fixed Mode

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
        Radio_Strobe(TI_CCxxx0_SRX,CC1101); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
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
      ctl_timeout_wait(ctl_get_current_time()+26);  //25 ms delay to flush 30 possible remaining bytes.  Before we turn off power amplifier
      P6OUT &= ~RF_SW1;                             //Set T/R switches to receive
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



   } //end for loop

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

