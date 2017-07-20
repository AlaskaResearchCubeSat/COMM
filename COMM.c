#include <msp430.h>
#include <MSP430F6779A.h>
#include <ctl.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <SDlib.h>
#include "COMM.h"
#include "COMM_Events.h"
#include "Radio_functions.h"
#include "i2c.h"
#include "AX25_EncodeDecode.h"


COMM_STAT status;
CTL_EVENT_SET_t COMM_evt;

short beacon_on=0, beacon_flag=0,data_mode=TX_DATA_BUFFER;
unsigned char data_seed, IMG_Blk, Tx1Buffer[600], RxBuffer[600], RxTemp[30];
unsigned int Tx1Buffer_Len, TxBufferPos=0, TxBytesRemaining, RxBuffer_Len=0, RxBufferPos=0, RxBytesRemaining, state, sec = 0;

/****************************************************** Subsystem Events ******************************************************************************
//flags for events handled by the subsystem
enum{SUB_EV_PWR_OFF=(1<<0),SUB_EV_PWR_ON=(1<<1),SUB_EV_SEND_STAT=(1<<2),SUB_EV_SPI_DAT=(1<<3),
     SUB_EV_SPI_ERR_CRC=(1<<4),SUB_EV_SPI_ERR_BUSY=(1<<5),SUB_EV_ASYNC_OPEN=(1<<6),SUB_EV_ASYNC_CLOSE=(1<<7),
     SUB_EV_INT_0=(1<< 8),SUB_EV_INT_1=(1<< 9),SUB_EV_INT_2=(1<<10),SUB_EV_INT_3=(1<<11),
     SUB_EV_INT_4=(1<<12),SUB_EV_INT_5=(1<<13),SUB_EV_INT_6=(1<<14),SUB_EV_INT_7=(1<<15)
     };
 
//******************************************************************************************************************************************************/
void sub_events(void *p) __toplevel{ // note most of this setup is taken care of in ARCbus.h
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
//NOTE when I2C commands are fixed this can be updated for use 
  /*  if(e&SUB_EV_SEND_STAT){
      puts("Sending status\r\n");                                                     //send status
      ptr=BUS_cmd_init(buf,CMD_COMM_STAT);                                            //setup packet
      for(i=0;i<sizeof(COMM_STAT);i++){                                               //fill in telemetry data
        ptr[i]=((unsigned char*)(&status))[i];
      }
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(COMM_STAT),0,BUS_I2C_SEND_FOREGROUND);  //send command
      ctl_events_set_clear(&COMM_sys_events,COMM_sys_events_STATUS_REQ,0);                          //call status update
      if(resp!=RET_SUCCESS){
        printf("Failed to send status %s\r\n",BUS_error_str(resp));
      }
    }
*/
// ******************* RECEIVING DATA OVER SPI *************************
//TODO update this function to save data upon SPI transfer. 
    if(e&SUB_EV_SPI_DAT){
      //puts("SPI data recived:\r");
      printf("SUB_EV_SPI_DAT called\r\n");
      status.CC2500_1 = Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC2500_1);
      printf("status.CC2500_1 = 0X%02X \r\n",status.CC2500_1);
      //First byte contains data type
      //Second byte contains sender address
      //Both bytes are removed before the data is passed on to COMM
      type=arcBus_stat.spi_stat.rx[0]; // this comes in the sent SPI packet 
      src=arcBus_stat.spi_stat.rx[1]; 
      printf("SPI: type = 0x%02x, src = 0x%02x\r\n",type, src);
      printf("Trying to send beacon ON = %d FLAG = %d\r\n",beacon_on, beacon_flag);
      //PrintBuffer(arcBus_stat.spi_stat.rx, arcBus_stat.spi_stat.len);  //TEST

      switch(type){
      case SPI_BEACON_DAT:// SPI_BEACON_DAT lives in ARCbus.h and = 'B'
        break;
      //other data, write to SD card
      default:
        //write data to SD card
        //TODO FIX THIS 
        //writeSD_Data(src,type,arcBus_stat.spi_stat.rx+2);
        if(src==BUS_ADDR_IMG && *(unsigned short*)(arcBus_stat.spi_stat.rx+2) == 0x990F){
          IMG_Blk = arcBus_stat.spi_stat.rx[5];
        }
        
  
        ctl_events_set_clear(&ev_SPI_data,SPI_EV_DATA_REC,0);
        //free buffer
        BUS_free_buffer_from_event();
        break;
      }
    }

    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
      type=arcBus_stat.spi_stat.rx[0];
      src=arcBus_stat.spi_stat.rx[1];
      printf("SPI: type = 0x%02x, src = 0x%02x\r\n",type, src);
      printf("Trying to send beacon ON = %d FLAG = %d\r\n",beacon_on, beacon_flag);
    }
  } 
}

//**************************************************************** radio commands ******************************************************************
//
//**************************************************************************************************************************************************
//gen data for transmit , random or pattern  
unsigned char tx_data_gen(unsigned char *dest,unsigned short size,int mode,unsigned char seed){
  int i;
  for(i=0;i<=size;i++){
    switch(mode){
      case TX_DATA_PATTERN:  //101... packet 
              dest[i]=seed;  // uses seed as patten              
      break;
      case TX_DATA_RANDOM:  //rand packet 
        seed=(seed>>1)^(-(seed&1)&0xB8);
        dest[i]=seed;
      break;
    }
  }
  return seed;
}


//handle COMM specific commands don't wait here.
int SUB_parseCmd(unsigned char src, unsigned char cmd, unsigned char *dat, unsigned short len){
  int i;
  
  switch(cmd){
    case CMD_BEACON_ON_OFF:
     beacon_flag = 1;
     return RET_SUCCESS;
  }
  return ERR_UNKNOWN_CMD;
}

//************************************************************************** COMM Events *******************************************************************************
//
//**********************************************************************************************************************************************************************
void COMM_events(void *p) __toplevel{
  unsigned int e, count;
  int i, resp; 
// NOTE should we clear flags or does rest do this ? 
    Reset_Radio(CC2500_1);                 // Reset Radios/initialize status
    Reset_Radio(CC1101);                 // Reset Radios/initialize status


  __delay_cycles(800);                         // Wait for radio to be ready before writing registers.cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
                                               // After reset chip is in IDLE state
  Write_RF_Settings(CC1101);             // Write radios Settings
  Write_RF_Settings(CC2500_1);             // Write radios Settings

  //Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);
  Radio_Interrupt_Setup();
    
  //TODO Need to set up two radio setups. Might have to have them done separtely  
  Radio_Strobe(TI_CCxxx0_SRX, CC2500_1);          //Initialize CCxxxx in Rx mode
  Radio_Strobe(TI_CCxxx0_SRX, CC1101);          //Initialize CCxxxx in Rx mode

  //NOTE in flight code this should be called after insertion delay timer
  COMM_beacon_setup();                            // start beacon timer (dose not start transmission)

  ctl_events_init(&COMM_evt,0);                 //Initialize Event

  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&COMM_evt,COMM_EVT_ALL,CTL_TIMEOUT_NONE,0);

    //******************************************************************************************* COMM_EVT_STATUS_REQ
    if(e&COMM_EVT_STATUS_REQ){
//TODO fix this for multiple radios and figure out if it needs to be used 
        status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);
        status.CC2500_1 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1);
               //check radio status
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer  CC2500_1
      {
        Radio_Strobe(TI_CCxxx0_SFRX,CC2500_1); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC2500_1);
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1));
      }
        status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);
               //check radio status
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer   CC1101
      {
        Radio_Strobe(TI_CCxxx0_SFRX,CC1101); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
      }
    }
//***********************************************Radio RX/TX events**************************************************

//******************************************************************************************* COMM_EVT_CC1101_RX_READ
 if(e & COMM_EVT_CC1101_RX_READ){                  //READ RX FIFO
      // Triggered by GDO0 interrupt     
      // Entering here indicates that the RX FIFO is more than half filed.
      // Need to read RXThrBytes into RXBuffer then move RxBufferPos by RxThrBytes
      // Then wait until interrupt received again.
      radio_select = CC1101;  // sel radio
      Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxTemp, RxThrBytes, radio_select);
      status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select);
      printf("Radio State: 0x%02x \n\rCC1101 RX.\r\n", status.CC1101);
    }
  //******************************************************************************************* COMM_EVT_CC1101_TX_START
    if(e&COMM_EVT_CC1101_TX_START){                 //INITIALIZE TX START
      state = TX_START; 
      TxBufferPos = 0;
      radio_select = CC1101;  // sel radio
      P1IE |= CC1101_GDO2;
      // "package" beacon data 
        for(i=0;i<COMM_TXHEADER_LEN;i++){                                             //LOAD UP HEADER
          Tx1Buffer[i]=__bit_reverse_char(Tx1_Header[i]);                             //AX.25 octets are sent LSB first
        }

        if(!beacon_flag){                                                                        //SEND HELLO MESSAGE (predefined) if flag is low
          Tx1Buffer_Len=COMM_TXHEADER_LEN+sizeof(Packet_WBitshort);                            //Set length of message
          for(i=0;i<sizeof(Packet_WBitshort);i++){                 
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(Packet_WBitshort[i]);              //load message after header
          }
        } 
        else {                                                                      //SEND STATUS MESSAGE from SPI stuff
          Tx1Buffer_Len=COMM_TXHEADER_LEN+(arcBus_stat.spi_stat.len)+1;               //Set length of message: HeaderLen+(arcbusLen)+1 for carriage return
          for(i=0;i<arcBus_stat.spi_stat.len;i++) {                                   //load message after header
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(arcBus_stat.spi_stat.rx[i]);
          }
                  Tx1Buffer[Tx1Buffer_Len-1]=__bit_reverse_char(COMM_CR);                     //Add carriage return :Note this is included in prefabbed packets not statuses from CDH
        }
        
        //**** Create AX.25 packet (needs to include FCS, bit stuffed, flags) ***
        printf("Tx1Buffer_Len=%d  COMM_TXHEADER_LEN=%d\r\n", Tx1Buffer_Len,COMM_TXHEADER_LEN);

        CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);                           //Generate FCS
        Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);                     //Bit stuff - Encode for transitions - Scramble data

// Switch on the length of the initial packet.  
// If packet is > 256 then radio set up as INFINITE.
// If packet is > 64 and < 256 then radio is set up as Fixed Mode
// If packet is < 64 then radio is set up as Fixed Mode

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFRX,radio_select); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        Radio_Strobe(TI_CCxxx0_SRX,radio_select); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }

      if(data_mode==TX_DATA_BUFFER){
        TxBytesRemaining = Tx1Buffer_Len;

        if(TxBytesRemaining > 64)
        {
          if(TxBytesRemaining > 256)
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, (Tx1Buffer_Len % 256), radio_select); // Pre-program the packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          }
          else
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          }
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          TxBytesRemaining = TxBytesRemaining - 64;
          TxBufferPos = TxBufferPos + 64;
          state = TX_RUNNING;
        }
        else
        {
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, Tx1Buffer_Len, radio_select); // Write TX data
          TxBytesRemaining = 0;
          TxBufferPos = TxBufferPos+Tx1Buffer_Len;
          state = TX_END;
        }
      }else{
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, 1, radio_select); // Pre-program the packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          //fill buffer with data
          data_seed=tx_data_gen(Tx1Buffer,64,data_mode,data_seed);
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          state = TX_RUNNING;
      }

      Radio_Strobe(TI_CCxxx0_STX, radio_select);                                                  // Set radio state to Tx
   }

  //******************************************************************************************* COMM_EVT_CC1101_TX_THR
    if(e & COMM_EVT_CC1101_TX_THR)
    {
      //printf("TX THR TxBytesRemaining = %d\r\n", TxBytesRemaining);        
      // Entering here indicates that the TX FIFO has emptied to below TX threshold.
      // Need to write TXThrBytes (30 bytes) from TXBuffer then move TxBufferPos by TxThrBytes
      // Then wait until interrupt received again or go to TX_END.
      radio_select = CC1101;  // sel radio
      P1IE |= CC1101_GDO2;

      if(data_mode==TX_DATA_BUFFER){
          if(TxBytesRemaining > TxThrBytes)
          {
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxThrBytes, radio_select);
             TxBufferPos += TxThrBytes;
             TxBytesRemaining = TxBytesRemaining - TxThrBytes;
             state = TX_RUNNING;
          }
          else
          {
             Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select); // Enter fixed length mode to transmit the last of the bytes
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxBytesRemaining, radio_select);
             TxBufferPos += TxBytesRemaining;
             TxBytesRemaining = 0;
             state = TX_END;
          }
        }
        else 
        {
         data_seed=tx_data_gen(Tx1Buffer,TxThrBytes,data_mode,data_seed);
         Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, TxThrBytes, radio_select);
         state = TX_RUNNING;
        }
    }

  //******************************************************************************************* COMM_EVT_CC1101_TX_END        
    if(e & COMM_EVT_CC1101_TX_END)
    {
      // Entering here indicates that the TX FIFO has emptied to the last byte sent
      // No more bytes to send.
      // Need to change interrupts.     
      radio_select = CC1101;  
      P7OUT ^= BIT5;// debug LED

      ctl_timeout_wait(ctl_get_current_time()+26);  //25 ms delay to flush 30 possible remaining bytes.  Before we turn off power amplifier
     P1IE &= ~CC1101_GDO2;                         // Disable Port 2 GDO2 interrupt
     P1IFG &= ~CC1101_GDO2;                        // Clear flag

      while (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x13) // 0x13 is TX mode
      {
         __delay_cycles(500);
               printf("TX Loop!\r\n");

      }

      Radio_Write_Registers(TI_CCxxx0_PKTLEN,0xFF,radio_select);        //Reset PKTLEN
      Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);    //Reset infinite packet length mode set
      printf("TX End\r\n");
      printf("TxBufferPos = %d\r\n", TxBufferPos);

      // Check for TX FIFO underflow, if yes then flush dat buffer
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        Radio_Strobe(TI_CCxxx0_SRX,radio_select);
        __delay_cycles(16000); //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select)); 
      }
    }

    // TODO add in and fix once we get one radio transmitting and receiving. 

  //******************************************************************************************* COMM_EVT_CC2500_1_RX_READ
    if(e & COMM_EVT_CC2500_1_RX_READ){                  //READ RX FIFO
      // Triggered by GDO0 interrupt     
      // Entering here indicates that the RX FIFO is more than half filed.
      // Need to read RXThrBytes into RXBuffer then move RxBufferPos by RxThrBytes
      // Then wait until interrupt received again.
      radio_select = CC2500_1;  // sel radio
      Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxTemp, RxThrBytes, radio_select);
      status.CC2500_1 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select);
      printf("Radio State: 0x%02x \n\rCC2500_1 RX.\r\n", status.CC2500_1);
    }
  //******************************************************************************************* COMM_EVT_CC2500_1_TX_START
    if(e&COMM_EVT_CC2500_1_TX_START){                 //INITIALIZE TX START
      state = TX_START; 
      TxBufferPos = 0;
      radio_select = CC2500_1;  // sel radio
      P1IE |= CC1101_GDO2;
      // "package" beacon data 
        for(i=0;i<COMM_TXHEADER_LEN;i++){                                             //LOAD UP HEADER
          Tx1Buffer[i]=__bit_reverse_char(Tx1_Header[i]);                             //AX.25 octets are sent LSB first
        }

        if(!beacon_flag){                                                                        //SEND HELLO MESSAGE (predefined) if flag is low
          Tx1Buffer_Len=COMM_TXHEADER_LEN+sizeof(Packet_WBitshort);                            //Set length of message
          for(i=0;i<sizeof(Packet_WBitshort);i++){                 
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(Packet_WBitshort[i]);              //load message after header
          }
        } 
        else {                                                                      //SEND STATUS MESSAGE from SPI stuff
          Tx1Buffer_Len=COMM_TXHEADER_LEN+(arcBus_stat.spi_stat.len)+1;               //Set length of message: HeaderLen+(arcbusLen)+1 for carriage return
          for(i=0;i<arcBus_stat.spi_stat.len;i++) {                                   //load message after header
            Tx1Buffer[i+COMM_TXHEADER_LEN]=__bit_reverse_char(arcBus_stat.spi_stat.rx[i]);
          }
                  Tx1Buffer[Tx1Buffer_Len-1]=__bit_reverse_char(COMM_CR);                     //Add carriage return :Note this is included in prefabbed packets not statuses from CDH
        }
        
        //**** Create AX.25 packet (needs to include FCS, bit stuffed, flags) ***
        printf("Tx1Buffer_Len=%d  COMM_TXHEADER_LEN=%d\r\n", Tx1Buffer_Len,COMM_TXHEADER_LEN);

        CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);                           //Generate FCS
        Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);                     //Bit stuff - Encode for transitions - Scramble data

// Switch on the length of the initial packet.  
// If packet is > 256 then radio set up as INFINITE.
// If packet is > 64 and < 256 then radio is set up as Fixed Mode
// If packet is < 64 then radio is set up as Fixed Mode

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFRX,radio_select); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        Radio_Strobe(TI_CCxxx0_SRX,radio_select); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }

      if(data_mode==TX_DATA_BUFFER){
        TxBytesRemaining = Tx1Buffer_Len;

        if(TxBytesRemaining > 64)
        {
          if(TxBytesRemaining > 256)
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, (Tx1Buffer_Len % 256), radio_select); // Pre-program the packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          }
          else
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          }
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          TxBytesRemaining = TxBytesRemaining - 64;
          TxBufferPos = TxBufferPos + 64;
          state = TX_RUNNING;
        }
        else
        {
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, Tx1Buffer_Len, radio_select); // Write TX data
          TxBytesRemaining = 0;
          TxBufferPos = TxBufferPos+Tx1Buffer_Len;
          state = TX_END;
        }
      }else{
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, 1, radio_select); // Pre-program the packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          //fill buffer with data
          data_seed=tx_data_gen(Tx1Buffer,64,data_mode,data_seed);
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          state = TX_RUNNING;
      }

      Radio_Strobe(TI_CCxxx0_STX, radio_select);                                                  // Set radio state to Tx
   }

  //******************************************************************************************* COMM_EVT_CC2500_1_TX_THR
    if(e & COMM_EVT_CC2500_1_TX_THR)
    {
      //printf("TX THR TxBytesRemaining = %d\r\n", TxBytesRemaining);        
      // Entering here indicates that the TX FIFO has emptied to below TX threshold.
      // Need to write TXThrBytes (30 bytes) from TXBuffer then move TxBufferPos by TxThrBytes
      // Then wait until interrupt received again or go to TX_END.
      radio_select = CC2500_1;  // sel radio
      P1IE |= CC1101_GDO2;

      if(data_mode==TX_DATA_BUFFER){
          if(TxBytesRemaining > TxThrBytes)
          {
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxThrBytes, radio_select);
             TxBufferPos += TxThrBytes;
             TxBytesRemaining = TxBytesRemaining - TxThrBytes;
             state = TX_RUNNING;
          }
          else
          {
             Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select); // Enter fixed length mode to transmit the last of the bytes
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxBytesRemaining, radio_select);
             TxBufferPos += TxBytesRemaining;
             TxBytesRemaining = 0;
             state = TX_END;
          }
        }
        else 
        {
         data_seed=tx_data_gen(Tx1Buffer,TxThrBytes,data_mode,data_seed);
         Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, TxThrBytes, radio_select);
         state = TX_RUNNING;
        }
    }

  //******************************************************************************************* COMM_EVT_CC2500_1_TX_END        
    if(e & COMM_EVT_CC2500_1_TX_END)
    {
      // Entering here indicates that the TX FIFO has emptied to the last byte sent
      // No more bytes to send.
      // Need to change interrupts.     
      radio_select = CC2500_1;  
      P7OUT ^= BIT5;// debug LED

      ctl_timeout_wait(ctl_get_current_time()+26);  //25 ms delay to flush 30 possible remaining bytes.  Before we turn off power amplifier
     P1IE &= ~CC1101_GDO2;                         // Disable Port 2 GDO2 interrupt
     P1IFG &= ~CC1101_GDO2;                        // Clear flag

      while (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x13) // 0x13 is TX mode
      {
         __delay_cycles(500);
               printf("TX Loop!\r\n");

      }

      Radio_Write_Registers(TI_CCxxx0_PKTLEN,0xFF,radio_select);        //Reset PKTLEN
      Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);    //Reset infinite packet length mode set
      printf("TX End\r\n");
      printf("TxBufferPos = %d\r\n", TxBufferPos);

      // Check for TX FIFO underflow, if yes then flush dat buffer
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        Radio_Strobe(TI_CCxxx0_SRX,radio_select);
        __delay_cycles(16000); //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select)); 
      }
    }

    // TODO add in and fix once we get one radio transmitting and receiving. 
    } 
} //end for loop
 


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
//************************************************************** radio IR code****************************************************************************************
void Radio_Interrupt_Setup(void){ // Enable RX interrupts only!  TX interrupt enabled in TX Start
  // Use GDO0 and GDO2 as interrupts to control TX/RX of radio
  P1DIR = 0;			        // Port 1 configured as inputs (i.e. GDO0 and GDO2 are inputs)
  P1IES = 0;
  P1IES |= CC1101_GDO2|CC2500_1_GDO2; // GDO0 interrupts on rising edge = 0 (RX), GDO2 interrupts on falling edge = 1 (TX) 

  P1IFG = 0;// Clear all flags <-- do this after IES as it will set a BIT2 high (pg 413 fam)
  P1IE |= CC2500_1_GDO2|CC1101_GDO0|CC1101_GDO2; // Enable GDO0 interrupt only (RX interrupt) add this back after single TX works CC1101_GDO0
    
    // For testing with only the CC1101 RX
    P1IE |= CC1101_GDO2;
}

void Port1_ISR (void) __ctl_interrupt[PORT1_VECTOR]{
    //read P1IV to determine which interrupt happened
    //reading automatically resets the flag for the returned state
    switch(P1IV){
// RADIO CC2500_1 interrupts
       case P1IV_P1IFG0: // [GDO0_1] is set up to assert when RX FIFO is greater than FIFO_THR.  This is an RX function only
            ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_RX_READ,0);
        break;
    // TX state
        case P1IV_P1IFG1: //[GDO2_1]is set up to assert when TX FIFO is above FIFO_THR threshold.  
    // Actual interrupt SR                
            switch(state)
            {
                case IDLE:
                     break;
                case TX_START:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                      state = TX_RUNNING;
                      ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_THR,0); 
                     break;
                case TX_RUNNING: //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                      ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_THR,0); 
                     break;
                case TX_END:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Last part of packet to transmit
                     state = IDLE;
                      ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_END,0);
                     break;
                default:
                  break;          
            }
        break;
// Radio CC1101 interrupts
        case P1IV_P1IFG2: // [GDO0_2] is set up to assert when RX FIFO is greater than FIFO_THR.  This is an RX function only
          P7OUT ^= BIT4;
          ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_RX_READ,0);
        break; 
    //TX state
        case P1IV_P1IFG3: //[GDO2_2] is set up to assert when TX FIFO is above FIFO_THR threshold.  
                                 //Interrupts on falling edge, i.e. when TX FIFO falls below FIFO_THR
    // Actual interrupt SR
            switch(state)
            {
                case IDLE:
                     break;
                case TX_START:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                      state = TX_RUNNING;
                      //ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_TX_THR,0);
                     break;
                case TX_RUNNING: //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                      //ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_TX_THR,0); 
                     break;
                case TX_END:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Last part of packet to transmit
                     state = IDLE;
                     // ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_TX_END,0);
                     break;
                default:
                  break;          
            }
        break; 
        }
}

//**************************************************** Beacon set up and timer IR
void COMM_beacon_setup(void){    //enable 10 sec interrupt
  int timer_check;

//************************************ Set up clock [0] 
  TA2CTL |= TASSEL__ACLK | MC_2;                            // Setting Timer_A to ACLK(TASSEL_1) 32kHz to continuous mode(MC_2)

//Set timer interrupt enable [1] 
  TA2CCTL0 |= CCIE;                                          // Capture/compare interrupt enable #0

//Set the timer count IR value [2] 
  TA2CCR0 = 32767;                                           // Timer0_A3 Capture/Compare --> 1s
}

//================[Time Tick interrupt]=========================
void beacon_tick(void) __ctl_interrupt[TIMER2_A0_VECTOR]{
      P7OUT^=BIT7; //toggle bit 7
      sec++; // increment sec
      if(sec == 1){  // reset counter for beacon @ 10 seconds
        P7OUT^=BIT6; //toggle bit 5
        if (beacon_on){
          ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_TX_START,0);     //Send to Radio to transmit mode
        }
        sec=0;  // reset 
      }
}




