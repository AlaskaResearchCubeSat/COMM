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


int comm_evt_gs_decode(void){
  int i, len, resp;
  unsigned char FCS[2];
  unsigned char buf[BUS_I2C_HDR_LEN+30+BUS_I2C_CRC_LEN],*ptr;

        printf("COMM_EVT_GS_DECODE\r\n");

    // Check destination address (all bit reversed!)
        if((RxBuffer[0] == 0x69) && (RxBuffer[1] == 0x19) && (RxBuffer[2] == 0x66) && (RxBuffer[3] == 0x29) && (RxBuffer[4] == 0x05) && (RxBuffer[5] == 0x02)){
//            printf("Destination address good!\r\n");
        } else{ return ERR_BAD_COMM_DEST_ADDR; }
    // Check source address (all bit reversed!)
        if((RxBuffer[7] == 0x75) && (RxBuffer[8] == 0x19) && (RxBuffer[9] == 0x76) && (RxBuffer[10] == 0x61) && (RxBuffer[11] == 0x0D) && (RxBuffer[12] == 0x21)){
//           printf("Source address good!\r\n");
        } else{ return ERR_BAD_COMM_SRC_ADDR;}
    // Verify CRC  
        FCS[0]=RxBuffer[RxBuffer_Len-2];
        FCS[1]=RxBuffer[RxBuffer_Len-1];
        RxBuffer[RxBuffer_Len-2]=0x00;
        RxBuffer[RxBuffer_Len-1]=0x00;
        RxBuffer_Len = RxBuffer_Len-2;
        CRC_CCITT_Generator(RxBuffer, &RxBuffer_Len);
        if((FCS[0] == RxBuffer[RxBuffer_Len-2]) && (FCS[1] == RxBuffer[RxBuffer_Len-1])){
//          printf("CRC checked\r\n");
        } else{ return ERR_BAD_COMM_CRC; }
    //GS command to COMM

    status.Num_CMD++; //Increment number of commands received

        if(RxBuffer[16] == 0xC8) { 
          printf("subsystem address: 0x%02x\r\n",RxBuffer[16]);
//          printf("num: 0x%02x\r\n",RxBuffer[17]);
//          printf("cmd: 0x%02x\r\n",RxBuffer[18]);
          switch(__bit_reverse_char(RxBuffer[18])){
            case COMM_RF_OFF:
              beacon_on=0;
              return RET_SUCCESS;
            case COMM_RF_ON:
              beacon_on=1;
              return RET_SUCCESS;
            case COMM_BEACON_STATUS:
              beacon_flag=1;
              return RET_SUCCESS;
            case COMM_BEACON_HELLO:
              beacon_flag=0;
              return RET_SUCCESS;
            case COMM_RESET_CDH:
              return COMM_CDH_reset();
            case COMM_GET_DATA:
              len = __bit_reverse_char(RxBuffer[17]);
              for(i=0;i<len;i++){
                  buf[i]=__bit_reverse_char(RxBuffer[19+i]);
              }
              return COMM_Get_Data(buf);
            case COMM_SEND_DATA:
              len = __bit_reverse_char(RxBuffer[17]);
              for(i=0;i<len;i++){
                  buf[i]=__bit_reverse_char(RxBuffer[19+i]);
              }
              return COMM_Send_Data(buf);
            default:
              return ERR_UNKNOWN_COMM_CMD;
          }
       } 
    //else send to CDH  send CMD_GS_DATA I2C command to CDH with payload RxBuffer[16] - end    
       else { 
         printf("Sending GS CMD to CDH\r\n");
         len = __bit_reverse_char(RxBuffer[17])+3;
         printf("subsystem address: 0x%02x, len: %d\r\n",__bit_reverse_char(RxBuffer[16]), len);
         ptr=BUS_cmd_init(buf,CMD_GS_DATA);         //setup packet
         for(i=0;i<len;i++){             //fill in telemetry data
           ptr[i]=__bit_reverse_char(RxBuffer[16+i]);
          }
          resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,len,0,BUS_I2C_SEND_FOREGROUND);  //send command
          if(resp!=RET_SUCCESS){ printf("Failed to send GS CMD to CDH %s\r\n",BUS_error_str(resp));}
       }

      return RET_SUCCESS;
}


//return error strings for error code
const char *COMM_error_str(int error){
  //check for error
  switch(error){
    case RET_SUCCESS:
      return "SUCCESS";
    case ERR_BAD_COMM_DEST_ADDR:
      return "ERROR BAD DESTINATION ADDRESS";
    case ERR_BAD_COMM_SRC_ADDR:
      return "ERROR BAD SOURCE ADDRESS";
    case ERR_BAD_COMM_CRC:
      return "ERROR BAD CRC";
    case ERR_UNKNOWN_COMM_CMD:
      return "ERROR UNKNOWN COMM COMMAND";
    case ERR_UNKNOWN_SUBADDR:
      return "ERROR UNKNOWN SUBSYSTEM ADDRESS";
    case ERR_DATA_NOT_TRANSFERED:
      return "ERROR TIMEOUT DATA NOT TRANSFERED";
    //Error was not found
    default:
      return "UNKNOWN ERROR";
  }
}

int COMM_CDH_reset(void){
    P6DIR |= CDH_RESET;
    ctl_timeout_wait(ctl_get_current_time()+3);
    P6DIR &= ~CDH_RESET;
    return RET_SUCCESS;
}

CTL_EVENT_SET_t ev_SPI_data;

int COMM_Get_Data(unsigned char *data){
  int i;
  unsigned int e;
  unsigned long start;
  unsigned char buf[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;

  switch(data[0]){
    case BUS_ADDR_ACDS:
      start = ((unsigned long) data[1])<<16;
      start |=((unsigned long) data[2])<<8;
      start |=((unsigned long) data[3]);
      for(i=0;i<data[4];i++){
         ptr=BUS_cmd_init(buf,CMD_ACDS_READ_BLOCK);
         ptr[0]=start>>16;
         ptr[1]=start>>8;
         ptr[2]=start;
         BUS_cmd_tx(BUS_ADDR_ACDS,buf,3,0,BUS_I2C_SEND_FOREGROUND);
         e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ev_SPI_data,SPI_EV_DATA_REC,CTL_TIMEOUT_DELAY,1024);
         if(!(e&SPI_EV_DATA_REC)){
          //data not received
          return ERR_DATA_NOT_TRANSFERED;
         }
         start++;
      }
      return RET_SUCCESS;
    case BUS_ADDR_LEDL:
      start = ((unsigned long) data[1])<<16;
      start |=((unsigned long) data[2])<<8;
      start |=((unsigned long) data[3]);
      for(i=0;i<data[4];i++){
         ptr=BUS_cmd_init(buf,CMD_LEDL_READ_BLOCK);
         ptr[0]=start>>16;
         ptr[1]=start>>8;
         ptr[2]=start;
         BUS_cmd_tx(BUS_ADDR_LEDL,buf,3,0,BUS_I2C_SEND_FOREGROUND);
         e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ev_SPI_data,SPI_EV_DATA_REC,CTL_TIMEOUT_DELAY,1024);
         if(!(e&SPI_EV_DATA_REC)){
          //data not received
          return ERR_DATA_NOT_TRANSFERED;
         }
         start++;
      }
      return RET_SUCCESS;
    case BUS_ADDR_IMG:
      //Read Image Start Block Definition Image ID, block number
      ptr=BUS_cmd_init(buf,CMD_IMG_READ_PIC);
      ptr[0]=data[3]; //Image number is one byte long.
      ptr[1]=0;
      BUS_cmd_tx(BUS_ADDR_IMG,buf,2,0,BUS_I2C_SEND_FOREGROUND);
      e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ev_SPI_data,SPI_EV_DATA_REC,CTL_TIMEOUT_DELAY,1024);
      if(!(e&SPI_EV_DATA_REC)){
        //data not received
        return ERR_DATA_NOT_TRANSFERED;
      }
      for(i=1;i<IMG_Blk;i++){
         ptr[1]=i;
         BUS_cmd_tx(BUS_ADDR_IMG,buf,2,0,BUS_I2C_SEND_FOREGROUND);
         e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ev_SPI_data,SPI_EV_DATA_REC,CTL_TIMEOUT_DELAY,1024);
         if(!(e&SPI_EV_DATA_REC)){
          //data not received
          return ERR_DATA_NOT_TRANSFERED;
         }
      }
      return RET_SUCCESS;
    default:
      return ERR_UNKNOWN_SUBADDR;
   }

}

int COMM_Send_Data(unsigned char *data){
  int i,j;
  unsigned long start;
  unsigned int e;
  beacon_on = 0;

  for(i=0;i<COMM_TXHEADER_LEN;i++){                                             //LOAD UP HEADER
    Tx1Buffer[i]=__bit_reverse_char(Tx2_Header[i]);                             //AX.25 octets are sent LSB first
  }
  Tx1Buffer_Len=COMM_TXHEADER_LEN+(512)+1; // Set length of message: HeaderLen+Blocksize+1

  start = ((unsigned long) data[1])<<16;
  start |=((unsigned long) data[2])<<8;
  start |=((unsigned long) data[3]);

  for(i=0;i<data[4];i++){
     readSD_Data(data[0],start,Tx1Buffer+COMM_TXHEADER_LEN);
     for(j=0;j<512;j++) { //data needs to be bit reversed
        Tx1Buffer[j+COMM_TXHEADER_LEN]=__bit_reverse_char(Tx1Buffer[j+COMM_TXHEADER_LEN]);
     }
     Tx1Buffer[Tx1Buffer_Len-1]=__bit_reverse_char(COMM_CR);                     //Add carriage return

     //**** Create AX.25 packet (needs to include FCS, bit stuffed, flags) ***
     CRC_CCITT_Generator(Tx1Buffer, &Tx1Buffer_Len);                           //Generate FCS
     Stuff_Transition_Scramble(Tx1Buffer, &Tx1Buffer_Len);                     //Bit stuff - Encode for transitions - Scramble data
     ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_START,0);                     //Send to Radio to transmit
     e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ev_SPI_data,SPI_EV_DATA_TX,CTL_TIMEOUT_DELAY,1024);
     if(!(e&SPI_EV_DATA_TX)){
          //data not received
          beacon_on = 1;
          return ERR_DATA_NOT_TRANSFERED;
      }
      start++;
  }

  beacon_on = 1;
  return RET_SUCCESS;
}


