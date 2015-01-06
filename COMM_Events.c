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
              return RET_SUCCESS;
            case COMM_DATA_TRANSFER:
              return RET_SUCCESS;
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
    //Error was not found
    default:
      return "UNKNOWN ERROR";
  }
}


