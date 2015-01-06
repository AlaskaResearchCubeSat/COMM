#include <Error.h>
#include "COMM_errors.h"
#include <SDlib.h>

//decode errors from CDH code
char *err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case COMM_ERR_SRC_DAT_STORE:
      switch(err){
        case COMM_ERR_DAT_INIT_CRC:
            return "Data Storage : invalid data table CRC";
        case COMM_ERR_DAT_INIT_MAGIC:
            sprintf(buf,"Data Storage : Incorrect block ID (0x%04X) for data table",argument);
            return buf;
        case COMM_ERR_DAT_INIT_READ:
            sprintf(buf,"Data Storage : initilization failed, SD card returned error %s",SD_error_str(argument));  
            return buf;
        case COMM_ERR_DAT_INIT_BUFFER:
            return "Data Storage : initilization failed, buffer busy";
        case COMM_ERR_DAT_STORE_BAD_SUBSYSTEM:
            sprintf(buf,"Data Storage : data storage failed, unknown subsystem 0x%02X",argument);  
            return buf;
        case COMM_ERR_DAT_STORE_BLOCK_WRITE:
            sprintf(buf,"Data Storage : data block write failed, SD card returned error %s",SD_error_str(argument));  
            return buf;
        case COMM_ERR_DAT_STORE_TABLE_WRITE:
            sprintf(buf,"Data Storage : data table write failed, SD card returned error %s",SD_error_str(argument));  
            return buf;
      }
    break;         
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}

