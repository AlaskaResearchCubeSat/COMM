#ifndef __COMM_ERRORS_H
  #define __COMM_ERRORS_H
  #include <Error.h>
  //error sources for BUS test program
  enum{COMM_ERR_SRC_SUBSYSTEM=ERR_SRC_SUBSYSTEM,COMM_ERR_SRC_DAT_STORE};
    
  //errors for data storage
  enum{COMM_ERR_DAT_INIT_CRC,COMM_ERR_DAT_INIT_MAGIC,COMM_ERR_DAT_INIT_READ,COMM_ERR_DAT_INIT_BUFFER,COMM_ERR_DAT_INIT_WRITE,COMM_ERR_DAT_STORE_BAD_SUBSYSTEM,COMM_ERR_DAT_STORE_BLOCK_WRITE,COMM_ERR_DAT_STORE_TABLE_WRITE};
        
#endif
  