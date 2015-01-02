#ifndef __COMM_Events_H
#define __COMM_Events_H

  int comm_evt_gs_decode(void);

//Return values from COMM Event functions (RET_SUCCESS = 0 defined in ARCbus.h)
enum{ERR_BAD_COMM_DEST_ADDR=-1,ERR_BAD_COMM_SRC_ADDR=-2,ERR_BAD_COMM_CRC=-3,ERR_UNKNOWN_COMM_CMD=-4};

const char *COMM_error_str(int error);


#endif




