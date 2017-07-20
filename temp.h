#ifndef __TEMP_H
#define __TEMP_H

//fixed address bits for AD7416
#define BASE_ADDR     0x48

//AD7416 register addresses
#define TEMP_VAL  0x00
#define CONFIG1   0x01
#define THYST_SP  0x02
#define TOTI_SP   0x03
#define ADC_VAL   0x04
#define CONFIG2   0x05

//address for X_Minus face
#define X_MINUS_ADDR   (0x00)
#define X_PLUS_ADDR    (0x48)
#define Y_MINUS_ADDR   (0x4C)
#define Y_PLUS_ADDR    (0x49)
#define Z_MINUS_ADDR   (0x4D)
#define Z_PLUS_ADDR    (0x4E)
#define INT_TEMP_ADDR  (0x4F)
#define CLYDE_ADDR     (0x01)

#endif
