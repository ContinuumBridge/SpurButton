/*******************************************************
 glue.h:    Glue definitions, required for portability
 Author:    B.Harris
 Date:      28/01/2016
 *******************************************************/
#ifndef __GLUE_H
#define __GLUE_H


/*********************
 Prototypes
 Author:    B.Harris
 Date:      28/01/16
 *********************/
extern int misc_sgn(int n);
extern uint8_t spi_hw(uint8_t dat);
extern void timers_sdelay(uint32_t ms);
extern void spi_init(void);
extern void spi_deinit(void);
extern uint8_t misc_reverse_byte(uint8_t byteval);


#endif
