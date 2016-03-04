/****************************************
 EDO20OC1.H:    EINK driver definitions
 Author:        B.Harris
 Date:          15/03/2013
 ****************************************/
#ifndef __ecog_driver__header
#define __ecog_driver__header
#define DISPLAY_PRESENT                                                                 /* Indicate presence of display */

/*********************
 General definitions
 Author:  B.Harris
 Date:    15/10/2013
 *********************/
#define ECOG_ON      0                                                                  /* Mode to turn pixel on */
#define ECOG_OFF     1                                                                  /* Mode to turn pixel off */
#define ECOG_XOR     2                                                                  /* Mode to XOR pixel */
#define ECOG_WIDTH   200
#define ECOG_HEIGHT  96
#define IMAGE_COUNT 2400                                                                /* Size of display buffer (200*96)/8 */



/*************************
 ECOG display line types
 Author:  B.Harris
 Date:    13/11/2015
 *************************/
#define LINE_TYPE_WHITE          0                                                      /* To display white lines */
#define LINE_TYPE_BLACK          1                                                      /* To display black lines */
#define LINE_TYPE_NORMAL         2                                                      /* To display normal lines */
#define LINE_TYPE_INVERSE        3                                                      /* To display inverted lines */
#define LINE_TYPE_SKIP           4                                                      /* To display skip lines */


/*********************
 I/O definitions
 Author:  B.Harris
 Date:    15/10/2013
 *********************/
//OUTPUT
// PCX
//#define ECOG_CS_ON               (GPIOA_ODR&=0xffef)
#define ECOG_CS_ON               HAL_GPIO_WritePin(_SPI_CS_GPIO_Port, _SPI_CS_Pin, GPIO_PIN_RESET)
//#define ECOG_CS_OFF              (GPIOA_ODR|=0x0010)
#define ECOG_CS_OFF              HAL_GPIO_WritePin(_SPI_CS_GPIO_Port, _SPI_CS_Pin, GPIO_PIN_SET)
//#define ECOG_DISCHARGE_ON        (GPIOA_ODR|=0x1000)
#define ECOG_DISCHARGE_ON        HAL_GPIO_WritePin(GPIOA, DISPLAY_DISCHARGE_Pin, GPIO_PIN_SET);
//#define ECOG_DISCHARGE_OFF       (GPIOA_ODR&=0xefff)
#define ECOG_DISCHARGE_OFF       HAL_GPIO_WritePin(GPIOA, DISPLAY_DISCHARGE_Pin, GPIO_PIN_RESET);
//#define ECOG_RESET_ON            (GPIOB_ODR&=0xffdf)
#define ECOG_RESET_ON            HAL_GPIO_WritePin(GPIOB, _RESET_DISPLAY_Pin, GPIO_PIN_RESET);
//#define ECOG_RESET_OFF           (GPIOB_ODR|=0x0020)
#define ECOG_RESET_OFF           HAL_GPIO_WritePin(GPIOB, _RESET_DISPLAY_Pin, GPIO_PIN_SET);
//#define ECOG_POWER_ON            (GPIOB_ODR|=0x0040)
#define ECOG_POWER_ON            HAL_GPIO_WritePin(GPIOB, DISPLAY_ON_PB6_Pin, GPIO_PIN_SET)
//#define ECOG_POWER_OFF           (GPIOB_ODR&=0xffbf)
#define ECOG_POWER_OFF           HAL_GPIO_WritePin(GPIOB, DISPLAY_ON_PB6_Pin, GPIO_PIN_RESET)

//INPUT
//#define ECOG_BUSY                ((GPIOA_IDR&0x0100)? 1:0)
#define ECOG_BUSY                ((HAL_GPIO_ReadPin(_DISPLAY_BUSY_GPIO_Port, _DISPLAY_BUSY_Pin))? 1:0)


/*********************
 Prototypes
 Author:  B.Harris
 Date:    15/10/2013
 *********************/
extern uint8_t ecog_test(void);
extern void check_busy(void);
extern uint8_t ecog_init(void);
extern void ecog_barcode(uint8_t height,uint8_t * text);
extern void ecog_copy(unsigned char const *img);
extern void ecog_icopy(unsigned char const *img);
extern void ecog_printf(const uint8_t * font,const char *pstr);
extern void ecog_cls(void);
extern void ecog_position(int16_t x,int16_t y);
extern uint16_t ecog_text_width(const uint8_t * font,uint8_t *pstr);
extern uint16_t ecog_character_width(const uint8_t * font,uint8_t c);
extern void ecog_box(int16_t x,int16_t y,unsigned char w,unsigned char h,unsigned char m);
extern void ecog_printfc(const uint8_t * font,int16_t y,const char *pstr);
extern void ecog_line(int x1,int y1,int x2,int y2,unsigned char m);
extern uint8_t ecog_disable_display(void);
extern uint8_t ecog_enable_display(void);
extern uint8_t ecog_display_enabled;
extern uint8_t ecog_update_display(uint8_t powered);
uint8_t ecog_write_inverse(uint8_t powered);
extern void ecog_plot(int16_t x,int16_t y,uint8_t m);
extern uint8_t ecog_buffer[];
extern void ecog_dump(void);


/*********************
 Interface
 Author:    B.Harris
 Date:      14/11/13
 *********************/
//#define display_test        ecog_test
//#define display_init        ecog_init

#endif
