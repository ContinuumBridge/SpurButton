/********************************
 define.h - General definitions
 Author:  B.Harris
 Date:    22/10/2013
 ********************************/
#ifndef __mydefine__header
#define __mydefine__header
/*********************************************************************************************************************************************************************************************

 Revision list

 Version 0.00 - 22/10/2015 - BJH          Derrived from Version 0.42 TeakSense.
 Version 0.01 - 28/10/2015 - BJH          Update all I/O and implement I/O test in Diagnostics.
 Version 0.02 - 06/11/2015 - BJH          Display driver routines implemented.
 Version 0.03 - 07/12/2015 - BJH          Demo to cycle through different fonts.
 Version 0.04 - 16/12/2015 - BJH          Tidied fonts up.
                                          Added optional horizontal and vertical borders to EEPROM settings menu.
 Version 0.05 - 27/01/2016 - BJH          New font added for improved small font Verdana_Pro_SemiBold18x16.
                                          Correct fault in rendering for display driver.

 To Do:

 *********************************************************************************************************************************************************************************************/
#define NO_RTC_CALIBRATION                                                              /* Include to prevent RTC calibration */
//#define DEBUG_MODE                                                                      /* Debug mode */
#define USE_DEBUG_SETTINGS  (1)                                                         /* Use debug settings flag (1=on, 0=off) */
#define CONSERVE_POWER                                                                  /* Set to conserve power where possible */
//#define BASIC_LIBRARY                                                                   /* Indicate that this uses BASIC library */
#define DEBUG_REGISTERS                                                                 /* Set to include debug registers */

// PCX
//#define STM32L151xB

// PCX
#include "stm32l1xx_hal.h"
#include <string.h>                                                                     /* Include string definitions */
#include <stdint.h>                                                                     /* Include custom types */
#include <stdio.h>                                                                      /* Include standard I/O definitions */
#include <stdarg.h>                                                                     /* Include standard arguments */
#include <stdlib.h>                                                                     /* Include standard library definitions */
#include <ctype.h>                                                                      /* Include ctype definitions */
#include "glue.h"                                                                       /* Include glue definitions */
//#include "__Time.h"
#include "sysclk_driver.h"                                                              /* Include system clock definitions */
//#include "sleep_driver.h"                                                               /* Include sleep driver definitions */
//#include "internal_rtc_driver.h"                                                        /* Include internal RTC routines */
//#include "uart.h"                                                                       /* Include UART definitions */
//#include "uart_driver.h"                                                                /* Include UART driver definitions */
//#include "misc.h"                                                                       /* Include miscellaneous definitions */
//#include "spi_driver.h"                                                                 /* Include SPI driver definitions */
//#include "spi.h"                                                                        /* Include SPI definitions */
//#include "eeprom_settings.h"                                                            /* Include EEPROM handler definitions */
//#include "internal_eeprom_driver.h"                                                     /* Include internal EEPROM handler definitions */
//#include "debug_log_driver.h"                                                           /* Include logger definitions */
//#include "io_driver.h"                                                                  /* Include I/O definitions */
//#include "io.h"                                                                         /* Include I/O definitions */
//#include "i2csw_driver.h"                                                               /* Include I2C driver definitions */
//#include "timers.h"                                                                     /* Include timer definitions */
//#include "adc.h"                                                                        /* Include ADC definitions */
//#include "diagnostics_151_driver.h"                                                     /* Include diagnostics driver */
//#include "process.h"                                                                    /* Include main application processes */
//#include "system_flags.h"                                                               /* Include system flags */
//#include "application_flags.h"                                                          /* Include application flags */
#include "ecog_driver.h"                                                                /* Include ECOG driver */
//#include "Sanus_Webissimo16x22.h"                                                       /* Include font */
//#include "icons32x30.h"                                                                 /* Include icons */
//#include "Courier13x17.h"                                                               /* Include font */
//#include "Sanus_Webissimo24x32.h"                                                       /* Include font */
#include "Arial_Rounded_MT_Bold26x27.h"                                                 /* Include font */
//#include "Copperplate_Gothic_Bold30x24.h"                                               /* Include font */
//#include "Copperplate_Gothic_Bold27x22.h"                                               /* Include font */
#include "Arial_Rounded_MT_Bold19x20.h"                                                 /* Include font */
//#include "Verdana_Pro_SemiBold18x16.h"                                                  /* Include font */
//#include "Arial_Narrow16x20.h"                                                          /* Include font */
#include "Arial_Narrow14x20.h"                                                          /* Include font */
#include "Arial_Narrow16x23.h"                                                          /* Include font */
#include "Arial_Narrow18x26.h"                                                          /* Include font */


extern int _Lib_Sprintf__doprntf(char  * pb,  const char * f, register va_list ap);     /* Required for printf style routines */
// PCX
//extern sbit TFT_BLED;                                                                   /* TFT backlight */

#define full_debug                                                                      /* Enable full debugging */
//#define extra_debug                                                                     /* Enable extra debugging */
//#define SPI_DEBUG                                                                       /* Enable SPI debugging */
//#define full_gsm_response
// #define PROGRAM_VALIDATION_REQUIRED                                                     /* Program validation required flag */

// #define DISABLE_RESET                                                                   /* Disable reset from windows software */
#define VERSION_NUMBER      6                                                           /* Version number */
#define VERSION_TEXT        "ContinuumBridge V0.05-27/01/2016"                          /* Current version text */
#define VERSION_STRING      "V0.05-27/01/2016"                                          /* Version number and date string */
#define COPYRIGHT_MESSAGE   "(C)Creative Systems Technology Ltd. 2004-2016"             /* Copyright message string */
#define HW_STRING           "ContinuumBridge V1"                                        /* Hardare string */

#define M_DATE              260515

#define NEW_PROGRAM         1                                                           /* Set to indicate new program */
#define DEVICE_TYPE_ID      "ContinuumBridge"                                           /* Device ID string */
#define MCLK_SPEED          32000000L                                                   /* Main clock speed in Hz */
#define DIAGNOSTICS_ENTRY   "DIAG"                                                      /* Diagnostics entry code */


/***************************
 Invalid number definition
 Author:        B.Harris
 Date:          09/01/07
 ***************************/
#define INVALID_NUMBER    0x7fffffff                                                    /* Number indicating invalid value */


#endif
