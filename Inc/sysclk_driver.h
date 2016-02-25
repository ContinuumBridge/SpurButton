/**********************************************
 sysclk.h -   System clock driver definitions
 Author:      B.Harris
 Date:        30/04/15
 **********************************************/
#ifndef __sysclk_header
#define __sysclk_header
/************************************************************************************************************************************

 Revision list

 Version 1.00 - 04/11/2015 - BJH          Original code dates to around April 2013.
                                          First version with revision information.
 ************************************************************************************************************************************/
#define VERSION_SYSCLK_DRIVER 100                                                       /* Module version number */



/*********************
 Prototypes
 Author:    B.Harris
 Date:      15/04/13
 *********************/
extern void SystemInit12(void);
extern void SystemInit16(void);
extern void SystemInit32(void);


/*********************
 Interface
 Author:    B.Harris
 Date:      14/11/13
 *********************/
//#define        CLOCK_SETTABLE
//#define rtc_set_time                internal_rtc_set_time
//#define rtc_set_date                internal_rtc_set_date
//#define rtc_set_alarm               internal_rtc_set_alarm
//#define rtc_get_time                internal_rtc_get_time
//#define rtc_get_date                internal_rtc_get_date
//#define        RTC_TEST_EXISTS
//#define rtc_test                    internal_rtc_test
//#define rtc_init                    internal_rtc_init

#endif