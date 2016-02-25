/*******************************************************
 PROCESS.H -    Main application processes definitions
 Author:        B.Harris
 Date:          02/06/15
 *******************************************************/
#ifndef __process__header
#define __process__header

#define ENTER_LOW_POWER_ROUTINE                          process_enter_low_power        /* Enter low power routine */
#define EXIT_LOW_POWER_ROUTINE                           process_exit_low_power         /* Exit low power routine */


/************************************
 PROCESS_EVENT structure
 Author:        B.Harris
 Date:          10/02/14
 ************************************/
typedef struct
{
  uint8_t * flag;
  void(*event_routine)(void);
} PROCESS_EVENT;


/***************************
 Prototypes
 Author:        B.Harris
 Date:          02/06/2015
 ***************************/
extern void process_main(void);
extern uint8_t event_got_gps_fix;
extern int16_t x_axis;
extern int16_t y_axis;
extern int16_t z_axis;
extern uint8_t battery_percentage;
extern uint16_t battery_level;
extern int8_t internal_temperature;
extern void process_schedule_temperature_report(void);
extern void process_schedule_temperature_average(void);
extern void process_reset_temperature_alarm_resend_timer(void);
extern void process_start_temperature_failure_timer(void);
extern void process_schedule_repeat_temperature_alarm(void);
extern void process_read_altitude(void);
// PCX
//extern void void process_schedule_altitude_report(void);
extern void process_schedule_altitude_report(void);
extern void process_schedule_altitude_average(void);
extern void process_reset_altitude_alarm_resend_timer(void);
extern void process_start_altitude_failure_timer(void);
extern uint8_t event_initialise_required;
extern void process_schedule_light_report(void);
extern void process_schedule_shock_report(void);
extern void process_schedule_alarm_holdoff(void);
extern uint8_t event_failed_gps_fix;
extern void process_enable_light_interrupt(void);
extern void process_disable_light_interrupt(void);
extern uint8_t event_light_out_of_range;
extern void process_light_out_of_range(void);
extern void process_schedule_light_interrupt_enable(void);
extern uint8_t event_light_interrupt_enable_required;
extern void process_com_detected(void);
extern void process_enable_com_detect_ineterrupt(void);
extern uint8_t event_com_detected;
extern void process_enter_low_power(void);
extern void process_exit_low_power(void);



#endif
