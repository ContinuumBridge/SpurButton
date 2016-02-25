/*******************************************
 PROCESS.C -    Main application processes
 Author:        B.Harris
 Date:          02/06/15
********************************************/
#include "define.h"                                                                     /* Include general definitions */
#include "cbutils.h"
#define NOF_SCENES         4                                                            /* Number of different scenes */


/********************************
 RAM            RAM definitions
 Author:        B.Harris
 Date:          02/06/15
*********************************/
uint8_t event_initialise_required;                                                      /* Initialise events required flag */
uint8_t event_minute_passed;                                                            /* Minute event event flag */
uint8_t event_com_detected;                                                             /* RS232 communications detected flag */
uint8_t event_battery_read_complete;                                                    /* Battery read completed flag */
uint8_t event_user_button_pressed;                                                      /* User button pressed event flag */

uint8_t scene;                                                                          /* Scene currently being displayed */
uint8_t battery_read_counter;                                                           /* Battery minute read counter */
uint8_t battery_percentage;                                                             /* Battery percentage */
uint16_t battery_level;                                                                 /* Battery level in mV */


/**************************************************
 process_button_irq -  Button interrupt triggered
 Author:               B.Harris
 Inputs:               Interrupt counter
 Outputs:              Nothing
 Date:                 13/11/15
 **************************************************/
void process_button_irq(uint8_t source)
{
//  wakeup_required=1;                                                                    /* Force wakeup */
//  sleep_timer=1;                                                                        /* Force wakeup */
  event_user_button_pressed=1;                                                          /* User button pressed */
//  dprintf("Button pressed\r\n");
}


/******************************
 scene0 -     Display scene 0
 Author:      B.Harris
 Inputs:      Nothing
 Outputs:     Nothing
 Date:        13/11/15
 ******************************/
void scene0(void)
{

//  dprintf("Showing scene 0\r\n");

  ecog_printfc(Arial_Rounded_MT_Bold26x27,6,"PRESS");
  ecog_printfc(Arial_Rounded_MT_Bold26x27,33,"TO CALL FOR");
  ecog_printfc(Arial_Rounded_MT_Bold26x27,61,"SERVICE");
  ecog_box(0,0,200,96,ECOG_ON);
  ecog_box(1,1,198,96,ECOG_ON);
}


/******************************
 scene1 -     Display scene 1
 Author:      B.Harris
 Inputs:      Nothing
 Outputs:     Nothing
 Date:        13/11/15
 ******************************/
void scene1(void)
{

//  dprintf("Showing scene 1\r\n");

  ecog_printfc(Sanus_Webissimo24x32,3,"YOUR REQUEST");
  ecog_printfc(Sanus_Webissimo24x32,32,"HAS BEEN");
  ecog_printfc(Sanus_Webissimo24x32,61,"SENT");
  ecog_box(0,0,200,96,ECOG_ON);
  ecog_box(1,1,198,96,ECOG_ON);
}


/******************************
 scene2 -     Display scene 2
 Author:      B.Harris
 Inputs:      Nothing
 Outputs:     Nothing
 Date:        13/11/15
 ******************************/
void scene2(void)
{

//  dprintf("Showing scene 2\r\n");

  /*
  ecog_printfc(Verdana_Pro_SemiBold18x16,4,"PRESS HERE");
  ecog_printfc(Verdana_Pro_SemiBold18x16,26,"IF MACHINE");
  ecog_printfc(Verdana_Pro_SemiBold18x16,48,"HAS RUN OUT");
  ecog_printfc(Verdana_Pro_SemiBold18x16,70,"OF COFFEE");*/

  ecog_printfc(Arial_Rounded_MT_Bold19x20,4,"Press here");
  ecog_printfc(Arial_Rounded_MT_Bold19x20,26,"if this machine");
  ecog_printfc(Arial_Rounded_MT_Bold19x20,48,"has run out");
  ecog_printfc(Arial_Rounded_MT_Bold19x20,70, "of coffee");
  //ecog_box(0,0,200,96,ECOG_ON);
  //ecog_box(1,1,198,96,ECOG_ON);
}


/******************************
 scene3 -     Display scene 3
 Author:      B.Harris
 Inputs:      Nothing
 Outputs:     Nothing
 Date:        24/01/16
 ******************************/
void scene3(void)
{
//  dprintf("Showing scene 3\r\n");
  ecog_printfc(Verdana_Pro_SemiBold18x16,4,"Meeting Room 10");
  ecog_position(4,20);
  ecog_printf(Verdana_Pro_SemiBold18x16,"14:00 Push to use");
  ecog_position(4,40);
  ecog_printf(Verdana_Pro_SemiBold18x16,"15:00 Jeremy Corbyn");
  ecog_position(4,60);
  ecog_printf(Verdana_Pro_SemiBold18x16,"16:30 Free");
  ecog_position(4,80);
  ecog_printf(Verdana_Pro_SemiBold18x16,"17:00 David Cameron");
}


/***************************************************************
 process_show_default_display -     Show default display image
 Author:                            B.Harris
 Inputs:                            Interrupt counter
 Outputs:                           Nothing
 Date:                              12/11/15
 ***************************************************************/
void process_show_default_display(void)
{
  DEBUG_TX("process_show_default_display\r\n");
  ecog_cls();                                                                           /* Clear display */
  ecog_update_display();
  switch(scene)                                                                         /* Act on current scene */
  {
    case 0: scene0();                                                                   /* Display Scene 0 */
            break;

    case 1: scene1();                                                                   /* Display scene 1 */
            break;

    case 2: scene2();                                                                   /* Display scene 2 */
            break;

    case 3: scene3();                                                                   /* Display scene 3 */
            break;
  }
//  dprintf("Updating display\r\n");
//  UART1_Write_Text("Updating display\r\n");
  ecog_update_display();
//  UART1_Write_Text("Display updated\r\n");

//  dprintf("Display updated\r\n");
}


/************************************************************************************
 process_event_initialise -     Initialise events from power up or settings changed
 Author:                        B.Harris
 Inputs:                        Interrupt counter
 Outputs:                       Nothing
 Date:                          31/07/15
 ************************************************************************************/
void process_event_initialise(void)
{
//  debug_log("Initialising events from power up or settings change");
//  clock_clear_events();                                                                 /* Reset current events */
//  clock_add_event(30,"Half-Minute scheduled event",&event_minute_passed);               /* Add event to wake every 30 seconds */
//  process_enable_com_detect_ineterrupt();                                               /* Enable interrupt to detect com activity */
//  io_interrupt_enable(0x0008,IO_INT_FALLING,process_button_irq);                        /* Enable falling interrupt for user button detect */
  process_show_default_display();                                                       /* Show default display image */
}


/************************************************
 process_event_minute -     Minute passed event
 Author:                    B.Harris
 Inputs:                    Interrupt counter
 Outputs:                   Nothing
 Date:                      24/06/15
 ************************************************/
void process_event_minute(void)
{
//  clock_add_event(30,"Half-Minute scheduled event",&event_minute_passed);               /* Add event to wake every 30 seconds */
  scene++;                                                                              /* Next scene */
  if(scene>=NOF_SCENES)                                                                 /* All scenes done? */
  {
    scene=0;                                                                            /* Loop around scenes */
  }
  process_show_default_display();                                                       /* Show default display image */
}


/*****************************************************
 process_user_button_pressed -   User button pressed
 Author:                         B.Harris
 Inputs:                         Nothing
 Outputs:                        Nothing
 Date:                           13/11/15
 *****************************************************/
void process_user_button_pressed(void)
{
  scene++;                                                                              /* Next scene */
  if(scene>=NOF_SCENES)                                                                 /* All scenes done? */
  {
    scene=0;                                                                            /* Loop around scenes */
  }
  process_show_default_display();                                                       /* Show default display image */
}


/***********************************************************************
 process_battery_read_complete -   Process battery level read complete
 Author:                           B.Harris
 Inputs:                           Nothing
 Outputs:                          Nothing
 Date:                             25/09/15
 ***********************************************************************/
void process_battery_read_complete(void)
{
//  debug_log("Battery is %scharging",testflag(F_BATTERY_CHARGING)? "":"not ");
//  debug_log("Battery is %sfully charged",testflag(F_BATTERY_CHARGED)? "":"not ");
//  debug_log("Battery is %slow (according to EEPROM settings)",testflag(F_BATTERY_LOW)? "":"not ");
//  debug_log("Battery level is %umV",battery_level);
}


/***************************************************************************************
 process_timed_event_table -    Table containing all event flags and handling routines
 Author:                        B.Harris
 Date:                          01/07/15
****************************************************************************************/
const PROCESS_EVENT process_timed_event_table[]=
{
  { &event_initialise_required,process_event_initialise },
  { &event_minute_passed,process_event_minute },
  { &event_user_button_pressed,process_user_button_pressed },
  { &event_battery_read_complete,process_battery_read_complete },
  { 0,0 }
};


/***********************************************************************
 process_timed_events -    Process any timed events that have occurred
 Author:                   B.Harris
 Inputs:                   Nothing
 Outputs:                  Nothing
 Date:                     24/06/15
 ***********************************************************************/
void process_timed_events(void)
{
  PROCESS_EVENT *ep;                                                                    /* Pointer to event */

  ep=process_timed_event_table;                                                         /* Point at event table */
//  clock_trigger_events();                                                               /* Trigger any pending events */
  while(ep->flag&&ep->event_routine)                                                    /* Loop until all done */
  {
    if(*(ep->flag))                                                                     /* Event triggered? */
    {
      *ep->flag=0;                                                                      /* Reset flag */
      ep->event_routine();                                                              /* Trigger event */
    }
    ep++;
  }
}


/********************************************************
 process_main -    Main application process entry point
 Author:           B.Harris
 Inputs:           Nothing
 Outputs:          Never returns
 Date:             02/06/15
 ********************************************************/
void process_main(void)
{
  uint32_t timer;

  ecog_init();                                                                          /* Initialise display */
  event_initialise_required=1;                                                          /* Initialisation required */
  scene=0;                                                                              /* Start with first scene */
  timer=10000000;
  while(1)
  {
    if(timer)
    {
      timer--;
    }
    else
    {
      process_event_minute();
      timer=10000000;
    }
    process_timed_events();                                                             /* Process timed events */
  }
}
