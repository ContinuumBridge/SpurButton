/*****************************************
 ecog_driver.c:  ECOG INK display driver
 Author:         B.Harris
 Date:           02/11/2015
 *****************************************/
#include "define.h"
#include "usart.h"
#include "cbutils.h"


/**********************
 RAM
 Author:  B.Harris
 Date:    12/03/2013
 **********************/
uint8_t ecog_buffer[IMAGE_COUNT];                                                       /* New displayed screen data */
int16_t ecogx;                                                                          /* X position */
int16_t ecogy;                                                                          /* Y position */
uint8_t ecog_display_enabled;                                                           /* Display enabled flag */

/**********************************
 ecog_dump -   Dump screen buffer
 Author:       B.Harris
 Enters:       Nothing
 Exits:        Nothing
 Date:         26/01/16
 **********************************/
//void ecog_dump(void)
//{
//  uint32_t f;                                                                           /* Loop counter */
//
//  for(f=0;f<IMAGE_COUNT;f++)
//  {
//    if((f%25)==0) dprintf("\r\n%04X: ",f);
//    dprintf("%02X",ecog_buffer[f]);
//  }
//}


/****************************************************
 check_busy -   Check and wait for display not busy
 Author:        B.Harris
 Enters:        Nothing
 Exits:         Nothing
 Date:          02/11/15
 ****************************************************/
void check_busy(void)
{
  uint32_t f;                                                                           /* General loop counter */

  f=1000000;                                                                             /* Set loop counter */
  while(ECOG_BUSY&&f--);                                                                /* Ensure not busy */
}


/*************************************
 ecog_position - Set cursor position
 Author:        B.Harris
 Enters:        X character position
                Y character position
 Exits:         Nothing
 Date:          08/10/13
 *************************************/
void ecog_position(int16_t x,int16_t y)
{
  ecogx=x;                                                                              /* Set X position */
  ecogy=y;                                                                              /* Set Y position */
}


/*****************************************
 ecog_plot -   Plot point on EDO display
 Author:       B.Harris
 Enters:       X position
               Y position
               Mode
 Exits:        Nothing
 Date:         03/11/15
 *****************************************/
void ecog_plot(int16_t x,int16_t y,uint8_t m)
{
  uint16_t pixel;                                                                       /* Pixel offset */
  uint16_t boff;                                                                        /* Byte offset in display array */
  uint8_t pdata;                                                                        /* Pixel data */

  if(x<ECOG_WIDTH&&y<ECOG_HEIGHT)                                                       /* Plottable? */
  {
//
//dprintf("Plot %d,%d\r\n",x,y);
//
//
    pixel=y*ECOG_WIDTH+x;                                                               /* Set pixel to effect */
    boff=pixel>>3;                                                                      /* Set pixel byte offset */
    pdata=0x01<<(pixel&7);                                                              /* Set pixel data */
//
//dprintf("pixel=%d,boff=%d,pdata=%d\r\n",pixel,boff,pdata);
//
    switch(m)                                                                           /* Act on mode? */
    {
      case ECOG_ON:                                                                     /* On mode? */
        ecog_buffer[boff]|=pdata;                                                       /* Store data in buffer */
        break;

      case ECOG_OFF:                                                                    /* Off mode? */
        ecog_buffer[boff]&=(255-pdata);                                                 /* Store data in buffer */
        break;

      case ECOG_XOR:                                                                    /* On mode? */
        ecog_buffer[boff]^=pdata;                                                       /* Store data in buffer */
        break;
    }
  }
}


/************************************
 ecog_box -  Display box on display
 Author:     B.Harris
 Enters:     X position
             Y position
             Width
             Height
             Mode
 Exits:      Nothing
 Date:       03/11/15
 ************************************/
void ecog_box(int16_t x,int16_t y,unsigned char w,unsigned char h,unsigned char m)
{
  unsigned char f;                                                                      /* Loop counter */

  for(f=0;f<w;f++)                                                                      /* Loop for width */
  {
    ecog_plot(x+f,y,m);                                                                 /* Top line */
    ecog_plot(x+f,y+h-1,m);                                                             /* Bottom line */
  }
  for(f=0;f<h;f++)                                                                      /* Loop for height */
  {
    ecog_plot(x,y+f,m);                                                                 /* Left line */
    ecog_plot(x+w-1,y+f,m);                                                             /* Right line */
  }
}


/************************************
 ecog_line - Draw a line on display
 Author:     B.Harris
 Enters:     X1 position
             Y1 position
             X2 position
             Y3 position
             Mode
 Exits:      Nothing
 Date:       03/11/15
 ************************************/
void ecog_line(int x1,int y1,int x2,int y2,unsigned char m)
{
  int n, deltax, deltay, sgndeltax, sgndeltay, deltaxabs, deltayabs, x, y, drawx, drawy; 

  deltax=x2-x1;
  deltay=y2-y1;
  deltaxabs=abs(deltax);
  deltayabs=abs(deltay);
  sgndeltax=misc_sgn(deltax);
  sgndeltay=misc_sgn(deltay);
  x=deltayabs>>1;
  y=deltaxabs>>1;
  drawx=x1;
  drawy=y1;
  ecog_plot(drawx, drawy,m);
  if(deltaxabs>=deltayabs)
  {
    for(n=0;n<deltaxabs;n++)
    {
      y+=deltayabs;
      if(y>=deltaxabs)
      {
        y-=deltaxabs;
        drawy+=sgndeltay;
      }
      drawx+=sgndeltax;
      ecog_plot(drawx,drawy,m);
    }
  }
  else
  {
    for(n=0;n<deltayabs;n++)
    {
      x+=deltaxabs;
      if(x>=deltayabs)
      {
        x-=deltayabs;
        drawx+=sgndeltax;
      }
      drawy+=sgndeltay;
      ecog_plot(drawx,drawy,m);
    }
  }
}


/******************************************
 ecog_cursor_move -  Move cursor position
 Author:             B.Harris
 Enters:             Character width
                     Character height
 Exits:              Nothing
 Date:               03/11/15
 *****************************************/
void ecog_cursor_move(uint8_t width,uint8_t height)
{
  ecogx+=width;                                                                         /* Update X position */
  if(ecogx>=ECOG_WIDTH)                                                                 /* X out of range? */
  {
    ecogx=0;                                                                            /* Reset X position */
    ecogy+=height;                                                                      /* Update Y position */
    if(ecogy>=ECOG_HEIGHT)                                                              /* Y out of range? */
    {
      ecogy=0;                                                                          /* Reset Y position */
    }
    ecog_position(ecogx,ecogy);                                                         /* Set display address */
  }
}


/***********************************
 ecog_putchar -  Display character
 Author:         B.Harris
 Enters:         Character
 Exits:          Nothing
 Date:           03/11/15
 **********************************/
void ecog_putchar(const uint8_t * font,unsigned char c)
{
  uint8_t f;                                                                            /* General loop counter */
  uint8_t g;                                                                            /* General loop counter */
  uint32_t d;                                                                           /* Line data */
  uint8_t fw;                                                                           /* Font width */
  uint8_t fh;                                                                           /* Font height */
  uint16_t bpc;                                                                         /* Bytes per character */
  uint8_t dw;                                                                           /* Display width */

  fw=*font++;                                                                           /* Pick up font width */
  fh=*font++;                                                                           /* Pick up font height */
  if(c<font[0]||c>font[1])                                                              /* Out of range? */
  {
    c=font[0];                                                                          /* Use first available character */
  }
  c-=font[0];                                                                           /* Set character offset */
  font+=2;                                                                              /* Point at start of font data */
  bpc=((fh+7)/8)*fw+1;                                                                  /* Set bytes per character */
  font=font+bpc*c;                                                                      /* Point at correct character data */
  dw=*font++;                                                                           /* Pick up display width */
  if(c==0)                                                                              /* Blank character? */
  {
    dw=dw/2;                                                                            /* Half display width */
  }
//
//  dprintf("Display character %d, dw=%d, BPC=%d, Address=%08lX\r\n",c,dw,bpc,font);
//
  for(f=0;f<dw;f++)                                                                     /* Loop for all lines */
  {
    d=*font++;                                                                          /* Pick up data */
    if(fh>8)                                                                            /* 2 bytes? */
    {
      d=d|(*font++)<<8;                                                                 /* Include second byte data */
    }
    if(fh>16)                                                                           /* 3 bytes? */
    {
      d=d|(uint32_t)(*font++)<<16;                                                      /* Include third byte data */
    }
    if(fh>24)                                                                           /* 4 bytes? */
    {
      d=d|(uint32_t)(*font++)<<24;                                                      /* Include fourth byte data */
    }
//
//    dprintf("fh=%d, data=%02X\r\n",fh,d);
//
    for(g=0;g<fh;g++)                                                                   /* Loop for all columns */
    {
      if(d&1)                                                                           /* Plot point? */
      {
        ecog_plot(ecogx+f,ecogy+g,ECOG_ON);                                             /* Plot point */
      }
      else
      {
        ecog_plot(ecogx+f,ecogy+g,ECOG_OFF);                                            /* Erase point */
      }
      d>>=1;                                                                            /* Shift data */
    }
  }
  ecog_cursor_move(dw+1,fh);                                                            /* Move cursor */
}


/*******************************************
 ecog_printf -   printf equivalent for LCD
 Author:         B.Harris
 Enters:         Font pointer
                 string
 Exits:          Nothing
 Date:           28/01/16
 *******************************************/
void ecog_printf(const uint8_t * font,const char *pstr)
{
  while(*pstr)                                                                          /* Loop for all characters */
  {
    ecog_putchar(font,*pstr++);                                                         /* Write to LCD */
  }
}


/**************************************************
 ecog_printfc -    LCD printf with text centering
 Author:           B.Harris
 Enters:           font
                   y position
                   string
 Exits:            Nothing
 Date:             28/01/16
 **************************************************/
void ecog_printfc(const uint8_t * font,int16_t y,const char *pstr)
{
  ecog_position((ECOG_WIDTH-ecog_text_width(font,pstr))/2,y);                           /* Position cursor */
  while(*pstr)                                                                          /* Loop for all characters */
  {
    ecog_putchar(font,*pstr++);                                                         /* Write to LCD */
  }
}

/**************************************************
 ecog_printfl -    LCD printf with text centering, left half of screen
 Author:           Peter Claydon
 Enters:           font
                   y position
                   string
 Exits:            Nothing
 Date:             28/01/16
 **************************************************/
void ecog_printfl(const uint8_t * font,int16_t y,const char *pstr)
{
  ecog_position((ECOG_WIDTH/2-ecog_text_width(font,pstr))/2,y);                         /* Position cursor */
  while(*pstr)                                                                          /* Loop for all characters */
  {
    ecog_putchar(font,*pstr++);                                                         /* Write to LCD */
  }
}

/**************************************************
 ecog_printfr -    LCD printf with text centering, right half of screen
 Author:           Peter Claydon
 Enters:           font
                   y position
                   string
 Exits:            Nothing
 Date:             28/01/16
 **************************************************/
void ecog_printfr(const uint8_t * font,int16_t y,const char *pstr)
{
  ecog_position(ECOG_WIDTH*3/4-(ecog_text_width(font,pstr))/2,y);                           /* Position cursor */
  while(*pstr)                                                                          /* Loop for all characters */
  {
    ecog_putchar(font,*pstr++);                                                         /* Write to LCD */
  }
}


/******************************************************************************
 ecog_character_width -    Return width of given character and font in pixels
 Author:                   B.Harris
 Enters:                   Character
 Exits:                    Nothing
 Date:                     30/10/13
 ******************************************************************************/
uint16_t ecog_character_width(const uint8_t * font,uint8_t c)
{
  uint8_t fw;                                                                           /* Font width */
  uint8_t fh;                                                                           /* Font height */
  uint16_t bpc;                                                                         /* Bytes per character */
  uint8_t dw;                                                                           /* Display width */

  fw=*font++;                                                                           /* Pick up font width */
  fh=*font++;                                                                           /* Pick up font height */
  if(c<font[0]||c>font[1])                                                              /* Out of range? */
  {
    c=font[0];                                                                          /* Use first available character */
  }
  c-=font[0];                                                                           /* Set character offset */
  font+=2;                                                                              /* Point at start of font data */
  bpc=((fh+7)/8)*fw+1;                                                                  /* Set bytes per character */
  font=font+bpc*c;                                                                      /* Point at correct character data */
  dw=*font++;                                                                           /* Pick up display width */
  if(c==0)                                                                              /* Blank character? */
  {
    dw=dw/2;                                                                            /* Half display width */
  }
  return(dw+1);                                                                         /* Return display width */
}


/*******************************************************************
 ecog_text_width -   Return width in pixels of given text and font
 Author:             B.Harris
 Enters:             font
                     display text
 Exits:              Width of text in pixels
 Date:               30/10/13
 *******************************************************************/
uint16_t ecog_text_width(const uint8_t * font,uint8_t *pstr)
{
  uint16_t width;                                                                       /* String width in pixels */

  width=0;                                                                              /* Reset width */
  while(*pstr)                                                                          /* Loop for entire string */
  {
    width+=ecog_character_width(font,*pstr++);                                           /* Include character in width */
  }
  return(width);                                                                        /* Return character width */
}


/***********************
 ecog_cls -  Clear LCD
 Author:     B.Harris
 Enters:     Nothing
 Exits:      Nothing
 Date:       03/11/15
 ***********************/
void ecog_cls(void)
{
  uint16_t f;                                                                           /* Loop counter */

  ecog_position(0,0);                                                                   /* Reset cursor position */
  for(f=0;f<IMAGE_COUNT;f++)                                                            /* Loop for all display data */
  {
    ecog_buffer[f]=0;                                                                   /* Clear data */
  }
}


/********************************************
 ecog_copy -   Copy image to display buffer
 Author:       B.Harris
 Enters:       Pointer to image data
 Exits:        Nothing
 Date:         11/10/13
 ********************************************/
void ecog_copy(unsigned char const *img)
{
  unsigned int i;

  for(i=0;i<IMAGE_COUNT;i++)
  {
    ecog_buffer[i]=*img++;                                                              /* Copy to RAM buffer */
  }
}


/*************************************************
 ecog_send      Write a byte to the ECOG display
 Author:        B.Harris
 Inputs:        Data to send
 Outputs:       Nothing
 Date:          02/11/15
 *************************************************/
uint8_t ecog_send(unsigned char c)
{
   ECOG_CS_ON;                                                                          /* Enable chip */
   return(spi_hw(c));                                                                   /* Write command */
}


/********************************************
 ecog_single -  Send single command to ECOG
 Author:        B.Harris
 Inputs:        Data to send
 Outputs:       Data received
 Date:          02/11/15
 ********************************************/
uint8_t ecog_single(char c)
{
  uint8_t dat;                                                                          /* Data received */

  dat=ecog_send(c);                                                                     /* Send command */
  ECOG_CS_OFF;                                                                          /* Disable chip */
  return(dat);                                                                          /* Return data */
}


/**************************************************
 ecog_read      Read a byte from the ECOG display
 Author:        B.Harris
 Inputs:        Index to read
 Outputs:       Nothing
 Date:          02/11/15
 **************************************************/
uint8_t ecog_read(uint8_t index)
{
   ecog_send(0x70);                                                                     /* Set index command */
   ecog_single(index);                                                                  /* Send index */
   ecog_send(0x73);                                                                     /* Read command */
   return(ecog_single(0x00));                                                           /* Return data read */
}


/*************************************************************
 ecog_write -   Write a value to the register at given index
 Author:        B.Harris
 Inputs:        Index to write to
                Data to write
 Outputs:       Nothing
 Date:          02/11/15
 *************************************************************/
void ecog_write(uint8_t index,uint8_t dat)
{
   ecog_send(0x70);                                                                     /* Set index command */
   ecog_single(index);                                                                  /* Send index */
   ecog_send(0x72);                                                                     /* Write command */
   ecog_single(dat);                                                                    /* Write data */
}


/*****************************************************************
 ecog_set_display_data -   Write display data for display in use
 Author:                   B.Harris
 Inputs:                   Nothing
 Outputs:                  Nothing
 Date:                     02/11/15
 *****************************************************************/
void ecog_set_display_data(void)
{
   ecog_send(0x70);                                                                     /* Set index command */
   ecog_single(0x01);                                                                   /* Send index */
   ecog_send(0x72);                                                                     /* Write command */
   ecog_send(0x00);                                                                     /* Output data for 2inch display - 0x0000 0000 01ff e000 */
   ecog_send(0x00);
   ecog_send(0x00);
   ecog_send(0x00);
   ecog_send(0x01);
   ecog_send(0xff);
   ecog_send(0xe0);
   ecog_single(0x00);
}


/********************************
 ecog_read_id -   Read COG_ID
 Author:          B.Harris
 Inputs:          Index to read
 Outputs:         Nothing
 Date:            02/11/15
 ********************************/
uint8_t ecog_read_id(void)
{
   ecog_send(0x71);                                                                     /* Set index command */
   return(ecog_single(0x00));                                                           /* Return data read */
}


/*************************************************
 ecog_power_display -   Power up the COG display
 Author:                B.Harris
 Inputs:                Nothing
 Outputs:               Nothing
 Date:                  03/11/15
 *************************************************/
void ecog_power_display(void)
{
  ECOG_POWER_ON;                                                                        /* Power up display */
  ECOG_DISCHARGE_OFF;                                                                   /* Discharge off */
  ECOG_CS_OFF;                                                                          /* Ensure device is deselected */
  ECOG_RESET_OFF;                                                                       /* Take out of reset */
  timers_sdelay(5);                                                                     /* 5ms delay */
  ECOG_RESET_ON;                                                                        /* Put display into reset */
  timers_sdelay(5);                                                                     /* 5ms delay */
  ECOG_RESET_OFF;                                                                       /* Take out of reset */
  timers_sdelay(5);                                                                     /* Short delay */
  spi_init();                                                                          /* Initialise SPI */
 // PCX
 // GPIOA_MODER=(GPIOA_MODER&0xffff33ff)|0x00008800;                                      /* Configure SPI lines as outputs */

}


/***********************************************
 ecog_initialise_cog -   Initialise COG driver
 Author:                 B.Harris
 Inputs:                 Nothing
 Outputs:                1=success, 0=fail
 Date:                   03/11/15
 ***********************************************/
uint8_t ecog_initialise_cog(void)
{
  uint8_t spidat;                                                                       /* SPI data */
  uint8_t attempts;                                                                     /* Charge attempt counter */

  check_busy();                                                                         /* Ensure we're not busy */
  spidat=ecog_read_id();                                                                /* Read COG_ID */
  if(spidat!=0x12)                                                                      /* Wrong ID? */
  {
    spidat=ecog_read_id();                                                              /* Read COG_ID */
    if(spidat!=0x12)                                                                    /* Wrong ID? */
    {
//      debug_log("Got unexpected COG_ID=%02X",spidat);                                   /* Indicate error */
      char buffer[30] = {" "};
  	  sprintf(buffer, "Got unexpected COG_ID=%02X\r\n", spidat);
  	  DEBUG_TX(buffer);
      //UART1_Write_Text("Got unexpected COG_ID\r\n");
      return(0);                                                                        /* Fail */
    }
  }
  ecog_write(0x02,0x40);                                                                /* Disable OE */
  if((ecog_read(0x0f)&0x80)==0)                                                         /* Check breakage */
  {
//    debug_log("Display is broken");
	DEBUG_TX("Display is broken");
    return(0);                                                                          /* Fail */
  }
  ecog_write(0x0b,0x02);                                                                /* Power saving mode */
  ecog_set_display_data();                                                              /* Set display data */
  ecog_write(0x07,0xd1);                                                                /* High power mode Osc setting */
  ecog_write(0x08,0x02);                                                                /* Power setting */
  ecog_write(0x09,0xc2);                                                                /* Set Vcom level */
  ecog_write(0x04,0x03);                                                                /* Power setting */
  ecog_write(0x03,0x01);                                                                /* Driver latch on */
  ecog_write(0x03,0x00);                                                                /* Driver latch off */
  timers_sdelay(5);                                                                     /* 5ms delay */
  attempts=1;                                                                           /* First chargepump */
  while(1)                                                                              /* Loop until charge complete */
  {
    ecog_write(0x05,0x01);                                                              /* Start chargepump positive */
    timers_sdelay(150);                                                                 /* 150ms delay */
    ecog_write(0x05,0x03);                                                              /* Start chargepump negative */
    timers_sdelay(90);                                                                  /* 40ms delay */
    ecog_write(0x05,0x0f);                                                              /* Set chargepump Vcom driver to on */
    timers_sdelay(40);                                                                  /* 40ms delay */
    if((ecog_read(0x0f)&0x40)==0x40)                                                    /* Chargepump ready? */
    {
      ecog_write(0x02,0x06);                                                            /* Output enable to disable */
      break;                                                                            /* Ready to update */
    }
    else
    {
      attempts++;                                                                       /* Another attempt */
      if(attempts==4)                                                                   /* Fail? */
      {
//        debug_log("Chargepump failure");
        return(0);                                                                      /* Fail */
      }
    }
  }
  return(1);                                                                            /* Success */
}


/************************************************
 ecog_send_scan_line -   Initialise COG driver
 Author:                 B.Harris
 Inputs:                 Scan line,
                         Pointer to data buffer
                         LINE_TYPE
 Outputs:                1=success, 0=fail
 Date:                   03/11/15
 ************************************************/
void ecog_send_scan_line(uint8_t y,uint8_t * dp,uint8_t type)
{
  uint8_t scan;                                                                         /* Scan data loop counter */
  uint8_t db;                                                                           /* Data byte loop counter */
  uint8_t dat;                                                                          /* Data to write to display */
  uint8_t dat1;
//
//dprintf("Start of scan data, line %u\r\n",y);
//
  check_busy();                                                                         /* Ensure we're not busy */
  ecog_send(0x70);                                                                      /* Set index command */
  ecog_single(0x0a);                                                                    /* Send index */
  ecog_send(0x72);                                                                      /* Ready to send data */
  ecog_send(0x00);                                                                      /* Border byte */
  for(db=0;db<25;db++)                                                                  /* Loop for all odd x pixels */
  {
    switch(type)                                                                        /* Act on scan type */
    {
      case LINE_TYPE_WHITE:                                                             /* Display white lines? */
           dat=0b10101010;                                                              /* White data */
           break;

      case LINE_TYPE_BLACK:                                                             /* Display black lines? */
           dat=0b11111111;                                                              /* Black data? */
           break;

      case LINE_TYPE_NORMAL:                                                            /* Display normal lines? */
           dat=dp[24-db]&0b01010101;                                                    /* Pick up odd screen data */

           dat1=dat;

           dat=dat|0b10101010;                                                          /* Set to black or white */
           break;

      case LINE_TYPE_INVERSE:                                                           /* Display inverted lines? */
           dat=(dp[24-db]^255)&0b01010101;                                              /* Pick up inverted odd screen data */
           dat=dat|0b10101010;                                                          /* Set to black or white */
           break;

      case LINE_TYPE_SKIP:                                                              /* Dsplay skip lines? */
           dat=0b00000000;                                                              /* Set skip data */
           break;
    }
//dprintf("Odd %u=%02X=%02X\r\n",db,dat1,dat);
    ecog_send(dat);                                                                     /* Output pixel data */
  }
//dprintf("Sending scan data\r\n");
  for(scan=0;scan<24;scan++)                                                            /* Loop for all scan lines */
  {
    if(scan==((95-y)>>2))                                                               /* Scan line found? */
    {
      switch(y&3)                                                                       /* Act on scan line */
      {
        case 0:
             ecog_send(0x03);                                                           /* Select this line */
             break;
 
        case 1:
             ecog_send(0x0c);                                                           /* Select this line */
             break;

        case 2:
             ecog_send(0x30);                                                           /* Select this line */
             break;

        case 3:
             ecog_send(0xc0);                                                           /* Select this line */
             break;
      }
    }
    else
    {
      ecog_send(0x00);                                                                  /* No scan lines selected */
    }
  }
  for(db=0;db<25;db++)                                                                  /* Loop for even x pixels */
  {
    switch(type)                                                                        /* Act on scan type */
    {
      case LINE_TYPE_WHITE:                                                             /* Display white lines? */
           dat=0b10101010;                                                              /* White data */
           break;

      case LINE_TYPE_BLACK:                                                             /* Display black lines? */
           dat=0b11111111;                                                              /* Black data? */
           break;

      case LINE_TYPE_NORMAL:                                                            /* Display normal lines? */
           dat=misc_reverse_byte(dp[db]);
           dat1=dat;
           dat|=0b10101010;                                                             /* Set write pattern */
           break;

      case LINE_TYPE_INVERSE:                                                           /* Display inverted lines? */
           dat=(misc_reverse_byte(dp[db])^255)|0b10101010;                              /* Set write pattern */
           break;

      case LINE_TYPE_SKIP:                                                              /* Dsplay skip lines? */
           dat=0b00000000;                                                              /* Set skip data */
           break;
    }
//dprintf("Even %u=%02X=%02X\r\n",db,dat1,dat);
    ecog_send(dat);                                                                     /* Output pixel data */
  }
  ECOG_CS_OFF;                                                                          /* Disable chip */
  ecog_write(0x02,0x07);                                                                /* Turn on output enable */
//
//  dprintf("End of scan data\r\n");
//
}


/*********************************************************
 ecog_write_screen_data -   Write screen data to display
 Author:                    B.Harris
 Inputs:                    Nothing
 Outputs:                   Nothing
 Date:                      10/11/15
 *********************************************************/
uint8_t ecog_write_screen_data(void)
{
  uint8_t f;                                                                            /* General loop counter */
  uint8_t y;                                                                            /* Y loop counter */
  uint8_t multiple = 4;
  //uint8_t buffer[25];                                                                   /* 1 scan line data */
  //uint8_t * bp;                                                                         /* Buffer pointer */

  //for(f=0; f<1; f++)                                                                    /* Default is 4 */
  for(f=0; f<multiple; f++)                                                                    /* Default is 4 */
	  for(y=0;y<96;y++)                                                                 /* Loop for all Y data */
	  {
		  ecog_send_scan_line(y,&ecog_buffer[y*25],LINE_TYPE_BLACK);                    /* Scan line of data */
	  }
  //timers_sdelay(150);  // PCX added
  //for(f=0; f<2; f++)                                                                    /* Default is 4 */
  for(f=0; f<multiple; f++)                                                                    /* Default is 4 */
	  for(y=0;y<96;y++)                                                                 /* Loop for all Y data */
	  {
		  ecog_send_scan_line(y,&ecog_buffer[y*25],LINE_TYPE_WHITE);                    /* Scan line of data */
	  }
  //timers_sdelay(150);  // PCX added
  ecog_write_inverse(1);

  //for(f=0; f<6; f++)   // "Low temp"
  for(f=0; f<multiple; f++)   // "Test temp" Default is 12
	  for(y=0;y<96;y++)                                                                 /* Loop for all Y data */
	  {
		  ecog_send_scan_line(y,&ecog_buffer[y*25],LINE_TYPE_NORMAL);                    /* Scan line of data */
	  }
}

void ecog_blank_screen(int line_type)
{
	uint8_t f;                                                                            /* General loop counter */
	uint8_t y;                                                                            /* Y loop counter */
	for(y=0;y<96;y++)                                                                 	  /* Loop for all Y data */
	{
		ecog_send_scan_line(y,&ecog_buffer[y*25],line_type);                              /* Scan line of data */
	}
}

/*************************************************************
 ecog_block_type -   Write block type screen data to display
 Author:             B.Harris
 Inputs:             Repeat number
                     Block
                     Step
                     Inverse display flag
 Outputs:            Nothing
 Date:               10/11/15
 *************************************************************/
//uint8_t ecog_block_type(uint8_t repeat,uint8_t block,uint8_t step,uint8_t inverse)
//{
//  uint8_t f;                                                                            /* General loop counter */
//  uint8_t buffer[25];                                                                   /* 1 scan line data */
//  uint8_t * bp;                                                                         /* Buffer pointer */
//  uint8_t x,y,z;                                                                        /* Loop counters */
//
//
//  for(x=0;x<repeat;x++)                                                                 /* Loop for all repeat cycles */
//  {
//    for(y=0;y<96;y++)                                                                   /* Loop for all scan lines */
//    {
//      z=1;                                                                              /* Reset Z */
//
//
//    if(thisscaninrangeofblock)
//    {
//      ecog_send_scan_line(scanline,&ecog_buffer[scanline*25],inverseflag);              /* Send scan line */
//
//  bp=ecog_buffer;                                                                       /* Point at start of display buffer */
//  for(y=0;y<96;y++)                                                                     /* Loop for all Y data */
//  {
//    for(f=0;f<25;f++)                                                                   /* Loop for 1 scan line */
//    {
//      buffer[f]=*bp++^255;                                                              /* Store inverted image */
//    }
//    ecog_send_scan_line(y,buffer);                                                      /* Scan line of data */
//  }
//  timers_sdelay(150);
//  for(y=0;y<96;y++)                                                                     /* Loop for all Y data */
//  {
//    ecog_send_scan_line(y,&ecog_buffer[y*25]);                                          /* Scan line of data */
//  }
//  timers_sdelay(150);
//}


/*************************************************************
 ecog_write_nothing_frame -   Write nothing frame to display
 Author:                      B.Harris
 Inputs:                      Nothing
 Outputs:                     Nothing
 Date:                        03/11/15
 *************************************************************/
uint8_t ecog_write_nothing_frame(void)
{
  uint8_t y;                                                                            /* Y loop counter */
  uint8_t scan;                                                                         /* Scan data loop counter */
  uint8_t db;                                                                           /* Data byte loop counter */

  for(y=0;y<96;y++)                                                                     /* Loop for all Y data */
  {
    check_busy();                                                                       /* Ensure we're not busy */
    ecog_send(0x70);                                                                    /* Set index command */
    ecog_single(0x0a);                                                                  /* Send index */
    ecog_send(0x72);                                                                    /* Prepare to send data */
    ecog_send(0x00);                                                                    /* Send border byte */
    for(db=0;db<25;db++)                                                                /* Loop for all odd x pixels */
    {
      ecog_send(0x00);                                                                  /* All pixels skipped */
    }
    for(scan=0;scan<24;scan++)                                                          /* Loop for all scan lines */
    {
      if(scan==(y/4))                                                                   /* Scan line found? */
      {
        switch(y&3)                                                                     /* Act on scan line */
        {
          case 0:
               ecog_send(0x03);                                                         /* Select this line */
               break;

          case 1:
               ecog_send(0x0c);                                                         /* Select this line */
               break;

          case 2:
               ecog_send(0x30);                                                         /* Select this line */
               break;

          case 3:
               ecog_send(0xc0);                                                         /* Select this line */
               break;
        }
      }
      else
      {
        ecog_send(0x00);                                                                /* No scan lines selected */
      }
    }
    for(db=0;db<25;db++)                                                                /* Loop for even x pixels */
    {
      ecog_send(0x00);                                                                  /* All pixels skipped */
    }
    ECOG_CS_OFF;                                                                        /* Disable chip */
    ecog_write(0x02,0x07);                                                              /* Turn on output enable */
  }
}


/************************************
 ecog_write_border -   Write border
 Author:               B.Harris
 Inputs:               Nothing
 Outputs:              Nothing
 Date:                 03/11/15
 ************************************/
uint8_t ecog_write_border(void)
{
  uint8_t scan;                                                                         /* Scan data loop counter */
  uint8_t db;                                                                           /* Data byte loop counter */

  check_busy();                                                                         /* Ensure we're not busy */
  ecog_send(0x70);                                                                      /* Set index command */
  ecog_single(0x0a);                                                                    /* Send index */
  ecog_send(0x72);                                                                      /* Prepare to send data */
  ecog_send(0xaa);                                                                      /* Send border byte */
  for(db=0;db<25;db++)                                                                  /* Loop for all odd x pixels */
  {
    ecog_send(0x00);                                                                    /* All pixels skipped */
  }
  for(scan=0;scan<24;scan++)                                                            /* Loop for all scan lines */
  {
    ecog_send(0x00);                                                                    /* No scan lines selected */
  }
  for(db=0;db<25;db++)                                                                  /* Loop for even x pixels */
  {
    ecog_send(0x00);                                                                    /* All pixels skipped */
  }
  ECOG_CS_OFF;                                                                          /* Disable chip */
  ecog_write(0x02,0x07);                                                                /* Turn on output enable */
  timers_sdelay(150);
}


/****************************************************
 ecog_discharge_capacitors -   Discharge capacitors
 Author:                       B.Harris
 Inputs:                       Nothing
 Outputs:                      Nothing
 Date:                         03/11/15
 ****************************************************/
void ecog_discharge_capacitors(void)
{
  int discharge_time = 150;
  ecog_write(0x0b,0x00);
  ecog_write(0x03,0x01);                                                                /* Latch reset turn on */
  ecog_write(0x05,0x03);                                                                /* Power off chargepump */
  ecog_write(0x05,0x01);                                                                /* Power off negative chargepump */
//  io_output(O_BORDER_CONTROL,0);
  // PCX Was 1000
  timers_sdelay(300);                                                                  /* Delay to allow discharge */
  //timers_sdelay(1000);                                                                  /* Delay to allow discharge */
//  io_output(O_BORDER_CONTROL,1);
  ecog_write(0x04,0x80);                                                                /* Discharge internal */
  ecog_write(0x05,0x00);                                                                /* Turn off all chargepumps */
  ecog_write(0x07,0x01);                                                                /* Turn off oscillator */
  timers_sdelay(50);                                                                    /* Delay 50ms */
  spi_deinit();                                                                         /* Deinitialise SPI */

  // PCX Moved here
  ECOG_POWER_OFF;                                                                       /* Power off display */
  //timers_sdelay(10);                                                                    /* Delay 50ms */
  timers_sdelay(50);                                                                    /* Delay 50ms */
  // PCX
  //spi_deinit();                                                                         /* Deinitialise SPI was here*/
  // PCX
  //GPIOA_ODR&=0xff5f;                                                                    /* PA5 and PA7 low */
  //GPIOA_MODER=(GPIOA_MODER&0xffff33ff)|0x00004400;                                      /* Configure SPI lines PA5 and PA7 as outputs */

//  io_output(PORTA+5,0);                                                                 /* SPI clock low */
//  io_output(PORTA+7,0);
//
//
//
//
//dprintf("Done that\r\n");
//

  timers_sdelay(10);                                                                    /* Delay 10ms */
  ECOG_RESET_ON;                                                                        /* Reset line low */
  ECOG_CS_ON;                                                                           /* Select device */
  ECOG_DISCHARGE_ON;                                                                    /* Discharge display */
  if(temperature > 25)
	  discharge_time = 10;
  else if(temperature > 15)
	  discharge_time = 200;
  else
	  discharge_time = 400;
  discharge_time = 200;
  //sprintf(debug_buff, "Discharge: %d\r\n", discharge_time);
  //DEBUG_TX(debug_buff);
  timers_sdelay(discharge_time);                                                        /* Delay 150ms */
  ECOG_DISCHARGE_OFF;                                                                   /* Discharge off */
}


/*****************************************************
 ecog_update_display -   Update display image on LCD
 Author:                 B.Harris
 Enters:                 Nothing
 Exits:                  1=passed, 0=failed
 Date:                   03/11/15
 *****************************************************/
uint8_t ecog_update_display(uint8_t powered)
{
  uint8_t init_ok;
  if (!powered)
  {
	  //UART1_Write_Text("Powering display\r\n");
	  ecog_power_display();                                                              /* Power up the display */
	  //UART1_Write_Text("Display powered\r\n");
	  init_ok = ecog_initialise_cog();
  }
  else
	  init_ok = 1;
  if(init_ok)
  {
    //UART1_Write_Text("Writing screen data\r\n");
    ecog_write_screen_data();                                                           /* Write screen buffer data */
    //UART1_Write_Text("Writing nothing frame\r\n");
    ecog_write_nothing_frame();                                                         /* Write nothing frame to stabilise display */
    //UART1_Write_Text("Writing border frame\r\n");
    ecog_write_border();                                                                /* Write border data */
    //UART1_Write_Text("Discharging capacitors\r\n");
    ecog_discharge_capacitors();                                                        /* Discharge capacitors and power down display */

    return(1);                                                                          /* Success */
  }
  else
  {
    UART1_Write_Text("Failed initialising COG\r\n");
    UART1_Write_Text("Discharging capacitors\r\n");
    ecog_discharge_capacitors();                                                          /* Discharge capacitors */
    return(0);
  }
}

uint8_t ecog_write_inverse(uint8_t powered)
{
  uint8_t f;                                                                          /* General loop counter */
  uint8_t y;                                                                          /* Y loop counter */

  if(!powered)
  {
	  //UART1_Write_Text("Powering display\r\n");
	  ecog_power_display();                                                           /* Power up the display */
	  //UART1_Write_Text("Display powered\r\n");
  }
  if(ecog_initialise_cog())                                                           /* Initialise COG driver? */
  {
	  for(f=0; f<1; f++)                                                              /* Default is 3 */
	  {
		  //for(y=0;y<96;y++)                                                           /* Loop for all Y data */
		  //{
			  //ecog_send_scan_line(y,&ecog_buffer[y*25],LINE_TYPE_INVERSE);            /* Scan line of data */
		  //}
	  }
  }
  else
  {
	  UART1_Write_Text("Failed initialising COG\r\n");
      UART1_Write_Text("Discharging capacitors\r\n");
      ecog_discharge_capacitors();                                                    /* Discharge capacitors */
      return(0);
  }
}



/******************************
 ecog_init -   Initialise LCD
 Author:       B.Harris
 Enters:       Nothing
 Exits:        Nothing
 Date:         02/11/15
 ******************************/
uint8_t ecog_init(void)
{
  ecog_cls();                                                                           /* Clear screen */
  return(ecog_update_display(0));                                                       /* Update display */
}


/******************************************
 ecog_disable_display -    Disable display
 Author:                  B.Harris
 Enters:                  Nothing
 Exits:                   Nothing
 Date:                    27/01/14
 ******************************************/
uint8_t ecog_disable_display(void)
{
  if(ecog_display_enabled)                                                              /* Currently enabled? */
  {
    ecog_cls();                                                                         /* Clear display */
    ecog_update_display(0);                                                             /* Update display */
    ecog_display_enabled=0;                                                             /* Display is disabled */
  }
}


/*****************************************
 ecog_enable_display -     Enable display
 Author:                  B.Harris
 Enters:                  Nothing
 Exits:                   Nothing
 Date:                    27/01/14
 *****************************************/
uint8_t ecog_enable_display(void)
{
  if(!ecog_display_enabled)                                                             /* Currently enabled? */
  {
    ecog_display_enabled=1;
    ecog_update_display(0);                                                              /* Update display */
  }
}
