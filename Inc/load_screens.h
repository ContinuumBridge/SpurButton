/**
  ******************************************************************************
  * @file    load_screens.h
  * @brief   Loads screen text & lines for system & demo screens
  ******************************************************************************
  *
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOADSCREENS_H
#define __LOADSCREENS

#ifdef __cplusplus
 extern "C" {
#endif 

#define MAX_SCREEN 			38
void Load_Normal_Screens(void);
void Load_Demo_Screens(void);

#ifdef __cplusplus
}
#endif

#endif /* __CBUTILS_H */

/*****************************END OF FILE****/
