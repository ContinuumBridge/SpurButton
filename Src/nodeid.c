/**
  ******************************************************************************
  * File Name          : nodeid.c
  * Description        : Sets the Node ID
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 ContinuumBridge Ltd
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <inttypes.h>
#include "nodeid.h"
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x2F};  // Battery 47
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x0A};  // Development
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x10};  // Brexit
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x11};  // 17 - Bench Test
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x12};  // 18 - Martin's
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x13};  // Smart IoT 19
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x14};  // Smart IoT 20
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x15};  // Smart IoT 21
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x16};  // Smart IoT 22
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x17};  // 23
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x18};  // 24
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x19};  // 25
 //uint8_t 			node_id[] 			= {0x00, 0x00, 0x00, 0x1A};  // 26
// 27 MobileWorks
// 28 Martin Spur 2
// 29 Martin Spur 2
// 30 Spare
// 31 Martin Spur 2
// 32 Steve Spur 2
// 33 Replacement old one
// 37

const uint32_t node_id_int = 30;

void Set_Node_ID(uint8_t *node_id)
{
  node_id[3] = (node_id_int & 0xFF);
  node_id[2] = (node_id_int >> 8) & 0xFF;
  node_id[1] = (node_id_int >> 16) & 0xFF;
  node_id[0] = (node_id_int >> 24) & 0xFF;
}


