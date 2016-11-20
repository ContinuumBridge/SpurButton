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

const uint32_t node_id_int = 17;

void Set_Node_ID(uint8_t *node_id)
{
  node_id[3] = (node_id_int & 0xFF);
  node_id[2] = (node_id_int >> 8) & 0xFF;
  node_id[1] = (node_id_int >> 16) & 0xFF;
  node_id[0] = (node_id_int >> 24) & 0xFF;
}


