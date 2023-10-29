/***********************************************************************************

    Filename:     usb_cdc_hooks.h

    Description:  USB Virtual UART interface.

***********************************************************************************/
#ifndef USB_CDC_HOOKS_H
#define USB_CDC_HOOKS_H

#include "hal_types.h"

typedef struct {
  uint32 dteRate;
  uint8 charFormat;
  uint8 parityType;
  uint8 dataBits;
} CDC_LINE_CODING_STRUCTURE;

extern CDC_LINE_CODING_STRUCTURE currentLineCoding;


#endif
