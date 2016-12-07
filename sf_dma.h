#ifndef SF_DMA_H
#define SF_DMA_H

#include "FreeRTOS.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xiicps.h"
#include "semphr.h"
#include "sf_coms.h"

extern XScuGic xInterruptController;
int test_acc();

#endif
