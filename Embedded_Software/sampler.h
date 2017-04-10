
#ifndef ROACHZSTACK_ADC_H
#define ROACHZSTACK_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "osal.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern byte RoachZStack_ADC_TaskID;


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Serial Transfer Application
 */
extern void Sampler_Init( byte task_id );

/*
 * Task Event Processor for the Serial Transfer Application
 */
extern UINT16 Sampler_ProcessEvent( byte task_id, UINT16 events );

/*
 * Task Event Processor for the Serial Transfer Application
 */
extern void Sampler_SetState( bool isEnabled );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ROACHZSTACK_H */
