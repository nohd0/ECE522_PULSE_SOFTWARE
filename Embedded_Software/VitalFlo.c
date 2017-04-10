/**************************************************************************************************
  Filename:       sleepiBand.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_i2c.h"
#include "hal_uart.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "vitalProfile.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "VitalFlo.h"
#include "sampler.h"
#include "cma3000d.h"
#include "i2c_wrapper.h"
#include "stdio.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     6

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          2000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */
   
  //Period for sampling EEG
#define INIT_period          1000


//Delay Offset for CTL
#define ctl_offset -1
   
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 vitalFlo_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

uint8 buffer_count = 0;
uint8 adc_buffer[20];

//Characteristics
uint8 deviceEnable = 0;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E',
  'x',
  'a',
  'm',
  'p',
  'l',
  'e',
  ' ',
  'B',
  'L',
  'E',
  '4',
  '.',
  '0',
  '*',
  '*',
  '*',
  '*',
  '*',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
*  INTERRUPTS
*/

//Variables for Interrupt
uint8 adc_reg = 0x00;
uint8 adc_reading;

#pragma vector = T1_VECTOR
__interrupt void T1_ISR(void)
{
  HAL_ENTER_ISR();
  T1IF = 0;
  
  HalI2CInit(0x64, i2cClock_123KHZ);
  HalI2CWrite(1, &adc_reg, TRUE);
  HalI2CRead(1, &adc_reading);
  
  osal_memcpy(adc_buffer + buffer_count, &adc_reading, 1);
  
  if(buffer_count == 19){
    buffer_count = 0;
    osal_set_event(vitalFlo_TaskID, ADC_EVT);
  }else
    buffer_count++;
  
  
  HAL_EXIT_ISR();
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void vitalFlo_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void vitalProfileChangeCB( uint8 paramID );   


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t vitalFlo_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t vitalFlo_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t vitalFlo_SimpleProfileCBs =
{
  vitalProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      sleepiBand_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  non
 */
void VitalFlo_Init( uint8 task_id )
{
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT);
  HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE );
  
  
  vitalFlo_TaskID = task_id;

  /*************PIN SETUP*************/

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFF;         //P0.0-P0.7 - Output
  P1DIR = 0xC7;         //P1.3-P1.5 - Input (Rest Output)
  P2DIR = 0x07;         //P2.0-P2.2 - Output

  APCFG = 0x00; //Or 0x7F?
  
  P1_0 = 0x1;   //Turn on LEDs
  P1_1 = 0x1;
  P1_2 = 0x0;   //TURN OFF, OFF PINOUT
  
  /**************TIMER 1 SETUP********************/
  PERCFG &= ~(0x1 << 6);        //Timer 1 Alternative Location 1
  P0DIR = (0x2 << 6);           //Timer 1 Channel 0/1 Priority
  T1CTL = (0x3 << 2);           //Tick Freq / 128
  P0SEL |= (0x1 << 2);          //P0_2 Peripheral (Timer 1 Channel 0)
  IEN1 |= (0x1 << 1);           //Enable Timer 1 Interrupt
  printf("%d", CLKCONCMD);      //Use this to check clock speed
  
  T1CCTL1 = 0x00;
  T1CCTL2 = 0x00;
  T1CCTL3 = 0x00;
  T1CCTL4 = 0x00;
  
  T1CCTL0 = (0x1 << 6) |        //Enable Channel 0 Interrupt
            (0x1 << 2) |        //Compare Mode
            (0x0);              //Default - No Capture, etc.
  
  T1CC0L = 0xC4;                //10ms Duty Cycle
  T1CC0H = 0x09;
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  
  VitalProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  
  // Register callback with SimpleGATTprofile
  VOID VitalProfile_RegisterAppCBs( &vitalFlo_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  HalAdcInit();
  HalAdcSetReference(HAL_ADC_REF_AVDD);
  
  // Setup a delayed profile startup
  osal_set_event( vitalFlo_TaskID, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      vitalFlo_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */

uint16 VitalFlo_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( vitalFlo_TaskID )) != NULL )
    {
      vitalFlo_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &vitalFlo_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &vitalFlo_BondMgrCBs );
    
    osal_pwrmgr_device( PWRMGR_BATTERY );
    
    return ( events ^ START_DEVICE_EVT );
  }
  
  if ( events & ACCEL_EVT){
    readAccel();
    
    if(deviceEnable){
      osal_start_timerEx( vitalFlo_TaskID, ACCEL_EVT, 100); 
    }
    return ( events ^ ACCEL_EVT);
  }
  
  if ( events & ADC_EVT){
    
    VitalProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, adc_buffer);
    
    return ( events ^ ADC_EVT);
  }
  
  return 0;
}

static void initAccel(){
    uint8 write2[2];
    
    HalI2CInit(0x1D, i2cClock_123KHZ);
    write2[0] = 0x2A; //CTRL_REG1
    write2[1] = 0x02; //Fast-Read
    HalI2CWrite(2, write2, FALSE);
    
    write2[1] = 0x03; //Active Mode
    HalI2CWrite(2, write2, FALSE);
}

static void readAccel(){
    uint8 write = 0x00;
    uint8 accelData[4];
    
    HalI2CInit(0x1D, i2cClock_123KHZ);
    
    HalI2CWrite(1, &write, FALSE);
    HalI2CRead(4, accelData);
    printf("%d, %d, %d, %d", accelData[0], accelData[1], accelData[2], accelData[3]);
    VitalProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, accelData);
}

static void initADC(){
    uint8 write2[2];
    HalI2CInit(0x64, i2cClock_123KHZ);
    
    write2[0] = 0x02;
    write2[1] = 0x00;   //Default - 0x00 - Conversion on Read
    HalI2CWrite(2, write2, FALSE);
}


static void waitUs(uint16 a){
  uint16 i = 0;
  for(i = 0; i < a; i++){
    /* 32 NOPs == 1 usecs */
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
       asm("nop"); asm("nop");
  }
}



/*********************************************************************
 * @fn      sleepiBand_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void vitalFlo_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {

  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      {
      }
      break;

    case GAPROLE_CONNECTED:
      {
          
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
      }
      break;      
    case GAPROLE_WAITING:
      {
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
      }
      break;

    default:
      {
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}
/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void vitalProfileChangeCB( uint8 paramID )
{
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:    
      VitalProfile_GetParameter( SIMPLEPROFILE_CHAR1, &deviceEnable);
      
      if(deviceEnable){
        initADC();
        initAccel();
    
        osal_start_timerEx( vitalFlo_TaskID, ACCEL_EVT, 100); 
        T1CTL |= (0x2 << 0);
      }else{
        T1CTL &= ~(0x2 << 0);
      }
      
      break;
    
    default:
      // should not reach here!
      break;
  }
}


