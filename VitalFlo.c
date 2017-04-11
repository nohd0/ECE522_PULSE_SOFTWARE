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
#include "sleepiProfile.h"
#include "accelerometer.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

  #include "peripheral.h"

#include "gapbondmgr.h"

#include "VitalFlo.h"
#include "sampler.h"
#include "cma3000d.h"
#include "i2c_wrapper.h"


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

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

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

// How often (in ms) to read the accelerometer
#define ACCEL_READ_PERIOD             50

// Minimum change in accelerometer before sending a notification
#define ACCEL_CHANGE_THRESHOLD        2

/*********************************************************************
 * TYPEDEFS
 */

   // Delay for which each LED1 stays ON (in ms)
#define LED_1_period      1000

// Delay for which each LED2 stays ON (in ms)
#define LED_2_period      1000

// Dead Delay for which no LED stays ON (in ms)
#define LED_off_period      100
   
   
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
static uint8 sleepiBand_TaskID;   // Task ID for internal task/event processing
uint8 glob_buffer_pointer = 0;
uint8 adc_buffer[60];

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Accelerometer Profile Parameters
static uint8 accelEnabler = FALSE;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'l',
  'e',
  'e',
  'p',
  'i',
  'B',
  'a',
  'n',
  'd',
  ' ',
  'v',
  '2',
  '.',
  '0',
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

#pragma vector = T1_VECTOR
__interrupt void T1_ISR(void)
{
  HAL_ENTER_ISR();
  T1IF = 0;
  
  
  HAL_EXIT_ISR();
}

#pragma vector = T3_VECTOR
__interrupt void T3_ISR(void)
{
  HAL_ENTER_ISR();
  T3IF = 0;
  uint8 adc_reading[2];
  
  /*
  //sample from adc (PCF8591)
  HalI2CInit( 0x48, i2cClock_33KHZ );
  uint8 writeData[1] = {0x04};
  HalI2CWrite( 1, writeData, FALSE );
  HalI2CRead( 1, adc_reading );
  */
  adc_reading[0] = glob_buffer_pointer;
  osal_memcpy(adc_buffer+glob_buffer_pointer,adc_reading,1);
  
  if(glob_buffer_pointer == 59){
    glob_buffer_pointer = 0;
    osal_set_event(sleepiBand_TaskID, INIT_evt);
  }
  else{
    glob_buffer_pointer++;
  }
  
  HAL_EXIT_ISR();
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sleepiBand_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void accelEnablerChangeCB( void );
static void accelRead( void );              // Amey

static void Pot1ChangeCB( void );       //James
static void Pot2ChangeCB( void );       //James



static void WaitUs( uint16 microseconds );       //James


static void Init_I2C( void ); //Allie
static void Init_SI1143( void ); //Allie
static void Init_LMP( void );
static void battery_init(void);

static void ads_init( void );
static void afe_init( void );

static void adxl_init( void );
static void ads1115_init(uint16 channel);
static void stc3105_init(void);


static void set_pot2( uint8 channel, uint8 data );       //James

#if defined( CC2540_MINIDK )
static void sleepiBand_HandleKeys( uint8 shift, uint8 keys );
#endif 

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static uint16 glob_inc = 4;
static uint8 global_assist_ozone[2];
static int UV_first_time=1;

uint8 combined_readings[11];

static int8 glob_inc1 = 0;
static int8 glob_inc2 = 0;

static uint8 glob_uv=0;

static uint8 glob_motion[2];

static uint24 glob_dat1 = 0;
static uint24 glob_dat2 = 0;
static uint24 glob_dat3 = 0;

static uint8 timer4_state = 0;

static uint8 simpleProfilePot1[2]; 
static uint8 simpleProfilePot2[2];
bool enable_all_notes = false;

// Pin assignments for SPI of MCP42100 ***************

#define CS_MCP      P1_2
#define CS_HIGH     1
#define CS_LOW      0  

#define SCK_MCP     P1_3
#define SCK_HIGH    1
#define SCK_LOW     0

#define SI_MCP      P1_5
#define SI_HIGH     1
#define SI_LOW      0  

#define SO_MCP      P1_4
#define SO_HIGH     1
#define SO_LOW      0  

// ***************************************************

// Pin assignments for SPI2 of MCP42100 ***************

#define CS_MCP2     P2_2

#define SCK_MCP2     P2_1

#define SI_MCP2      P2_0

// ***************************************************


// Pin assignments for SPI2 of MCP42100 ***************

#define CS_ADS     P1_6

#define SCK_ADS     P1_3

#define SI_ADS      P1_5

#define START_ADS       P0_7

// ***************************************************




/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t sleepiBand_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t sleepiBand_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t sleepiBand_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

// Accelerometer Profile Callbacks
static accelCBs_t sleepiBand_AccelCBs =
{
  accelEnablerChangeCB,          // Called when Enabler attribute changes
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
 * @return  none
 */
void SleepiBand_Init( uint8 task_id )
{
  sleepiBand_TaskID = task_id;
  
  PERCFG &= ~0x20; //Timer3 Alternate Location 1 (Channel 0)
  // P0.1 is the input pin for the ECG signal
  P0SEL |= (0x1 << 1); //Select P0.1 - Peripheral
  P0DIR &= ~(0x1 << 1); // Set P0.1 as an input
  P1SEL |= (0x1 << 3); //Select P1_6 - Peripheral
  P1DIR |= (0x1 << 3); //Select P1_6 - Output
  P2SEL |= (0x1 << 5); //Set Timer 3 -High Priority (Over UART1)
  
  T3CTL = (0x7 << 5) | //Tick 128
          (0x0 << 4) | //Timer OFF
          (0x0 << 3) | //Overflow OFF
          (0x1 << 2) | //Clear Count
          (0x2);       //Modulo Mode
  
  T3CCTL0 = (0x1 << 6) | //Channel 0 Interrupt Enabled
            (0x2 << 3) | //Toggle output on compare
            (0x1 << 2) | //Compare Mode
            (0x0);       //No Capture            
  
  T3CC0 = 230; //CC Value
  IEN1 |= 1 << 3;       //Enable interrupts
  
  T3CTL |= (0x1 << 4); //Start Timer
  
  
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = TRUE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

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
  SleepiProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  Accel_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }


#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( sleepiBand_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  /*************PIN SETUP*************/

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFF;         //P0.0-P0.7 - Output
  P1DIR = 0xC7;         //P1.3-P1.5 - Input (Rest Output)
  P2DIR = 0x07;         //P2.0-P2.2 - Output

  APCFG = 0x7F;
  
  P1_0 = 0x1;   //Turn on LEDs
  P1_1 = 0x1;
  
  /**************TIMER 1 SETUP********************/
  PERCFG &= ~(0x1 << 6);        //Timer 1 Alternative Location 1
  P0DIR = (0x2 << 6);           //Timer 1 Channel 0/1 Priority
  T1CTL = (0x3 << 2);           //Tick Freq / 128
  P0SEL |= (0x1 << 2);          //P0_2 Peripheral (Timer 1 Channel 0)
  
  printf("%d", CLKCONCMD);      //Use this to check clock speed
  
  T1CCTL1 = 0x00;
  T1CCTL2 = 0x00;
  T1CCTL3 = 0x00;
  T1CCTL4 = 0x00;
  
  T1CCTL0 = (0x1 << 6) |        //Interrupt Mask - Enable Interrupt Request
            (0x1 << 2) |        //Compare Mode
            (0x0);              //Default - No Capture, etc.
  
  T1CC0L = 0xFA;                //1ms Duty Cycle
  T1CC0H = 0x00;
  
  T1CTL &= ~(0x2);
  T1CTL |= (0x2);       //Turn on Timer 1
    
    
    
    
    
    
    
#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD
  

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &sleepiBand_SimpleProfileCBs );
  
  
  // Start the Accelerometer Profile
  VOID Accel_RegisterAppCBs( &sleepiBand_AccelCBs );

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

  /*
  HalUARTInit();
  halUARTCfg_t config;
  config.callBackFunc = NULL;
  HalUARTOpen(HAL_UART_PORT_1, &config);
  */
  P1_1 = 1;
  P1_0 = 0;     //UV
  //Reset_I2C();
  
  accInit();    //SPI
  
  // Setup a delayed profile startup
  osal_set_event( sleepiBand_TaskID, SBP_START_DEVICE_EVT );
  //osal_start_timerEx( sleepiBand_TaskID, LED_1_evt, LED_off_period );
  //osal_start_timerEx( sleepiBand_TaskID, INIT_evt, INIT_period );

}

/*********************************************************************
 * @fn      sleepiBand_ProcessEvent
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
uint16 SleepiBand_ProcessEvent( uint8 task_id, uint16 events )
{

  uint16 EEG_full;
  uint8 EEG_buff[2];
  uint8 acc_motion[2];
  uint8 adc_data[2];
  uint8 combined_nirs_eeg[15];
  uint8 reader[2];
  uint8 lead_status[2];
  uint8 motion[3];
  uint8 bat_level[2];
  uint8 bat_info[2];
  uint8 combined_battery[6];
  uint16 reading_ozone;
  
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( sleepiBand_TaskID )) != NULL )
    {
      sleepiBand_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &sleepiBand_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &sleepiBand_BondMgrCBs );
    
    osal_pwrmgr_device( PWRMGR_BATTERY );

    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  
  if ( events & INIT_evt ){
    halIntState_t intState;
    if(enable_all_notes)
    {
      HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

      uint8 send_array[20];
      bStatus_t e = SUCCESS;
      
      osal_memcpy(send_array, adc_buffer, 20);
      e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, 20, send_array );
      osal_memcpy(send_array, adc_buffer+20, 20);
      e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, 20, send_array );
      osal_memcpy(send_array, adc_buffer+40, 20);
      e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, 20, send_array );
      
    }
    
      HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
    
    
    return (events ^ INIT_evt);
  }
  
  if ( events & UV_evt ){
    uint16 ozone_m=0;
    P1_0=1;
    
    
    
    ozone_m = global_assist_ozone[0] <<8;
    ozone_m |= global_assist_ozone[1];
    
    if(UV_first_time==0){
      
    
      if(ozone_m>0x257f){ //add 2k if cots is on
        //2edf - 10k
        //2aa0 - 12k
        //2710 - 14k
        //257f - 15k
        P1_0=0;
        UV_first_time=1;
        osal_start_timerEx( sleepiBand_TaskID, UV_evt, 180000);
      }
      else{
        osal_start_timerEx( sleepiBand_TaskID, UV_evt, 100); 
      }
    }
    else{
      UV_first_time=0;
      osal_start_timerEx( sleepiBand_TaskID, UV_evt, 500); 
    }
      
    return (events ^ UV_evt);
  }
  
  
  if ( events & BATTERY_evt ){
    bStatus_t e = SUCCESS;
    if(enable_all_notes==1){
      
           
      //Read battery
      
      HalI2CInit( 0x70, i2cClock_267KHZ );
      uint8 writeData[1] = { 0x02 };
      HalI2CWrite( 1, writeData , TRUE );
      HalI2CRead( 2, bat_level );
      

      osal_memcpy(combined_battery,bat_level,2);
     
      
      e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, 6, combined_battery );
    }
    //P1_1 = ~P1_1;
    osal_start_timerEx( sleepiBand_TaskID, BATTERY_evt, 2000);
    
    
    return (events ^ BATTERY_evt);
  }  
  
  if ( events & NIRS_EEG_evt )
  {
    bStatus_t e = SUCCESS;
    //P1_1 = 0;
    if(enable_all_notes==1){
      
      
      if(glob_inc<8){      
          //Read a conversion
          HalI2CInit( 0x48, i2cClock_267KHZ );
          uint8 writeData[1] = { 0x00 };
          HalI2CWrite( 1, writeData , TRUE );
          HalI2CRead( 2, adc_data );
          osal_memcpy(combined_readings+((glob_inc-4)*2),adc_data,2);
          if(glob_inc==5)
            osal_memcpy(global_assist_ozone,adc_data,2);
        }
      
      if(glob_inc==8){
        adxl_read_reg(0x08,acc_motion);
        motion[0] = acc_motion[0];
        adxl_read_reg(0x09,acc_motion);   
        motion[1] = acc_motion[0];
        adxl_read_reg(0x0A,acc_motion);
        motion[2] = acc_motion[0];  
        osal_memcpy(combined_readings+8,motion,3);
      }
      
      glob_inc++;
      if(glob_inc>8){
        glob_inc=4;
        e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, 11, combined_readings );
      }
      ads1115_init(glob_inc);
     
      
      /*
      
          
          ads1115_init(0x04);
          WaitUs(1000);
          HalI2CInit( 0x48, i2cClock_267KHZ );
          uint8 writeData[1] = { 0x00 };
          HalI2CWrite( 1, writeData , TRUE );
          HalI2CRead( 2, adc_data );
          osal_memcpy(combined_readings+0,adc_data,2);
          
          ads1115_init(0x05);
          WaitUs(1000);
          HalI2CWrite( 1, writeData , TRUE );
          HalI2CRead( 2, adc_data );
          osal_memcpy(combined_readings+2,adc_data,2);
          
          ads1115_init(0x06);
          WaitUs(1000);
          HalI2CWrite( 1, writeData , TRUE );
          HalI2CRead( 2, adc_data );
          osal_memcpy(combined_readings+4,adc_data,2);
          
          ads1115_init(0x07);
          WaitUs(1000);
          HalI2CWrite( 1, writeData , TRUE );
          HalI2CRead( 2, adc_data );
          osal_memcpy(combined_readings+6,adc_data,2);
          
          
          adxl_read_reg(0x08,acc_motion);
          motion[0] = acc_motion[0];
          adxl_read_reg(0x09,acc_motion);   
          motion[1] = acc_motion[0];
          adxl_read_reg(0x0A,acc_motion);
          motion[2] = acc_motion[0];  
          osal_memcpy(combined_readings+8,motion,3);
          
          
          
          e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, 11, combined_readings );
        */
        
    }
      
    osal_start_timerEx( sleepiBand_TaskID, NIRS_EEG_evt, 35);
    return (events ^ NIRS_EEG_evt);
  }
  
  /*
  if ( events & ADS_evt )
  {
    bStatus_t e = SUCCESS;
    if(enable_all_notes==1){
      ads_read(ads_data);
      e = SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, 9, ads_data );
    }
      
    osal_start_timerEx( sleepiBand_TaskID, ADS_evt, 500);
    return (events ^ ADS_evt);
  }
  */
  
  
  if ( events & ACCEL_READ_EVT )
  {
    //bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );
    bStatus_t status = SUCCESS;
    
    if (status == SUCCESS)
    {
      if (accelEnabler)
      {
        // Restart timer
        if (ACCEL_READ_PERIOD)
          osal_start_timerEx( sleepiBand_TaskID, ACCEL_READ_EVT, ACCEL_READ_PERIOD );

        // Read accelerometer data
        if(enable_all_notes)
        accelRead();
      }
      else
      {
        // Stop the acceleromter
        osal_stop_timerEx( sleepiBand_TaskID, ACCEL_READ_EVT);
      }
    }
    else
    {
        //??
    }
    return (events ^ ACCEL_READ_EVT);
  }
  
  
  
  
  // Discard unknown events
  return 0;
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
static void sleepiBand_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      sleepiBand_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      sleepiBand_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void sleepiBand_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
    }
#endif // PLUS_BROADCASTER
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

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

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
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
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
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
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      /*if(enable_all_notes==1){
        //P1_1 = 1;
        //P1_0 = 1;
        //osal_start_timerEx( sleepiBand_TaskID, UV_evt, 5000); 
      }
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      Sampler_SetState(newValue == 1);
      enable_all_notes = (newValue == 1);
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      */
      
      enable_all_notes = true;
      
      
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

  case SIMPLEPROFILE_CHAR2:
    P1_1 = 1;
    P1_0 = 1;
    osal_start_timerEx( sleepiBand_TaskID, UV_evt, 10000); 
    break;
    
    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
 * @fn      accelEnablerChangeCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void accelEnablerChangeCB( void )
{
  bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

  if (status == SUCCESS){
    if (accelEnabler){
      // Initialize accelerometer
      // Setup timer for accelerometer task
      osal_start_timerEx( sleepiBand_TaskID, ACCEL_READ_EVT, ACCEL_READ_PERIOD );
    } else {
      // Stop the acceleromter
      osal_stop_timerEx( sleepiBand_TaskID, ACCEL_READ_EVT);
    }
  } else {
      //??
  }
}

/*********************************************************************
 * @fn      accelRead
 *
 * @brief   Called by the application to read accelerometer data
 *          and put data in accelerometer profile
 *
 * @param   none
 *
 * @return  none
 */

static void accelRead( void )
{

  uint8 AccelData[3];
  static int8 x, y, z;
  int8 new_x, new_y, new_z;

  // Read data for each axis of the accelerometer
  //FetchAccelData(AccelData); jpd
  

  // Check if x-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  //if( (x < (new_x-ACCEL_CHANGE_THRESHOLD)) || (x > (new_x+ACCEL_CHANGE_THRESHOLD)) || (y < (new_y-ACCEL_CHANGE_THRESHOLD)) || (y > (new_y+ACCEL_CHANGE_THRESHOLD)) || (z < (new_z-ACCEL_CHANGE_THRESHOLD)) || (z > (new_z+ACCEL_CHANGE_THRESHOLD)) )
  //{
  
  /*
    x = AccelData[0];
	y = AccelData[1];
	z = AccelData[2];  
    
  */ //jpd
  if(false){
  x=0;
  y=0;
  z=0;
    Accel_SetParameter(ACCEL_XYZ_ATTR, 3, &x, &y, &z);
  }
    
  //}
  
  
  
}

/*********************************************************************
*********************************************************************/

/*
void afe_write(uint8 channel, uint24 data){
  
    //HAL_TURN_ON_LED1();
    uint8 command_byte;    
    uint24 data_byte;
    uint8 mask_short = 0x80;
    uint24 mask_long = 0x800000, i=0; //        binary  1000 0000 0000 0000 0000 0000; mask for msb
  
    command_byte = channel;
    data_byte = data;
    
    
    //data_byte = 0x00;
    
      SCK_MCP = SCK_LOW;
      CS_MCP = CS_LOW;
      
      for (i=0;i<8;i++){
          if (command_byte & (mask_short>>i))
            SI_MCP  = 1;
          else 
            SI_MCP  = 0;
          
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_MCP = SCK_HIGH;
          WaitUs(1000);
          SCK_MCP = SCK_LOW;
      }
      
      for (i=0;i<24;i++){
          if (data_byte & (mask_long>>i))
            SI_MCP  = 1;
          else 
            SI_MCP  = 0;
          
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_MCP = SCK_HIGH;
          WaitUs(1000);
          SCK_MCP = SCK_LOW;
      }
        
      CS_MCP = CS_HIGH;
      SI_MCP  = 0;
      WaitUs(1000);
}

*/

/*
void ads_long_write(uint24 data){
  
  
    uint24 data_byte;
    uint24 mask_long = 0x800000, i=0; //        binary  1000 0000 0000 0000 0000 0000; mask for msb
    data_byte = data;
    
    
    //data_byte = 0x00;
    
      SCK_ADS = SCK_LOW;
      CS_ADS = CS_LOW;
     
      for (i=0;i<24;i++){
          if (data_byte & (mask_long>>i))
            SI_ADS  = 1;
          else 
            SI_ADS  = 0;
          
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_ADS = SCK_HIGH;
          WaitUs(1000);
          SCK_ADS = SCK_LOW;
      }
        
      CS_ADS = CS_HIGH;
      SI_ADS  = 0;
      WaitUs(1000);
}
*/
/*
void ads_short_write(uint8 data){
  
    //HAL_TURN_ON_LED1();
    uint8 data_byte;
    uint8 mask_short = 0x80, i=0;
  
    data_byte = data;
    
      SCK_ADS = SCK_LOW;
      CS_ADS = CS_LOW;
      
      for (i=0;i<8;i++){
          if (data_byte & (mask_short>>i))
            SI_ADS  = 1;
          else 
            SI_ADS  = 0;
          
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_ADS = SCK_HIGH;
          WaitUs(1000);
          SCK_ADS = SCK_LOW;
      }
      
      CS_ADS = CS_HIGH;
      SI_ADS  = 0;
      WaitUs(1000);
}
*/

void ads_init(){
    uint8 reader[2];
    //START_ADS = 0;
    //ads_short_write(0x08);
    P1_6=0;
    //WaitUs(5000);
 
    //ads_short_write(0x06);
    WaitUs(50000);
    
    ads_short_write(0x11);

 
    WaitUs(5000);
    /*
    ads_read_reg(0x21,reader);
    ads_read_reg(0x22,reader);
    ads_read_reg(0x23,reader);
    ads_read_reg(0x24,reader);
    ads_read_reg(0x25,reader);
    ads_read_reg(0x26,reader);
    ads_read_reg(0x27,reader);
    ads_read_reg(0x28,reader);
    ads_read_reg(0x29,reader);
    ads_read_reg(0x2A,reader);
    ads_read_reg(0x2B,reader);
    */
    
    
    ads_long_write(0x41,0x00,0xD6); 
    WaitUs(50);
    ads_long_write(0x42,0x00,0xD0); 
    WaitUs(50);
    ads_long_write(0x43,0x00,0xED);
    WaitUs(50);
    ads_long_write(0x44,0x00,0x40);  //Threshold for leads
    WaitUs(50);
    ads_long_write(0x45,0x00,0x60); //Channel 1
    WaitUs(50);
    ads_long_write(0x46,0x00,0x60); //Channel 2
    WaitUs(50);
    ads_long_write(0x47,0x00,0x60); //Channel 3
    WaitUs(50);
    ads_long_write(0x48,0x00,0x60); //Channel 4
    WaitUs(50);
    ads_long_write(0x49,0x00,0x81); //Channel 5
    WaitUs(50);
    ads_long_write(0x4A,0x00,0x81); //Channel 6
    WaitUs(50);
    ads_long_write(0x4B,0x00,0x81); //Channel 7
    WaitUs(50);
    ads_long_write(0x4C,0x00,0x81); //Channel 8
    WaitUs(50);
    ads_long_write(0x4D,0x00,0x0F); //RLD_P
    WaitUs(50);
    ads_long_write(0x4E,0x00,0x0F); //RLD_N
    WaitUs(50);
    ads_long_write(0x4F,0x00,0x0F); //LOFF_SENSP
    WaitUs(50);
    ads_long_write(0x50,0x00,0x0F); //LOFF_SENSN
    WaitUs(50);
    ads_long_write(0x51,0x00,0x00); 
    WaitUs(50);
    
    ads_long_write(0x57,0x00,0x08); 
    WaitUs(50);
    
    
    
    

    /*
    ads_read_reg(0x21,reader);
    ads_read_reg(0x22,reader);
    ads_read_reg(0x23,reader);
    ads_read_reg(0x24,reader);
    ads_read_reg(0x25,reader);
    ads_read_reg(0x26,reader);
    ads_read_reg(0x27,reader);
    ads_read_reg(0x28,reader);
    ads_read_reg(0x29,reader);
    ads_read_reg(0x2A,reader);
    ads_read_reg(0x2B,reader);
    */
    
    //ads_short_write(0x10);
    WaitUs(500);
    START_ADS = 1;


}

void afe_init(){
  
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x00,0x00,0x00,0x08);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x00,0x00,0x00,0x08);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x01,0x00,0x17,0xC0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x02,0x00,0x1F,0x3E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x03,0x00,0x17,0x70);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x04,0x00,0x1F,0x3F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x05,0x00,0x00,0x50);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x06,0x00,0x07,0xCE);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x07,0x00,0x08,0x20);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x08,0x00,0x0F,0x9E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x09,0x00,0x07,0xD0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0A,0x00,0x0F,0x9F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0B,0x00,0x0F,0xF0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0C,0x00,0x17,0x6E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0D,0x00,0x00,0x06);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0E,0x00,0x07,0xCF);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0F,0x00,0x07,0xD6);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x10,0x00,0x0F,0x9F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x11,0x00,0x0F,0xA6);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x12,0x00,0x17,0x6F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x13,0x00,0x17,0x76);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x14,0x00,0x1F,0x3F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x16,0x00,0x00,0x05);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x17,0x00,0x07,0xD0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x18,0x00,0x07,0xD5);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x19,0x00,0x0F,0xA0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1A,0x00,0x0F,0xA5);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1B,0x00,0x17,0x70);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1C,0x00,0x17,0x75);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1D,0x00,0x1F,0x3F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1E,0x00,0x01,0x02);  //was 00 01 01

    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x20,0x00,0x32,0x11);     //TIA gain 
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x21,0x08,0x41,0x40);     //TIA AMB gain 

    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x22,0x00,0xFF,0xFF);     //LED current
    //
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x01,0x00,0x17,0xC0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x02,0x00,0x1F,0x3E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x05,0x00,0x00,0x50);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x06,0x00,0x07,0xCE);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x07,0x00,0x08,0x20);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x08,0x00,0x0F,0x9E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0B,0x00,0x0F,0xF0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0C,0x00,0x17,0x6E);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x03,0x00,0x17,0x70);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x04,0x00,0x1F,0x3F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x09,0x00,0x07,0xD0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0A,0x00,0x0F,0x9F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x15,0x00,0x00,0x00);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x16,0x00,0x00,0x05);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0D,0x00,0x00,0x06);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0E,0x00,0x07,0xCF);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x17,0x00,0x07,0xD0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x18,0x00,0x07,0xD5);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x0F,0x00,0x07,0xD6);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x10,0x00,0x0F,0x9F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x19,0x00,0x0F,0xA0);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1A,0x00,0x0F,0xA5);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x11,0x00,0x0F,0xA6);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x12,0x00,0x17,0x6F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1B,0x00,0x17,0x70);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x1C,0x00,0x17,0x75);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x13,0x00,0x17,0x76);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x14,0x00,0x1F,0x3F);
    
    afe_write(0x00,0x00,0x00,0x00);
    afe_write(0x23,0x01,0x00,0x00);
  
  
}



/*
uint24 afe_read_old(uint8 channel){
  
    //HAL_TURN_ON_LED1();
    uint8 command_byte;    
    uint24 data_byte=0;
    uint8 mask_short = 0x80;
    uint24 mask_long = 0x800000, i=0; //        binary  1000 0000 0000 0000 0000 0000; mask for msb
  
    command_byte = channel;
    
    afe_write(0x00,0x000001);
    //data_byte = 0x00;
    
      SCK_MCP = SCK_LOW;
      CS_MCP = CS_LOW;
      
      for (i=0;i<8;i++){
          if (command_byte & (mask_short>>i))
            SI_MCP  = 1;
          else 
            SI_MCP  = 0;
          
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_MCP = SCK_HIGH;
          WaitUs(1000);
          SCK_MCP = SCK_LOW;
      }
      
      for (i=0;i<24;i++){
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_MCP = SCK_HIGH;  
          WaitUs(1000);
          data_byte = (P1_4)<<(24-i);
          SCK_MCP = SCK_LOW;
      }
        
      CS_MCP = CS_HIGH;
      SI_MCP  = 0;
      WaitUs(1000);
      
      afe_write(0x00,0x000000);
      
      return data_byte;
}

*/

/*
void ads_read(){
    
    uint24 data_byte1=0;
    uint24 data_byte2=0;
    uint24 data_byte3=0;
    uint24 mask_long = 0x800000, i=0; //        binary  1000 0000 0000 0000 0000 0000; mask for msb
  
    
    
      SCK_ADS = SCK_LOW;
      CS_ADS = CS_LOW;
      
     
      for (i=0;i<24;i++){
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_ADS = SCK_HIGH;  
          WaitUs(1000);
          data_byte1 = (P1_4)<<(72-i);
          SCK_ADS = SCK_LOW;
      }
      for (i=0;i<24;i++){
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_ADS = SCK_HIGH;  
          WaitUs(1000);
          data_byte2 = (P1_4)<<(72-i);
          SCK_ADS = SCK_LOW;
      }
      for (i=0;i<24;i++){
          WaitUs(1000);       // wait 10us before raising clock high
          SCK_ADS = SCK_HIGH;  
          WaitUs(1000);
          data_byte3 = (P1_4)<<(72-i);
          SCK_ADS = SCK_LOW;
      }
        
      CS_ADS = CS_HIGH;
      SI_ADS  = 0;
      WaitUs(1000);
      
      glob_dat1 = data_byte1;
      glob_dat2 = data_byte2;
      glob_dat3 = data_byte3;
      
      
}
*/





void set_pot2 (uint8 channel, uint8 data){
  
    //HAL_TURN_ON_LED1();
    uint8 command_byte, data_byte;    
    uint8 mask = 0x80, i=0; //        binary  1000 0000 ; mask for msb
  
    if (channel == 0)  
        command_byte = 0x11;      // 0001 0001
    else if (channel == 1)  
        command_byte = 0x12;      // 0001 0010
    else if (channel == 2)  
      command_byte = 0x13;      // 0001 0011 write on both channels
    
    data_byte = data;
    
      SCK_MCP2 = SCK_LOW;
      CS_MCP2 = CS_LOW;
      
      for (i=0;i<8;i++){
          if (command_byte & (mask>>i))
            SI_MCP  = 1;
          else 
            SI_MCP  = 0;
          
          WaitUs(10000);       // wait 10us before raising clock high
          SCK_MCP2 = SCK_HIGH;
          WaitUs(10000);
          SCK_MCP2 = SCK_LOW;
      }
      
      for (i=0;i<8;i++){
          if (data_byte & (mask>>i))
            SI_MCP2  = 1;
          else 
            SI_MCP2  = 0;
          
          WaitUs(10000);       // wait 10us before raising clock high
          SCK_MCP2 = SCK_HIGH;
          WaitUs(10000);
          SCK_MCP2 = SCK_LOW;
      }
        
      CS_MCP2 = CS_HIGH;
      SI_MCP2  = 0;
      WaitUs(10000);
}



static void Pot1ChangeCB( void )
{
  
  bStatus_t status = SimpleProfile_GetParameter( SIMPLEPROFILE_POT1, &simpleProfilePot1 );
  
  
        
      //Need to auto-calib here
      
      // This part below won't be required then
      // Write to channel 0 of pot
        set_pot2 (0, simpleProfilePot1[0]);   // update channel 0 of digipot
        set_pot2 (0, simpleProfilePot1[1]);   // update channel 0 of digipot
     // set_pot (0, 0x1f);
      
    
}


static void Pot2ChangeCB( void )
{
  
  bStatus_t status = SimpleProfile_GetParameter( SIMPLEPROFILE_POT2, &simpleProfilePot2 );
  
 
      //Need to auto-calib here
      
      // This part below won't be required then
      // Write to channel 0 of pot
        set_pot2 (0, simpleProfilePot2[0]);   // update channel 0 of digipot
        set_pot2 (0, simpleProfilePot2[1]);   // update channel 0 of digipot
     // set_pot (0, 0x1f);
      
    
}



static void WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
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


static void Init_I2C( void ) {
  // Init Slaves
//  Init_SI1143();
//  Init_TMP006();
  Init_LMP();
}

static void Init_LMP( void ) {
  /*
  uint8 INT_Z = 0x01;
  uint8 BIAS_SIGN = 0x00;
  uint8 BIAS = 0x00;  
  
  HalI2CInit( 0x48, i2cClock_533KHZ );  //Slave ID 
  writeReg_I2C( 0x10, 0x0B ); //set TIA gain
  writeReg_I2C( 0x11, 0x00|(INT_Z<<5)|(BIAS_SIGN<<4)|(BIAS)); //Bias
  writeReg_I2C( 0x12, 0x01 );
  */
  HalI2CInit( 0x48, i2cClock_533KHZ );  //Slave ID 
  P1_6=0;
  writeReg_I2C( 0x01, 0x00 ); 
  P1_6=1;
  
  P1_6=0;
  writeReg_I2C( 0x10, 0x14 );
  P1_6=1;
  
  P1_6=0;
  writeReg_I2C( 0x11, 0x80 );
  P1_6=1;
  
  P1_6=0;
  writeReg_I2C( 0x12, 0x03 );
  P1_6=1;
}

static void writeParam_I2C( uint8 param, uint8 paramValue )
{
  writeReg_I2C( 0x17, paramValue ); //PARAM_WR
  writeReg_I2C( 0x18, param | 0xA0); //COMMAND, PARAM_SET
  WaitUs(50); // Needed to ensure that the param values are stored before proceeding
}

static void Init_SI1143( void ) {
  HalI2CInit( 0x5A, i2cClock_533KHZ ); // Slave ID for SI1141/2/3
  
  writeReg_I2C( 0x18, 0x01 ); //COMMAND, Reset
  WaitUs(200);   
  writeReg_I2C( 0x07, 0x17 ); //HW_KEY
  WaitUs(50);   
  writeReg_I2C( 0x04, 0x08 ); //IRQ_ENABLE      after led2 is read 
  WaitUs(20);
  writeReg_I2C( 0x05, 0x00 ); //IRQ_MODE1
  writeReg_I2C( 0x06, 0x00 ); //IRQ_MODE2
  writeReg_I2C( 0x21, 0x00 ); //IRQ_STATUS
  writeReg_I2C( 0x03, 0x03 ); //IRQ_CFG
  writeReg_I2C( 0x13, 0x00 ); //PS2_TH0
  writeReg_I2C( 0x14, 0x00 ); //PS2_TH1
  
  writeReg_I2C( 0x08, 0x40 ); //MEAS_RATE (was 84)
  writeReg_I2C( 0x09, 0x08 ); //ALS_RATE
  writeReg_I2C( 0x0A, 0x08 ); //PS_RATE
  writeReg_I2C( 0x0F, 0xFF ); //PS_LED21, sets current for LEDs 1&2
  writeReg_I2C( 0x10, 0x0F ); //PS_LED3, sets current for LED 3
  
  // 0x67
  writeParam_I2C( 0x01, 0x47 ); //CHLIST, enable measurements
  writeParam_I2C( 0x02, 0x21 ); //PSLED12_SELECT, enables LEDs 1&2
  writeParam_I2C( 0x03, 0x04 ); //PSLED3_SELECT, enables LED 3
  writeParam_I2C( 0x07, 0x03 ); //PS1_ADCMUX, selects Large IR Photodiode
  writeParam_I2C( 0x08, 0x03 ); //PS2_ADCMUX, selects Large IR Photodiode
  writeParam_I2C( 0x09, 0x03 ); //PS3_ADCMUX, selects Large IR Photodiode
  writeParam_I2C( 0x0E, 0x03 ); //ALS2_ADCMUX, selects Large IR Photodiode
  writeParam_I2C( 0x0F, 0x65 ); //AUX_ADCMUX, selects Temperature for AUX measurements
  writeParam_I2C( 0x0A, 0x70 ); //PS_ADC_COUNTER
  
  writeParam_I2C( 0x11, 0x06 ); //ALS_VIS_ADC_GAIN
  
  writeParam_I2C( 0x1D, 0x70 ); //ALSIR_ADC_COUNTER
  
  writeReg_I2C( 0x18, 0x0F ); //COMMAND, PSALS_AUTO_CMD, starts an autonomous loop
  WaitUs(10);  
}

void adxl_init(){
  adxl_write_reg(0x2D,0x02);
  return;
}

static void stc3105_init(){
  HalI2CInit( 0x70, i2cClock_267KHZ );
  writeReg_I2C( 0x00, 0x10 );
  
}


static void ads1115_init(uint16 channel){
  uint16 OS = (0x01 << 15);       //Operating status
                                //When writing
                                //0x00 - no effect
                                //0x01 - begin a conversion
                                //when reading
                                //0x00 - device is performing conversion
                                //0x01 - device is not performing conversion
  uint16 MUX = (channel << 12);      //Input mux config
                                //0x00 - AINp = AIN0 and AINn = AIN1
                                //0x01 - AINp = AIN0 and AINn = AIN3
                                //0x02 - AINp = AIN1 and AINn = AIN3
                                //0x03 - AINp = AIN2 and AINn = AIN3
                                //0x04 - AINp = AIN0 and AINn = GND
                                //0x05 - AINp = AIN1 and AINn = GND
                                //0x06 - AINp = AIN2 and AINn = GND
                                //0x07 - AINp = AIN3 and AINn = GND
  uint16 PGA = (0x01 << 9);      //Input mux config
                                //0x00 - 6.144V
                                //0x01 - 4.096V
                                //0x02 - 2.048V
                                //0x03 - 1.024V
                                //0x04 - 0.512V
                                //0x05 - 0.256V
                                //0x06 - 0.256V
                                //0x07 - 0.256V
  uint16 MODE = (0x01 << 8);     //Device operating mode
                                //0x00 - Continuous Conversion Mode
                                //0x01 - Power-down single-shot mode
  uint16 DATA_RATE = (0x04 << 5); //Sampling rate
                                //0x00 - 8SPS
                                //0x01 - 16SPS
                                //0x02 - 32SPS
                                //0x03 - 64SPS
                                //0x04 - 128SPS
                                //0x05 - 250SPS
                                //0x06 - 475SPS
                                //0x07 - 860SPS
  uint16 COMP_MODE = (0x00 << 4); //Comparator Mode
                                //0x00 - Traditional comparator with hysteresis
                                //0x01 - Window comparator
  uint16 COMP_POL = (0x00 << 3); //Comparator polarity
                                //0x00 - Active low
                                //0x01 - Active high
  uint16 COMP_LAT = (0x00 << 2); //Comparator Latching
                                //0x00 - Non-latching comparator
                                //0x01 - Latching comparator
  uint16 COMP_QUE = 0x03;        //Comparator queue and disable
                                //0x00 - Assert after one conversion
                                //0x01 - Assert after two conversions
                                //0x02 - Assert after four conversions
                                //Disable comparator
  HalI2CInit( 0x48, i2cClock_267KHZ );
  writeReg16_I2C( 0x01, (OS|MUX|PGA|MODE|DATA_RATE|COMP_MODE|COMP_POL|COMP_LAT|COMP_QUE) ); //Write to config reg
  //writeReg16_I2C( 0x01, 0x2483 ); //Write to config reg
  return;
  }

void battery_init(void){
  HalI2CInit( 0x36, i2cClock_267KHZ );
  writeReg16_I2C(0xFE,0x0054);
  writeReg16_I2C(0x0C,0x0054);
  return;
}