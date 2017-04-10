
#include "sampler.h"
#include "hal_adc.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_dma.h"
#include "hal_i2c.h"
#include "hal_sleep.h"
#include "OSAL.h"
#include "bcomdef.h"
#include "VitalFlo.h"
#include "sleepiProfile.h"
#include "stdio.h"

#define MSG_SIZE 60




typedef struct bufferNode bufferNode;
struct bufferNode{
  union {
    uint8 buffer[MSG_SIZE];
    struct {
      uint8 buffer1[MSG_SIZE/3];
      uint8 buffer2[MSG_SIZE/3];
      uint8 buffer3[MSG_SIZE/3];
    } subBuffers;
  } data;
  bufferNode* next;
  uint8 flags;
};

bufferNode doneRoot;
bufferNode* pBufferReading = NULL;
bufferNode* pBufferNext = NULL;

// Task ID not initialized
#define NO_TASK_ID 0xFF


#define HAL_ADC_EOC         0x80    /* End of Conversion bit */
#define HAL_ADC_START       0x40    /* Starts Conversion */

#define HAL_ADC_STSEL_EXT   0x00    /* External Trigger */
#define HAL_ADC_STSEL_FULL  0x10    /* Full Speed, No Trigger */
#define HAL_ADC_STSEL_T1C0  0x20    /* Timer1, Channel 0 Compare Event Trigger */
#define HAL_ADC_STSEL_ST    0x30    /* ADCCON1.ST =1 Trigger */

#define HAL_ADC_RAND_NORM   0x00    /* Normal Operation */
#define HAL_ADC_RAND_LFSR   0x04    /* Clock LFSR */
#define HAL_ADC_RAND_SEED   0x08    /* Seed Modulator */
#define HAL_ADC_RAND_STOP   0x0c    /* Stop Random Generator */
#define HAL_ADC_RAND_BITS   0x0c    /* Bits [3:2] */

#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */

#define HAL_ADC_STSEL       HAL_ADC_STSEL_ST
#define HAL_ADC_RAND_GEN    HAL_ADC_RAND_STOP
#define HAL_ADC_REF_VOLT    HAL_ADC_REF_AVDD
#define HAL_ADC_DEC_RATE    HAL_ADC_DEC_064
#define HAL_ADC_SCHN        HAL_ADC_CHN_VDD3
#define HAL_ADC_ECHN        HAL_ADC_CHN_GND


// Registered keys task ID, initialized to NOT USED.
extern volatile uint8 allocCount;
uint8 Sampler_TaskID = NO_TASK_ID;    // Task ID for internal task/event processing.

uint16 data = 0;

uint16 overflow = 0;

uint8 seqNum = 0;
uint8 pacNum = 0;

uint32 fail = 0;
uint32 dropped = 0;
uint32 success = 0;

static bool isEnabled;
/*
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
  
  adc_reading[0] = glob_buffer_pointer;
  osal_memcpy(adc_buffer+glob_buffer_pointer,adc_reading,1);
  
  if(glob_buffer_pointer == 59){
    glob_buffer_pointer = 0;
    osal_set_event(, INIT_evt);
  }
  else{
    glob_buffer_pointer++;
  }
  
  HAL_EXIT_ISR();
}
*/


/*********************************************************************
* @fn      RoachZStack_Init
*
* @brief   This is called during OSAL tasks' initialization.
*
* @param   task_id - the Task ID assigned by OSAL.
*
* @return  none
*/
void Sampler_Init( uint8 task_id )
{

  Sampler_TaskID = task_id;   
  isEnabled = FALSE;
  /*
  PERCFG |= 0x20; //Timer3 Alternate Location 1 (Channel 0)
  P1SEL |= (0x1 << 6); //Select P1_6 - Peripheral
  P1DIR |= (0x1 << 6); //Select P1_6 - Output
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
  */
  
  
  
  
  /**
  PERCFG &= ~0x20; //Timer3 Alternate Location 1 (Channel 0)
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
  
  
  
  
  /*APCFG = 0x00 | (1 << HAL_ADC_CHANNEL_7);
  ADCCON1 = HAL_ADC_STSEL_T1C0 | 0x03; // 0x03 reserved
  ADCCON2 = HAL_ADC_REF_VOLT | HAL_ADC_DEC_512 | 0x07; //stop at channel 7
 
  P0DIR |= 0x1 << 2;
  P0SEL |= 0x1 << 2;
  P2DIR |= 0x80;
  P2DIR &= ~0x60;
  //PERCFG |= 0x1<<6;
  T1CTL = 0x00 | 0x0C | 0x02;
  
  uint16 counter = 230; // 32 Mhz / 128 / counter (230 = 1 kHz)
  
  T1CC0H = counter >> 8;
  T1CC0L = (uint8)counter;
  // no rf, no interrupt, set->clear, compare, no capture
  T1CCTL0 = 0<<7 | 0<<6 | 0x18 | 0x04 | 0x00; // 5C;
  DMAIE = 1;
  //T1IE = 1;
  
  HalDmaInit();
  HAL_DMA_SET_SOURCE(HAL_DMA_GET_DESC1234(1), &X_ADCH);
  HAL_DMA_SET_VLEN(HAL_DMA_GET_DESC1234(1), HAL_DMA_VLEN_USE_LEN);
  HAL_DMA_SET_LEN(HAL_DMA_GET_DESC1234(1), MSG_SIZE);
  HAL_DMA_SET_WORD_SIZE(HAL_DMA_GET_DESC1234(1), HAL_DMA_WORDSIZE_BYTE);
  HAL_DMA_SET_TRIG_MODE(HAL_DMA_GET_DESC1234(1), HAL_DMA_TMODE_SINGLE);
  HAL_DMA_SET_TRIG_SRC(HAL_DMA_GET_DESC1234(1), HAL_DMA_TRIG_ADC_CHALL);
  HAL_DMA_SET_SRC_INC(HAL_DMA_GET_DESC1234(1), HAL_DMA_SRCINC_0);
  HAL_DMA_SET_DST_INC(HAL_DMA_GET_DESC1234(1), HAL_DMA_DSTINC_1);
  HAL_DMA_SET_IRQ(HAL_DMA_GET_DESC1234(1), HAL_DMA_IRQMASK_ENABLE);
  HAL_DMA_SET_PRIORITY(HAL_DMA_GET_DESC1234(1), HAL_DMA_PRI_GUARANTEED); 
  */
}

/*********************************************************************
* @fn      RoachZStack_ProcessEvent
*
* @brief   Generic Application Task event processor.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events   - Bit map of events to process.
*
* @return  Event flags of all unprocessed events.
*/
UINT16 Sampler_ProcessEvent( uint8 task_id, UINT16 events )
{
    return events ^ SAMPLER_FLUSH_BUFFER;
}

void Sampler_SetState( bool state )
{
  isEnabled = state;
  osal_set_event(Sampler_TaskID, SAMPLER_FLUSH_BUFFER );
}





