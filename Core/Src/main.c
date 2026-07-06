/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
    uint8_t priority;
    uint8_t sender;
    uint8_t receiver;
    uint16_t message_id;
    uint8_t seq_type;
    uint8_t seq_count;
} METUCube_CAN_ID_t;

typedef struct{
  uint8_t priority;
  uint8_t sender;
  uint8_t receiver;
  uint16_t message_id;
  uint8_t seq_type;
  uint8_t seq_count;
  uint8_t dlc;
  uint8_t payload[8];
}CAN_queue_element;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_ID_ADC_REQUEST   0x010
#define MSG_ID_ADC_RESPONSE  0x020 // bunları rastgele seçtim
#define MSG_ID_EPS_HOUSEKEEPING 0x66
#define MSG_ID_READ_VOLTAGE_CHANNEL 0x6F
#define MSG_ID_READ_CURRENT_CHANNEL 0x70
#define MSG_ID_EPS_HEARTBEAT 0x65
#define MSG_ID_POWER_GOOD_SIGNAL 0x71
#define MSG_ID_OPEN_BUCK_SIGNAL 0x72
#define MSG_ID_OPEN_CHANNEL_SIGNAL 0x73
#define MSG_ID_OBC_HEARTBEAT 0x00
#define CAN_QUEUE_SIZE 50
#define EPS_ID 0x03
#define BROADCAST_ID 0x0F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txHeader;
uint8_t txData[8];
uint32_t txMailbox;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];

uint16_t resistors[10];

uint32_t last_heartbeat_time = 0;
uint32_t last_OBC_heartbeat_time = 0;
const uint32_t heartbeat_interval = 1000;
const uint32_t OBC_heartbeat_reset_time = 10000;

uint32_t resetOBCCounter = 0;
const uint32_t resetOBCCounterMax = 5;

uint8_t initProtocoleCompleted = 0;

volatile uint8_t high_head = 0;
volatile uint8_t low_head = 0;
volatile uint8_t high_tail = 0;
volatile uint8_t low_tail = 0;
volatile uint16_t queue_overflow_counter = 0;

CAN_queue_element high_queue[CAN_QUEUE_SIZE];
CAN_queue_element low_queue[CAN_QUEUE_SIZE];

uint32_t canRecCounter = 0;

uint16_t deger;
uint16_t deger2;
HAL_StatusTypeDef generalStatus;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} GPIOPin_Map_t;

const GPIOPin_Map_t gpioBUCK_map[] = {
		{EN_B_2_GPIO_Port, EN_B_2_Pin}, // placeholder for int0
		{EN_B_2_GPIO_Port, EN_B_2_Pin}, // placeholder for int1
		{EN_B_2_GPIO_Port, EN_B_2_Pin}, // int2
		{EN_B_3_GPIO_Port, EN_B_3_Pin}, // int3
		{EN_B_4_GPIO_Port, EN_B_4_Pin}, // int4
		{EN_B_5_GPIO_Port, EN_B_5_Pin}  // int5
};

const GPIOPin_Map_t gpioFUSE_map[] = {
		{EN_F_1_GPIO_Port, EN_F_1_Pin}, // placeholder for int0
		{EN_F_1_GPIO_Port, EN_F_1_Pin},
		{EN_F_2_GPIO_Port, EN_F_2_Pin},
		{EN_F_3_GPIO_Port, EN_F_3_Pin},
		{EN_F_4_GPIO_Port, EN_F_4_Pin},
		{EN_F_5_GPIO_Port, EN_F_5_Pin},
		{EN_F_6_GPIO_Port, EN_F_6_Pin},
		{EN_F_7_GPIO_Port, EN_F_7_Pin},
		{EN_F_8_GPIO_Port, EN_F_8_Pin},
		{EN_F_9_GPIO_Port, EN_F_9_Pin},
		{EN_F_10_GPIO_Port, EN_F_10_Pin},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef EPS_Set_Fuse_State(GPIO_TypeDef*, uint16_t, GPIO_PinState);
HAL_StatusTypeDef EPS_Set_Buck_State(GPIO_TypeDef*, uint16_t, GPIO_PinState);
HAL_StatusTypeDef RESET_OBC_CHANNEL(void);
HAL_StatusTypeDef INIT_Protocol(void);

uint32_t Build_CAN_ID_FromStruct(METUCube_CAN_ID_t *id);
HAL_StatusTypeDef CAN_Send_ADC15_Segmented(void);
HAL_StatusTypeDef CAN_Send_EPS_Heartbeat(void);
HAL_StatusTypeDef CAN_Send_EPS_Housekeeping(void);
HAL_StatusTypeDef CAN_Send_Voltages(void);
HAL_StatusTypeDef CAN_Send_Currents(void);
HAL_StatusTypeDef CAN_Send_PowerGood(void);
HAL_StatusTypeDef CAN_Verify_Open(uint16_t, uint8_t, HAL_StatusTypeDef, uint8_t);

 // newly added 
uint8_t CAN_IsHighPriority(uint16_t message_id);
uint8_t CAN_QueuePush(CAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail,CAN_queue_element *element);
uint8_t CAN_QueuePop(CAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, CAN_queue_element *element);
HAL_StatusTypeDef CAN_ProcessQueue(void);
HAL_StatusTypeDef CAN_ProcessQueueElement(CAN_queue_element *element);
HAL_StatusTypeDef Handle_OpenBuck(CAN_queue_element *element);
HAL_StatusTypeDef Handle_OpenChannel(CAN_queue_element *element);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int errorCounter = 0;

volatile uint16_t adc_buffer[15]; // first 10 reads fuse I, remaining 5 reads buck V

GPIO_PinState killSwitchStatus;
GPIO_PinState pGodF1status;
GPIO_PinState pGodF2status;
GPIO_PinState pGodF3status;
GPIO_PinState pGodF4status;
GPIO_PinState pGodF5status;
GPIO_PinState pGodF6status;
GPIO_PinState pGodF7status;
GPIO_PinState pGodF8status;
GPIO_PinState pGodF9status;
GPIO_PinState pGodF10status;

GPIO_PinState EN7READ;
GPIO_PinState EN8READ;

HAL_StatusTypeDef EPS_Set_Fuse_State(GPIO_TypeDef* fuse_port, uint16_t fuse_pin, GPIO_PinState state){
	HAL_GPIO_WritePin(fuse_port, fuse_pin, state);
    return HAL_OK;
}

HAL_StatusTypeDef EPS_Set_Buck_State(GPIO_TypeDef* buck_port, uint16_t buck_pin, GPIO_PinState state){
	HAL_GPIO_WritePin(buck_port, buck_pin, state);
    return HAL_OK;
}

HAL_StatusTypeDef RESET_OBC_CHANNEL(void) {
	HAL_StatusTypeDef jobStatus;

	jobStatus = EPS_Set_Fuse_State(EN_F_5_GPIO_Port, EN_F_5_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	jobStatus = EPS_Set_Fuse_State(EN_F_5_GPIO_Port, EN_F_5_Pin, GPIO_PIN_SET);

	return jobStatus;
}

HAL_StatusTypeDef INIT_Protocol(void) {
	HAL_StatusTypeDef jobStatus;

	jobStatus = EPS_Set_Buck_State(EN_B_4_GPIO_Port, EN_B_4_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	jobStatus = EPS_Set_Fuse_State(EN_F_5_GPIO_Port, EN_F_5_Pin, GPIO_PIN_SET);

	return jobStatus;
}

uint32_t Build_CAN_ID_FromStruct(METUCube_CAN_ID_t *id) {

    uint32_t ext_id = 0;

    ext_id |= ((id->priority   & 0x03) << 27);
    ext_id |= ((id->sender     & 0x0F) << 23);
    ext_id |= ((id->receiver   & 0x0F) << 19);
    ext_id |= ((id->message_id & 0x3FF) << 9);
    ext_id |= ((id->seq_type   & 0x03) << 7);
    ext_id |= ((id->seq_count  & 0x7F));

    return ext_id;
}

uint8_t CAN_IsHighPriority(uint16_t message_id){
  return (message_id == MSG_ID_OPEN_BUCK_SIGNAL || message_id == MSG_ID_OPEN_CHANNEL_SIGNAL || message_id == MSG_ID_POWER_GOOD_SIGNAL);
}

uint8_t CAN_QueuePush(CAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, CAN_queue_element *element) {
  uint8_t next = (*head + 1) % CAN_QUEUE_SIZE;
  if (next == *tail) return 0;   // queue full
  queue[*head] = *element;
  *head = next;
  return 1;
}

uint8_t CAN_QueuePop(CAN_queue_element *queue, volatile uint8_t *head, volatile uint8_t *tail, CAN_queue_element *element){
  __disable_irq();
  if (*tail == *head) {
    __enable_irq();
    return 0;   // empty
  }
  *element = queue[*tail];
  *tail = (*tail + 1) % CAN_QUEUE_SIZE;
  __enable_irq();
  return 1;
}

HAL_StatusTypeDef CAN_ProcessQueueElement(CAN_queue_element *element){

  switch (element->message_id){
    case MSG_ID_ADC_REQUEST:
    	return CAN_Send_ADC15_Segmented();
    case MSG_ID_EPS_HOUSEKEEPING:
    	return CAN_Send_EPS_Housekeeping();
    case MSG_ID_READ_VOLTAGE_CHANNEL:
    	return CAN_Send_Voltages();
    case MSG_ID_READ_CURRENT_CHANNEL:
    	return CAN_Send_Currents();
    case MSG_ID_POWER_GOOD_SIGNAL:
    	return CAN_Send_PowerGood();
    case MSG_ID_OPEN_BUCK_SIGNAL:
    	return Handle_OpenBuck(element);
    case MSG_ID_OPEN_CHANNEL_SIGNAL:
    	return Handle_OpenChannel(element);
    case MSG_ID_OBC_HEARTBEAT:
    	last_OBC_heartbeat_time = HAL_GetTick();
    	resetOBCCounter = 0;
      return HAL_OK;
    default:
      return HAL_ERROR;
    }
}

HAL_StatusTypeDef CAN_ProcessQueue(void){
  CAN_queue_element element;
  if(CAN_QueuePop(high_queue, &high_head, &high_tail, &element)){
    return CAN_ProcessQueueElement(&element);
  } else if(CAN_QueuePop(low_queue, &low_head, &low_tail, &element)){
    return CAN_ProcessQueueElement(&element);
  }
  return HAL_OK;
}

HAL_StatusTypeDef Handle_OpenBuck(CAN_queue_element *element){
  if(element->dlc < 5) return HAL_ERROR; 
  uint16_t buck_id = element->payload[0] | (element->payload[1] << 8);
  uint8_t is_to_open = element->payload[4];

  if(buck_id < 2 || buck_id > 5) return CAN_Verify_Open(buck_id, MSG_ID_OPEN_BUCK_SIGNAL, HAL_ERROR, is_to_open); // invalid buck id

  if(is_to_open){
    generalStatus = EPS_Set_Buck_State(gpioBUCK_map[buck_id].port, gpioBUCK_map[buck_id].pin, GPIO_PIN_SET);
  } else {
    generalStatus = EPS_Set_Buck_State(gpioBUCK_map[buck_id].port, gpioBUCK_map[buck_id].pin, GPIO_PIN_RESET);
  }
  return CAN_Verify_Open(buck_id, MSG_ID_OPEN_BUCK_SIGNAL, generalStatus, is_to_open);
}

HAL_StatusTypeDef Handle_OpenChannel(CAN_queue_element *element){
  if(element->dlc < 5) return HAL_ERROR; 
  uint16_t channel_id = element->payload[0] | (element->payload[1] << 8);
  uint8_t is_to_open = element->payload[4];
  if(channel_id < 1 || channel_id > 10) return CAN_Verify_Open(channel_id, MSG_ID_OPEN_CHANNEL_SIGNAL, HAL_ERROR, is_to_open); 

  if(is_to_open){
    generalStatus = EPS_Set_Fuse_State(gpioFUSE_map[channel_id].port, gpioFUSE_map[channel_id].pin, GPIO_PIN_SET);
  } else {
    generalStatus = EPS_Set_Fuse_State(gpioFUSE_map[channel_id].port, gpioFUSE_map[channel_id].pin, GPIO_PIN_RESET);
  }
  return CAN_Verify_Open(channel_id, MSG_ID_OPEN_CHANNEL_SIGNAL, generalStatus, is_to_open);
}

HAL_StatusTypeDef CAN_Send_ADC15_Segmented(void){

	METUCube_CAN_ID_t can_id;
	for(uint8_t segment_number = 0; segment_number<4; segment_number++ ){
		can_id.priority = 0x03; //low
		can_id.sender = 0x03; //eps
		can_id.receiver = 0x00; //obc
		can_id.message_id = MSG_ID_ADC_RESPONSE;

		can_id.seq_count = segment_number;

		if (segment_number == 0) can_id.seq_type = 0x01; // first
		if (segment_number == 3) can_id.seq_type = 0x02;
		if (segment_number == 1 || segment_number == 2) can_id.seq_type = 0x00;

		txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
		txHeader.IDE = CAN_ID_EXT;
		txHeader.RTR = CAN_RTR_DATA;
		if (segment_number == 3) {
			txHeader.DLC = 6;
		}else {
			txHeader.DLC = 8;
		}
		txHeader.TransmitGlobalTime = DISABLE;

		uint8_t buffer_start = segment_number*4;

		for(uint8_t i = 0; i<4; i++){
			uint8_t adc_index = buffer_start+i;

			if(adc_index < 15){
				txData[2*i] = (uint8_t) (adc_buffer[adc_index] & 0xFF); // lower 8 bits
				txData[2*i +1] = (uint8_t) ((adc_buffer[adc_index] >>8) & 0xFF);
			}else{
				txData[2*i] = 0;
				txData[2*i +1] = 0;
			}
		}
		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
    if (status != HAL_OK) return status;
    
		HAL_Delay(2);

	}
	return HAL_OK;
}

HAL_StatusTypeDef CAN_Send_EPS_Heartbeat(void){
	METUCube_CAN_ID_t can_id;
	can_id.priority = 0x03;
	can_id.sender = 0x03;
	can_id.receiver = 0x00;
	can_id.message_id = MSG_ID_EPS_HEARTBEAT;
	can_id.seq_type = 0x03;
	can_id.seq_count = 0;

	txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
	txHeader.IDE = CAN_ID_EXT;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 1;
	txHeader.TransmitGlobalTime = DISABLE;

	txData[0] = 0x65;

	return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

}

HAL_StatusTypeDef CAN_Send_EPS_Housekeeping(void){
	METUCube_CAN_ID_t can_id;
	uint8_t payload[22];
	// current readings
	for (uint8_t i = 0; i<10; i++){
		float adc_voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
		float current = (adc_voltage / (float)resistors[i]) * 20600.0f;
		if (current < 0.0f) current = 0.0f;
		if (current > 6.18f) current = 6.18f;
		payload[i] = (uint8_t)((current * 255.0f) / 6.18f);
	}

	// voltage readings
	for(uint8_t i = 0; i<5; i++){
		uint16_t voltage = (uint16_t)((adc_buffer[10+i] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));
		payload[10 + 2*i] = (uint8_t) (voltage & 0xFF); // lower bitleri yazıyo
		payload[10 + 2*i +1] = (uint8_t)((voltage >> 8) & 0xFF); // higher bits
	}

	uint16_t pgod_bits = 0;
	pgod_bits |= ((pGodF1status  == GPIO_PIN_SET) ? 1 : 0);
	pgod_bits |= ((pGodF2status  == GPIO_PIN_SET) ? 1 : 0) << 1;
	pgod_bits |= ((pGodF3status  == GPIO_PIN_SET) ? 1 : 0) << 2;
	pgod_bits |= ((pGodF4status  == GPIO_PIN_SET) ? 1 : 0) << 3;
	pgod_bits |= ((pGodF5status  == GPIO_PIN_SET) ? 1 : 0) << 4;
	pgod_bits |= ((pGodF6status  == GPIO_PIN_SET) ? 1 : 0) << 5;
	pgod_bits |= ((pGodF7status  == GPIO_PIN_SET) ? 1 : 0) << 6;
	pgod_bits |= ((pGodF8status  == GPIO_PIN_SET) ? 1 : 0) << 7;
  pgod_bits |= ((pGodF9status  == GPIO_PIN_SET) ? 1 : 0) << 8;
  pgod_bits |= ((pGodF10status == GPIO_PIN_SET) ? 1 : 0) << 9;

  payload[20] = (uint8_t)(pgod_bits & 0xFF);  // lower
  payload[21] = (uint8_t)((pgod_bits >> 8) & 0xFF); // higher

  can_id.priority = 0x03;
  can_id.sender = 0x03;
  can_id.receiver = 0x00;
  can_id.message_id = MSG_ID_EPS_HOUSEKEEPING;

	for(uint8_t segment_number = 0; segment_number<3; segment_number++){
		can_id.seq_count = segment_number;
		if(segment_number == 0) can_id.seq_type = 0x01;
		if(segment_number == 1) can_id.seq_type = 0x00;
		if(segment_number == 2) can_id.seq_type = 0x02;

		txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
		txHeader.IDE = CAN_ID_EXT;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;

		if(segment_number == 0){
			txData[0] = payload[0];
			txData[1] = payload[1];
			txData[2] = payload[2];
			txData[3] = payload[3];
			txData[4] = payload[4];
			txData[5] = payload[5];
			txData[6] = payload[6];
			txData[7] = payload[7];
			txHeader.DLC = 8;
		} else if(segment_number == 1){
			txData[0] = payload[8];
			txData[1] = payload[9];
			txData[2] = payload[10];
			txData[3] = payload[11];
			txData[4] = payload[12];
			txData[5] = payload[13];
			txData[6] = payload[14];
			txData[7] = payload[15];
			txHeader.DLC = 8;
		} else if(segment_number == 2){
			txData[0] = payload[16];
			txData[1] = payload[17];
			txData[2] = payload[18];
			txData[3] = payload[19];
			txData[4] = payload[20];
			txData[5] = payload[21];
			txHeader.DLC = 6;
		}
		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
		if (status != HAL_OK) {
			return status;
		}
		HAL_Delay(2);

	}
  return HAL_OK;
}

HAL_StatusTypeDef CAN_Send_Voltages(void){
	METUCube_CAN_ID_t can_id;
	uint8_t payload[10];
	uint16_t voltage1 = (uint16_t)((adc_buffer[10] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));
	uint16_t voltage2 = (uint16_t)((adc_buffer[11] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));
	uint16_t voltage3 = (uint16_t)((adc_buffer[12] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));
	uint16_t voltage4 = (uint16_t)((adc_buffer[13] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));
	uint16_t voltage5 = (uint16_t)((adc_buffer[14] * 3300.0f / 4095.0f) * ((47000.0f + 15800.0f) / 15800.0f));


	payload[0] = voltage1 & 0xFF; //lower
	payload[1] = (voltage1>>8) & 0xFF; //higher

	payload[2] = voltage2 & 0xFF; //lower
	payload[3] = (voltage2>>8) & 0xFF; //higher

	payload[4] = voltage3 & 0xFF; //lower
	payload[5] = (voltage3>>8) & 0xFF; //higher

	payload[6] = voltage4 & 0xFF; //lower
	payload[7] = (voltage4>>8) & 0xFF; //higher

	payload[8] = voltage5& 0xFF; //lower
	payload[9] = (voltage5>>8) & 0xFF; //higher

	can_id.priority = 0x03;
	can_id.sender = 0x03;
	can_id.receiver = 0x00;
	can_id.message_id =  MSG_ID_READ_VOLTAGE_CHANNEL;

	for(uint8_t segment_number = 0; segment_number<2; segment_number ++ ){
		can_id.seq_count = segment_number;

		if(segment_number == 0) can_id.seq_type = 0x01;
		if(segment_number == 1) can_id.seq_type = 0x02;

		txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
		txHeader.IDE = CAN_ID_EXT;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;

		if(segment_number == 0){
			txHeader.DLC = 8;
			txData[0]=payload[0];
			txData[1] = payload[1];
			txData[2] = payload[2];
			txData[3] = payload[3];
			txData[4] = payload[4];
			txData[5] = payload[5];
			txData[6] = payload[6];
			txData[7] = payload[7];
		}else{
			txHeader.DLC = 2;
			txData[0] = payload[8];
			txData[1] = payload[9];
		}
		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
		if (status != HAL_OK) {
			return status;
		}
		HAL_Delay(2);
	}
  return HAL_OK;

}

HAL_StatusTypeDef CAN_Send_Currents(void){
    METUCube_CAN_ID_t can_id;
    uint8_t payload[10];

    for (uint8_t i = 0; i < 10; i++){
    	float adc_voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
    	float current = (adc_voltage / (float)resistors[i]) * 20600.0f;
    	if (current < 0.0f) current = 0.0f;
    	if (current > 6.18f) current = 6.18f;
    	payload[i] = (uint8_t)((current* 255.0f) / 6.18f);
    }

    can_id.priority = 0x03;
    can_id.sender = 0x03;
    can_id.receiver = 0x00;
    can_id.message_id = MSG_ID_READ_CURRENT_CHANNEL;

    for (uint8_t segment_number = 0; segment_number < 2; segment_number++)
    {
        can_id.seq_count = segment_number;

        if (segment_number == 0)
            can_id.seq_type = 0x01;   // first
        else
            can_id.seq_type = 0x02;   // last

        txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
        txHeader.IDE = CAN_ID_EXT;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.TransmitGlobalTime = DISABLE;

        if (segment_number == 0)
        {
            txHeader.DLC = 8;

            txData[0] = payload[0];
            txData[1] = payload[1];
            txData[2] = payload[2];
            txData[3] = payload[3];
            txData[4] = payload[4];
            txData[5] = payload[5];
            txData[6] = payload[6];
            txData[7] = payload[7];
        }
        else
        {
            txHeader.DLC = 2;

            txData[0] = payload[8];
            txData[1] = payload[9];
        }

        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
        if (status != HAL_OK) {
            return status;
        }
        HAL_Delay(2);
    }
  return HAL_OK;
}

HAL_StatusTypeDef CAN_Verify_Open(uint16_t channelBuckId, uint8_t messageId, HAL_StatusTypeDef endStatus, uint8_t isToOpen){
    METUCube_CAN_ID_t can_id;
    can_id.priority = 0x03;
    can_id.sender = 0x03;
    can_id.receiver = 0x00;
    can_id.message_id = messageId;
    can_id.seq_type = 0x03;
	  can_id.seq_count = 0x00;

	uint8_t statusToSend = (uint8_t)endStatus;

	txData[0] = channelBuckId & 0xFF;
	txData[1] = (channelBuckId >> 8) & 0xFF;
	txData[2] = statusToSend & 0xFF;
	txData[3] = (statusToSend >> 8) & 0xFF;
	txData[4] = isToOpen;

    txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.DLC = 5;

  return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

}


HAL_StatusTypeDef CAN_Send_PowerGood(void){

    METUCube_CAN_ID_t can_id;
    uint16_t power_good = 0;

    power_good |= ((pGodF1status  == GPIO_PIN_SET) ? 1 : 0);
    power_good |= ((pGodF2status  == GPIO_PIN_SET) ? 1 : 0) << 1;
    power_good |= ((pGodF3status  == GPIO_PIN_SET) ? 1 : 0) << 2;
    power_good |= ((pGodF4status  == GPIO_PIN_SET) ? 1 : 0) << 3;
    power_good |= ((pGodF5status  == GPIO_PIN_SET) ? 1 : 0) << 4;
    power_good |= ((pGodF6status  == GPIO_PIN_SET) ? 1 : 0) << 5;
    power_good |= ((pGodF7status  == GPIO_PIN_SET) ? 1 : 0) << 6;
    power_good |= ((pGodF8status  == GPIO_PIN_SET) ? 1 : 0) << 7;
    power_good |= ((pGodF9status  == GPIO_PIN_SET) ? 1 : 0) << 8;
    power_good |= ((pGodF10status == GPIO_PIN_SET) ? 1 : 0) << 9;

    can_id.priority = 0x03;
    can_id.sender = 0x03;      // EPS
    can_id.receiver = 0x00;    // OBC
    can_id.message_id = MSG_ID_POWER_GOOD_SIGNAL;
    can_id.seq_type = 0x03;    // unsegmented
    can_id.seq_count = 0;

    txHeader.ExtId = Build_CAN_ID_FromStruct(&can_id);
    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 2;
    txHeader.TransmitGlobalTime = DISABLE;
    txData[0] = (uint8_t)(power_good & 0xFF);
    txData[1] = (uint8_t)((power_good >> 8) & 0xFF);
    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  if(EPS_Set_Buck_State(EN_B_2_GPIO_Port, EN_B_2_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Buck_State(EN_B_3_GPIO_Port, EN_B_3_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Buck_State(EN_B_4_GPIO_Port, EN_B_4_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Buck_State(EN_B_5_GPIO_Port, EN_B_5_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};

  HAL_Delay(100);

  if(EPS_Set_Fuse_State(EN_F_1_GPIO_Port, EN_F_1_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_2_GPIO_Port, EN_F_2_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_3_GPIO_Port, EN_F_3_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_4_GPIO_Port, EN_F_4_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_5_GPIO_Port, EN_F_5_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_6_GPIO_Port, EN_F_6_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_7_GPIO_Port, EN_F_7_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_8_GPIO_Port, EN_F_8_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_9_GPIO_Port, EN_F_9_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
  if(EPS_Set_Fuse_State(EN_F_10_GPIO_Port, EN_F_10_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};


  HAL_Delay(500);

  // start DMA for V and I Read
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 15) != HAL_OK){
	  errorCounter++;
  }

  CAN_FilterTypeDef canFilter;

  canFilter.FilterActivation = CAN_FILTER_ENABLE;
  canFilter.FilterBank = 0;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
      errorCounter++;
  }
  if(HAL_CAN_Start(&hcan1) != HAL_OK) {
      errorCounter++;
  }
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      errorCounter++;
  }


  for(uint8_t i = 0; i<10; i++){
  		resistors[i] = 2000;
  	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  killSwitchStatus = HAL_GPIO_ReadPin(GPIOC, KILL_SWITCH_Pin);

	  pGodF1status = HAL_GPIO_ReadPin(PGOD_F_1_GPIO_Port, PGOD_F_1_Pin);
	  pGodF2status = HAL_GPIO_ReadPin(PGOD_F_2_GPIO_Port, PGOD_F_2_Pin);
	  pGodF3status = HAL_GPIO_ReadPin(PGOD_F_3_GPIO_Port, PGOD_F_3_Pin);
	  pGodF4status = HAL_GPIO_ReadPin(PGOD_F_4_GPIO_Port, PGOD_F_4_Pin);
	  pGodF5status = HAL_GPIO_ReadPin(PGOD_F_5_GPIO_Port, PGOD_F_5_Pin);
	  pGodF6status = HAL_GPIO_ReadPin(PGOD_F_6_GPIO_Port, PGOD_F_6_Pin);
	  pGodF7status = HAL_GPIO_ReadPin(PGOD_F_7_GPIO_Port, PGOD_F_7_Pin);
	  pGodF8status = HAL_GPIO_ReadPin(PGOD_F_8_GPIO_Port, PGOD_F_8_Pin);
	  pGodF9status = HAL_GPIO_ReadPin(PGOD_F_9_GPIO_Port, PGOD_F_9_Pin);
	  pGodF10status = HAL_GPIO_ReadPin(PGOD_F_10_GPIO_Port, PGOD_F_10_Pin);

	  if (killSwitchStatus == GPIO_PIN_RESET) {

		  if(EPS_Set_Buck_State(EN_B_2_GPIO_Port, EN_B_2_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Buck_State(EN_B_3_GPIO_Port, EN_B_3_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Buck_State(EN_B_4_GPIO_Port, EN_B_4_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Buck_State(EN_B_5_GPIO_Port, EN_B_5_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};

		  HAL_Delay(100);

		  if(EPS_Set_Fuse_State(EN_F_1_GPIO_Port, EN_F_1_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_2_GPIO_Port, EN_F_2_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_3_GPIO_Port, EN_F_3_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_4_GPIO_Port, EN_F_4_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_5_GPIO_Port, EN_F_5_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_6_GPIO_Port, EN_F_6_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_7_GPIO_Port, EN_F_7_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_8_GPIO_Port, EN_F_8_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_9_GPIO_Port, EN_F_9_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  if(EPS_Set_Fuse_State(EN_F_10_GPIO_Port, EN_F_10_Pin, GPIO_PIN_RESET) == HAL_OK) {} else {errorCounter++;};
		  initProtocoleCompleted = 0;
		  HAL_Delay(1000);

		  continue;
	  } else if (initProtocoleCompleted == 0) {
		  if(INIT_Protocol() == HAL_OK){initProtocoleCompleted = 1;}
	  }

	  if(HAL_GetTick() - last_heartbeat_time >= heartbeat_interval) {
		  last_heartbeat_time = HAL_GetTick();
	      if(CAN_Send_EPS_Heartbeat() != HAL_OK) {
	          errorCounter++;
	      }
	  }

	  if(HAL_GetTick() - last_OBC_heartbeat_time >= OBC_heartbeat_reset_time && resetOBCCounter <= resetOBCCounterMax) {
		  // reset OBC, this assumes EPS is working perfectly
		  if(RESET_OBC_CHANNEL() != HAL_OK) {
			  errorCounter++;
		  }
		  resetOBCCounter++;
	  }

	  if(CAN_ProcessQueue() != HAL_OK) {
		  errorCounter++;
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 26;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 15;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_F_6_Pin|EN_F_5_Pin|EN_B_4_Pin|EN_B_5_Pin
                          |EN_F_9_Pin|EN_F_10_Pin|EN_B_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_F_2_GPIO_Port, EN_F_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_F_1_Pin|EN_B_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_F_3_Pin|EN_F_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_F_7_Pin|EN_F_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_F_6_Pin EN_F_5_Pin EN_B_4_Pin EN_B_5_Pin
                           EN_F_9_Pin EN_F_10_Pin EN_B_2_Pin */
  GPIO_InitStruct.Pin = EN_F_6_Pin|EN_F_5_Pin|EN_B_4_Pin|EN_B_5_Pin
                          |EN_F_9_Pin|EN_F_10_Pin|EN_B_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PGOD_F_9_Pin PGOD_F_10_Pin */
  GPIO_InitStruct.Pin = PGOD_F_9_Pin|PGOD_F_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KILL_SWITCH_Pin PGOD_F_2_Pin */
  GPIO_InitStruct.Pin = KILL_SWITCH_Pin|PGOD_F_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_F_2_Pin */
  GPIO_InitStruct.Pin = EN_F_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_F_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_F_1_Pin EN_B_3_Pin */
  GPIO_InitStruct.Pin = EN_F_1_Pin|EN_B_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PGOD_F_1_Pin */
  GPIO_InitStruct.Pin = PGOD_F_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PGOD_F_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_F_3_Pin EN_F_4_Pin */
  GPIO_InitStruct.Pin = EN_F_3_Pin|EN_F_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PGOD_F_3_Pin PGOD_F_4_Pin PGOD_F_5_Pin PGOD_F_6_Pin
                           PGOD_F_7_Pin PGOD_F_8_Pin */
  GPIO_InitStruct.Pin = PGOD_F_3_Pin|PGOD_F_4_Pin|PGOD_F_5_Pin|PGOD_F_6_Pin
                          |PGOD_F_7_Pin|PGOD_F_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_F_7_Pin EN_F_8_Pin */
  GPIO_InitStruct.Pin = EN_F_7_Pin|EN_F_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
    errorCounter++;
		return;
	}
	canRecCounter = canRecCounter + 1;

  if (rxHeader.IDE != CAN_ID_EXT) return;
  
  CAN_queue_element element;
  
  element.priority = (rxHeader.ExtId >> 27) & 0x03;
  element.sender = (rxHeader.ExtId >> 23) & 0x0F;
  element.receiver = (rxHeader.ExtId >> 19) & 0x0F;
  element.message_id = (rxHeader.ExtId >> 9) & 0x3FF;
  element.seq_type = (rxHeader.ExtId >> 7) & 0x03;
  element.seq_count = rxHeader.ExtId & 0x7F;
  element.dlc = rxHeader.DLC;

  if(element.receiver != EPS_ID && element.receiver != BROADCAST_ID) return;
  for (uint8_t i = 0; i < element.dlc; i++) {
      element.payload[i] = rxData[i];
  }
  uint8_t ok;
  if(CAN_IsHighPriority(element.message_id)) {
    ok = CAN_QueuePush(high_queue, &high_head, &high_tail, &element);
  }else {
    ok = CAN_QueuePush(low_queue, &low_head, &low_tail, &element);
  }
  if(!ok) {
    queue_overflow_counter++;
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
