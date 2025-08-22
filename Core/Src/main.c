/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include <stdlib.h>

#include "lsm6dso.h"
#include "custom_bus.h"
#include "GUI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_NUM_CONVERSIONS 4
#define ADC_RAW_TO_VOLTAGE 0.000100711f    // =(3.3/(2^15-1) for 3.3V full scale
#define ADC_RAW_TO_VOLTAGE2  0.001612     //12 bit ADC for 3.3V (3.3/(2^11-1) for 3.3V

#define PWR_WAKEUP_PIN_FLAGS  (PWR_WAKEUP_FLAG1 | PWR_WAKEUP_FLAG2  | \
                               PWR_WAKEUP_FLAG4  | PWR_WAKEUP_FLAG6)

#define ADC_BUFFER_SIZE 96
#define ADC3_BUFFER_SIZE 32
//
//ALIGN_32BYTES (uint16_t adc3Data[ADC3_BUFFER_SIZE]) __attribute__ ((section(".ARM.__at_0x38000000")));
ALIGN_32BYTES (static uint16_t   adcData[ADC_BUFFER_SIZE]);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t loopnumber;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan2;

IWDG_HandleTypeDef hiwdg1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#include "variables.h"
#include "DoorControl.h"

PWREx_WakeupPinTypeDef sPinParams;
//uint16_t adc3Data[ADC_NUM_CONVERSIONS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_IWDG1_Init(void);
/* USER CODE BEGIN PFP */
//static void MEMS_Init(void);

int _write(int32_t file, uint8_t *ptr, int32_t len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

void _Error_Handler(char *file, int line)
{
      /* USER CODE BEGIN Error_Handler_Debug */
      /* User can add his own implementation to report the HAL error return state */
      while(1)
      {
        // Optional: Send error information via UART for debugging
         printf("Error occurred in file: %s, line: %d\r\n", file, line);
        HAL_Delay(500);
      }
      /* USER CODE END Error_Handler_Debug */
}

uint32_t motoroutput;

void RunOutputCycle(void)
{
//	DO.Clutch = TRUE;
//			  if (motoroutput <500)
//			  {
//				  DO.Ramp_Light = TRUE;
////					DO.Clutch = TRUE;
////					DO.Latch =TRUE;
////					DO.LEDYellow= FALSE;
////					DO.LEDWhite= TRUE;
////					DO.LEDRed= TRUE;
////					MagnaOperation(DOOR_LOCK);
//
//	//				DO.Release = FALSE;
//	//			DO.Wakeup = TRUE;
//	//			  Start_Open(50);
//			  }
////			  else if (motoroutput <1000){
////			  Start_Close(100);
////				  Stop_Motors();
////			  }
////			  else if (motoroutput <3000){
////				  Start_Close(100);
////			  }
//			  else
//			  {
//				  DO.Ramp_Light = FALSE;
////				  Start_Close(50);
////				  DO.Clutch = FALSE;
////				  DO.Latch = FALSE;
//				  DO.LEDYellow= TRUE;
//				  DO.LEDWhite= FALSE;
//				  DO.LEDRed= FALSE;
//	//			  MagnaOperation(DOOR_UNLOCK);
//	//   		      DO.Release = FALSE;
//	//			  DO.Wakeup = TRUE;
//   //				  Stop_Motors();
//			  }
			  if (motoroutput>1200)
				  motoroutput =0;

			 if (motoroutput ==800)
				 LatchOperation(LATCH_EXTRACT);//MagnaOperation(DOOR_LOCK);
//
			  if (motoroutput == 0)
				  LatchOperation(LATCH_WITHDRAW);//MagnaOperation(DOOR_UNLOCK);

			  if (motoroutput ==400)
				  LatchOperation(LATCH_DISABLE);//MagnaOperation(DOOR_RELEASE);
}


__IO uint16_t uhADCxConvertedValue = 0;

/**
 * Read the digital input to the variables
 */
void UpdateAllInputs(void)
{
	DI.Braun_Op_Switch = !(char) HAL_GPIO_ReadPin(Braun_Op_GPIO_Port,Braun_Op_Pin);
	DI.ConvDisabled = !(char) HAL_GPIO_ReadPin(Conversion_Dis_GPIO_Port,	Conversion_Dis_Pin);
	DI.Door_ajar = (char) HAL_GPIO_ReadPin(Door_Ajar_GPIO_Port, Door_Ajar_Pin);   //full closed
	DI.ERR_CAN = !(char) HAL_GPIO_ReadPin(ERR_CAN_GPIO_Port, ERR_CAN_Pin);
//	DI.Door_Latched_Switch = !(char) HAL_GPIO_ReadPin(Door_Latched_GPIO_Port,	Door_Latched_Pin);
	DI.Kneel_disabled = !(char) HAL_GPIO_ReadPin(Kneel_Disable_GPIO_Port,	Kneel_Disable_Pin);
	DI.Lock_status_switch = !(char) HAL_GPIO_ReadPin(Lock_Status_GPIO_Port,	Lock_Status_Pin);   //0: Locked   1: Unlocked
	DI.Out_side_handle = !(char) HAL_GPIO_ReadPin(Outside_Handle_GPIO_Port,	Outside_Handle_Pin);
	DI.full_door_open = !(char) HAL_GPIO_ReadPin(Door_Open_GPIO_Port,	Door_Open_Pin);

	if (DI.Door_ajar == TRUE)
		Full_Closed_Encode = adc->Door_Encode_Signal;
	if (DI.full_door_open == TRUE)
		Full_Opened_Encode = adc->Door_Encode_Signal;

	if ((DI.Braun_Op_Switch == TRUE) && (Pre_DI.Braun_Op_Switch ==FALSE))
	{
		Braun_Op_Pressed(DI.Braun_Op_Switch );
	}
	Pre_DI.Braun_Op_Switch =  DI.Braun_Op_Switch;
//	Pre_Braun_Op_Switch = DI.Braun_Op_Switch;

	if ((DI.Out_side_handle == TRUE) && (Pre_DI.Out_side_handle ==FALSE))
	{
#ifndef NO_VEHICLE_CAN
		if (DI.Lock_status_switch == TRUE)   //outside handle only work when unlocked.
#endif
			Outside_handle_Pressed();
	}
	Pre_DI.Out_side_handle = DI.Out_side_handle;

	//if ((DI.ConvDisabled == TRUE) && (Pre_DI.ConvDisabled ==FALSE))
	if (DI.ConvDisabled != Pre_DI.ConvDisabled )
	{
		DisableConv_Time.flag = TRUE;
		DisConv_toggle_count++;
	}
	Pre_DI.ConvDisabled =  DI.ConvDisabled;
}


/**
 * Write the digital output from the variables.
 */
void UpdateAlloutputs(void)
{
//	DO.Wakeup = FALSE;
//	DO.Clutch

	//DO.Clutch = FALSE;
	if (DO.Clutch == TRUE)
		HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, GPIO_PIN_RESET);

	if (DO.LEDRed == TRUE)
		HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

	if (DO.LEDWhite == TRUE)
	{
		HAL_GPIO_WritePin(White_LED_GPIO_Port, White_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(White_LED_12V_GPIO_Port, White_LED_12V_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(White_LED_GPIO_Port, White_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(White_LED_12V_GPIO_Port, White_LED_12V_Pin, GPIO_PIN_RESET);
	}

	if (DO.LEDYellow == TRUE)
		HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);

	if  (DO.Latch == TRUE)//|| (DI.Out_side_handle == FALSE))
		HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);

	if (DO.Wakeup == TRUE)
		HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, GPIO_PIN_RESET);

//	if ((DO.Release == TRUE)|| ((DI.Out_side_handle == FALSE) && (m_doorstep==DOORIDLE)))
	if (DO.ReleaseEnable == TRUE)// || (DI.Out_side_handle == FALSE))
		HAL_GPIO_WritePin(Lock_Release_Enable_GPIO_Port, Lock_Release_Enable_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Lock_Release_Enable_GPIO_Port, Lock_Release_Enable_Pin, GPIO_PIN_RESET);

	if  (DO.ReleaseTrig == TRUE)// || (DI.Out_side_handle == FALSE))
		HAL_GPIO_WritePin(Lock_Release_Trigger_GPIO_Port, Lock_Release_Trigger_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Lock_Release_Trigger_GPIO_Port, Lock_Release_Trigger_Pin, GPIO_PIN_RESET);

	if  (DO.Lock_Coil == TRUE)// || (DI.Out_side_handle == FALSE))
		HAL_GPIO_WritePin(Lock_Unlock_Coil_GPIO_Port, Lock_Unlock_Coil_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Lock_Unlock_Coil_GPIO_Port, Lock_Unlock_Coil_Pin, GPIO_PIN_RESET);


	if (DO.Ramp_Light == TRUE)
		HAL_GPIO_WritePin(Ramp_Light_GPIO_Port, Ramp_Light_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Ramp_Light_GPIO_Port, Ramp_Light_Pin, GPIO_PIN_RESET);


	if (DO.Latch_DIR == TRUE)
		HAL_GPIO_WritePin(LATCH_DIR_GPIO_Port, LATCH_DIR_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LATCH_DIR_GPIO_Port, LATCH_DIR_Pin, GPIO_PIN_RESET);

	if (DO.Latch_Disable == TRUE)
		HAL_GPIO_WritePin(LATCH_Disable_GPIO_Port, LATCH_Disable_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LATCH_Disable_GPIO_Port, LATCH_Disable_Pin, GPIO_PIN_RESET);

	if (DO.Latch_PWM == TRUE)
		HAL_GPIO_WritePin(LATCH_PWM_GPIO_Port, LATCH_PWM_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LATCH_PWM_GPIO_Port, LATCH_PWM_Pin, GPIO_PIN_RESET);

	if (DO.Latch_VSO == TRUE)
		HAL_GPIO_WritePin(LATCH_VSO_GPIO_Port, LATCH_VSO_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LATCH_VSO_GPIO_Port, LATCH_VSO_Pin, GPIO_PIN_RESET);

//	if   (DO.Latch_DIR==)
//	{
//		HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Magna_Power_Release_GPIO_Port, Magna_Power_Release_Pin, GPIO_PIN_SET);
//	}
	//printf("test IO:Output =%d;%d;%d;\r\n",DO.Clutch, DO.Latch, DO.Release);//,DI.Door_Latched_Switch,DI.Kneel_disabled,DI.Lock_status_switch,DI.Out_side_handle,DI.full_door_open);
}


char tim6_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	motoroutput++;

//	DO.Wakeup = TRUE;
	//motoroutput =(motoroutput>5000)? 1000:motoroutput;
	if (htim->Instance == TIM6) {       						//TIM6, period around 32ms
		  //Ken Test
//		printf("ADC=%d,%d,%d,%d\r\n",adcData[0],adcData[1],adcData[2],adcData[3]);
		  testcount++;
		  if (testcount >10)
		  {
			  testcount = 0;
		  }
		  mDebugPrintCount++;
		  if (mDebugPrintCount <100)			 //Need
		  {
			  mDebugPrintCount++;
			  mDebugPrint = TRUE;
		  }
		  else
		  {
			  mDebugPrint = FALSE;
		  }
///////////////////////////////
		tim6_flag = 1;
		TIM6CountEvent(&Door_flag, &Door_Count);    //Do the ifxxx_flag then xxx_count++
		//printf("sleep_wait_count= %d;\r\n",sleep_wait_count);

		//printf("Door speed = %f; encode=%d;\r\n",doorspeed_cal,adc->Door_Encode_Signal);
		//printf ("SensorReading.NoseAngle=%f\r\n",SensorReading.NoseAngle);
//		if (Enable_PI_Moniter == TRUE)
//		{
//			if (UARTIOUpdateCount > 6)
//				UARTIOUpdateCount = 0;
//
//			SendIOstatus(UARTIOUpdateCount);
//			UARTIOUpdateCount++;
//		}
//		sleep_wait_count++;
	}

	if (htim->Instance == TIM7) {							//TIM7 period 10ms timer
		/***********************************
		 *  CAN Status Messages Update every 100ms
		 ***********************************/

		ChangeMotorSpeedFlag = TRUE;
		Timer_Check();   //Check the Timer flags for TIM7
		DoorTimerCountSub();      //process time out count
	//	update_CAN = 1;
		can_count++;
		fault_count++;
		fault_code = CheckFaultCode(DI,adc,&Error_trig);
		FaultIndicator(fault_code,fault_count,&flashnumber);
		doorspeed_cal = calculateDoorSpeed();
//		printf("Motor Current:%f,%f;\r\n",	adc->ADC_Cur_SNS1,	adc->ADC_Cur_SNS2);
//		printf("ADC=%d;%d\r\n",loopnumber,motoroutput);
//		printf("Clutch=%d; Motor=%d\r\n",DO.Clutch, Moto_Status);
		if (can_count == 1)
			send_dSw_stat = 1;
		else if (can_count == 2)
			send_dCur_stat = 1;
		else if (can_count == 3)
			send_swM_stat = 1;
		else if (can_count == 10)
		{
			can_count = 0;
			/***********************************
			 *  Error Messages Send Twice
			 ***********************************/
			ErrorMsgSend(&Error_flag, &Error_send, &Error_count);
		}
		LEDFunction(SWITCH_STUCK_flag);  //For testing LED purpose
	}
	CheckBraun_Op_Switch(DI);           //CheckIf switch is stuck or reset error.
}

/**
 * @brief  Conversion complete callback in non-blocking mode
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
//  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[0], ADC_CONVERTED_DATA_BUFFER_SIZE);
}
uint32_t encodecount;
float pre_Encode;
float pre_Cur_SNS1;
float pre_Cur_SNS2;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	float temp_encode;
	encodecount++;
	if (hadc->Instance ==ADC1)
	{
		adc->Motor_Cur_Open_Raw = adcData[1];
		adc->Motor_Cur_Close_Raw = adcData[3];
		adc->ADC_Cur_SNS1 =  (float) (adcData[3])/65535*3.3 ;
		//pre_Cur_SNS1 = adc->ADC_Cur_SNS1;
		adc->ADC_Cur_SNS2= (float)(adcData[1]) /65535*3.3;

		adc->ADC_Cur_SNS1 = 0.99 * pre_Cur_SNS1	+ 0.01* adc->ADC_Cur_SNS1;//(ADC_RAW_TO_VOLTAGE	* (float) (adcData[3]- MOTORCUROFFSET1)) ;
		pre_Cur_SNS1 = adc->ADC_Cur_SNS1;
		adc->ADC_Cur_SNS2= 0.99*pre_Cur_SNS2 + 0.01* adc->ADC_Cur_SNS2;//0.1 * (ADC_RAW_TO_VOLTAGE*(float)(adcData[1]- MOTORCUROFFSET1)) ;
		pre_Cur_SNS2 = adc->ADC_Cur_SNS2;

		adc->Bump_Strip_Signal = ADC_RAW_TO_VOLTAGE*(float) adcData[2];
		temp_encode = 0.9*pre_Encode+0.1*(float)adcData[0];
		pre_Encode = temp_encode;
		adc->Door_Encode_Signal = (int)temp_encode;

		adc->Motor_Cur_Close = (adc->ADC_Cur_SNS1  - MOTOR_CUR_CALI_INTERCEPT) * MOTOR_CUR_CALI_SLOP;
		adc->Motor_Cur_Open = (adc->ADC_Cur_SNS2  - MOTOR_CUR_CALI_INTERCEPT) * MOTOR_CUR_CALI_SLOP;

		if (error == 0)
		{
			if (adc->Bump_Strip_Signal < 1)
				CheckBumpStripEvent();
			else
				Door_flag.bump_deb_flag = FALSE;
		}
//	   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
		  SCB_InvalidateDCache_by_Addr((uint32_t *) &adcData, ADC_BUFFER_SIZE);
	}
}

void Outside_handle_Pressed(void )
{
	 if (CAN_Data.lock_status == TRUE)
			 return;

#ifndef NO_VEHICLE_CAN
	 if ((CAN_Data.lock_status == TRUE) && (CAN_Data.fob_nearby == FALSE))   //if door is locked and no fob is near, then do nothing.
	{
		 return;
	}
	 else
	 {
#endif
		  if (DI.Door_ajar == TRUE)														//if door is  closed.
		  {
			  SensorReading4OP = SensorReading;
			  if ((SensorReading4OP.NoseAngle > ANGLE_NOSE_CLUTCH_BEFORE_RELEASE_MAX)
				  || (SensorReading4OP.SideAngle >  ANGLE_PITCH_CLUTCH_BEFORE_RELEASE_MAX))
			  {
				  m_doorstep = OUTHANDLE_ACTIVATED; // Cause the hardfault, why?? need debugging
			  }
			  else
			  {
				  Outside_Handle_Wait_Time.flag = TRUE;
//				  MagnaOperation(DOOR_RELEASE);
//				  DO.Clutch = FALSE;
			  }
		  }
		  if (DI.full_door_open == TRUE)														//if door is  open.
		  {
			  SensorReading4OP = SensorReading;
			  if ((SensorReading4OP.NoseAngle < ANGLE_NOSE_CLUTCH_BEFORE_RELEASE_MIN)
				  || (SensorReading4OP.SideAngle > ANGLE_PITCH_CLUTCH_BEFORE_RELEASE_MAX))
			  {
				  m_doorstep = OUTHANDLE_ACTIVATED;
			  }
			  else
			  {
				  DO.Latch = TRUE;
				  DO.Clutch = FALSE;
			  }
		  }
#ifndef NO_VEHICLE_CAN
	 }
#endif
}


void Braun_Op_Pressed()
{
	if (initialDone == FALSE)
		return;

		if (DI.Door_ajar == TRUE)                    															//If Door is fully closed
		{
			m_doorstep = PREOPEN;
			PRINTF("Go to the PREOPEN.\r\n");
		}
		else if (DI.full_door_open == TRUE)																	//If Door is fully opened
		{
			ramp_stow_timeout.flag = TRUE;
			ramp_stow_timeout.count = 0;
			m_doorstep = PRECLOSE;
		}
		else if ((m_doorstep>=START_CLOSING) && (m_doorstep<=FULLY_CLOSED))
		{
			 m_doorstep = REVERSEOPEN;
		}
		else if ((m_doorstep>=START_OPEN) && (m_doorstep<=FULLY_OPENED))
		{
			 m_doorstep = REVERSCLOSE;
		}
		if (HAL_GPIO_ReadPin(Braun_Op_GPIO_Port,Braun_Op_Pin)== GPIO_PIN_SET)         //Keep holding button 3 seconds to reset Errors.
		{
			Door_flag.reset_error_flag = TRUE;
//			Door_Count.reset_error_count =0;
		}
		if (HAL_GPIO_ReadPin(Braun_Op_GPIO_Port,Braun_Op_Pin)== GPIO_PIN_RESET)         //Start operation when release the operation button.
		{
			printf("Door_flag.op_deb_flag = 1\r\n");
			Door_flag.op_deb_flag = 1;
			Door_Count.op_count = 0;
			Door_flag.reset_error_flag = FALSE;
		}
		ResetFlagsInIDLE();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	sleep_count = 0;
	maint_reset_flag = 1;
//	status_update_flag = 1;
	stat_countup = 0;
	debounce_flag = 1;
	debounce_count = 0;
	bus_sleep = 0;
	Resetdebounce(TRUE);

	/****************************
//	 * Full Latched Switch INT
//	 ***************************/

	/****************************
	 * Braun Operate Switch INT
	 ***************************/
	if (GPIO_Pin == Braun_Op_Pin) {
//		Braun_Op_Pressed();
	}

	/****************************
	 *
	 * Full Open Switch INT
	 *
	 ***************************/
	if (GPIO_Pin == Door_Open_Pin)
	{
		DIchanged=TRUE;
		//sprintf(buffer,"full open\r\n");
		// drew_print(buffer);
		if (error == 0)
		{
//				Door_flag.f_open_deb_flag = TRUE;
//				Door_Count.f_open_count = 0;
		}
	}

	/****************************
	 * Outside Handle Switch INT
	 *handle_sw_pressed
	 ***************************/
	if (GPIO_Pin == Outside_Handle_Pin) {
		//sprintf(buffer,"Handle\r\n");
		// drew_print(buffer);
//		DO.Release = TRUE;
//		Door_flag.release_hold_flag = TRUE;
//		Door_Count.release_hold_count = 0;
//		Door_flag.handle_deb_flag = 1;
//		Door_Count.handle_count = 0;

//		if (error == 0) {
//			if (HAL_GPIO_ReadPin(Outside_Handle_GPIO_Port,Outside_Handle_Pin)== GPIO_PIN_SET)
//			{
//				DI.Out_side_handle = TRUE;
//				handle_sw_pressed = TRUE;
//				Door_flag.handle_deb_flag = 1;
//				Door_Count.handle_count = 0;
//			}
//			else
//				handle_sw_pressed = FALSE;
//		}
	}

	/****************************
	 * Door Ajar Switch INT
	 ***************************/
	if (GPIO_Pin == Door_Ajar_Pin)
	{
		//sprintf(buffer,"ajar\r\n");
		DIchanged=TRUE;
		// drew_print(buffer);
		if (error == 0)
		{
//			if (HAL_GPIO_ReadPin(Door_Ajar_GPIO_Port,Door_Ajar_Pin)== GPIO_PIN_RESET)    //Ajar 1->0 during closing means fully close. Need verification.
//			{
				DI.Door_ajar = TRUE;
				door_full_closed =1;
				Door_flag.ajar_deb_flag = 1;
				Door_Count.ajar_count = 0;

//				if (m_doorstep == DOORIDLE)
//					m_doorstep = PREOPEN;
//			}
		}
	}

}
//END GPIO INT

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void JumpToBootloader(void) {
	uint32_t i = 0;
	void (*SysMemBootJump)(void);
	/* Set the address of the entry point to bootloader */
	volatile uint32_t BootAddr = 0x1FF09800;
	/* Disable all interrupts */
	__disable_irq();
	/* Disable Systick timer */
	SysTick->CTRL = 0;
	/* Set the clock to the default state */
	HAL_RCC_DeInit();
	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (i = 0; i < 5; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Re-enable all interrupts */
	__enable_irq();
	/* Set up the jump to booloader address + 4 */

	SysMemBootJump = (void (*)(void)) (*((uint32_t*) ((BootAddr + 4))));
	/* Set the main stack pointer to the bootloader stack */
	__set_MSP(*(uint32_t*) BootAddr);
	/* Call the function to jump to bootloader location */
	SysMemBootJump();

	/* Jump is done successfully */
	while (1) {
		/* Code should never reach this loop */
//    	 sprintf(buffer, "Broke");
//    	 drew_print(buffer);
	}
}

/**
 * Disable IWDG at runtime
 */
void SetIWDGForStandbyMode(void)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	// Initialize the Option Bytes structure
	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASHEx_OBGetConfig(&OBInit);

	// Set the IWDG_SW bit
	OBInit.OptionType = OPTIONBYTE_USER;
	OBInit.USERType = OB_USER_IWDG_STDBY;

//	OBInit.USERConfig = OB_IWDG_STDBY_ACTIVE;//OB_IWDG_STDBY_FREEZE; //to enable, use OB_IWDG_STOP_RUN
	OBInit.USERConfig = OB_IWDG_STDBY_FREEZE; //to enable, use OB_IWDG_STOP_RUN
	// Write the Option Bytes
	HAL_FLASHEx_OBProgram(&OBInit);

	// Launch Option Bytes programming
	HAL_FLASH_OB_Launch();

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

/********************************************
 * Sleep Count Exceeded, start sleep cycle
 ********************************************/
void Current_Sleep_Mode_Check(uint32_t wait_count)
{
	if (DEBUGSLEEPFlag == TRUE)
	   return;
	if ((wait_count >=(MAX_SLEEP_COUNT-10))	 && (Moto_Status ==FALSE))					//Give two time cycle to operate the Wake mode
	{
		HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, GPIO_PIN_RESET);
//		LSM6DSO_ACC_Disable(&MotionSensor);
		//Set_Lsm6dso_Sleep_Mode(TRUE);
		//LSM6DSO_ACC_Disable(&MotionSensor);             //Power off sensor;
	}
	if ((wait_count > MAX_SLEEP_COUNT+1)	 && (Moto_Status ==FALSE))					//20 seconds
	{
		ReadWritEEPROMVariables(&eepromcount_t, WRITE);
		LSM6DSO_ACC_Disable(&MotionSensor);             //Power off sensor;
		Set_Lsm6dso_Sleep_Mode(TRUE);
		HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, GPIO_PIN_RESET);
		go_to_sleep = FALSE;
		sleep_wait = FALSE;
		start_sleep = FALSE;
		sleeping = 1;

		HAL_TIM_Base_Stop_IT(&htim6);
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		if (HAL_FDCAN_DeactivateNotification(&hfdcan2,
				FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK) {
			_Error_Handler("Current_Sleep_Mode_Check",724);//Error_Handler();Error_Handler();
		}
		HAL_FDCAN_Stop(&hfdcan2);
		HAL_TIM_Base_Stop(&htim6);

		HAL_UART_MspDeInit(&huart2);
		HAL_ADC_DeInit(&hadc1);


		HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, 1);
		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, 0);
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); //disable wakeup pin
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); //disable wakeup pin
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); //disable wakeup pin
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); //disable wakeup pin


		  /* Clear all related wakeup flags */
		  HAL_PWREx_ClearWakeupFlag(PWR_WAKEUP_PIN_FLAGS);

		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);//clear WU flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP6); //clear WU flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP4); //clear WU flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP2); //clear WU flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP1); //clear WU flag


		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		 /* Enable WakeUp Pin PWR_WAKEUP_PIN4 connected to PC.13 User Button */
		  sPinParams.WakeUpPin    = PWR_WAKEUP_PIN6 | PWR_WAKEUP_PIN4;
		  sPinParams.PinPolarity  = PWR_PIN_POLARITY_LOW;
		  sPinParams.PinPull      = PWR_PIN_NO_PULL;
		  HAL_PWREx_EnableWakeUpPin(&sPinParams);
		  SetIWDGForStandbyMode();
		//HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6_LOW);  //enable WU pin
		  HAL_SuspendTick();
		  HAL_PWR_EnterSTANDBYMode();
	}
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
	AUTOCYCLE = FALSE;
	initialDone = FALSE;
	memset(EncodeBuf, 0, sizeof EncodeBuf);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);

  	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  	HAL_Delay(10);
  //	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcData, ADC_NUM_CONVERSIONS);
  //	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *) adc3Data, ADC3_BUFFER_SIZE)!= HAL_OK)
  //	{
  //		printf("hadc3 error with HAL_ADC_Start_DMA.\r\n");
  //		Error_Handler();
  //	}

  	if (WATCHDOEENABLE)
  	{
  //		DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  //		DBGMCU->APB1FZ = DBGMCU->APB1FZ | DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  //		__HAL_DBGMCU_FREEZE_IWDG();
  		if (HAL_IWDG_Init(&hiwdg1) != HAL_OK) {
  			_Error_Handler("main",825);//Error_Handler();
  		}
  	}
  //
  	HAL_Delay(10);
  	HAL_TIM_Base_Start(&htim8);
  	HAL_Delay(10);
  	MEMS_Init();
  	SetIWDGForStandbyMode();
  	/*****************************************
  	 *
  	 * Reset the Wakeup flags for operation
  	 *
  	 *****************************************/

  	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
  		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); //clear WU flag
  		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP4); //clear WU flag
  		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP6); //clear WU flag
  		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP1); //clear WU flag

  		//Reset the CAN pins for operation
  		HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, 1);
  		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, 1);
  	}

  	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); //disable wakeup pin
  	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); //disable wakeup pin
  	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); //disable wakeup pin
  	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); //disable wakeup pin

  	/*****************************************
  	 * If CAN error, then reset
  	 ****************************************/

  	if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
  		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
  		HAL_Delay(2);
  		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
  	}
  	//HAL_GPIO_WritePin(CAN_RX_GPIO_Port, CAN_RX_Pin,GPIO_PIN_SET);

  	/*************************************
  	 *Set up all the CAN Filtering
  	 *************************************/

  	Set_CAN_Filters();
  	HAL_TIM_Base_Start_IT(&htim6);
  	HAL_TIM_Base_Start_IT(&htim7);
  	HAL_ADC_Start_IT(&hadc1);

  	/* Enable Back up SRAM */
  	/* Enable write access to Backup domain */
  	PWR->CR1 |= PWR_CR1_DBP;
  	while ((PWR->CR1 & PWR_CR1_DBP) == RESET) {
  	}
  	/*Enable BKPRAM clock*/
  	__HAL_RCC_BKPRAM_CLK_ENABLE();
  //
  	Read_Variable_From_Memory();

  	status_update_count = 0;
  	wake_door_flag = 1;
  	Initial_Before_Loop();
  	HAL_UART_Receive_IT(&huart2,UART2_rxBuffer,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
			/*******************************
		 * Jump to Bootloader Section
		 *******************************/
		if (JTBL == 1) {
			JTBL = 0;
			JumpToBootloader();
		}
//		loopnumber++;
		Current_Sleep_Mode_Check(sleep_wait_count);//if Sleep Count Exceeded, start sleep cycle

		UpdateAllInputs();

		Check_MEMS();   						 // Get LSM6xyz and LSM6 temperature.
		Check_CAN_data();  					 //If CAN data is recieved, check it
		CheckReverseOperation(pre_Encode, adc);
//		Door_Operation(DI,adc->Door_Encode_Signal);
		RunOutputCycle();               			 //For test the DO purpose;
		RampLight_Operation();
		UpdateAlloutputs();
		Send_CAN_data();  						//Update/Send out the CAN data
		/* Refresh IWDG: reload counter */
		if (WATCHDOEENABLE)
		{
				if (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK) {
					/* Refresh Error */
					_Error_Handler("main",928);//Error_Handler();
				}
		}

		if (UartRecDoneFlag == TRUE)
		{
			processUartMsg(&uart_msg);
			UartRecDoneFlag = FALSE;
		}
		SetMotorSpeedbyInclination(SensorReading4OP);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (Enable_PI_Moniter == TRUE)
		{
			if (UARTIOUpdateCount > 6)
				UARTIOUpdateCount = 0;

			SendIOstatus(UARTIOUpdateCount);
			UARTIOUpdateCount++;
			Enable_PI_Moniter = FALSE;
		}

		initialDone = TRUE;
////		HAL_Delay(1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 3;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 10;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 5;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
	Set_CAN_Tx_Messages();
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */
	if (WATCHDOEENABLE) {
  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg1.Init.Window = 1000;
  hiwdg1.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */
	}
  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 259;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 259;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 40000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 17000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 501;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */


  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 43;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 62999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 275-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, Wakeup_Pin|Fault_LED_Pin|CLUTCH_Pin|Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LATCH_DIR_Pin|Red_LED_Pin|White_LED_Pin|Yellow_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LATCH_Disable_GPIO_Port, LATCH_Disable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, White_LED_12V_Pin|LATCH_VSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LATCH_PWM_Pin|Ramp_Light_Pin|Lock_Release_Enable_Pin|Lock_Release_Trigger_Pin
                          |Lock_Unlock_Coil_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_1_Pin|STB_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Braun_Op_Pin */
  GPIO_InitStruct.Pin = Braun_Op_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Braun_Op_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Wakeup_Pin Fault_LED_Pin Latch_Pin */
  GPIO_InitStruct.Pin = Wakeup_Pin|Fault_LED_Pin|Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLUTCH_Pin */
  GPIO_InitStruct.Pin = CLUTCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CLUTCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ERR_CAN_Pin ERR_1_Pin */
  GPIO_InitStruct.Pin = ERR_CAN_Pin|ERR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LATCH_DIR_Pin Red_LED_Pin White_LED_Pin Yellow_LED_Pin */
  GPIO_InitStruct.Pin = LATCH_DIR_Pin|Red_LED_Pin|White_LED_Pin|Yellow_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LATCH_Disable_Pin EN_1_Pin STB_1_Pin */
  GPIO_InitStruct.Pin = LATCH_Disable_Pin|EN_1_Pin|STB_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Door_Open_Pin */
  GPIO_InitStruct.Pin = Door_Open_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Door_Open_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : White_LED_12V_Pin LATCH_VSO_Pin */
  GPIO_InitStruct.Pin = White_LED_12V_Pin|LATCH_VSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Kneel_Disable_Pin */
  GPIO_InitStruct.Pin = Kneel_Disable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Kneel_Disable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Lock_Status_Pin Spare2_Pin Conversion_Dis_Pin */
  GPIO_InitStruct.Pin = Lock_Status_Pin|Spare2_Pin|Conversion_Dis_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Outside_Handle_Pin Door_Ajar_Pin */
  GPIO_InitStruct.Pin = Outside_Handle_Pin|Door_Ajar_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LATCH_PWM_Pin Ramp_Light_Pin Lock_Release_Enable_Pin Lock_Release_Trigger_Pin
                           Lock_Unlock_Coil_Pin */
  GPIO_InitStruct.Pin = LATCH_PWM_Pin|Ramp_Light_Pin|Lock_Release_Enable_Pin|Lock_Release_Trigger_Pin
                          |Lock_Unlock_Coil_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(Door_Open_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(Door_Open_EXTI_IRQn);

  HAL_NVIC_SetPriority(Outside_Handle_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(Outside_Handle_EXTI_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/***************************************
 *
 * USART IRQ for Jumping to bootloader
 *
 ***************************************/
void USART2_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart2);

	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
	//HAL_UART_Transmit(&huart2, UART2_rxBuffer, 12, 100);

	if (UART2_rxBuffer[0] == 0x7F) {
		//sprintf(buffer,"RX: %x\r\n",UART2_rxBuffer[0]);
		//drew_print(buffer);
//		SCB_DisableDCache();
		JTBL = 1;
	} else {
		JTBL = 0;
	}

	switch (UART2_rxBuffer[0])
	{
	case 33:   //"!"
		sleep_count = 0;
		diag_send = 1;
		if (diag_update == 0)
		{
			diag_update = 1;
//			printf("diag:%d%d%d%d%d%d%d%d%d%d%d", door_stat, ramp_state, park_stat, !operate_sw, !handle, full_door_open, !door_full_closed, unlock_timeout_flag, bumper, model_year);
//			printf("count:%d,%d,%d,%d,%d,%d,%d,%d,%d?%c%c%c", prev_open_count, prev_close_count, prev_hndl_count, prev_switch_count, prev_reverse_count, prev_overcurrent_count, maint_counter, encoder_data, cinch_data, rev_level[0], rev_level[1], rev_level[2]);
		}
		break;

	default :
		break;

	}

	if ((UART2_rxBuffer[0] != 0) && (UART2_rxBuffer[0]!='\n'))
		AddCharToUartBuf(UART2_rxBuffer[0]);
}


void Read_Variable_From_Memory(void)
{
	ReadWritEEPROMVariables(&eepromcount_t, READ);
	pre_eepromcount_t = eepromcount_t;
	Full_Opened_Encode = eepromcount_t.full_opened_encode;
	Full_Closed_Encode = eepromcount_t.full_closed_encode;
	if (Full_Opened_Encode == 0)
		Full_Opened_Encode = 20000;
	if (Full_Closed_Encode == 0)
		Full_Closed_Encode = 55000;

}


/******************************************************
 * CAN received IRQ, sets flag to be checked in MAIN
 ******************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan2,uint32_t RxFifo0ITs)
{
	sleep_wait_count = 0;
	update_CAN = 1;
	can_rx_int = 1;
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
      while(1)
      {
        // Optional: Send error information via UART for debugging
 //        printf("Error occurred in file: %s, line: %d\r\n", file, line);
        HAL_Delay(500);
      }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
