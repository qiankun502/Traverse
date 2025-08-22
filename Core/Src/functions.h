/*
 * functions.h
 *
 *  Created on: Jul 11, 2024
 *      Author: andrew.henseleit
 *      		Ken Qian
 */

#ifndef SRC_FUNCTIONS_H_
#define SRC_FUNCTIONS_H_
#include "variables.h"
#include "main.h"
static int32_t LSM6DSO_Read_All_FIFO_Data(void);
static int32_t LSM6DSO_Read_Single_FIFO_Data(uint16_t SampleIndex);
int16_t xyz[4];
//int16_t LSM6xyz[4];

//float LSM6Angle[3];
uint8_t door_stat=1;
/**
 * Initial the gyo sensor
 */
uint8_t MEMS_Retry_count;
void MEMS_Init(void) {
	LSM6DSO_IO_t io_ctx;
	uint8_t id;
	LSM6DSO_AxesRaw_t axes;

	/* Link I2C functions to the LSM6DSL driver */
	io_ctx.BusType = LSM6DSO_I2C_BUS;
	io_ctx.Address = LSM6DSO_I2C_ADD_L;
	io_ctx.Init = BSP_I2C4_Init;
	io_ctx.DeInit = BSP_I2C4_DeInit;
	io_ctx.ReadReg = BSP_I2C4_ReadReg;
	io_ctx.WriteReg = BSP_I2C4_WriteReg;
	io_ctx.GetTick = BSP_GetTick;
	LSM6DSO_RegisterBusIO(&MotionSensor, &io_ctx);

	/* Read the LSM6DSL WHO_AM_I register */
	LSM6DSO_ReadID(&MotionSensor, &id);
	printf("ID:%x\r\n", id);
	if (id != LSM6DSO_ID)
	{
		MEMS_Retry_count++;
		HAL_Delay(10);
		if (MEMS_Retry_count<4)
		{
			MEMS_Init();
		}
		else
		{
			Gyro_Sensor_Error_Flag = TRUE;     //Sensor initialization continue failed over 4 times
		}
//		Error_flag.dSw = TRUE;
	//	return;
		//_Error_Handler("MEMS_Init",42);
	}
	Gyro_Sensor_Error_Flag = FALSE;
//	/* Initialize the LSM6DSL sensor */
//	LSM6DSO_Init(&MotionSensor);
	/* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
	LSM6DSO_ACC_SetOutputDataRate(&MotionSensor, LSM6DSO_SAMPLE_ODR); /* 10 Hz */
	LSM6DSO_FIFO_ACC_Set_BDR(&MotionSensor, LSM6DSO_XL_BATCHED_AT_6667Hz);
	LSM6DSO_FIFO_Set_INT1_FIFO_Full(&MotionSensor, ENABLE); /* Enable DRDY */
	LSM6DSO_FIFO_Set_Watermark_Level(&MotionSensor, FIFO_WATERMARK);
	LSM6DSO_FIFO_Set_Stop_On_Fth(&MotionSensor, ENABLE);
	LSM6DSO_ACC_SetFullScale(&MotionSensor, 4); /* [-4000mg; +4000mg] */
	LSM6DSO_ACC_GetAxesRaw(&MotionSensor, &axes); /* Clear DRDY */

	LSM6DSO_Init(&MotionSensor);
	/* Start the LSM6DSL accelerometer */
	LSM6DSO_ACC_Enable(&MotionSensor);
	MEMS_Retry_count = 0;
}


/*
 * Set the sleep mode of the gyro sensor;
 * sleep = FALSE: wake up;
 *	sleep = TRUE: go to sleep;
 */
void  Set_Lsm6dso_Sleep_Mode(char sleep)
{
	 if (sleep)
	 {
		 //return lsm6dso_gy_sleep_mode_set(&(MotionSensor->Ctx), 0x00);
//		 LSM6DSO_Goto_Sleep(&MotionSensor, 0x1);
		 LSM6DSO_GYRO_Disable(&MotionSensor);
//		 LSM6DSO_ACC_Disable(&MotionSensor);
	 }
	 else
	 {
//		 return lsm6dso_gy_sleep_mode_set(MotionSensor->Ctx, 0x01);
//		 LSM6DSO_GYRO_Enable(&MotionSensor);
//		 LSM6DSO_Goto_Sleep(&MotionSensor, 0x0);
	 }
}
/**
 * Read X, Y, Z angles and temperature
 */
static int32_t LSM6DSO_Read_All_FIFO_Data(void) {
	int32_t ret;

	if ((ret = LSM6DSO_Read_Single_FIFO_Data(0)) != BSP_ERROR_NONE) {
			return ret;
		}
	if (Gyro_Sensor_Error_Flag ==TRUE)                //Sensor is failed.
	{
		SensorReading.NoseAngle = 0;
		SensorReading.SideAngle = 0;
		ret = -1;
	}
	return ret;
}
static int32_t LSM6DSO_Read_Single_FIFO_Data(uint16_t SampleIndex) {
	LSM6DSO_Axes_t angular_velocity;
	int32_t ret = BSP_ERROR_NONE;

	/* Read single FIFO data (angular velocity in all 3 axes) */
	if ((ret = LSM6DSO_ACC_GetAxes(&MotionSensor, &angular_velocity)) != BSP_ERROR_NONE) {
		return ret;
	}
	xyz[0] = (xyz[0] + angular_velocity.x) / 2;
	xyz[1] = (xyz[1] + angular_velocity.y) / 2;
	xyz[2] = (xyz[2] + angular_velocity.z) / 2;

	if ((ret = LSM6DSO_Read_Reg(&MotionSensor, 0x20, &TemperatureL))
			!= BSP_ERROR_NONE) {
		return ret;
	}
	if ((ret = LSM6DSO_Read_Reg(&MotionSensor, 0x21, &TemperatureH))
			!= BSP_ERROR_NONE) {
		return ret;
	}
	xyz[3] = (float) TemperatureH + ((float) TemperatureL) / 256
			+ 25; //format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25°C
	xyz[4] = 1;    //updated
	if (ret != BSP_ERROR_NONE) {
		return ret;
	}
	return ret;
}

int waitcount;
/**
 * Read angle and temperature from sensor and convert from the body coordination to the earth coordination
 * return 0:  get the data;
 * return 1: skip the reading.
 * return -1: reading error.
 */
uint8_t Check_MEMS(void) {
	waitcount++;
	if (waitcount<100)
	{
		return 1;
	}
	waitcount =0;
	if (LSM6DSO_Read_All_FIFO_Data() != BSP_ERROR_NONE)
	{
		MEMS_Init();
		Gyro_Sensor_Error_Flag = TRUE;     //Sensor is failed
//			Error_flag.dSw = TRUE;
		return -1;
//			_Error_Handler("Check_MEMS",113);
	}
	Gyro_Sensor_Error_Flag= FALSE;
	if (xyz[4] == 1)   // result updated.
	{
		SensorReading.LSM6AngleX = RAD_TO_DEG * (atan2((float) (-xyz[1]), (float) (-xyz[2])) + PI);
		SensorReading.LSM6AngleY = RAD_TO_DEG * (atan2((float) (-xyz[0]), (float) (-xyz[2])) + PI);
		SensorReading.LSM6AngleZ = RAD_TO_DEG * (atan2((float) (-xyz[1]), (float) (-xyz[0])) + PI);
		SensorReading.LSM6DSOtemperature =xyz[3];
		if (SensorReading.LSM6AngleX>180)
			SensorReading.LSM6AngleX = SensorReading.LSM6AngleX-360;
		if (SensorReading.LSM6AngleY>180)
			SensorReading.LSM6AngleY = SensorReading.LSM6AngleY-360;
		if (SensorReading.LSM6AngleZ>180)
			SensorReading.LSM6AngleZ = SensorReading.LSM6AngleZ-360;
		xyz[4] = 0;

		if (SensorReading.LSM6AngleZ > 0)
			SensorReading.NoseAngle =  SensorReading.LSM6AngleZ - 180+1.1;  //nose down
		else
			SensorReading.NoseAngle =  SensorReading.LSM6AngleZ + 180+1.1;   //nose up

		SensorReading.SideAngle =SensorReading.LSM6AngleY+90-2.4;	// 90+SensorReading.LSM6AngleY;	//Side left is Negative, Side right is positive

		SensorReading.NoseAngle = 0.05 * SensorReading.NoseAngle + 0.95 * PreSensorReading.NoseAngle;
		SensorReading.SideAngle = 0.05 * SensorReading.SideAngle + 0.95 * PreSensorReading.SideAngle;

		PreSensorReading =SensorReading;

//		//Force angles to 0 for debugging purpose.
		if (FORCE_ANGLE)
		{
			SensorReading.NoseAngle =1;
			SensorReading.SideAngle = 1.5;
		}
		return 0;
	}
/************************************************
 * The reading of inclination sensor:
 * 		NO slop   Nose up(45)      Nose Down(45)    slap right(45)  slap left(45)
 * x	90			90				90				53					135
 * y	-109		90				-90				0					177
 * z	90			47				137				90					90
 ************************************************/
}


/**********************************************
 *
 * Motor Sections
 * PWM SPEED MIN=3, max=61.  Below 3 gives you GND, above 61 gives you solid 12V
 *
 **********************************************/

void Stop_Motors(void) {
	//HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	//printf("Stop_Motors;\r\n");
	Moto_Status = 0;
	//HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, 0);
	DO.Wakeup = FALSE;
}

void Start_Open(int pulser) {
	HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, 1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulser);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	//HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, 1);
	DO.Wakeup = TRUE;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	Moto_Status = 1;
	//printf("Start_Open at %d ;\r\n" , pulser);
}

void Start_Close(int pulser) {
	HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, 1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulser);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//	HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, 1);
	DO.Wakeup = TRUE;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	Moto_Status = 2;
	//printf("Start_Close at %d ;\r\n" , pulser);
}

void Pause_Hold(void) {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(CLUTCH_GPIO_Port, CLUTCH_Pin, 1);
	//HAL_GPIO_WritePin(Wakeup_GPIO_Port, Wakeup_Pin, 1);
	DO.Wakeup = TRUE;
	Moto_Status = 3;
	//printf("Pause_Hold;\r\n" );
}


/**
 * Calculate the Door speed with encode reading.
 * Use the different of encode reading with 0.5 seconds length (10msx50)
 */
float calculateDoorSpeed(void)
{
	float doorspeed;
	float X,Y;
	float SumXY=0;
	float SumX2=0;
	float SumY=0;
	float SumX=0;
	for (int i=ENCODEBUFSIZE-1;i>0;i--)			//EncodeBuf[0] is the latest reading EncodeBuf[ENCODEBUFSIZE-1] is the oldest reading
	{
		EncodeBuf[i] = EncodeBuf[i-1];
		EncodeBuf[0] = adc->Door_Encode_Signal;
	}
	//doorspeed = (float)(EncodeBuf[0]- EncodeBuf[ENCODEBUFSIZE-1]) / 0.64 /ENCODESPERMETER;
	if (EncodeBuf[ENCODEBUFSIZE-1] ==0 )
		doorspeed = 0;
	else
	{
		   for (int i = 0; i <ENCODEBUFSIZE-1; i++)
		   {
			   X = 0.01 *((float) i);                                   	 //second
			   Y =(float) EncodeBuf[i]*CMPerCODE;   		 //cm
			   SumXY = SumXY +Y*X;
			   SumX2 = SumX2 + X*X;
			   SumY = SumY + Y;
			   SumX = SumX + X;
		   }
		   doorspeed = ((ENCODEBUFSIZE-1) * SumXY - SumX * SumY) / ((ENCODEBUFSIZE-1) * SumX2 - SumX * SumX);
	}
	//	doorspeed = (float)(EncodeBuf[0]- EncodeBuf[ENCODEBUFSIZE-1]) / 0.2 /ENCODESPERMETER;
	if (adc->Door_Encode_Signal < (Full_Opened_Encode + ENCODE_FAST_RANGE))    //Encode change too fast near the fully opened, need more data for encode vs speed
		doorspeed = doorspeed /2;

	return -doorspeed;
}

/*
 * For Single dimension angle test. No compound of the angles.
 *If Pitch angle in (-5,5), then run Nose angle array. If Nose angle in (-5,5), then run Pitch angle array
 * Return -1 if no angle available, else return 0.
 */
int SetMotorSpeedBySingleAngle( int angle_type, LSM6SENSOR_READING sensor_reading)
{
	int i;
	int noseanglezone=0,pitchanglezone=0;

	if ((sensor_reading.NoseAngle > MAXINCLINATION) || (sensor_reading.NoseAngle < MININCLINATION))  //Nose angle over range
		return -1;
	if ((sensor_reading.SideAngle > MAXINCLINATION) || (sensor_reading.SideAngle < MININCLINATION))  //pitch angle over range
		return -2;

	int zonesize = (int) (sizeof(NoseAngleArray_Test)/sizeof(float));
	for (i=0;i<zonesize-1;i++)
	{
		if ((sensor_reading.NoseAngle <= NoseAngleArray_Test[i+1]) && (sensor_reading.NoseAngle > NoseAngleArray_Test[i]))
		{
			noseanglezone = i;
		}
	}
	zonesize = (int) (sizeof(NoseAngleArray_Test)/sizeof(float));      //REVIEW: Pitch angle for 9 zones
	for (i=0;i<zonesize-1;i++)
	{
		if ((sensor_reading.SideAngle <= PitchAngleArray_Test[i+1]) && (sensor_reading.SideAngle > PitchAngleArray_Test[i]))
		{
			pitchanglezone = i;
		}
	}

	 if (angle_type == NOSE_ANGLE_TEST)
	 {
		  open_door_speed_1 = V_Start_Open_Nose[noseanglezone];//OPEN_DOOR_SPEED_0 + compensation;
		  open_door_speed_2 = V_Open_Nose[noseanglezone];//OPEN_DOOR_SPEED_1 + compensation;
		  open_door_speed_3 = V_End_Open_Nose[noseanglezone];//OPEN_DOOR_SPEED_2 + compensation;
		  close_door_speed_1 = V_Start_Close_Nose[noseanglezone];//CLOSE_DOOR_SPEED_0 + compensation;
		  close_door_speed_2 = V_Close_Nose[noseanglezone];//CLOSE_DOOR_SPEED_1 + compensation;
		  close_door_speed_3 = V_End_Close_Nose[noseanglezone];//CLOSE_DOOR_SPEED_2 + compensation;

		  if ((m_doorstep <OUTHANDLE_ACTIVATED) && (m_doorstep != DOOR_MONITOR))         //only get the unlatch time from table during cycles.
			  open_door_time_unlatch = Time_Pull_Door_Latch_Nose[noseanglezone];
		  else
			  open_door_time_unlatch = 300;

		  open_door_speed_unlatch = Power_Pull_Door_Latch_Nose[noseanglezone];
	 }
	 if (angle_type == PITCH_ANGLE_TEST)
	 {
		  open_door_speed_1 = V_Start_Open_Pitch[pitchanglezone];//OPEN_DOOR_SPEED_0 + compensation;
		  open_door_speed_2 = V_Open_Pitch[pitchanglezone];//OPEN_DOOR_SPEED_1 + compensation;
		  open_door_speed_3 = V_End_Open_Pitch[pitchanglezone];//OPEN_DOOR_SPEED_2 + compensation;
		  close_door_speed_1 = V_Start_Close_Pitch[pitchanglezone];//CLOSE_DOOR_SPEED_0 + compensation;
		  close_door_speed_2 = V_Close_Pitch[pitchanglezone];//CLOSE_DOOR_SPEED_1 + compensation;
		  close_door_speed_3 = V_End_Close_Pitch[pitchanglezone];//CLOSE_DOOR_SPEED_2 + compensation;

		  if ((m_doorstep <OUTHANDLE_ACTIVATED) && (m_doorstep != DOOR_MONITOR))        //only get the unlatch time from table during cycles.
			  open_door_time_unlatch = Time_Pull_Door_Latch_Pitch[pitchanglezone];
		  else
			  open_door_time_unlatch = 300;

		  open_door_speed_unlatch = Power_Pull_Door_Latch_Pitch[pitchanglezone];
	 }
	 return 0;
}

/*
 * For Combined angle
 */
int SetMotorSpeedByCombinedAngle( LSM6SENSOR_READING sensor_reading)
{
	int compensation=0;
	int i;
	int noseanglezone=0,pitchanglezone=0;

	if ((sensor_reading.NoseAngle > MAXINCLINATION) || (sensor_reading.NoseAngle < MININCLINATION))  //Nose angle over range
		return -1;
	if ((sensor_reading.SideAngle > MAXINCLINATION) || (sensor_reading.SideAngle < MININCLINATION))  //pitch angle over range
		return -2;

	int zonesize = (int) (sizeof(NoseAngleArray)/sizeof(float));
	for (i=0;i<zonesize-1;i++)
	{
		if ((sensor_reading.NoseAngle <= NoseAngleArray[i]) && (sensor_reading.NoseAngle > NoseAngleArray[i+1]))
		{
			noseanglezone = i;
		}
	}
	zonesize = (int) (sizeof(PitchAngleArray)/sizeof(float));      //REVIEW: Pitch angle for 9 zones
	for (i=0;i<zonesize-1;i++)
	{
		if ((sensor_reading.SideAngle <= PitchAngleArray[i]) && (sensor_reading.SideAngle > PitchAngleArray[i+1]))
		{
			pitchanglezone = i;
		}
	}
	  open_door_speed_1 = V_Start_Open[pitchanglezone][noseanglezone];//OPEN_DOOR_SPEED_0 + compensation;
	  open_door_speed_2 = V_Open[pitchanglezone][noseanglezone];//OPEN_DOOR_SPEED_1 + compensation;
	  open_door_speed_3 = V_End_Open[pitchanglezone][noseanglezone];//OPEN_DOOR_SPEED_2 + compensation;
	  close_door_speed_1 = V_Start_Close[pitchanglezone][noseanglezone];//CLOSE_DOOR_SPEED_0 + compensation;
	  close_door_speed_2 = V_Close[pitchanglezone][noseanglezone];//CLOSE_DOOR_SPEED_1 + compensation;
	  close_door_speed_3 = V_End_Close[pitchanglezone][noseanglezone];//CLOSE_DOOR_SPEED_2 + compensation;

	  open_door_speed_unlatch = 100;
	return 0;
}

/**
 * Adjust the motor speed by inclination. Start with the linear compensation.
 */
int SetMotorSpeedbyInclination(LSM6SENSOR_READING sensor_reading)
{
//	/*******Temp code*******/
//		  open_door_speed_1 = 100;//OPEN_DOOR_SPEED_0 + compensation;
//		  open_door_speed_2 = 100;//OPEN_DOOR_SPEED_1 + compensation;
//		  open_door_speed_3 = 100;//OPEN_DOOR_SPEED_2 + compensation;
//		  close_door_speed_1 = 100;//CLOSE_DOOR_SPEED_0 + compensation;
//		  close_door_speed_2 = 100;//CLOSE_DOOR_SPEED_1 + compensation;
//		   close_door_speed_3 = 100;
//		   open_door_speed_unlatch = 100;
//		   open_door_time_unlatch =100;
//		   return 4;
//	///*****************************/

	if ((sensor_reading.SideAngle <3 && sensor_reading.SideAngle >-3) && (sensor_reading.NoseAngle <3 && sensor_reading.NoseAngle >-3))
	{
		  open_door_speed_1 = V_Start_Open_Flat_Ground;
		  open_door_speed_2 = V_Open_Flat_Ground;
		  open_door_speed_3= V_End_Open_Flat_Ground;
		  close_door_speed_1 = V_Start_Close_Flat_Ground;
		  close_door_speed_2 = V_Close_Flat_Ground;
		  close_door_speed_3 = V_End_Close_Flat_Ground;
		  if ((m_doorstep <OUTHANDLE_ACTIVATED) && (m_doorstep != DOOR_MONITOR))
			  	  open_door_time_unlatch = Time_Pull_Door_Latch_Flat_Ground;
		  else
			  open_door_time_unlatch = 300;
		  open_door_speed_unlatch = Power_Pull_Door_Latch_Flat_Ground;
		  return 0;
	}
	if (TWO_DIMESION_ANGLE)		//if using combined angle
		SetMotorSpeedByCombinedAngle(sensor_reading);
	else
	{
		if (abs(sensor_reading.SideAngle)>abs(sensor_reading.NoseAngle))
			SetMotorSpeedBySingleAngle(PITCH_ANGLE_TEST,sensor_reading);
		else
			SetMotorSpeedBySingleAngle(NOSE_ANGLE_TEST,sensor_reading);
		return 0;
	}
	return 0;
}

/**************************************
 * Update/Send out the CAN data
 * #define SW_Stat 0x60;
#define Dr_Stat 0x40;
#define Dr_Curr 0x41;
#define Dr_Sw_Er 0x300;
#define Dr_REV_OPEN_Er 0x301;
#define Dr_REV_CLOSE_Er 0x302;
#define Dr_En_Er 0x304;
#define Dr_To_Er 0x303;
#define Ci_To_Er 0x306;
 **************************************/
void Send_CAN_data(void)
{

	uint8_t door_state;
	uint16_t motor_current_in_int;
	char Reversed_OP_swich= (DI.Braun_Op_Switch) ^ CAN_Data.Fob_Op_Cmd;
	char Reversed_FullOpen= !(DI.full_door_open);
	can_sleep_count = 0;
	if (update_CAN == 0)
		return;

	update_CAN = 0;
	Check_Door_State(m_doorstep, DI);
	/**********************
	 * Door Switch Update
	 * Dr_Stat 0x40;
	 **********************/
	if (send_dSw_stat == 1)
	{
		send_dSw_stat = 0;
		/******Collect the data to send****/
		TxData[0] =  (DI.Door_ajar<<4) | Reversed_FullOpen; 	 //Door full open + Power_Door_Ajar //Power_Door_Full_open
		TxData[1] = door_stat;//CAN_Data.door_stat; ///Power_Door_Stat
		TxData[2] = 0;//adc->Door_Encode_Signal & 0xff;//encoder_data;			//Door_Postion
		TxData[3] = 0;//(adc->Door_Encode_Signal>>8) & 0xff;	//Cinch_Position, where is it come from?
		TxData[4] = 0;//Cinch_Position, where is it come from?
		TxData[5] = 0;
		TxData[6] =  ((VERSION0) + (VERSION1*10));
		TxData[7] =  VERSION2 & 0x0F;

		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Stat_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",368);
			}
		}
	}

	/**********************
	 * Door Current Update
	 * 	 * Dr_Stat 0x41;
	 **********************/
	if (send_dCur_stat == 1)
	{
		send_dCur_stat = 0;
		TxData[1] = ((int) (adc->Motor_Cur_Open_Raw)) &0xFF;         			//Power_Door_Open_Curr_L
		TxData[0] = ((int)(adc->Motor_Cur_Open_Raw)>> 8) &0xFF; 			//Power_Door_Open_Curr_H
		TxData[3] = ((int) (adc->Motor_Cur_Close_Raw)) &0xFF;  				//Power_Door_Close_Curr_L
		TxData[2] = ((int)(adc->Motor_Cur_Close_Raw)>> 8) &0xFF; 			//Power_Door_Close_Curr_H
		TxData[4] = (adc->Door_Encode_Signal>>8) & 0xff;          			//Encode  High
		TxData[5] = (adc->Door_Encode_Signal) & 0xff;							//Encode low Nibbles
		TxData[6] = 0;
		TxData[7] = 0;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0)
		{
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		}
		else
		{
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Curr_Header, TxData)!= HAL_OK)
			{
				_Error_Handler("Send_CAN_data",399);//Error_Handler();
			}
		}
	}

	/**********************
	 *
	 * Switch Module Update
	 *ID 60
	 **********************/
	if (send_swM_stat == 1) {
		send_swM_stat = 0;
		char ConvDisable;
		if (DI.Kneel_disabled)
			ConvDisable =1;
		else if (DI.ConvDisabled)
			ConvDisable = 2;
		else
			ConvDisable = 0;

		if (Braun_Op_Switch_UART_flag == TRUE)				//The operator switch command from UART
		{
			DI.Braun_Op_Switch= TRUE;
			Braun_Op_Switch_UART_flag = FALSE;
		}

		TxData[0] =  Reversed_OP_swich<<4  | ConvDisable  ;    //Ramp_Kneel_Disable and Operate_SW_Status
		TxData[1] =  (DI.Door_ajar<<4) | Reversed_FullOpen; 		//Power_Door_Full_open and Power_Door_Ajar
		TxData[2] = door_stat;										                  //Door status
		TxData[3] = Reversed_OP_swich;//DI.Braun_Op_Switch;  										//Operate_From_Switch
		TxData[4] = 9; // ((VERSION0) + (VERSION1*10));          //(REV_LEVEL_0 <<4) | REV_LEVEL_1)
		TxData[5] = 2; // VERSION2 & 0x0F;			//REV_LEVEL_2
		TxData[6] = 0;
		TxData[7] = 0;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &SW_Stat_Header, TxData)
					!= HAL_OK) {
				_Error_Handler("Send_CAN_data",442);//Error_Handler();
			}
		}
	}

	/**********************
	 *
	 * Door Switch Error
	 *0x300
	 **********************/
	if (Error_send.dSw == 1) {
		Error_send.dSw = 0;
		TxData[0] = (Reversed_OP_swich<<4) | CAN_Data.park_stat;
		TxData[1] = (fault_code == ENCODE_FAULT)? 1:0;					//e
		TxData[2] = (CAN_Data.kneel_switch) | CAN_Data.Unkneel_Switch;
		TxData[3] = door_stat;
		TxData[4] = (CAN_Data.Battery_Voltage_int>>8) & 0xFF;
		TxData[5] =  CAN_Data.Battery_Voltage_int & 0xFF;
		TxData[6] = (DI.full_door_open<<4) | DI.Door_ajar;
		TxData[7] =  Gyro_Sensor_Error_Flag;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Sw_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",471);//Error_Handler();
			}
		}
	}


	/**********************
	 * Door Over Current Reverse Open Error Error 301
	 **********************/
	if (Error_send.dRevsOpen == 1) {
		Error_send.dRevsOpen = 0;
		TxData[0] = 0;
		TxData[1] = 1;
		TxData[2] = 2;
		TxData[3] = 3;
		TxData[4] = 4;
		TxData[5] = 5;
		TxData[6] = 6;
		TxData[7] = 7;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_RevsOpen_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",527);//Error_Handler();
			}
		}
	}

	/**********************
	 * Door Over Current Reverse Close Error Error 302
	 **********************/
	if (Error_send.dRevsClose == 1) {
		Error_send.dRevsClose = 0;
		TxData[0] = 0;
		TxData[1] = 1;
		TxData[2] = 2;
		TxData[3] = 3;
		TxData[4] = 4;
		TxData[5] = 5;
		TxData[6] = 6;
		TxData[7] = 7;
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Queue
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_RevsClose_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",527);//Error_Handler();
			}
		}
	}

	/**********************
	 * Door Timeout Error Error 303
	 **********************/
	if (Error_send.dTo == 1) {
		Error_send.dTo = 0;
		TxData[0] = Time_Out_Error;
		TxData[1] = 1;
		TxData[2] = 2;
		TxData[3] = 3;
		TxData[4] = 4;
		TxData[5] = 5;
		TxData[6] = 6;
		TxData[7] = 7;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_To_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",527);//Error_Handler();
			}
		}
	}

	/**********************
	 * Encode_Error Error 305
	 **********************/
	if (Error_send.dEncode == 1) {
		Error_send.dEncode = 0;
		TxData[0] = (adc->Door_Encode_Signal>>8) & 0xff;          			//Encode  High
		TxData[1] = (adc->Door_Encode_Signal) & 0xff;							//Encode low Nibbles
		TxData[2] = 0;
		TxData[3] = 0;
		TxData[4] = 0;
		TxData[5] = 0;
		TxData[6] = 0;
		TxData[7] = 0;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Encode_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",527);//Error_Handler();
			}
		}
	}


	/**********************
	 * Door_Release_Timeout Error 306
	 **********************/
	if (Error_send.releaseTo == 1) {
		Error_send.releaseTo = 0;
		TxData[0] = Time_Out_Error;
		TxData[1] = 0;
		TxData[2] = 0;
		TxData[3] = 0;
		TxData[4] = 0;
		TxData[5] = 0;
		TxData[6] = 0;
		TxData[7] = 0;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Release_To_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",671);//Error_Handler();
			}
		}
	}

	/**********************
	 * Door_Unlatch_Timeout Error 307
	 **********************/
	if (Error_send.unlatchTo == 1) {
		Error_send.unlatchTo = 0;
		TxData[0] = Time_Out_Error;
		TxData[1] = 0;
		TxData[2] = 0;
		TxData[3] = 0;
		TxData[4] = 0;
		TxData[5] = 0;
		TxData[6] = 0;
		TxData[7] = 0;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Queue
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_Unlatch_To_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",671);//Error_Handler();
			}
		}
	}

	/**********************
	 *
	 * Door Enable Error
	 *
	 **********************/
	if (Error_send.dEn == 1) {
		Error_send.dEn = 0;
		TxData[0] = 0;
		TxData[1] = 1;
		TxData[2] = 2;
		TxData[3] = 3;
		TxData[4] = 4;
		TxData[5] = 5;
		TxData[6] = 6;
		TxData[7] = 7;
		//Check for CAN errors
		if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
		} else {
			//Add message to Que
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Dr_En_Er_Header,
					TxData) != HAL_OK) {
				_Error_Handler("Send_CAN_data",500);//Error_Handler();
			}
		}
	}
}

/********************************************************************
 *Check_Door_State: Check the door operate status for the CAN Database
#define	DOORMANUAL  0    //No power
#define	DOORPOWEROPENING 1
#define	DOORPOWERCLOSING 2
#define DOORMANUALOPENING 3
#define DOORMANUALCLOSING 4
#define	DOORFULLOPEN 5
#define	DOORFULLCLOSE 6
#define	DOORREVERSOPEN 7
#define	DOORREVERSCLOSE 8
#define	DOOFAILURE 0xB
**************************************************/
/*****
 * Assigned the door status code for CAN bus.
 */
void Check_Door_State(Door_Contol_Step doorstep, DI_st din)
{
	if (Time_Out_Error != 0)
		door_stat = DOOFAILURE;
	else if (din.full_door_open)
		door_stat = DOORFULLOPEN;
	else if (din.Door_ajar)
		door_stat = DOORFULLCLOSE;
	else if ((doorstep > START_OPEN) && (doorstep < FULLY_OPENED ))
		if (Reverse_OP_flag )										//reverse opening
			door_stat = DOORREVERSOPEN;
		else
			door_stat = DOORPOWEROPENING;
	else if ((doorstep > START_CLOSING) && (doorstep < FULLY_CLOSED ))
		if (Reverse_OP_flag )										//reverse closing
			door_stat = DOORREVERSCLOSE;
		else
			door_stat = DOORPOWERCLOSING;
	else if (doorstep == REVERSEOPEN)
		door_stat = DOORREVERSOPEN;									//reverse opening
	else if (doorstep >= REVERSCLOSE)
		door_stat = DOORREVERSCLOSE;								//reverse closing
	else if (doorspeed_cal <- 0.2)
		door_stat =DOORMANUALOPENING;
	else if (doorspeed_cal > 0.2)
		door_stat =DOORMANUALCLOSING;
	else
		door_stat = DOORMANUAL;

//	return door_stat;
}

/******************************************
 * If CAN data is recieved, check it
 * Where is the protocol of this CAN MSG?
 * fob status?
 * park status
 * ramp state
 * controller_data: controller_op, ramp_state, operate, ramp_moving
 ********************************************/
void Check_CAN_data(void)
{
	uint16_t  int_volt;
	if (can_rx_int == 0)
		return;
	can_rx_int = 0;

	has_can = 1;
	final_can = 0;

	if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData)	!= HAL_OK) {
		_Error_Handler("Check_CAN_data",627);//Error_Handler();
	}

	//STATUS_CAN_GATEWAY_Stat
	if ((RxHeader.Identifier == 0x01) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		if (RxData[0] == 0)																				//sleep command
			Current_Sleep_Mode_Check(10000);

		CAN_Data.park_stat = (RxData[1]==0) ? PARKED: NOPARKED;                  //0:Parked; 1:Not Parked
		CAN_Data.operator_stat = RxData[2];
		CAN_Data.buckle_stat = RxData[3];
		CAN_Data.lock_status = RxData[4]&0x0F;												//swapped as Seth requested.
		CAN_Data.Fob_Op_Cmd = (RxData[4]>>4) &0x0F;
		CAN_Data.fob_data = RxData[5] & 0x0F;                  						//Key or overhead switch
		CAN_Data.fob_nearby = (RxData[5] >>4) &0x0F;

		if (CAN_Data.Fob_Op_Cmd == 0x01)
			FOB_OPEN_Delay_Time.flag = TRUE;

//		if ((CAN_Data.fob_data == 0x01) && (unpressed == 0) )
//		{
//			unpressed = 1;
//			unlock_count++;
//			unlock_timeout_flag = 1;
//			unlock_timeout_count = 0;
//
//			if (unlock_count >= 3) //PRODUCTION IS 3
//			{
//				//start_OP_Door(DI,adc->Door_Encode_Signal);
//				FOB_OPEN_Delay_Time.flag = TRUE;
//				unlock_count = 0;
//				unlock_timeout_flag = 0;
//			}
//		}
//		else if ((CAN_Data.fob_data == 0x00) && (unpressed == 1))
//			unpressed = 0;

	}

	//STATUS_KNEEL_MODULE
	if ((RxHeader.Identifier == 0x20) && (RxHeader.IdType == FDCAN_STANDARD_ID))    //STATUS_Kneel_MOdule
	{
		CAN_Data.kneel_status= RxData[7];//(RxData[1]==0) ? TRUE: FALSE;
		//CAN_Data.kneel_status_unkneeled = (RxData[7]==0) ? TRUE: FALSE;
	}

	//////STATUS_Ramp_SW_STATUS////////
	if ((RxHeader.Identifier == 0x10) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		CAN_Data.ramp_switch = RxData[0];  					//0x00:  Both switches Active;	0x1 = full stow;  0x10=full deploy;  0x11=No Switches active.
		CAN_Data.ramp_motor_state = RxData[2];				//0: No motor Running; 1: Motor Deploying;  x10: Motor Stowing;    0x11 Motor Leads Locked.
		if (CAN_Data.ramp_switch == 0x01)        //Fully Stewed
			CAN_Data.ramp_state = SWITCH_STEW_ACTIVE;         // ramp is stewed
		else if (CAN_Data.ramp_switch == 0x10)        //Fully deployed
			CAN_Data.ramp_state = SWITCH_DEPLOY_ACTIVE;         // ramp is deployed
		else if (CAN_Data.ramp_switch == 0x11)        //Fully deployed
			CAN_Data.ramp_state = NO_SWITCH_ACTIVE;         // ramp is deployed
		else
			CAN_Data.ramp_state = BOTH_SWITCHS_ACTIVE;
	}

	//////STATUS_Ramp_RAMP_Motor_Current////////
	if ((RxHeader.Identifier == 0x11) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		int_volt = RxData[4]*256 +RxData[5];
		CAN_Data.Battery_Voltage_int= int_volt;
		CAN_Data.Battery_Voltage = (float)int_volt * 0.0205+0.5;  					//0x00:  Both switches Active;	0x1 = full stow;  0x10=full deploy;  0x11=No Switches active.
	}

	//////Date and Time from Diag module////////
	if ((RxHeader.Identifier == 0x51) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		CAN_Data.Year= RxData[0];
		CAN_Data.Month= RxData[1];
		CAN_Data.Day= RxData[2];
		CAN_Data.Hour= RxData[3];
		CAN_Data.Minute= RxData[4];
		CAN_Data.Second= RxData[5];
		CheckDateDays(CAN_Data);
	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if ((CAN_Data.park_stat == PARKED) && (sleep_wait_count < (MAX_SLEEP_COUNT-50)))						//If Parted or not going to sleep mode
	{
		DO.LEDWhite = TRUE;
	}
	else
	{
		DO.LEDWhite = FALSE;
	}

	CheckDTCError(CAN_Data);
}

/**
 * Check the fault status from CAN message
 */
void CheckDTCError(CAN_IN CAN_Data)
{
	DO.LEDRed = FALSE;

	 if (CAN_Data.ramp_state == BOTH_SWITCHS_ACTIVE)
	 {
		 SetDTCError(ERROR_RAMP_SW);    //Both ramp end switches active at the same time
		 DO.LEDRed = TRUE;
	 }
    if (CAN_Data.kneel_status == BOTH_SWITCHS_ACTIVE)
    {
    	SetDTCError(ERROR_KNEEL_SW);
    	DO.LEDRed = TRUE;
    }
		///Ramp overcurrent stops while deploying////////
		if ((RxHeader.Identifier == 0x101) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(RAMP_OC_DEPLOY);
		}
		///Ramp overcurrent stops while stowing////////
		if ((RxHeader.Identifier == 0x102) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(RAMP_OC_STOW);
		}
		///Ramp timeout stops while deploying///////
		if ((RxHeader.Identifier == 0x103) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(RAMP_TO_DEPLOY);
		}
		///Ramp timeout stops while stowing////////
		if ((RxHeader.Identifier == 0x104) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(RAMP_TO_STOW);
		}

		///Kneel overcurrent while lowering///////
		if ((RxHeader.Identifier == 0x201) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(KNEEL_OC_LOWERING);
		}
		///Kneel overcurrent while raising///////
		if ((RxHeader.Identifier == 0x202) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(KNEEL_OC_RAISING);
		}
		///Kneel timesout while lowering///////
		if ((RxHeader.Identifier == 0x203) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(KNEEL_TO_LOWERING);
		}
		///Kneel timesout while raising////////
		if ((RxHeader.Identifier == 0x204) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(KNEEL_TO_RAISING);
		}
		///Kneel timesout while raising////////
		if ((RxHeader.Identifier == 0x501) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			 SetDTCError(LOW_BATTERY_VOLTAGE);
		}
		///Door_SW check////////////////////////
		if ((DI.full_door_open == TRUE) &&(DI.Door_ajar == TRUE))
		{
			 SetDTCError(DOOR_SW);
			 DO.LEDRed = TRUE;
		}
		if ((DI.ConvDisabled == TRUE) && (DI.Kneel_disabled == TRUE))
		{
			SetDTCError(CONV_SW);
			DO.LEDRed = TRUE;
		}
		if ((adc->Motor_Cur_Close > MAXMOTORCURRENT) || (adc->Motor_Cur_Open > MAXMOTORCURRENT))   //if over 10A, set zero.
		{
			SetDTCError(REV_OPEN);
		}
}

void Set_CAN_Filters(void) {
	FDCAN_FilterTypeDef sFilterConfig;

	for (int q = 0; q < (int) (sizeof(ID) / sizeof(ID[0])); q++) {
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = q;
		sFilterConfig.FilterType = FDCAN_FILTER_MASK;
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		sFilterConfig.FilterID1 = ID[q];
		sFilterConfig.FilterID2 = 0x7FF;
		if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
			_Error_Handler("Set_CAN_Filters",701);//Error_Handler();
		}
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		_Error_Handler("Set_CAN_Filters",707);//Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		_Error_Handler("Set_CAN_Filters",712);//Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
		_Error_Handler("Set_CAN_Filters",716);//Error_Handler();
	}

	if (HAL_GPIO_ReadPin(ERR_1_GPIO_Port, ERR_1_Pin) == 0) {
		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_RESET);
		HAL_Delay(2);
		HAL_GPIO_WritePin(STB_1_GPIO_Port, STB_1_Pin, GPIO_PIN_SET);
	}
}


void Set_CAN_Tx_Messages(void) {
	SW_Stat_Header.Identifier = SW_Stat;
	SW_Stat_Header.IdType = FDCAN_STANDARD_ID;
	SW_Stat_Header.TxFrameType = FDCAN_DATA_FRAME;
	SW_Stat_Header.DataLength = FDCAN_DLC_BYTES_8;
	SW_Stat_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	SW_Stat_Header.BitRateSwitch = FDCAN_BRS_OFF;
	SW_Stat_Header.FDFormat = FDCAN_CLASSIC_CAN;
	SW_Stat_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	SW_Stat_Header.MessageMarker = 0;

	Dr_Stat_Header.Identifier = Dr_Stat;
	Dr_Stat_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Stat_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Stat_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Stat_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Stat_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Stat_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Stat_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Stat_Header.MessageMarker = 0;

	Dr_Curr_Header.Identifier = Dr_Curr;
	Dr_Curr_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Curr_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Curr_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Curr_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Curr_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Curr_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Curr_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Curr_Header.MessageMarker = 0;

	Dr_Sw_Er_Header.Identifier = Dr_Sw_Er;
	Dr_Sw_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Sw_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Sw_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Sw_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Sw_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Sw_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Sw_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Sw_Er_Header.MessageMarker = 0;

	Dr_En_Er_Header.Identifier = Dr_En_Er;
	Dr_En_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_En_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_En_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_En_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_En_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_En_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_En_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_En_Er_Header.MessageMarker = 0;

	Dr_To_Er_Header.Identifier = Dr_To_Er;
	Dr_To_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_To_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_To_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_To_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_To_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_To_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_To_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_To_Er_Header.MessageMarker = 0;

	Dr_RevsOpen_Er_Header.Identifier = Dr_REV_OPEN_Er;
	Dr_RevsOpen_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_RevsOpen_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_RevsOpen_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_RevsOpen_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_RevsOpen_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_RevsOpen_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_RevsOpen_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_RevsOpen_Er_Header.MessageMarker = 0;


	Dr_RevsClose_Er_Header.Identifier = Dr_REV_CLOSE_Er;
	Dr_RevsClose_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_RevsClose_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_RevsClose_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_RevsClose_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_RevsClose_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_RevsClose_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_RevsClose_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_RevsClose_Er_Header.MessageMarker = 0;

	Dr_Release_To_Er_Header.Identifier = Dr_Release_To_Er;
	Dr_Release_To_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Release_To_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Release_To_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Release_To_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Release_To_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Release_To_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Release_To_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Release_To_Er_Header.MessageMarker = 0;

	Dr_Unlatch_To_Er_Header.Identifier = Dr_Unlatch_To_Er;
	Dr_Unlatch_To_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Unlatch_To_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Unlatch_To_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Unlatch_To_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Unlatch_To_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Unlatch_To_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Unlatch_To_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Unlatch_To_Er_Header.MessageMarker = 0;

	Dr_Encode_Er_Header.Identifier = Dr_Encode_To_Er;
	Dr_Encode_Er_Header.IdType = FDCAN_STANDARD_ID;
	Dr_Encode_Er_Header.TxFrameType = FDCAN_DATA_FRAME;
	Dr_Encode_Er_Header.DataLength = FDCAN_DLC_BYTES_8;
	Dr_Encode_Er_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Dr_Encode_Er_Header.BitRateSwitch = FDCAN_BRS_OFF;
	Dr_Encode_Er_Header.FDFormat = FDCAN_CLASSIC_CAN;
	Dr_Encode_Er_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Dr_Encode_Er_Header.MessageMarker = 0;
}

extern char tim6_flag;
void Timer_Check(void)
{
	sleep_wait_count++;

	if (Door_Count.handle_count > 10) {
		Door_flag.handle_deb_flag = 0;
		Door_Count.handle_count = 0;
		Door_flag.handle_deb_done = TRUE;
	}

	if (Door_Count.Norma_Open_Wait_count > RELEASE_OFF_TIMER) {
		Door_flag.Normal_Open_Wait_flag = 0;
		Door_Count.Norma_Open_Wait_count = 0;
		Door_flag.Normal_Open_Wait_Done = TRUE;
//		DO.Release = FALSE;
	}

	if (Door_Count.bump_count > 10) {
		Door_flag.bump_deb_flag = 0;
		Door_Count.bump_count = 0;
		//Door_flag.bump_deb_done = TRUE;
		CheckBumpStripEvent();
	}


	if (Door_Count.Latch_Off_Count > LATCH_OFF_TIMER) {		 //wait time to turn off motor after latched.
		Door_flag.Latch_Hold_On_flag = FALSE;
		Door_Count.Latch_Off_Count = 0;
		Door_flag.Latch_Hold_On_done = TRUE;
	}

	if (Door_Count.Magna_Cinch_Wait_Count >MAGNA_CINCH_WAIT_TIMER) {		 //wait time to turn off motor after latched.
		Door_flag.Magna_Cinch_Wait_flag = FALSE;
		Door_Count.Magna_Cinch_Wait_Count = 0;
		Door_flag.Magna_Cinch_Wait_done = TRUE;
	}

	if (Door_Count.reverse_open_delay_count > REVERSE_OPEN_DELAY_TIMER) {		 //wait time to turn off motor after latched.
		Door_flag.reverse_open_delay_flag = FALSE;
		Door_Count.reverse_open_delay_count = 0;
		Door_flag.reverse_open_delay_done = TRUE;
	}
	if (Door_Count.reverse_close_delay_count > REVERSE_CLOSE_DELAY_TIMER) {		 //wait time to turn off motor after latched.
		Door_flag.reverse_close_delay_flag = FALSE;
		Door_Count.reverse_close_delay_count = 0;
		Door_flag.reverse_close_delay_done = TRUE;
	}

	if (Door_Count.Autocycle_delay_count > AutoCycleWaitTime) {		 //Autocyle wait timer 45s
		Door_flag.Autocycle_delay_flag = FALSE;
		Door_Count.Autocycle_delay_count = 0;
		Door_flag.Autocycle_delay_done = TRUE;
	}

	if (Door_Count.Clutch_to_Motor_On_count > 20) {		 //200ms
		Door_flag.Clutch_to_Motor_On_flag = FALSE;
		Door_Count.Clutch_to_Motor_On_count = 0;
		Door_flag.Clutch_to_Motor_On_done = TRUE;
	}


	if (Door_Count.general_delay_count > 10) {		 //general delay wait timer 0.1s
		Door_flag.general_delay_flag = FALSE;
		Door_Count.general_delay_count = 0;
		Door_flag.general_delay_done = TRUE;
	}

	if (Door_Count.ajar_count > 2) {
		Door_flag.ajar_deb_flag = 0;
		Door_Count.ajar_count = 0;
		Door_flag.ajar_deb_done = TRUE;

	}

	if (Door_Count.op_count > 10) {
	}

	if (error == 1) {     //Ken: where error come from?
		error = 0;
	}

}

/**
 *   Moved from the main loop, increase the counts
 *   TIM6, period around 32ms
 */
void TIM6CountEvent(Door_flag_st* door_flg,Door_Count_st* door_count)
{

	if ( m_doorstep == DOOR_MONITOR) //only run the backup_RAM save in idle.
	{
		if (Backup_RAM_count==100)
		{
			if ((pre_eepromcount_t.close_count != eepromcount_t.close_count)    || (pre_eepromcount_t.hndl_count!=eepromcount_t.hndl_count) || (pre_eepromcount_t.maint_saved!=eepromcount_t.maint_saved)
					|| (pre_eepromcount_t.open_count!=eepromcount_t.open_count) || (pre_eepromcount_t.overcurrent_count!=eepromcount_t.overcurrent_count)
					|| (pre_eepromcount_t.learned_enc!=eepromcount_t.learned_enc) || (pre_eepromcount_t.reverse_count!=eepromcount_t.reverse_count)
					|| (pre_eepromcount_t.full_opened_encode!=eepromcount_t.full_opened_encode) || (pre_eepromcount_t.full_closed_encode!=eepromcount_t.full_closed_encode)
					|| (pre_eepromcount_t.switch_count!=eepromcount_t.switch_count)|| (pre_eepromcount_t.pre_warning_days!=eepromcount_t.pre_warning_days))
			{
				ReadWritEEPROMVariables(&eepromcount_t, WRITE); 			//write to the backup memory
				pre_eepromcount_t = eepromcount_t;
			}

		}
		Backup_RAM_count++;
	}
	else
	{
		Backup_RAM_count = 0;
	}

	if (door_flg->ajar_deb_flag == 1)
		door_count->ajar_count++;
	else
		door_count->ajar_count = 0;

	if (door_flg->op_deb_flag == 1)
	{
//		printf("Door_flag->op_deb_flag = 1,count = %d;\r\n",door_count->op_count);
		door_count->op_count++;
	}
	else
		door_count->op_count = 0;

	if (door_flg->handle_deb_flag == 1)
		door_count->handle_count++;
	else
		door_count->handle_count = 0;

	if (door_flg->bump_deb_flag == 1)
		door_count->bump_count++;
	else
		door_count->bump_count = 0;

	if (door_flg->latch_deb_flag == 1)
		door_count->latch_deb_count++;
	else
		door_count->latch_deb_count = 0;

//	if (door_flg->reset_error_flag == 1)
//		door_count->reset_error_count++;
//	else
//		door_count->reset_error_count = 0;

	if (door_flg->reverse_open_delay_flag ==1)
		door_count->reverse_open_delay_count++;
	else
		door_count->reverse_open_delay_count = 0;

	if (door_flg->reverse_close_delay_flag ==1)
		door_count->reverse_close_delay_count++;
	else
		door_count->reverse_close_delay_count = 0;

	if (door_flg->Autocycle_delay_flag == 1)
		door_count->Autocycle_delay_count++;
	else
		door_count->Autocycle_delay_count = 0;

	if (door_flg->Magna_Cinch_Wait_flag == 1)
		door_count->Magna_Cinch_Wait_Count++;
	else
		door_count->Magna_Cinch_Wait_Count = 0;

	if (door_flg->Latch_Hold_On_flag == 1)
		door_count->Latch_Off_Count++;
	else
		door_count->Latch_Off_Count = 0;

	if (door_flg->general_delay_flag == 1)
		door_count->general_delay_count++;
	else
		door_count->general_delay_count = 0;

	if (debounce_flag == 1) {
		debounce_count++;
		if (debounce_count >= 5) {
			debounce_flag = 0;
			debounce_count = 0;
			debounce_done = 1;
		}
	} else
		debounce_count = 0;


	if (sleep_flag == TRUE)
		sleep_count++;
	else
		sleep_count = 0;

	if (status_update_flag == 1)
		status_update_count++;
	else
		status_update_count = 0;

	if (door_flg->Clutch_to_Motor_On_flag == 1)
		door_count->Clutch_to_Motor_On_count++;
	else
		door_count->Clutch_to_Motor_On_count = 0;

	if (status_update_count >= 25) {
		status_update_count = 0;
		stat_countup++;
		if (stat_countup >= 10) {
			status_update_flag = 0;
			stat_countup = 0;
		}
	}

//	if ((maint_reset_flag == 1) && (operate_sw == 0) && (maint_set == 1))
//		maint_reset_count++;
//	else
//	{
//		maint_reset_count = 0;
//		maint_reset_flag = 0;
//	}

	if (door_flg->Normal_Open_Wait_flag == 1)
		door_count->Norma_Open_Wait_count++;
	else
		door_count->Norma_Open_Wait_count = 0;

}


/**
 *   Moved from the main loop, increase the counts
 */
void  ErrorMsgSend(ERROR_FLAG_COUNT_SEND_ST* error_flag, ERROR_FLAG_COUNT_SEND_ST* error_send,ERROR_FLAG_COUNT_SEND_ST* error_count)
{
	//***********Door Switch Error************************
	if ((error_flag->dSw == 1) && (Error_count.dSw <= 2)) {
		Error_send.dSw = 1;
		Error_count.dSw++;
	} else {
		error_flag->dSw = 0;
		Error_count.dSw = 0;
	}
	//***********Door Enabled Error************************
	if ((error_flag->dEn == 1) && (Error_count.dEn <= 2)) {
		Error_send.dEn = 1;
		Error_count.dEn++;
	} else {
		error_flag->dEn = 0;
		Error_count.dEn = 0;
	}
	//***********Door Timeout Error************************
//	error_flag->dTo  = (Time_Out_Error == 0) ? 0:1;
	if ((error_flag->dTo == 1) && (Error_count.dTo <=2)) {
		Error_send.dTo = 1;
		Error_count.dTo++;
	} else {
		error_flag->dTo = 0;
		Error_count.dTo = 0;
		Time_Out_Error = 0;
	}
	//***********Release Timeout Error************************ // relese timeout
	if ((error_flag->releaseTo == 1) && (Error_count.releaseTo <= 2)) {
		Error_send.releaseTo = 1;
		Error_count.releaseTo++;
	} else {
		error_flag->releaseTo = 0;
		Error_count.releaseTo = 0;
	}
	//***********Unlatch Timeout Error************************ // unlatch timeout
	if ((error_flag->unlatchTo == 1) && (Error_count.unlatchTo <= 2)) {
		Error_send.unlatchTo = 1;
		Error_count.unlatchTo++;
	} else {
		error_flag->unlatchTo = 0;
		Error_count.unlatchTo = 0;
	}
	//***********Encode Error************************ // Encode Error
	if ((error_flag->dEncode == 1) && (Error_count.dEncode <= 2)) {
		Error_send.dEncode = 1;
		Error_count.dEncode++;
	} else {
		error_flag->dEncode = 0;
		Error_count.dEncode = 0;
	}

}


/**
 * Save the EEPROM variables, write == TRUE is write to, FALSE is read
 * var: the value to be write or read.
 * index: the index 32bits in EEPROM,
 */
void ReadWritEEPROMVariables(EEPROMCOUNT_t *var, char iswrite)
{
	if (iswrite == TRUE)
	{
		var->Saved = TRUE;
		*(__IO uint32_t*) (BKPSRAM_BASE) = var->maint_saved;
		*(__IO uint32_t*) (BKPSRAM_BASE+4) = var->Saved;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*2) = var->open_count;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*3) = var->close_count;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*4) = var->hndl_count;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*5) = var->switch_count;

		*(__IO uint32_t*) (BKPSRAM_BASE+4*6) = var->reverse_count;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*7) = var->overcurrent_count;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*8) = var->learned_enc;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*9) = Full_Opened_Encode;//var->full_opened_encode;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*10) = Full_Closed_Encode;//var->full_closed_encode;
		*(__IO uint32_t*) (BKPSRAM_BASE+4*11) = var->pre_warning_days;//var->full_closed_encode;
		PRINTF("ReadWritEEPROMVariables save.");
	}
	else
	{
		var->Saved = *(__IO uint32_t*) (BKPSRAM_BASE+4) ;
		if (var->Saved != TRUE)
		{
			//var->Saved = TRUE;
			var->open_count =0;
			var->close_count =0;
			var->hndl_count =0;
			var->switch_count =0;
			var->reverse_count =0;
			var->overcurrent_count =0;
			var->full_opened_encode = 20000;//*(__IO uint32_t*) (BKPSRAM_BASE+4*9);
			var->full_closed_encode = 50000;//*(__IO uint32_t*) (BKPSRAM_BASE+4*10);
			PRINTF("ReadWritEEPROMVariables Initial.");
		}
		else
		{
			var->maint_saved = *(__IO uint32_t*) (BKPSRAM_BASE);
			var->open_count = *(__IO uint32_t*) (BKPSRAM_BASE+4*2);
			var->close_count= *(__IO uint32_t*) (BKPSRAM_BASE+4*3);
			var->hndl_count= *(__IO uint32_t*) (BKPSRAM_BASE+4*4);
			var->switch_count = *(__IO uint32_t*) (BKPSRAM_BASE+4*5);
			var->reverse_count= *(__IO uint32_t*) (BKPSRAM_BASE+4*6);
			var->overcurrent_count = *(__IO uint32_t*) (BKPSRAM_BASE+4*7);
			var->learned_enc= *(__IO uint32_t*) (BKPSRAM_BASE+4*8);
			var->full_opened_encode = *(__IO uint32_t*) (BKPSRAM_BASE+4*9);
			var->full_closed_encode = *(__IO uint32_t*) (BKPSRAM_BASE+4*10);
			var->pre_warning_days = *(__IO uint32_t*) (BKPSRAM_BASE+4*11);
			PRINTF("ReadWritEEPROMVariables Read.");
		}
	}
	//return 0;

//	following code is used to read and write by block
//	// write to backup RAM
//	memcpy(*(__IO uint8_t *)D3_BKPSRAM_BASE,(uint8_t *)&w_data, sizeof(w_data));
//
//	//read from backup RAM
//	memcpy(*(__IO uint8_t *)D3_BKPSRAM_BASE,(uint8_t *)&r_data, sizeof(r_data));
}


/**
 * Reset the debounce count,
 * input setflag: if true set debounce flag to start a debounce count, if false, clear the debounce.
 */
void Resetdebounce(char setflag)
{
//	Door_flag.op_deb_flag = 1;
//	Door_Count.op_count = 0;
//	braun_sw_pressed = TRUE;
	debounce_flag = setflag;
	debounce_count = 0;
	debounce_done = FALSE;
}


void Initial_Before_Loop(void)
{
	CAN_Data.park_stat = PARKED;
	//printf(" First ECDR: %x %x DS: %x\r\n",prev_enc_h,prev_enc_l,door_stat);
	sleep_flag = TRUE;
	can_sleep_flag = TRUE;
	sleep_count = 0;
	HAL_Delay(200);
	closed_count_up = 4;
	startup = 1;
	Enable_PI_Moniter = TRUE; //FALSE;  Should be false for product
	if (door_full_closed == 1)
	{
		prev_door = DOORFULLCLOSE;
		status_update_flag = TRUE;
		stat_countup = 0;
		learned_enc = encoder_data;
	}
	else
	{
		if (eepromcount_t.learned_enc <= 0)
			pre_eepromcount_t.learned_enc = 800;
		else
			learned_enc = learned_enc;

		prev_door = DOORFULLOPEN;
		status_update_flag = 1;
		stat_countup = 0;
	}

	//printf("Init ENC: %d Model: %x\r",learned_enc,model_year);
	prev_enc = encoder_data;
	pre_eepromcount_t.maint_saved = eepromcount_t.maint_saved;

	UpdateAllInputs();

	Config_Value.ANGLE_CLINATION_MAX = 13;
	Config_Value.ANGLE_CLINATION_MIN = -13;

}


/**
 * FaultIndicator:  flash the indicator based on the fault, called every 10ms
 * if not fault, flash every 1s, else fast flash fault numbers then one slow flash.
 */
void FaultIndicator(char fault, int faultcount, int* flashnumber)
{
	if ((faultcount % 25) != 0 )			//toggle the LED every 500ms
		return;
	fault =0;
	if (fault == 0)
	{
		if ((faultcount % 50) != 0 )
			HAL_GPIO_TogglePin(Fault_LED_GPIO_Port, Fault_LED_Pin);
	}
	else
	{
		if ((*flashnumber > 0 ) && (*flashnumber < (2 * fault)))
		{								//If there is a fault, fast flash a certain time and then one slow flash.
			HAL_GPIO_TogglePin(Fault_LED_GPIO_Port, Fault_LED_Pin);
			*flashnumber--;
		}

		else
		{
			*flashnumber--;
			HAL_GPIO_WritePin(Fault_LED_GPIO_Port, Fault_LED_Pin, GPIO_PIN_SET);
			*flashnumber = 2 * fault;
		}
	}
}

/**
 * Smoothly change the motor speed to avoid the bump when start the reverse open
 */
void ChangeMotorSpeed(int targetspeed, int* currentspeed)
{
	int pwmspeed;
	if ((targetspeed>100) || (targetspeed <-100))
		return;

		if (ChangeMotorSpeedFlag==TRUE)
		{
			ChangeMotorSpeedFlag = FALSE;
			if (*currentspeed < targetspeed)
				(*currentspeed)++;
			else if (*currentspeed > targetspeed)
				(*currentspeed)--;

			pwmspeed = *currentspeed;
			printf("pwmspeed= %d", pwmspeed);
		}

	pwmspeed = *currentspeed;

	if ((pwmspeed > 0) && (pwmspeed <= 100))				//Positive is opening
		Start_Open(pwmspeed);
	else if ((pwmspeed < 0) && (pwmspeed >=-100))       //negative is closing
		Start_Close(-pwmspeed);
}

/**
 * Read the memory count from the flash memory
 * FLASH_USER_START_ADDR = 0x08040000UL
 */
EEPROMCOUNT_t  ReadFromFlashMemory()
{
	EEPROMCOUNT_t  eepromcount;
	Address = FLASH_USER_START_ADDR;
	eepromcount.maint_saved= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.Saved= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.open_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.close_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.hndl_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.switch_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.reverse_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.overcurrent_count= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.learned_enc= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.full_opened_encode= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.full_closed_encode= *(uint32_t*)Address;
	Address=Address+4;
	eepromcount.pre_warning_days= *(uint32_t*)Address;
	Address=Address+4;
    return eepromcount;
}

char SaveToFlashMemory(EEPROMCOUNT_t eepromcount)
{
	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();
	  /* Erase the user Flash area (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  /* Get the 1st sector to erase */
	  FirstSector = 2;//Section 2:0x08040000 GetSector(FLASH_USER_START_ADDR);
	  /* Get the number of sector to erase from 1st sector*/
	  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.Banks         = FLASH_BANK_1;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_4;
	  EraseInitStruct.Sector        = FirstSector;
	  EraseInitStruct.NbSectors     = NbOfSectors;

	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
		  HAL_FLASH_GetError();
	    /*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SECTORError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
	    */
	  }

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	Address = FLASH_USER_START_ADDR;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, eepromcount.maint_saved);
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.Saved));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.open_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.close_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.hndl_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.switch_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.reverse_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.overcurrent_count));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.learned_enc));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.full_opened_encode));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.full_closed_encode));
	Address = Address + 32;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, ((uint32_t)eepromcount.pre_warning_days));
	Address = Address + 32;
	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();
	  return TRUE;
}

/*
 * Check if Need reverse closing.  When Open Motor current is high and encode is not increase.
 * Called every 32ms.
 * Return the encoder as pre-encode reading
 */
int CheckReverseOperation(int pre_encoder,ADC_st* motorcurrent)
{
	if ((m_doorstep>START_OPEN) && (m_doorstep<FULLY_OPENED))
	{
		if ( (motorcurrent->Motor_Cur_Open) > MAXMOTORCURRENT)															// If Open current is too high
		{
			if ((motorcurrent->Door_Encode_Signal > (Full_Opened_Encode + 3000)) && (adc->Door_Encode_Signal < (Full_Closed_Encode - 3000)))          //add 300 and minus 300 to avoid the jump current at the end of open/close
			{
				//motorcurrent->Door_Encode_Signal > pre_encoder;									//if encode is not increasing
				 m_doorstep = DOOROPENSTUCKED;
				 Error_flag.dRevsClose = 1;
			}
			 //m_doorstep = DOOROPENSTUCKED;
		}
	}
	if ((m_doorstep>START_CLOSING) && (m_doorstep<ALMOST_CLOSED))
	{
		if ( (motorcurrent->Motor_Cur_Close) > MAXMOTORCURRENT)															// If Close current is too high
		{
			if ((motorcurrent->Door_Encode_Signal > (Full_Opened_Encode + 3000)) && (adc->Door_Encode_Signal < (Full_Closed_Encode - 3000)))          //add 300 and minus 300 to avoid the jump current at the end of open/close
			{
				//motorcurrent->Door_Encode_Signal < pre_encoder;									//if encode is not increasing
				m_doorstep = REVERSEOPEN;
				 Error_flag.dRevsOpen = 1;
			}
			//m_doorstep = REVERSEOPEN;
		}
	}
	return adc->Door_Encode_Signal;
}


/*
 *    Flash the LED
 */
char Flash_LED( LED_t   *led,  int timecount)
{
	 char led_status;
	if (led->FlashFreq == FAST_FLASH)
	{
		led->ON_Time = 25;                 // on 500ms
		led->OFF_Time =25;				  //off 500ms
	}
	if (led->FlashFreq == SLOW_FLASH)
	{
		led->ON_Time = 100;                 //On 1s
		led->OFF_Time =100;				//Off 1s
	}
	if (led->FlashFreq == SOLID_OFF)
	{
		led->ON_Time = 0;                 //On 1s
		led->OFF_Time =100;				//Off 1s
	}
	if (led->FlashFreq == SOLID_ON)
	{
		led->ON_Time = 100;                 //On 1s
		led->OFF_Time =0;				//Off 1s
	}
	if (timecount%(led->ON_Time + led->OFF_Time) <= led->ON_Time )
		led_status = TRUE;
	else
		led_status = FALSE;

	if (led->FlashFreq == FAST_FLASH)
	{
		if (led->flashedcount >= led->FlashCount)
			led_status = FALSE;
	}
	if (timecount%(led->ON_Time+ led->OFF_Time) == 0 )
		led->flashedcount++;

	if (led->flashedcount >= ((led->FlashCount)+3))
		led->flashedcount = 0;

//	printf("led_status= %d;\r\n",led_status);
	return led_status;
}
/*
 * Flash LED Based on the status code. Called every 10ms.
 * Input: error status code
 * Return: LED flash count
 */
void LEDFunction(int code)
{
	RedLED.FlashFreq = SOLID_OFF;
	WhiteLED.FlashFreq = SOLID_OFF;
	YellowLED.FlashFreq = SOLID_OFF;
//	if (code == LOW_BATTERY_VOLTAGE)                  //
//	{
//		RedLED.FlashFreq = SOLID_ON;
//	}
//	if ((code == RAMP_OC_DEPLOY) ||(code == RAMP_OC_STOW) ||(code == KNEEL_OC_LOWERING) ||(code == KNEEL_OC_RAISING) )
//	{
//		YellowLED.FlashFreq = FAST_FLASH;
//		YellowLED.FlashCount = 5 ;
//	}
//	if ((code == RAMP_TO_DEPLOY) ||(code == RAMP_TO_STOW) ||(code == KNEEL_TO_LOWERING) ||(code == KNEEL_TO_RAISING) ||(code == DOOR_TO))
//	{
//		YellowLED.FlashFreq = FAST_FLASH;
//		YellowLED.FlashCount = 4 ;
//	}
//	if (code == SWITCH_STUCK)                  //
//	{
//		RedLED.FlashFreq = FAST_FLASH;
//		RedLED.FlashCount =0;                 //always on
//	}
//	if (code == CAN_COMM_ERROR)
//	{
//		RedLED.FlashFreq = SLOW_FLASH;
//		RedLED.FlashCount = 0 ;
//	}
//	if (code == DOOR_ENCODE_ERROR)
//	{
//		RedLED.FlashFreq = FAST_FLASH;
//		RedLED.FlashCount = 3;
//	}
//	if (code == KNEEL_SW)
//	{
//		RedLED.FlashFreq = FAST_FLASH;
//		RedLED.FlashCount = 4 ;
//	}
//	if (code == DOOR_SW)
//	{
//		RedLED.FlashFreq = FAST_FLASH;
//		RedLED.FlashCount = 5 ;
//	}CAN_Data.ramp_state = BOTH_SWITCHS_ACTIVE;
	if ((DTC_Error == DOOR_SW) || (DTC_Error == ERROR_RAMP_SW)  || (DTC_Error == ERROR_KNEEL_SW) ||  (Gyro_Sensor_Error_Flag == TRUE))           //
	{
		RedLED.FlashFreq = SOLID_ON;
	}
//	 DO.LEDRed = Flash_LED( &RedLED, LEDTimeCount);
//	 DO.LEDYellow = Flash_LED( &YellowLED, LEDTimeCount);
//	 DO.LEDWhite = Flash_LED( &WhiteLED, LEDTimeCount);
	 int32_t  total_currentdays = convertDateToDays(CAN_Data);

	 if ((eepromcount_t.open_count > 300) || ((total_currentdays - eepromcount_t.pre_warning_days) > 183))              // over 300 cycle or 6 months.
	 {
		 DO.LEDYellow = TRUE;
	 }
	 else
		 DO.LEDYellow = FALSE;

	 LEDTimeCount++;
}

/**
 *  Function: DoorSpeedPIDControl
 *  Use PI to control the door speed.
 */
float Integral;
float Proportional;
//float KI, KP;
//
//float DoorSpeedPIDControl(float target_speed, float current_speed, PID_st *pid, float currentpower)
//{
//	 float u;
////	 KI =0.05;
////	 KP= 0.0002;
//	 pid->Proportional = target_speed - current_speed;
//	 pid->Integral = pid->Integral + pid->Proportional;
//	 u = KP * pid->Proportional + KI * pid->Integral;//+ currentpower;
//	 if (u<0)
//		 u=0;
//	 else if (u>100)
//		 u=100;
//
//     printf("u=%f; Proportional= %f; Integral=%f; \r\n", u,pid->Proportional ,pid->Integral);
//	 return u;
//}

float  DoorSpeedPControl(float target_speed, float current_speed, PID_st *pid, float currentpower)
{
	 float u;
//	 KI =0.05;
//	 KP= 0.0002;

	 if ((target_speed - current_speed)>0)
	 {
		 u = currentpower + 0.1;
	 }
	 else
		 u = currentpower -  0.1;

	 if (u<0)
		 u=0;
	 else if (u>100)
		 u=100;

     //printf("u=%f; Proportional= %f; Integral=%f; \r\n", u,pid->Proportional ,pid->Integral);
	 return u;
}

/*
 * CheckBraun_Op_Switch: Check the braun switch and DisableConv status. Called every 10ms
 * return 1; Set switch stuck error.   return 2: clear errors;
 */
char  CheckBraun_Op_Switch(DI_st DIswitch)
{
	 if (DIswitch.Braun_Op_Switch == TRUE)
		 Count_OP_Stuck ++;
	 else
		 Count_OP_Stuck = 0;

	 if (Count_OP_Stuck > 12000)            //over120 seconds
	 {
		 SetDTCError(SWITCH_STUCK);
		 Count_OP_Stuck = 0;
		 return 1;
	 }
	 if ((Count_OP_Stuck == 500)  || (DisConv_toggle_count >=3) )          //over 5 second or    // toggled over 3 times in 4 seconds
	 {
		 eepromcount_t.open_count =0;                        //Reset all error of
		 eepromcount_t.pre_warning_days = convertDateToDays(CAN_Data);
		 DisConv_toggle_count = 0;
		 return 2;
	 }

	 return 0;
}


void CheckDateDays(CAN_IN can_data)
{

}
/*
 * Function's limits (included):
     * bottom: 01/01/2000
     * top: 31/12/2100

 * Input: date from CAN.
 * Output: (int) number of days from 01/01/2000 to that date.
 */
int convertDateToDays(CAN_IN can_data)
{
	int totalDays=0;
	int numLeap = 0;
	int monthsAddFromYearStart[] = {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
	int i;
	int year = (int)can_data.Year;
	int month = (int)can_data.Month;
	int day =(int)can_data.Day;
    // First, calculate the number of leap year since year one (not including date's year).
    for(i = 1; i < year; i++)
        if((i % 4 == 0 && i % 100 != 0) || (i % 4 == 0 && i % 400 == 0))
            numLeap++;

    // If it is a leap year, as of March there has been an extra day.
    if((year % 4 == 0 && year % 100 != 0) || (year % 4 == 0 && year % 400 == 0))
        for(i = 2; i < 12; i++)
            monthsAddFromYearStart[i]++;

    // (Year - 1) * 356 + a day per leap year + days totaling the previous months + days of this month
    totalDays = (year ) * 365 + numLeap + monthsAddFromYearStart[month ] + day;

    return totalDays;
}


void SetDTCError(int errorindex)
{
	DTC_Error= errorindex;
}
/*
 * Let's do this:
1) If ramp is enabled
2) Start timer once door reaches full open (120seconds)
3) Activate lights
4) After timer reaches 120s, turn lights off
5) If door reaches full closed before 120s is reached, turn lights off

That should be close to what other people are wanting. After we show it off to the larger group we may have to tweak how that functions

Evan Rose
Let's do this:  1) If ramp is enabled  2) Start timer once door reaches full open (120seconds)  3) Activate lights  4) After timer reaches 120s, turn lights off  5) If door reaches full closed before 120s is reached, turn lights off
Should add two thing to this now that I'm thinking about all the possibilities:
6) reset timer whenever lights turn off
7) add a secondary condition to turning the lights on which needs to see a door closed signal before allowing the lights to turn on a second time
 */
void RampLight_Operation(void)
{
	if (DI.ConvDisabled)
		return;

	if ((Pre_DI.full_door_open == FALSE) && (DI.full_door_open==TRUE))
	{
		Ramp_Light_On_Time.flag = TRUE;
		DO.Ramp_Light = TRUE;
	}
	if ((PreCAN_Data.ramp_state != SWITCH_STEW_ACTIVE) && (CAN_Data.ramp_state == SWITCH_STEW_ACTIVE ))
	{
		Ramp_Light_On_Time.flag = TRUE;
		DO.Ramp_Light = TRUE;
	}

	if ( (Ramp_Light_On_Time.count > RAMP_LIGHT_ON_TIME) || (sleep_wait_count > (MAX_SLEEP_COUNT-50)) )
	{
		DO.Ramp_Light = FALSE;
		Ramp_Light_On_Time.flag = FALSE;
	}

	Pre_DI.full_door_open =DI.full_door_open;
	PreCAN_Data.ramp_state = CAN_Data.ramp_state;
}

#endif /* SRC_FUNCTIONS_H_ */
