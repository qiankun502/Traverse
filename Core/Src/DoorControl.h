/*
 * DoorControl.h
 *
 *  Created on: Aug 11, 2024
 *      Author:
 *      		Ken Qian
 */
#ifndef SRC_DOORCONTROL_H_
#define SRC_DOORCONTROL_H_
#include "variables.h"
#include "functions.h"

//float DoorSpeedPIDControl(float target_speed, float current_speed, PID_st *pid,float currentpower);
/*
 * Check the door operation condition.
 * In parking and inclination is acceptable and Ramp is fully stewed
 */
DOOR_OP_CONDITION_st CheckDoorOPCondition(void)
{
	if (CAN_Data.park_stat == NOPARKED)  											 //1: Not parking; 0: parked
		return NOTPARKING;
	if (CAN_Data.ramp_state != SWITCH_STEW_ACTIVE)   					//Ramp is externed.
		return RAMPOUT;
//	if ((SensorReading.LSM6AngleZ >MAXINCLINATION) &&(SensorReading.LSM6AngleZ < MININCLINATION))  //need to change it to z angle
//		return CLINATIONOVERRANGE;
	else
		return OP_ALLOW;
}


char GyoSensorInRange(LSM6SENSOR_READING sensorreading)
{

	if ((sensorreading.NoseAngle <MAXINCLINATION) &&(sensorreading.NoseAngle > MININCLINATION))
	{
//		if ((sensorreading.LSM6DSOtemperature < MAXTEMPERATURE) && (sensorreading.LSM6DSOtemperature > MINTEMPERATURE))
			return TRUE;
	}
	else
			return FALSE;
}

/***
 * Check if there is fault on the board. Only the last fault is returned if multiple faults are existed.
 */
char CheckFaultCode(DI_st din,ADC_st* adc, ERROR_TRIG_ST* error_trig)
{
	char fault = 0;

	//Check encode error

	if ((adc->Door_Encode_Signal > MAX_ENCODE_ERROR) ||	 (adc->Door_Encode_Signal < MIN_ENCODE_ERROR) )		  //if encode reading is too low or too high, treat it as in fault.
	{
		if (error_trig->encode_error_trig == FALSE )
		{
				fault = ENCODE_FAULT;
				error_trig->encode_error_trig = TRUE;
				Error_flag.dEncode = 1;
		}
	}
	else
	{
		error_trig->encode_error_trig = FALSE;
	}

	if (din.ERR_CAN == TRUE)
		fault = CAN_FAULT;

    //Check door
	if ((DI.Door_ajar == TRUE) && (DI.full_door_open == TRUE))								 //Door Full open and full closed switch can't be on at the same time.
	{
		fault = DOORSW_FAULT;
		if ((adc->Door_Encode_Signal < Full_Opened_Encode + 8000) && (error_trig->door_full_open_error_trig == FALSE))     // the door close to the full open side
		{
			error_trig->door_full_open_error_trig = TRUE;
			Error_flag.dSw = 1;
		}
		if ((adc->Door_Encode_Signal > Full_Closed_Encode - 8000) && (error_trig->door_ajar_error_trig == FALSE))     // the door close to the full open side
		{
			error_trig->door_ajar_error_trig = TRUE;
			Error_flag.dSw = 1;
		}
	}
	else
	{
		error_trig->door_full_open_error_trig = FALSE;
		error_trig->door_ajar_error_trig = FALSE;
	}
//	if ((fault != 0) && (fault != CAN_FAULT) )
//		Error_flag.dSw = 1;

	return fault;
}

/**
 * Increase Time out count, Called every 10ms
 */
void DoorTimerCountSub(void)
{
	if (ramp_stow_timeout.flag == TRUE)
		ramp_stow_timeout.count++;
	else
		ramp_stow_timeout.count =0;
/////////////////////////////////////////////////////////////////////////////////////////////////////
	if (release_timeout.flag == TRUE)
		release_timeout.count++;
	else
		release_timeout.count =0;

	if (DI.Door_ajar == FALSE)    //If Ajar switch is off, means released.
		release_timeout.flag = FALSE;

	if (release_timeout.count > RELEASE_TIMEOUT)    									//If the door have not been release in 2 seconds, set timeout error and return to Idle.
	{
			release_timeout.flag  = FALSE;
			Set_Timeout_Error(UNRELEASE_TO_ERROR);
			m_doorstep= DOOR_MONITOR;
			PRINTF("release_timeout.count>200 go to Idle.\r\n");
	}

//////////////////////////////////////////////////////////////////////////////////////////////////
	if (Ajar_Wait.flag == TRUE)                      	//time delay from release to ajar.
		Ajar_Wait.count++;
	else
		Ajar_Wait.count = 0;

	if (Ajar_Wait.count > 100)								//If Ajar is not off in 1 second
	{
			Ajar_Wait.flag  = FALSE;
		    flag_Need_Full_Power_Motor = TRUE;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (unlatch_timeout.flag == TRUE)
		unlatch_timeout.count++;
	else
		unlatch_timeout.count = 0;

	if (DI.full_door_open == FALSE)    				//If full open switch is off, means unlatched.
		unlatch_timeout.flag = FALSE;

	if (unlatch_timeout.count>UNLATCH_TIMEOUT)       					 					//if latch is not turned off in 2 seconds
	{
		Set_Timeout_Error(UNLATCH_TO_ERROR);      		//Open door fail
		PRINTF("Set_Timeout_Error(UNLATCh_TO_ERROR);\r\n");
		m_doorstep = STEP_AFTER_TO;
		unlatch_timeout.flag = FALSE;
	}
///////////////////////////////////////////////////////////////////////////////////////////////
	//Open door fail, possible motor or clutch fail
	if (door_open_timeout.flag == TRUE)
		door_open_timeout.count++;
	else
		door_open_timeout.count =0;

	if ((door_open_timeout.count>DOOR_OPEN_TIMEOUT))        					 					//if open time is longer than 150 seconds
	{
		if (m_doorstep == NORMAL_OPEN)
		{
			Set_Timeout_Error(DOOR_OPEN_TO_ERROR);
			PRINTF("Set_Timeout_Error(DOOR_OPEN_TO_ERROR);\r\n");
		}
//		else if (m_doorstep == ALMOST_OPENED)
//		{
//			Set_Timeout_Error(DOOR_ALMOST_OPEN_TO_ERROR);
//			printf("Set_Timeout_Error(DOOR_ALMOST_OPEN_TO_ERROR);\r\n");
//		}

		m_doorstep = STEP_AFTER_TO;
		door_open_timeout.flag = FALSE;
	}
///////////////////////////////////////////////////////////////////////////////////////////////
		//Close door fail, possible motor or clutch fail
		if (door_close_timeout.flag == TRUE)
			door_close_timeout.count++;
		else
			door_close_timeout.count =0;

		if ((door_close_timeout.count>DOOR_CLOSE_TIMEOUT))        					 					//if open time is longer than 150 seconds
		{
			Set_Timeout_Error(DOOR_CLOSE_TO_ERROR);
			PRINTF("Set_Timeout_Error(DOOR_CLOSE_TO_ERROR);\r\n");
			m_doorstep = STEP_AFTER_TO;
			door_close_timeout.flag = FALSE;
		}

////////////////////////////////////////////////////////////////////////////////////////////////////
		//Open door fail, The door full open switch might broken?
		if (door_final_open_timeout.flag == TRUE)
			door_final_open_timeout.count++;
		else
			door_final_open_timeout.count =0;

		if ((door_final_open_timeout.count>DOOR_FINAL_OPEN_TO_TIME))         		    				 //If open time is longer than 3 seconds
		{
			Set_Timeout_Error(DOOR_FULL_OPEN_TO_ERROR);
			m_doorstep = STEP_AFTER_TO;
		}
	////////////////////////////////////////////////////////////////////////////////////////////////////
		//Open door fail, The door full open switch might broken?
		if (door_final_close_timeout.flag == TRUE)
			door_final_close_timeout.count++;
		else
			door_final_close_timeout.count =0;

		if ((door_final_close_timeout.count>3000))         		    				 //If open time is longer than 30 seconds
		{
			Set_Timeout_Error(DOOR_FULL_CLOSE_TO_ERROR);
			m_doorstep = STEP_AFTER_TO;
		}
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	if (ramp_open_timeout.flag == TRUE)
		ramp_open_timeout.count++;
	else
		ramp_open_timeout.count =0;
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	//Hold the motor time after Fully opened
	if (Hold_Motor_Fully_Open.flag == TRUE)
		Hold_Motor_Fully_Open.count++;
	else
		Hold_Motor_Fully_Open.count = 0;
	////////////////////////////////////////////////////////////////////////////////////////////////////
//  Hold Cluth after Motor stop
	if (Hold_Clutch_After_Motor_Off.flag == TRUE)
		Hold_Clutch_After_Motor_Off.count++;
	else
		Hold_Clutch_After_Motor_Off.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
//	Delay time to Hold Clutch Before Motor On
	if (Hold_Clutch_Before_Motor_On.flag == TRUE)
		Hold_Clutch_Before_Motor_On.count++;
	else
		Hold_Clutch_Before_Motor_On.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Latch_to_OpenSW_off.flag == TRUE)
			Hold_Latch_to_OpenSW_off.count++;
		else
			Hold_Latch_to_OpenSW_off.count = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Motor_Before_Latch_On.flag == TRUE)
			Hold_Motor_Before_Latch_On.count++;
		else
			Hold_Motor_Before_Latch_On.count = 0;
//////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Motor_On_Full_Closed_Time.flag == TRUE)
			Hold_Motor_On_Full_Closed_Time.count++;
		else
			Hold_Motor_On_Full_Closed_Time.count = 0;
//////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Motor_Off_Full_Closed_Time.flag == TRUE)
			Hold_Motor_Off_Full_Closed_Time.count++;
		else
			Hold_Motor_Off_Full_Closed_Time.count = 0;
//////////////////////////////////////////////////////////////////////////////////////////////
		if (Wait_RAMP_STOW_SIG_Time.flag == TRUE)
			Wait_RAMP_STOW_SIG_Time.count++;
		else
			Wait_RAMP_STOW_SIG_Time.count = 0;

		if (Wait_RAMP_STOW_SIG_Time.count >100 )    // 1second
			Wait_RAMP_STOW_SIG_Time.done =TRUE;
		else
			Wait_RAMP_STOW_SIG_Time.done =FALSE;
//////////////////////////////////////////////////////////////////////////////////////////////
		if (Lock_Motor_Time.flag == TRUE)
			Lock_Motor_Time.count++;
		else
			Lock_Motor_Time.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_StopMotor_to_CloseDoor.flag == TRUE)
			Hold_StopMotor_to_CloseDoor.count++;
		else
			Hold_StopMotor_to_CloseDoor.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Close_Speed1_to_Normal_Close.flag == TRUE)
			Hold_Close_Speed1_to_Normal_Close.count++;
		else
			Hold_Close_Speed1_to_Normal_Close.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Latch_Time.flag == TRUE)
			Hold_Latch_Time.count++;
		else
			Hold_Latch_Time.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
		if (Hold_Motor_Stop_to_Start_Time.flag == TRUE)
			Hold_Motor_Stop_to_Start_Time.count++;
		else
			Hold_Motor_Stop_to_Start_Time.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
	//  DI DisableConv is changed.
		if (DisableConv_Time.flag == TRUE)
		{
			DisableConv_Time.count++;
			if (DisableConv_Time.count > 400)          //over 4 seconds
			{
				DisableConv_Time.flag = FALSE;
				DisConv_toggle_count = 0;
			}
		}
		else
			DisableConv_Time.count = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (DO.ReleaseEnable == TRUE)
			Release_Enable_Time.count++;
		else
			Release_Enable_Time.count = 0;

		if (Release_Enable_Time.count > 50)                           								//Keep DO.ReleaseEnable high for 200ms.
			DO.ReleaseEnable = FALSE;
/////---------------------------------------------------------
		if (DO.ReleaseTrig == TRUE)
			Release_Trig_Time.count++;
		else
			Release_Trig_Time.count = 0;

		if (Release_Trig_Time.count > 50)                           								//Keep DO.ReleaseTrig high for 200ms.
			DO.ReleaseTrig = FALSE;
///----------------------------------------------------------
		if (DO.Lock_Coil == TRUE)
			Lock_Coil_Time.count++;
		else
			Lock_Coil_Time.count = 0;

		if (Lock_Coil_Time.count > 50)                           								//Keep DO.Lock_Coil high for 200ms.
			DO.Lock_Coil = FALSE;
///----------------------------------------------------------
				if (DO.Latch_Disable == FALSE)    //if Latch is enabled
					Latch_Time.count++;
				else
					Latch_Time.count = 0;

				if (Latch_Time.count > open_door_time_unlatch)                           								//Keep DO.Lock_Coil high for 200ms.
					LatchOperation(LATCH_DISABLE);
					//DO.Latch = FALSE;
////----------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////
				if (Hold_Clutch_Outside_Handle_Time.flag == TRUE)
					Hold_Clutch_Outside_Handle_Time.count++;
				else
					Hold_Clutch_Outside_Handle_Time.count = 0;

////////////////////////////////////////////////////////////////////////////////////////////////
				if (Quick_Clutch_Time.flag == TRUE)
				{
					Quick_Clutch_Time.count++;
				}
				else
					Quick_Clutch_Time.count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////
		if (FOB_OPEN_Delay_Time.flag == TRUE)
		{
			FOB_OPEN_Delay_Time.count++;
			if (FOB_OPEN_Delay_Time.count>50)													//Wait 0.5 second to let door unlock.
			{
				start_OP_Door(DI,adc->Door_Encode_Signal);
				FOB_OPEN_Delay_Time.flag= FALSE;
			}
		}
		else
			FOB_OPEN_Delay_Time.count = 0;
		///////////////////////////////////////////////////////////////////////////////////////
		if (Ramp_Light_On_Time.flag == TRUE)
		{
			Ramp_Light_On_Time.count++;
		}
		else
			Ramp_Light_On_Time.count = 0;
		///////////////////////////////////////////////////////////////////////////////////////
		if (Ajar_Hold_Time.flag == TRUE)
		{
			Ajar_Hold_Time.count++;
		}
		else
			Ajar_Hold_Time.count = 0;
		///////////////////////////////////////////////////////////////////////////////////////
		if (Outside_Handle_Wait_Time.flag == TRUE)
		{
			Outside_Handle_Wait_Time.count++;
		}
		else
			Outside_Handle_Wait_Time.count = 0;

		if (Outside_Handle_Wait_Time.count >40)            //40ms delay to release the door.
		{
				MagnaOperation(DOOR_RELEASE);
				DO.Clutch = FALSE;
				Outside_Handle_Wait_Time.flag = FALSE;
		}
		///////////////////////////////////////////////////////////////////////////////////////
}



/**
 * Set Magna relays based on the operation
 */
void MagnaOperation(char operation)
{
	 switch (operation)
	 {
	 case DOOR_RELEASE:
		 DO.ReleaseEnable = TRUE;
		 DO.ReleaseTrig = TRUE;
		 DO.Lock_Coil = FALSE;
		 break;
	 case DOOR_LOCK:
		 DO.ReleaseEnable = FALSE;
		 DO.ReleaseTrig = TRUE;
		 DO.Lock_Coil = TRUE;
		 break;
	 case DOOR_UNLOCK:
		 DO.ReleaseEnable = FALSE;
		 DO.ReleaseTrig = TRUE;
		 DO.Lock_Coil = FALSE;
		 break;
	 default:
		 DO.ReleaseEnable = FALSE;
		 DO.ReleaseTrig = FALSE;
		 DO.Lock_Coil = FALSE;
		 break;
	 }
}
////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Set Magna relays based on the operation
 */
void LatchOperation(char operation)
{
	DO.Latch_VSO = TRUE;
	 switch (operation)
	 {
	 case LATCH_EXTRACT:
		 DO.Latch_DIR = TRUE;
		 DO.Latch_PWM = TRUE;
		 DO.Latch_Disable = FALSE;
		 break;
	 case LATCH_WITHDRAW:
		 DO.Latch_DIR = FALSE;
		 DO.Latch_PWM = TRUE;
		 DO.Latch_Disable = FALSE;
		 break;
	 case LATCH_DISABLE:
		 DO.Latch_DIR = FALSE;
		 DO.Latch_PWM = FALSE;
		 DO.Latch_Disable = TRUE;
		 break;
	 default:
		 DO.Latch_DIR = FALSE;
		 DO.Latch_PWM = FALSE;
		 DO.Latch_Disable = TRUE;
		 break;
	 }
}
////////////////////////////////////////////////////////////////////////////////////////////////
////Function: Reset most of the flags in the IDLS step
void ResetFlagsInIDLE(void )
{
	release_timeout.flag = FALSE;
	unlatch_timeout.flag = FALSE;
	door_open_timeout.flag=FALSE;
	door_close_timeout.flag=FALSE;
	door_final_open_timeout.flag=FALSE;
	door_final_close_timeout.flag=FALSE;
	ramp_open_timeout.flag=FALSE;
	ramp_stow_timeout.flag=FALSE;
	Position2_to_Close_timeout.flag=FALSE;
	Ajar_Wait.flag=FALSE;           //release to ajar wait time
	Hold_Motor_Fully_Open.flag=FALSE;
	Hold_Clutch_After_Motor_Off.flag=FALSE;
	Hold_Clutch_Before_Motor_On.flag=FALSE;
	Hold_Motor_Before_Latch_On.flag=FALSE;
	Hold_StopMotor_to_CloseDoor.flag=FALSE;
	Hold_Close_Speed1_to_Normal_Close.flag=FALSE;
	Hold_Latch_Time.flag=FALSE;
	Hold_Clutch_Outside_Handle_Time.flag =FALSE;
	Hold_Latch_to_OpenSW_off.flag=FALSE;
	Hold_Motor_On_Full_Closed_Time.flag=FALSE;
	Hold_Motor_Off_Full_Closed_Time.flag=FALSE;
	Hold_Motor_Stop_to_Start_Time.flag=FALSE;
	Wait_RAMP_STOW_SIG_Time.flag=FALSE;
	Lock_Motor_Time.flag=FALSE;
	Release_Trig_Time.flag = FALSE;
	Release_Enable_Time.flag = FALSE;
	Lock_Coil_Time.flag = FALSE;
	FullPowerOpenDoorTime.flag = FALSE;
	PID.Integral= 0;
	PID.Proportional = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//#define UNRELEASE_TO_ERROR 0x1  //release time out error.
////#define DOOR_START_OPEN_TO_ERROR 0x0002	//door open time out error
//#define DOOR_OPEN_TO_ERROR 0x2
//#define UNLATCH_TO_ERROR  0x3
//#define DOOR_CLOSE_TO_ERROR 0x4
////#define DOOR_ALMOST_OPEN_TO_ERROR 0x5
//#define DOOR_FULL_OPEN_TO_ERROR 0x5   //final close error, possible the full door open sw is broken
//#define DOOR_FULL_CLOSE_TO_ERROR 0x6
void Set_Timeout_Error(uint32_t errorcode)
{
	Time_Out_Error =  errorcode;
	if ((errorcode==DOOR_OPEN_TO_ERROR) || (errorcode==DOOR_CLOSE_TO_ERROR) ||
			(errorcode==DOOR_FULL_OPEN_TO_ERROR) || (errorcode==DOOR_FULL_CLOSE_TO_ERROR) )
	{
		Error_flag.dTo= TRUE;   //ERROR 303
	}

	if (errorcode==UNRELEASE_TO_ERROR)   //ERROR 306
	{
		Error_flag.releaseTo= TRUE;
	}
	if (errorcode==UNLATCH_TO_ERROR)   //ERROR 307
	{
		Error_flag.unlatchTo= TRUE;
	}
//	Error_flag.dTo  =TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//#define INCLINATION_OUT_RANGE 0x0001   //
void Set_Normal_Error(uint32_t errorcode)
{
	Normal_Error = Normal_Error | errorcode;
}
//////////////////////////////////////////////////////////////////////////////////////////////
void Reset_ERRORs(void)
{
	Time_Out_Error = 0;
	Normal_Error =0;
}

/***
 * Start normal operation door sequence by key.
 */
void start_OP_Door(DI_st din,uint16_t door_encode)
{
	if (din.full_door_open ==TRUE) 				//The door is fully, start close door steps
	{
		m_doorstep= PRECLOSE;
	}
	else if (din.Door_ajar ==TRUE)
	{
		m_doorstep= PREOPEN;
		PRINTF("Go to the PREOPEN.\r\n");
		Full_Closed_Encode = adc->Door_Encode_Signal;
	}
	else if ((din.Door_ajar == FALSE) && (door_encode>(Full_Closed_Encode-80)))		//Barely open from a fully closed position it will open the door.
	{
		//m_doorstep= SLOWOPEN;
	}

//	else if (door_encode < (Full_Closed_Encode-8000))		// the door passed a certain point and then stop it, it will close
//	{
//		m_doorstep= DOORBUMPSTRIPACTIVATE;
//	}
//	if ((m_doorstep < FULLYCLOSED) && (m_doorstep > STARTCLOSING) )
//	{
//		printf("m_doorstep = REVERSEOPEN1;\r\n");
//		m_doorstep = REVERSEOPEN1;
//	}
//	else if ((m_doorstep < FULL_OPENED) && (m_doorstep > SLOWOPEN) )
//	{
//		printf("m_doorstep = REVERSCLOSE1;\r\n");
//		m_doorstep = REVERSCLOSE1;
//	}
}

/**
 * The ramp switch must be stewed or deployed before running the door.
 */
void Check_Ramp_Switch(char rampswitchstat)
{
	if (rampswitchstat == NO_SWITCH_ACTIVE)
	{
		Set_Normal_Error(RAMP_NO_STEW_EXPLOY);
	}
}


void Door_Operation(DI_st din,uint16_t door_encode)
{
	//printf("Full_Closed_Encode=%d;\r\n",Full_Closed_Encode);
	if (DOORDEBUG == TRUE)
	{
//		CAN_Data.ramp_state = SWITCH_STEW_ACTIVE;
//		CAN_Data.kneel_switch = SWITCH_ACTIVE;
		CAN_Data.Battery_Voltage =13;
	}
	//printf("Motor Control:%d,%d,%d,%d,%d,%d;\r\n",	open_door_speed_1,	open_door_speed_2,open_door_speed_3,close_door_speed_1,	close_door_speed_2,close_door_speed_3);
	switch (m_doorstep)
	{
	case PREOPEN: //After seeing cycle request, check transmission position value (and/or other "safe to operate" values from vehicle CAN messaging),Check vehicle inclination value & temperature value, if either are outside established range do not allow power cycle until values are back within range
		if (CAN_Data.Battery_Voltage <  BAT_LIMIT_L)
		{
			 Set_Normal_Error(BAT_VOLT_LOW);
			 m_doorstep = DOOR_MONITOR;
			 break;
		}

		if (CAN_Data.lock_status==TRUE)          // Remove to idle                        							//If door is in locked status, back to door monitor.
		{
			m_doorstep = DOOR_MONITOR;
			break;
		}
//		DO.Latch = FALSE;
		LatchOperation(LATCH_DISABLE);
//		if ((CAN_Data.ramp_switch == NO_SWITCH_ACTIVE)	&& (Wait_RAMP_STOW_SIG_Time.done == FALSE)	)						//If ramp is not stowed, wait 1 second to stow it
//		{
//				Wait_RAMP_STOW_SIG_Time.flag = TRUE;
//				break;
//		}

		Check_Ramp_Switch(CAN_Data.ramp_switch );										//If not ramp stewed, set ramp error

//		if (CheckDoorOPCondition() == OP_ALLOW)											//Must be parked, ramp stewed and Angle and temperature in the range to start open the door.
//		{
				//SetMotorSpeedbyInclination(SensorReading);  								//Adjust motor speed based on the inclination.
				if (din.Door_ajar == TRUE)
				{
						Set_Next_Door_Step(&m_doorstep,START_OPEN);
						SensorReading4OP = SensorReading;                                          //Snap the sensor reading for the operation
//						Ajar_Wait.flag = TRUE;				 //REVIEW:Should move to after release on										 		// if Ajar is not deacativated, will do some temperature stuff.
						eepromcount_t.open_count++;
						PRINTF("Go to STARTOPEN.\r\n");
						release_timeout.flag = TRUE; 														//release_timeout_flag, 5 second
//						release_timeout.count = 0;																//release_timeout_count
						m_doorstep = START_OPEN;
				}
				else
				{
						m_doorstep = DOOR_MONITOR;
				}

				Wait_RAMP_STOW_SIG_Time.flag = FALSE;

				if (din.Door_ajar == TRUE)																	//Refresh the Full closed encode reading.
					Full_Closed_Encode=door_encode;
//		}
//		else
//		{
//				m_doorstep = DOOR_MONITOR;
//		}
		break;

	case START_OPEN:
		SensorReading4OP = SensorReading;																//Pull temperature and angles for open operation
		SetMotorSpeedbyInclination(SensorReading4OP);
		/*Depend on some condition, Need more feedback.  PROG in flow chart*/
		DO.Clutch = TRUE;
		Door_flag.Clutch_to_Motor_On_flag = TRUE;
		/*Depend on some condition, Need more feedback.  PROG in flow chart*/
		if (Door_flag.Clutch_to_Motor_On_done  == TRUE)
		{
			/*Depend on some condition, Need more feedback.  PROG in flow chart*/
				Start_Open(open_door_speed_1);
				//DO.Release = TRUE;
				MagnaOperation(DOOR_RELEASE);
				Ajar_Wait.flag = TRUE;										 		// if Ajar is not deacativated, will do some temperature stuff.
				Door_flag.Clutch_to_Motor_On_done  = FALSE;
		}

		//Door_flag.Latch_Hold_On_flag= FALSE;
		if ( DI.Door_ajar == FALSE )																	//If ajar is deactivated, Door opened, go to slow open;
		{
			Ajar_Wait.flag = FALSE;																	 //Reset Ajar wait time flag
//			Door_flag.ajar_deb_done  = FALSE;
			Start_Open(open_door_speed_1);
			Door_flag.Normal_Open_Wait_flag= TRUE;                  //REVIEW changed to normal open wait time
			if (Door_flag.Normal_Open_Wait_Done == TRUE)								//After delay reset release and go to Normal open status
			{
				m_doorstep = NORMAL_OPEN;
				if (USE_PID)																					  //USE_PID =1 means speed control
				{
					CurrentDoorPower = open_door_speed_1;
				}
				//Set_Next_Door_Step(&m_doorstep,NORMAL_OPEN);
				PRINTF("Go to Normal Open\r\n");
				//printf("Go to NormalOpening, door speed:%d.\r\n", open_door_speed_1);
				//m_doorstep = SLOWOPEN;
			}
//			door_timeout.flag = TRUE;
//			door_timeout.count = 0;
			release_timeout.flag  = FALSE;                                                           //Disable release timeout timer
			FullPowerOpenDoorTime.flag = FALSE;
		}
		else																											//if no released in 2 second, will trig release timeout error, and go to IDLE
		{
			if (flag_Need_Full_Power_Motor == TRUE)										// Ajar is not off in 1 second, Need try full power motor for 1 second.
			{
					Start_Open(100);
					PRINTF("Open again with Full Power Motor.");
					FullPowerOpenDoorTime.flag = TRUE;
					if (FullPowerOpenDoorTime.count>100)   // 100 persent power for 1 second
					{
						//??Set an Error?
						m_doorstep = DOOR_MONITOR;
						flag_Need_Full_Power_Motor = FALSE;
					}
			}
		}
		break;

	case NORMAL_OPEN:     																	       //After "X" value of movement, adjust door motor power/speed to next "zone" or range
//		DO.Release = FALSE;
		DO.Clutch = TRUE;
		door_open_timeout.flag = TRUE;
		if (USE_PID)                                                                               //USE_PID =1 means speed control
		{
			CurrentDoorPower= DoorSpeedPControl(OPEN_DOOR_SPEED_1, -doorspeed_cal, &PID, CurrentDoorPower);         //Use PI control to control the speed.
			open_door_speed_2 = (int) CurrentDoorPower;
		}
		Start_Open(open_door_speed_2);													        //Motor open speed 2
		Motor_PWM_Current_Count = open_door_speed_2;

//		printf("open_door_speed_2");// = %f \r\n", open_door_speed_2 );

		if (door_encode<=(Full_Opened_Encode + ALMOSTOPENAREA))							//Almost opened
		{
			m_doorstep = ALMOST_OPENED;
			printf("Go to ALMOST_OPENED, door speed:%d,%d.\r\n", open_door_speed_2,open_door_speed_3);
			PRINTF("Go to ALMOST_OPENED");
			Motor_PWM_Current_Count = open_door_speed_3;
			Start_Open(open_door_speed_3);

			door_final_open_timeout.flag = TRUE; 										//door_final_open_timeout
			door_final_open_timeout.count = 0;
		}
		if (DI.full_door_open == TRUE)												//In case encode does not work, the full door open DI will jump the step of the full opened
		{
				//Door_flag.f_open_deb_done  = FALSE;
				m_doorstep = FULLY_OPENED;
				PRINTF("Go to FULL_OPENED\r\n");
				Set_Normal_Error(DOOR_ENCODE_ERROR);				//Set Encode Error
		}
		break;
	case ALMOST_OPENED:																  //When reaching "XX" value of movement (near full open), adjust door motor power/speed to "approaching full open" range
		//printf("ALMOSTOPEN\r\n");
		//DO.Latch = FALSE;
		if (DI.full_door_open == TRUE)
		{
			//Door_flag.f_open_deb_done  = FALSE;
			m_doorstep = FULLY_OPENED;
			PRINTF("FULLY_OPENED.\r\n");
			door_open_timeout.flag = FALSE;										//stop door open time_out count.
//			printf("Go to FULL_OPENED\r\n");
		}

		break;
	case FULLY_OPENED:
		//printf("FULL_OPENED\r\n");
		door_final_open_timeout.flag = FALSE;
		                   	 									 	//Lock the Motor
		Hold_Motor_Fully_Open.flag = TRUE;

		if (Hold_Motor_Fully_Open.count < 25)                //250ms hold motor
		{
			Pause_Hold();
		}
		else
		{
			Hold_Clutch_After_Motor_Off.flag = TRUE;
			if (Hold_Clutch_After_Motor_Off.count < 20 )
			{
				Stop_Motors();
			}
			else
			{
					eepromcount_t.full_opened_encode = Full_Opened_Encode;
					DO.Clutch = FALSE;

					if (AUTOCYCLE==TRUE)
					{
						Door_flag.Autocycle_delay_flag = TRUE;
						m_doorstep = AUTOCYCLE_WAIT_CLOSE_DELAY;
						PRINTF("Go to AUTOCYCLE_WAIT_CLOSE_DELAY\r\n");
					}
					else
					{
							m_doorstep = DOOR_MONITOR;
							PRINTF("go to DOORIDLE.\r\n");
					}
					Hold_Motor_Fully_Open.flag = FALSE;
					Hold_Clutch_After_Motor_Off.flag = FALSE;
			}
		}
		break;

	case AUTOCYCLE_WAIT_CLOSE_DELAY:
		if (CAN_Data.ramp_state == NO_SWITCH_ACTIVE)
				Door_Count.Autocycle_delay_count = 0;

		if (Door_flag.Autocycle_delay_done == TRUE)
		{
			if (CAN_Data.ramp_state == SWITCH_DEPLOY_ACTIVE)         // ramp is stewed)
			{
					Braun_Op_Pressed();
					Braun_Op_Switch_UART_flag=TRUE;
					//m_doorstep = PRECLOSE;
					Door_flag.Autocycle_delay_done = FALSE;
					PRINTF("go to PRECLOSE\r\n");
			}
		}
		break;


/************************* close door steps****************************/
	case PRECLOSE: 																			//After seeing cycle request, stow ramp & kneel up (using hardware/software developed on next gen controller work)
//		if (DEBUG) {
//			CAN_Data.kneel_switch = SWITCH_ACTIVE;
//			CAN_Data.ramp_state = FALSE;
//		}
//		SensorReading4OP = SensorReading;
		if (CAN_Data.Battery_Voltage <  BAT_LIMIT_L)
		{
			 Set_Normal_Error(BAT_VOLT_LOW);
			 m_doorstep = DOOR_MONITOR;
		}

		Check_Ramp_Switch(CAN_Data.ramp_switch );

		if ((CAN_Data.ramp_state == SWITCH_STEW_ACTIVE))  //	&& (CAN_Data.kneel_switch == SWITCH_ACTIVE)) 										//Fully Stowed.
		{
			if (CheckDoorOPCondition() == OP_ALLOW)
			{
					//m_DoorError = m_DoorError & (NOTPARKING);
//				if (CAN_Data.park_stat == FALSE)
//				{
//					//LED.ParkingWarning == TRUE;
//				}
//				if (GyoSensorInRange(SensorReading4OP) == FALSE) //Inclination and temperature in range
//				{
//	//				LED.AngleWarning == TRUE;                                //Angle over the limit, give out the warning but still close the door.
//				}
				m_doorstep = START_CLOSING;
				eepromcount_t.close_count++;
				PRINTF("Go to STARTCLOSING.\r\n");
			}
			else
				m_doorstep = DOOR_MONITOR;
		}
		else
		{
			ramp_stow_timeout.flag = TRUE;
			if (ramp_stow_timeout.count >= 2000) 																		//stew waiting time >20 seconds. give up
			{
				m_doorstep = STEP_AFTER_TO;
				ramp_stow_timeout.flag = FALSE;
				PRINTF("Ramp_stew_timeout\r\n");
				//Set_Timeout_Error(RAMP_STEW_TO_ERROR);
			}
		}
		break;

	case START_CLOSING: 		//Check vehicle inclination value & temperature value, if either are outside established range do not allow power door cycle until values are back within range
		SensorReading4OP = SensorReading;																//Snap temperature and angles for open operation
		SetMotorSpeedbyInclination(SensorReading4OP);

		DO.Clutch = TRUE;
		Hold_Clutch_Before_Motor_On.flag = TRUE;
		if (Hold_Clutch_Before_Motor_On.count < OPEN_BEFORE_CLOSING_TIME)
			break;

		Hold_Motor_Before_Latch_On.flag = TRUE;
		if (Hold_Motor_Before_Latch_On.count<open_door_time_unlatch)
		{
			printf("Start_Closing = %d;%d.\r\n", open_door_speed_unlatch,open_door_time_unlatch);
			Start_Open(open_door_speed_unlatch);                                     //setup this power based on the angle.
			//Start_Open(100);
			break;
		}
		//DO.Latch = TRUE;
		LatchOperation(LATCH_WITHDRAW);
		Hold_Latch_to_OpenSW_off.flag = TRUE;
		if (Hold_Latch_to_OpenSW_off.count  < 20)                                     //Wait for the full open switch off.
			break;

//		Start_Close(close_door_speed_1);
		if (DI.full_door_open == FALSE)								//If unlatched ( door open switch is off)
		{
				Hold_StopMotor_to_CloseDoor.flag = TRUE;
				if (Hold_StopMotor_to_CloseDoor.count < 20)
				{
					//Stop_Motors();
					break;
				}
				else
				{
					Start_Close(close_door_speed_1);
					Motor_PWM_Current_Count = -close_door_speed_1;
					Hold_Close_Speed1_to_Normal_Close.flag = TRUE;
					if (Hold_Close_Speed1_to_Normal_Close.count>100)
					{
						m_doorstep = NORMAL_CLOSE;
						Hold_Motor_Before_Latch_On.flag = FALSE;
						Hold_StopMotor_to_CloseDoor.flag = FALSE;
						Hold_Close_Speed1_to_Normal_Close.flag = FALSE;
						if (USE_PID)																					  //USE_PID =1 means speed control
						{
							CurrentDoorPower = close_door_speed_1;
						}
						printf("Go to NormalClosing, door speed:%d,%d,%d.\r\n", close_door_speed_1,close_door_speed_2,close_door_speed_3);
						PRINTF("Go to NormalClosing\r\n");
					}
				}
		}
		else																	// If full open time out
		{
			unlatch_timeout.flag = TRUE;
		}

	case NORMAL_CLOSE: 		//Turn on clutch/door motor in CLOSE direction
		Hold_Latch_Time.flag = TRUE;
		Motor_PWM_Current_Count = -close_door_speed_2;                   //prepare for the reverse open
		if (Hold_Latch_Time.count  < 100)													//Hold the latch for another 1 second after full door open switch off.
			 break;

		//DO.Latch = FALSE;
		if ((door_encode>  32500))
			LatchOperation(LATCH_EXTRACT);

		if (USE_PID)                                                                               //USE_PID =1 means speed control
		{
			CurrentDoorPower= DoorSpeedPControl(CLOSE_DOOR_SPEED_1, doorspeed_cal, &PID, CurrentDoorPower);         //Use PI control to control the speed.
			close_door_speed_2 = (int) CurrentDoorPower;
		}
		Start_Close(close_door_speed_2);
		Motor_PWM_Current_Count = -close_door_speed_2;
		door_close_timeout.flag = TRUE;
		if ((door_encode>=(Full_Closed_Encode - 2500))	&&(adc->Motor_Cur_Close > MOTORCURRENT_FULLY_CLOSED))							//Almost Closed
		{
				m_doorstep = ALMOST_CLOSED;
				PRINTF("Go to ALMOSTCLOSED.\r\n");
				Start_Close(close_door_speed_3);
//				Door_flag.general_delay_flag = TRUE;
//				Door_flag.general_delay_done = FALSE;
				Hold_Latch_Time.flag = FALSE;
		}
//		else																						//If Ajar is activated and close motor current is over the threshold before encode, set encode error.
//		{
//			  if ((DI.Door_ajar == TRUE)&&(adc->Motor_Cur_Close > MOTORCURRENT_FULLY_CLOSED))
//			  {
//				  Set_Normal_Error(DOOR_ENCODE_ERROR);				//Set Encode Error
//				  m_doorstep = FULLY_CLOSED;
//				  PRINTF("Ajar ON and fully closed by current.\r\n");
//			  }
//		}
		break;

	case ALMOST_CLOSED:
		door_final_close_timeout.flag = TRUE;
		door_close_timeout.flag = FALSE;
//		if (DI.Door_ajar == TRUE)    // Old code, might fail the operation if spike happen.
//		{
//			m_doorstep = FULLY_CLOSED;
//		}
		if (DI.Door_ajar == TRUE)
		{
			Ajar_Hold_Time.flag = TRUE;
			if (Ajar_Hold_Time.count>2)         														 //The door_ajar need hold high for 20ms
			{
				m_doorstep = FULLY_CLOSED;
			}
		}
		else
		{
			Ajar_Hold_Time.flag = FALSE;
		}
		break;
	case FULLY_CLOSED:																					//When door ajar signal is no longer present, turn off door motor
//		Start_Close(close_door_speed_3);
		door_close_timeout.flag = FALSE;
		Hold_Motor_On_Full_Closed_Time.flag =TRUE;
		if (Hold_Motor_On_Full_Closed_Time.count<30)										 //Wait another 300ms before stop the motor.
		{
			Start_Close(30);
			break;
		}
		Stop_Motors();
		Hold_Motor_Off_Full_Closed_Time.flag = TRUE;
		if (Hold_Motor_Off_Full_Closed_Time.count>80)									 //Wait another 300ms before unclutch
		{
			DO.Clutch = FALSE;
			m_doorstep = DOOR_MONITOR;
		}

		break;

//	case AUTOCYCLE_WAIT_OPEN_DELAY:
////		if (Door_flag.general_delay_done == FALSE)
////			break;
//		DO.Clutch = FALSE;
//		//printf("Clutch Off1\r\n");
//		if (Door_flag.Autocycle_delay_done == TRUE)
//		{
//			Door_flag.Autocycle_delay_done = FALSE;
//			printf("Go to PREOPEN\r\n");
//			//m_doorstep = PREOPEN;
//			Braun_Op_Pressed();
//			Braun_Op_Switch_UART_flag=TRUE;
//		}
//		break;

	case DOORBUMPSTRIPACTIVATE:
		Stop_Motors();
		m_doorstep = REVERSEOPEN;
		eepromcount_t.reverse_count++;
		PRINTF("Go to REVERSEOPEN1.");
		Door_flag.reverse_open_delay_flag = TRUE;
		break;
	case DOOROPENSTUCKED:
		Stop_Motors();
		m_doorstep = REVERSCLOSE;
		eepromcount_t.reverse_count++;
		PRINTF("Go to REVERSECLOSE.");
		Door_flag.reverse_close_delay_flag = TRUE;
		break;
	case REVERSEOPEN:
//		if (Door_flag.reverse_open_delay_done == TRUE)
//		{
			//Start_Open(open_door_speed_2);
			ChangeMotorSpeed(open_door_speed_2, &Motor_PWM_Current_Count);
			//ChangeMotorSpeed(20, &Motor_PWM_Current_Count);
			Reverse_OP_flag = TRUE;
			if (Motor_PWM_Current_Count == open_door_speed_2)
			{
				PRINTF("REVERSEOPEN2 Go to NORMAL_OPEN\r\n");
				m_doorstep = NORMAL_OPEN;
			}

			if (DI.full_door_open == TRUE)
			{
				m_doorstep = DOOR_MONITOR;
				PRINTF("Door Fully Opened.");
			}
//		}
		break;
	case REVERSCLOSE:
		Hold_Motor_Stop_to_Start_Time.flag = TRUE;
		if (Hold_Motor_Stop_to_Start_Time.count<1)				// stop motor for 0.5 second
		{
			Stop_Motors();
			Motor_PWM_Current_Count =0;
		}
		else
		{
			ChangeMotorSpeed(-close_door_speed_2, &Motor_PWM_Current_Count);
			if (Motor_PWM_Current_Count == -close_door_speed_2)
			{
				PRINTF("REVERSECLOSE Go to NORMAL_OPEN\r\n");
				m_doorstep = NORMAL_CLOSE;
			}
		}

		if (DI.Door_ajar == TRUE)
		{
			m_doorstep = DOOR_MONITOR;
			PRINTF("Door Fully Closed.");
		}

		door_open_timeout.flag = FALSE;
		Reverse_OP_flag = TRUE;

		break;

	case STEP_AFTER_TO:
		Lock_Motor_Time.flag = TRUE;
		if (Lock_Motor_Time.count>20)
		{
			Stop_Motors();
			m_doorstep = DOOR_MONITOR;
			Lock_Motor_Time.flag = FALSE;
			Lock_Motor_Time.count =0;
		}
		else
		{
			Pause_Hold();
		}
		break;

	case DOOR_MONITOR:
		/**********************************************************************
		 * These code are comment for bypass the outside handle function, since the wire harness is not correct.
		 */
		//DO.Release = FALSE;
		 open_door_time_unlatch = 300;
		//CheckErrorState() ;                                          //Need more information
		if (CAN_Data.lock_status == TRUE)
			break;

		Stop_Motors();

		Reverse_OP_flag = FALSE;
		ResetFlagsInIDLE();																					//Reset most of the flags in the IDLS step
/****************************************************************************************************/
		if ((DI.Door_ajar == TRUE)	|| (DI.full_door_open == TRUE))						//Fully Closed or fully opened, Nothing need to to.
		{
			DO.Clutch = FALSE;
			//printf("Clutch Off2\r\n");
			break;
		}
		else if (doorspeed_cal>(DOOR_SPEED_LIMIT_IN_IDLE)	)				// Opening too fast
		{
			SensorReading4OP = SensorReading;																//Snap temperature and angles for open operation
			SetMotorSpeedbyInclination(SensorReading4OP);
			Motor_PWM_Current_Count = open_door_speed_2;
			DO.Clutch = TRUE;
			ChangeMotorSpeed(-close_door_speed_2, &Motor_PWM_Current_Count);
//			if (Motor_PWM_Current_Count == -close_door_speed_2)
//			{
//				printf("REVERSECLOSE Go to NORMAL_OPEN\r\n");
				m_doorstep = REVERSCLOSE;//NORMAL_CLOSE;
//			}
//			if (door_encode<=(Full_Opened_Encode + 3000))			//Almost Open
//			    m_doorstep = SLOW_CLOSE;
//			else
//				m_doorstep = NORMAL_CLOSE;

				PRINTF("m_doorstep =SLOW_CLOSE\r\n");
			//Set_Next_Door_Step(&m_doorstep,OPENLATCH);    					 //Closing
		}
		else if (doorspeed_cal< -(DOOR_SPEED_LIMIT_IN_IDLE))        		//Closing too fast
		{
			DO.Clutch = TRUE;//m_doorstep = MAGRELEASEOFF;
			if (door_encode>=(Full_Opened_Encode + 3000))			//Almost Open
				m_doorstep = SLOW_OPEN;
			else
				m_doorstep = NORMAL_OPEN;
			//Set_Next_Door_Step(&m_doorstep,MAGRELEASEOFF);		//Slow Open
		}
		else if ((SensorReading.NoseAngle > Config_Value.ANGLE_CLINATION_MAX+1) || (SensorReading.NoseAngle < Config_Value.ANGLE_CLINATION_MIN-1) || (SensorReading.SideAngle < Config_Value.ANGLE_CLINATION_MIN-1))  //for three position may move the door fast
		{
			DO.Clutch = TRUE;
			if ((doorspeed_cal>DOOR_SPEED_LIMIT1 ) && (adc->Door_Encode_Signal > Full_Closed_Encode - 26000))   //The door_speed_cal is nonlinear for the door position. encode changing is faster on the open side.
			{
				//Set_Next_Door_Step(&m_doorstep,OPENLATCH);      //Closing
				DO.Clutch = TRUE;
			}
			else if ((doorspeed_cal<-DOOR_SPEED_LIMIT1) && (adc->Door_Encode_Signal < Full_Closed_Encode - 26000))   //close door half side
			{
				//Set_Next_Door_Step(&m_doorstep,MAGRELEASEOFF);    //Slow open
				DO.Clutch = TRUE;
			}
		}
		else
		{
			Stop_Motors();
			DO.Clutch = FALSE;
		}

//		else if ((SensorReading.NoseAngle < ANGLE_CLINATION_MAX-1) && (SensorReading.NoseAngle > ANGLE_CLINATION_MIN+1))
//		{
//			if ((doorspeed_cal>DOOR_SPEED_LIMIT1 ) && (adc->Door_Encode_Signal > Full_Closed_Encode - 26000))   //The door_speed_cal is nonlinear for the door position. encode changing is faster on the open side.
//			{
//				//Set_Next_Door_Step(&m_doorstep,OPENLATCH);      //Closing
//				DO.Clutch = TRUE;
//			}
//			else if ((doorspeed_cal<-DOOR_SPEED_LIMIT1) && (adc->Door_Encode_Signal < Full_Closed_Encode - 26000))   //close door half side
//			{
//				//Set_Next_Door_Step(&m_doorstep,MAGRELEASEOFF);    //Slow open
//				DO.Clutch = TRUE;
//			}
//
//			else
//			{
//				DO.Clutch = FALSE;
//			}
//		}

		//If door speed goes above established threshold (runaway or abuse type situation), engage clutch & circuit for power bank until door speed falls below threshold.

		break;

	case OUTHANDLE_ACTIVATED:
		//release or unlatch door.
		DO.Clutch = TRUE;

		if (Hold_Clutch_Outside_Handle_Time.flag == FALSE)
		{
			if (DI.Door_ajar == TRUE)
			{
				Outside_Handle_Wait_Time.flag = TRUE;
				//MagnaOperation(DOOR_RELEASE);
			}
			if (DI.full_door_open == TRUE)
			{
				//DO.Latch = TRUE;
				LatchOperation(LATCH_WITHDRAW);
			}
			//open_door_time_unlatch= 1000;
			Hold_Clutch_Outside_Handle_Time.flag = TRUE;
		}
		else
		{
			if ((DI.Door_ajar == FALSE))//  && (DI.full_door_open == FALSE))
			{
				m_doorstep = AFTER_OUTHANDLE_ACTIVATED;
				//DO.Latch = FALSE;
				Hold_Clutch_Outside_Handle_Time.flag = FALSE;
				Quick_Clutch_Time.flag = TRUE;
				PRINTF("OUTHANDLE_ACTIVATED to SLOW_OPEN;\r\n");
			}
		}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Used for the slow open/close for big angle operation.
//		DO.Clutch = TRUE;
//		MagnaOperation(DOOR_RELEASE);
//		if (DI.Door_ajar == TRUE)
//			m_doorstep = SLOW_OPEN;
//
//		if (DI.full_door_open == TRUE)
//			m_doorstep = PRECLOSE;
//////////////////////////////////////////////////////////////////////////////////////////////////////

		break;

	case AFTER_OUTHANDLE_ACTIVATED:
////   for quick on/off clutch to
//		if (Quick_Clutch_Time.count>=10)
//			DO.Clutch = TRUE;
//		else
//			DO.Clutch = TRUE;
//
//		 if (Quick_Clutch_Time.count>=20)
//			 Quick_Clutch_Time.count =0;

		if ((DI.Door_ajar == TRUE)  && (DI.full_door_open == TRUE))
		{
			Hold_Motor_Off_Full_Closed_Time.flag = TRUE;
			if (Hold_Motor_Off_Full_Closed_Time.count>80)										 //Wait another 800ms before unclutch
			{
				DO.Clutch = FALSE;
				m_doorstep = DOOR_MONITOR;
			}
			Quick_Clutch_Time.flag = TRUE;
		}
		else
		{
			Hold_Motor_Off_Full_Closed_Time.flag = FALSE;
		}
		break;
	case SLOW_OPEN:
		SensorReading4OP = SensorReading;															//Snap temperature and angles for open operation
		SetMotorSpeedbyInclination(SensorReading4OP);
		DO.Clutch = TRUE;
		Start_Open(open_door_speed_1);
		door_open_timeout.flag = TRUE;
		if (DI.full_door_open == TRUE)
		{
			Hold_Clutch_After_Motor_Off.flag = TRUE;
			if (Hold_Clutch_After_Motor_Off.count<25)
			{
				 Stop_Motors();
			}
			else
			{
				 DO.Clutch = FALSE;
				 m_doorstep= DOOR_MONITOR;
				 Hold_Clutch_After_Motor_Off.flag = FALSE;
			}
		}
		break;

	case SLOW_CLOSE:
		SensorReading4OP = SensorReading;															//Snap temperature and angles for open operation
		SetMotorSpeedbyInclination(SensorReading4OP);
		DO.Clutch = TRUE;
		Start_Close(close_door_speed_1);
		door_close_timeout.flag = TRUE;
		if (DI.Door_ajar == TRUE)
		{
			Hold_Clutch_After_Motor_Off.flag = TRUE;
			if (Hold_Clutch_After_Motor_Off.count<25)
			{
				 Stop_Motors();
			}
			else																												//Wait 250ms to clutch off after stop motor
			{
				 DO.Clutch = FALSE;
				 m_doorstep= DOOR_MONITOR;
				 Hold_Clutch_After_Motor_Off.flag = FALSE;
			}
		}
		break;

	default:  // IDLE?
		break;
	}

	if (Door_flag.reset_error_done == TRUE)
	{
		Reset_ERRORs();
	}
}

/**
 * Bump strip (for obstacle detection) ALWAYS active on close cycle
 * If at any time, the bump strip is activated - shut down door motor - pause - then run set subroutine for "safety" door open.
 * Need more details on the safety sequence.
 */
void CheckBumpStripEvent(void)
{
	if (AUTOCYCLE == TRUE)
	{
		m_doorstep = DOOR_MONITOR;
		Stop_Motors();
		DO.Clutch = FALSE;
		return;
	}

	if ((m_doorstep >= START_CLOSING) && (m_doorstep <= FULLY_CLOSED) )
	{
		m_doorstep = DOORBUMPSTRIPACTIVATE;
	}

}

void Set_Next_Door_Step(char* step, char nextstep)
{
	*step = nextstep;
	Door_flag.general_delay_flag = TRUE;
	Door_Count.general_delay_count = 0;
	Door_flag.general_delay_done = FALSE;
}

void ManualOperator(void)
{
	switch (m_manual_step)
	{
	case MANUAL_PREOPEN:
		if (GyoSensorInRange(SensorReading) == TRUE)		//Check vehicle inclination value
		{
			m_manual_step = MANUAL_OPEN_RELEASE;
		}
		break;
	case MANUAL_OPEN_RELEASE:										//If inclination value is "X - Y" range (normal/level ground) - activate Braun full open latch release & leave clutch disengaged
		//DO.Latch=TRUE;
		LatchOperation(LATCH_WITHDRAW);
		if ( SensorReading.LSM6AngleZ < INCLINATION_NORMAL_RANGE )
		{
			DO.Clutch = FALSE;
		}
		else
		{
			DO.Clutch = TRUE;												 //If inclination value is "Y - Z" range (nose down extreme) - activate Braun full open latch release, engage clutch & circuit for power bank
		}
		break;

	case MANUAL_PRECLOSE:
		if (GyoSensorInRange(SensorReading) == TRUE)	//Check vehicle inclination value
		{
			m_manual_step = MANUAL_CLOSE_RELEASE;
		}
		break;
	case MANUAL_CLOSE_RELEASE:
		LatchOperation(LATCH_WITHDRAW);//DO.Latch=TRUE;
		if ( SensorReading.LSM6AngleZ > -INCLINATION_NORMAL_RANGE )	//If inclination value is "X - Y" range (normal/level ground) - activate Braun full open latch release & leave clutch disengaged
		{
			DO.Clutch = FALSE;
		}
		else
		{
			DO.Clutch = TRUE;			 							//If inclination value is "Y - Z" range (nose down extreme) - activate Braun full open latch release, engage clutch & circuit for power bank
		}
		break;
	default:  // IDLE?
		break;
	}
}

#endif /* SRC_DOORCONTROL_H_ */
