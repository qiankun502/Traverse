/*
 * variables.h
 *
 *  Created on: Jul 11, 2024
 *      Author: andrew.henseleit
 *      		Ken Qian
 */

#ifndef SRC_VARIABLES_H_
#define SRC_VARIABLES_H_

#define NO_VEHICLE_CAN
#define VERSION0  9 // Major 0-9
#define VERSION1  0 // Minor 0-9   //version number 0.02    //ver 0.02: Added sensor temperature reading to display.
#define VERSION2  2 // Patch 0-9

#define NOSE_ANGLE_TEST 1
#define PITCH_ANGLE_TEST 2

//#define AUTOCYCLE 0
#define FLASH_USER_START_ADDR   (FLASH_BASE + (FLASH_SECTOR_SIZE * 2))   /* Start @ of user Flash area 0x08040000UL */
#define FLASH_USER_END_ADDR      (FLASH_BASE + (FLASH_SECTOR_SIZE * 3))     /* End @ of user Flash area 0x08060000UL */
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

//static FDCAN_RxHeaderTypeDef RxHeader;
#define SW_Stat 0x60;
#define Dr_Stat 0x40;    //The diag modue use 0x40 and 0x41
#define Dr_Curr 0x41;
#define Dr_Sw_Er 0x300;
#define Dr_REV_OPEN_Er 0x301;
#define Dr_REV_CLOSE_Er 0x302;
#define Dr_To_Er 0x303;
#define Dr_En_Er 0x304;
#define Dr_Encode_To_Er 0x305;
#define Dr_Release_To_Er 0x306;
#define Dr_Unlatch_To_Er 0x307;

#define BAT_LIMIT_L 11.5                        //Battery low limit 11.5V.
#define PI 3.1415926
#define RAD_TO_DEG 57.2958
#define MAX_BUF_SIZE 256
#define FIFO_WATERMARK   31 /*!< FIFO size limit */
#define SAMPLE_LIST_MAX  10U /*!< Max. number of values (X,Y,Z) to be printed to UART */
#define ENCODE_FAST_RANGE  7000        // Encode change fast in these range from full opened.
#define LSM6DSO_SAMPLE_ODR      10.0f /*!< Sample Output Data Rate [Hz] */
#define LSM6DSO_FIFO_MAX_ODR  6600.0f /*!< LSM6DSO FIFO maximum ODR */

#define CMPerCODE  0.0025

#define ENABLE  1 /*!< Enable LSM6DSO FIFO functions */

#define INDICATION_DELAY  100 /* LED is ON for this period [ms]. */
#define DOOR_SPEED_LIMIT_IN_IDLE 400   //200cm/s speed limit under IDLE status
#define DOOR_SPEED_LIMIT1 1000   //1000 cm/s on door open side

#define RAMP_LIGHT_ON_TIME  12000					// Keep the ramp light on for 120 seconds
#define OPEN_BEFORE_CLOSING_TIME 100         //time to open the door for release latch before closing the door.

#define	DOORMANUAL  0    //No power
#define	DOORPOWEROPENING 1
#define	DOORPOWERCLOSING 2
#define    DOORMANUALOPENING 3
#define    DOORMANUALCLOSING 4
#define	DOORFULLOPEN 5
#define	DOORFULLCLOSE 6
#define	DOORREVERSOPEN 7
#define	DOORREVERSCLOSE 8
#define	DOOFAILURE 0xB

#define RAMP_UP 300
#define ENCODEBUFSIZE 21

#define	DOOR_IDLE  0    //No power
#define	DOOR_OPEN 1
#define	DOOR_CLOSE 2
#define	DOOR_STOP 3

#define CAN_FAULT 1
#define ENCODE_FAULT 2               //encode fail, reading out of range.
#define DOORSW_FAULT 3

#define DOOR_RELEASE 1
#define DOOR_LOCK 2
#define DOOR_UNLOCK 3

#define LATCH_EXTRACT 1
#define LATCH_WITHDRAW 2
#define LATCH_DISABLE 3

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Timeout time define
#define DOOR_FINAL_OPEN_TO_TIME    500

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Error definition
#define ERROR_RAMP_SW  100// Both ramp end switches active at the same time
#define RAMP_OC_DEPLOY  101// Ramp overcurrent stops while deploying
#define RAMP_OC_STOW  102//: Ramp overcurrent stops while stowing
#define RAMP_TO_DEPLOY  103//: Ramp timesout while deploying
#define RAMP_TO_STOW  104//: Ramp timesout while stowing

#define  ERROR_KNEEL_SW 200//: Both kneel end switches active at the same time
#define KNEEL_OC_LOWERING 201//: Kneel overcurrents while lowering
#define KNEEL_OC_RAISING 202//: Kneel overcurrents while raising
#define KNEEL_TO_LOWERING 203//: Kneel timesout while lowering
#define KNEEL_TO_RAISING 204//: Kneel timesout while raising

#define DOOR_SW 300//: Both door end active at the same time
#define REV_OPEN 301//: Door reverses open durring operation w/o switch
#define REV_CLOSE 302//: Door reverses close durring operation w/o switch
#define DOOR_TO 303//: Door reverses close durring operation w/o switch
#define DOOR_ENA 304//: Door is disabled but attempted to operate

#define CAN_HW 400  //: CAN bus has issue
#define NO_CAN_COM 401  //: CAN bus has no comunication
#define CAN_SHORT_TO_PWR 402  //: CAN wire is shorted to 12V
#define CAN_SHOTR_TO_GND 403  //: CAN wire is shorted to gnd
#define LOW_BATTERY_VOLTAGE 404  //: Battery voltage dips below 10.25V
#define SWITCH_STUCK 405  //: Braun button is held for more than 120 seconds

#define CONV_SW 500
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define UNRELEASE_TO_ERROR 0x1  //release time out error.
//#define DOOR_START_OPEN_TO_ERROR 0x0002	//door open time out error
#define DOOR_OPEN_TO_ERROR 0x2
#define UNLATCH_TO_ERROR  0x3
#define DOOR_CLOSE_TO_ERROR 0x4
//#define DOOR_ALMOST_OPEN_TO_ERROR 0x5
#define DOOR_FULL_OPEN_TO_ERROR 0x5   //final close error, possible the full door open sw is broken
#define DOOR_FULL_CLOSE_TO_ERROR 0x6

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RAMP_STEW_TO_ERROR 0x0100     //ramp stew timeout error
#define BAT_VOLT_LOW  0x0200						//battery voltage is too low
#define DOOR_ENCODE_ERROR 0x400
#define DOOR_IS_NOT_MOVING 0x800
#define RAMP_NO_STEW_EXPLOY 0x1000
#define CAN_COMM_ERROR 0x2000
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Ramp_Motor_Status
#define NO_RAMP_MOTOR_RUNNING 0
#define RAMP_MOTOR_DEPLOYING 1
#define RAMP_MOTOR_STOWING 2
#define RAMP_MOTOR_LOCKED 3
////Ramp_SW_Data1
//#define RAMP_BOTH_SWITCHES_ACTIVE 0
//#define RAMP_SWITCH1_ACTIVE 0x01
//#define RAMP_SWITCH2_ACTIVE 0x10
//#define RAMP_NO_SWITCHES_ACTIVE 0x11

#define OPEN 1
#define CLOSE 0

#define PARKED 0
#define NOPARKED 1

//Flash frequence;  0:Solide off, 1: solide on,  2: Fast, 3:Slow
#define SOLID_OFF  0
#define SOLID_ON  1
#define FAST_FLASH  2
#define SLOW_FLASH  3

#define DEBOUNCETIME 10    //10x10ms
#define ENCODESPERMETER 31431     //(59047-27616)/1   ;range of the door is around 1 meter,

#define ALMOSTOPENAREA  7000            // The encode reading close to the fully open
#define MAXMOTORCURRENT 15         //15A
#define MOTORCURRENT_FULLY_CLOSED  5         //The motor current when door is fully closed

char mDebugPrint;
uint32_t mDebugPrintCount;
char AUTOCYCLE;
uint8_t TemperatureL, TemperatureH;
float doorspeed_cal;
char flip_flop = 0;

LSM6DSO_Object_t MotionSensor;
volatile uint32_t dataRdyIntReceived;

uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;
__IO uint32_t MemoryProgramStatus = 0;

char PIprintstr[80];
char PRINTF_Flag;
char Enable_PI_Moniter;
char Moto_Status;
char uart_msg[32];
int JTBL=0;
int sleep_wait=0;
int start_sleep=0;
int wait_for_release=0;
uint32_t sleep_wait_count=0;
int can_count=0;
int go_to_sleep_count = 0;
int fault_count = 0;
char fault_code = 0;	//0: No fault, 1:CAN_Chip Fault;
int flashnumber=0;
int DisConv_toggle_count =0;
uint32_t Time_Out_Error;
uint32_t Normal_Error;
uint32_t DTC_Error;
int AutoCycleWaitTime=1;
char UartRecDoneFlag;
int recindex;
char recvbuffer[32];
int msg_come_in_wait_count;
int Motor_Target_Speed;
int Motor_PWM_Current_Count;
//int16_t xyz[4]={0,0,0,0};
//float LSM6DSOtemperature;
//double Angle[3];
//static char dataOut[MAX_BUF_SIZE];
//static uint8_t fifo_full_status = 0;
//static uint16_t num_samples = 0;
//static uint16_t prev_num_samples = 0;
char ChangeMotorSpeedFlag = FALSE;
char go_to_sleep = 0;
char sleeping = 0;
char one_last_send = 0;
char can_rx_int=0;
char send_dSw_stat = 0;
char send_dCur_stat = 0;
char send_swM_stat=0;
char DIchanged;



char Reverse_OP_flag;
char Braun_Op_Switch_UART_flag;                //command of OP from UART
//char error_dSw_send=0;
//char error_dEn_send=0;
//char error_dTo_send=0;
//char error_cTo_send=0;

#define REVERSE_OPEN_DELAY_TIMER 10
#define REVERSE_CLOSE_DELAY_TIMER 10
#define LATCH_BEFORE_MOTOR_OFF_TIMER 100    //wait time to turn off motor after latched.  1 second.
#define LATCH_AFTER_MOTOR_ON_TIMER 50    //wait time to turn on latch after motor on, 0.5 second
#define LATCH_OFF_TIMER  50   //Wait time to turn latch off after start closing.
#define RELEASE_OFF_TIMER 10  //Wait time to turn off release
#define MAGNA_CINCH_WAIT_TIMER   50// wait the cinch to fully close the door 2 seconds


#define  ANGLE_NOSE_CLUTCH_BEFORE_RELEASE_MAX  5
#define  ANGLE_PITCH_CLUTCH_BEFORE_RELEASE_MAX  7
#define  ANGLE_NOSE_CLUTCH_BEFORE_RELEASE_MIN  -5
#define  ANGLE_PITCH_CLUTCH_BEFORE_RELEASE_MIN  -7


//
//typedef struct
//{
//	float Temperatur;
//	float Nose_Angle;
//	float Pitch_Angle;
//} Condition_st;
//Condition_st OP_Condition;
uint32_t Count_OP_Stuck;
uint32_t Backup_RAM_count;
typedef struct
{
	float ADC_Cur_SNS1;
	float ADC_Cur_SNS2;
	float Motor_Cur_Open;
	float Motor_Cur_Close;
	uint16_t Motor_Cur_Open_Raw;
	uint16_t Motor_Cur_Close_Raw;
	float Bump_Strip_Signal;
	uint16_t Door_Encode_Signal;   // min: 27616 (full opened), Max:59047 (full closed)
}ADC_st;
ADC_st* adc;
#define MOTOR_CUR_CALI_SLOP  11.2
#define MOTOR_CUR_CALI_INTERCEPT 0//0.23

typedef struct{
	char encode_error_trig;
	char door_full_open_error_trig;      //if encode almost close and ajar and full open on at the same time.
	char door_ajar_error_trig;
}ERROR_TRIG_ST;
ERROR_TRIG_ST Error_trig;

typedef struct{
char dSw;
char dEn;
char dTo;
char dRevsOpen;
char dRevsClose;
char cTo;
char dEncode;
char releaseTo;
char unlatchTo;
} ERROR_FLAG_COUNT_SEND_ST;
ERROR_FLAG_COUNT_SEND_ST Error_flag;
ERROR_FLAG_COUNT_SEND_ST Error_count;
ERROR_FLAG_COUNT_SEND_ST Error_send;

char update_CAN=0;

typedef struct
{
	float Proportional;
	float Integral;
} PID_st;
PID_st PID;

int GO_TO_SLEEP_COUNT=0;
int m_DoorError=0;
char initialDone;
int UARTIOUpdateCount;
typedef struct
{
	char full_door_open;  //Door_Open
	char Braun_Op_Switch;
	char ERR_CAN;
	char Out_side_handle;
	char Door_ajar;
	char Lock_status_switch;  //Ken: =door_motor_closed?
	char Kneel_disabled;
	char Door_Latched_Switch;
	char ConvDisabled;
} DI_st;
static DI_st DI;
static DI_st Pre_DI;

typedef struct
{
	volatile char Wakeup;
	volatile char Clutch;
	volatile char Latch;				//Magna latch release
	volatile char LEDRed;
	volatile char LEDWhite;
	volatile char LEDWhite_12V;
	volatile char LEDYellow;
	volatile char ReleaseTrig;
	volatile char ReleaseEnable;
	volatile char Lock_Coil;
	volatile char Ramp_Light;
	volatile char Latch_PWM;
	volatile char Latch_DIR;
	volatile char Latch_Disable;
	volatile char Latch_VSO;    //wakeup.

}DO_st;
DO_st DO;

typedef struct
{
	volatile int CUR_SNS1;
	volatile int CUR_SNS2;
	volatile int BumpStrip;
	volatile int Door_encode;
}AI_st;
AI_st AnalogIn;

typedef struct
{
	int ER_RN;
	int RD_EN;
}PWM_st;
PWM_st PWMMotorControl;


//char Pre_Braun_Op_Switch;
//char Pre_Out_side_handle;
char adc_done;
//char tim6_flag;
char buffer[100];
char usb_rx_data[7];
char error;
char diag_send;
char has_can;
char final_can;
char diag_update;

char SWITCH_STUCK_flag;
uint8_t UART2_rxBuffer[1] ;
typedef struct
{
	float LSM6AngleX;
	float LSM6AngleY;
	float LSM6AngleZ;
	int LSM6DSOtemperature;
	float NoseAngle;			//Nose down is Negative, Nose up id positive
	float SideAngle;			//Side left is Negative, Side right is positive
} LSM6SENSOR_READING;
LSM6SENSOR_READING SensorReading;
LSM6SENSOR_READING PreSensorReading;
LSM6SENSOR_READING SensorReading4OP;

typedef struct {
//uint16_t f_open_count ;
uint16_t latch_deb_count;
uint16_t ajar_count ;
uint16_t handle_count ;
uint16_t bump_count ;
uint16_t op_count ;
//uint16_t Release_to_Motor_OFF_count ;
//uint16_t Motor_to_Latch_ON_count ;
//uint16_t Delay_Open_SW_off_count;
uint16_t reverse_open_delay_count;
uint16_t reverse_close_delay_count;
//uint16_t wake_door_count;
//uint16_t reset_error_count;
uint16_t Autocycle_delay_count;
uint16_t general_delay_count;
uint16_t Norma_Open_Wait_count;
uint16_t Latch_Off_Count;
uint16_t Magna_Cinch_Wait_Count;
uint16_t Clutch_to_Motor_On_count;
} Door_Count_st;
Door_Count_st Door_Count;

typedef struct {			//structure of the door status flag
//	char f_open_deb_flag;
	char ajar_deb_flag;
	char handle_deb_flag;
	char bump_deb_flag;
	char op_deb_flag;        //Braun operator switch
	char latch_deb_flag;
	char reverse_open_delay_flag;  	//wait time of reverse open from stop to slow open and speed1 to speed2
	char reverse_close_delay_flag;
	char reset_error_flag;
	char Autocycle_delay_flag;
	char general_delay_flag;
	char Latch_Hold_On_flag;
	char	 Magna_Cinch_Wait_flag;		//wait the cinch time
	char Normal_Open_Wait_flag;

	char Normal_Open_Wait_Done;
//	char f_open_deb_done;
	char ajar_deb_done;
	char handle_deb_done;
	char bump_deb_done;
	char op_deb_done;        //Braun operator switch
	char latch_done;
	char reverse_open_delay_done;	////wait time of reverse open from stop to slow open and speed1 to speed2
	char reverse_close_delay_done;
	char reset_error_done;
	char Autocycle_delay_done;
	char general_delay_done;
	char Latch_Hold_On_done;
	char Magna_Cinch_Wait_done;
	char Clutch_to_Motor_On_flag;
	char Clutch_to_Motor_On_done;
}Door_flag_st;
Door_flag_st Door_flag;

typedef struct
{
	char done;
	char flag;
	uint32_t count;
}TIMEOUT_st;
TIMEOUT_st release_timeout;
TIMEOUT_st unlatch_timeout;
TIMEOUT_st door_open_timeout;
TIMEOUT_st door_close_timeout;
TIMEOUT_st door_final_open_timeout;
TIMEOUT_st door_final_close_timeout;
TIMEOUT_st ramp_open_timeout;
TIMEOUT_st ramp_stow_timeout;
//TIMEOUT_st Door_Close_Timeout;
TIMEOUT_st Position2_to_Close_timeout;
TIMEOUT_st Ajar_Wait;           //release to ajar wait time
TIMEOUT_st Hold_Motor_Fully_Open;
TIMEOUT_st Hold_Clutch_After_Motor_Off;
TIMEOUT_st Hold_Clutch_Before_Motor_On;
TIMEOUT_st Hold_Motor_Before_Latch_On;
TIMEOUT_st Hold_StopMotor_to_CloseDoor;
TIMEOUT_st Hold_Close_Speed1_to_Normal_Close;
TIMEOUT_st Hold_Latch_Time;
TIMEOUT_st Hold_Latch_to_OpenSW_off;
TIMEOUT_st Hold_Motor_On_Full_Closed_Time;
TIMEOUT_st Hold_Motor_Off_Full_Closed_Time;
TIMEOUT_st Hold_Motor_Stop_to_Start_Time;
TIMEOUT_st Wait_RAMP_STOW_SIG_Time;
TIMEOUT_st Lock_Motor_Time;
TIMEOUT_st Release_Trig_Time;
TIMEOUT_st Release_Enable_Time;
TIMEOUT_st Lock_Coil_Time;
TIMEOUT_st Latch_Time;
TIMEOUT_st FullPowerOpenDoorTime;
TIMEOUT_st DisableConv_Time;
TIMEOUT_st Hold_Clutch_Outside_Handle_Time;
TIMEOUT_st Quick_Clutch_Time;
TIMEOUT_st FOB_OPEN_Delay_Time;
TIMEOUT_st Ramp_Light_On_Time;
TIMEOUT_st Ajar_Hold_Time;
TIMEOUT_st Outside_Handle_Wait_Time;

uint16_t m_pulser=0;    //PWM SPEED MIN=3, max=61.  Below gives you GND, above gives you solid 12V
uint16_t p_reset=40;  //RESET START VALUE
uint16_t p_max=65;  //MAX SOFT OPEN SPEED
uint16_t p_final=40;  //COME BACK DOWN ON SOFT START TO THIS
uint16_t rel_pos = 0;
uint16_t run_count = 0;
uint16_t open_count_up =0;
uint16_t closed_count_up =0;

char Upload_Open_0_flag;
char Upload_Close_0_flag;
char Upload_Open_1_flag;
char Upload_Close_1_flag;
char Upload_Open_2_flag;
char Upload_Close_2_flag;
char Upload_Open_3_flag;
char Upload_Close_3_flag;
char Upload_Open_4_flag;
char Upload_Close_4_flag;
char Upload_Open_5_flag;
char Upload_Close_5_flag;
char Upload_Open_6_flag;
char Upload_Close_6_flag;
char Upload_Open_7_flag;
char Upload_Close_7_flag;
char Upload_Open_Pitch_Angle_Test_flag;
char Upload_Open_Nose_Angle_Test_flag;
char Upload_Close_Pitch_Angle_Test_flag;
char Upload_Close_Nose_Angle_Test_flag;
char Upload_Pitch_Unlatch_Time_flag;
char Upload_Nose_Unlatch_Time_flag;
#define TWO_DIMESION_ANGLE  0         																		// true when use the nose and pitch angle
//float NoseAngleArray_Test[] = { 12, 10, 8, 5,-5, -8, -10, -12 };                              //Positive: head up;           Negative:Head down
//float PitchAngleArray_Test[] =  { 12, 10, 8, 5,-5, -8, -10, -12 };                             //Positive: passage side up. Negative: drive side up
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Flat ground angle parameters
int  V_Start_Open_Flat_Ground=	100;
int V_Open_Flat_Ground=	100;
int V_End_Open_Flat_Ground=	80;

int  V_Start_Close_Flat_Ground=	100;
int V_Close_Flat_Ground=	100;
int V_End_Close_Flat_Ground=	100;

int Time_Pull_Door_Latch_Flat_Ground =  40;//30;
int Power_Pull_Door_Latch_Flat_Ground = 75;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float NoseAngleArray_Test[] = {- 180, -12, -9, -5, 0, 5, 9, 12, 180 };                              //Positive: head up;           Negative:Head down
float PitchAngleArray_Test[] = { -180, -12, -9, -5, 0, 5, 9, 12, 180 };                              //Positive: passage side up. Negative: drive side up

//                                            Passage side up--------------------------Flat Ground----------------------------------- Driver Side Up
//Pitch Angle:  MININCLINATION to -12.1,   -12 to -10.1,  -10 to -5.1,   -5 to 0,  0 to 5,  5.1 to 9,    9.1 to 1   12.1 to MAXINCLINATION
int  V_Start_Open_Pitch[8]=	{   100,           100,               	100,        100,         65,          55,      		70,         70};
int V_Open_Pitch[8]=          	{   100,           100,              	 100,        100,         100,      100,          100,		100};
int V_End_Open_Pitch[8]=	    {   55,           	55,               55,        65,		   	100,      100,          100,		100};
int  V_Start_Close_Pitch[8]=	{   100,             100,             	 100,        100,        	  100,      100,        70,		70};
int V_Close_Pitch[8]=				{   100,           100,               100,        100,         100,      100,          100,		100};
//int V_End_Close_Pitch[8]=		{   100,           100,               100,        100,         100,      100,          100,		100};
int V_End_Close_Pitch[8]=		{   50,           50,               50,        50,         50,      50,          50,		50};
int Time_Pull_Door_Latch_Pitch[8] = {20,     20, 				  20, 		  		20, 		100, 	      100,			   100,        100 };             //Pull door back time to unlatch in Nose angle.
int Power_Pull_Door_Latch_Pitch[8] ={20,     20, 			  20, 		  		20,       100,         100,       	100,		100};
//int Time_Pull_Door_Latch_Pitch[8] = {100,     100, 				  100, 		  		100, 		100, 	      100,			   100,        100 };             //Pull door back time to unlatch in Nose angle.
//int Power_Pull_Door_Latch_Pitch[8] ={20,     20, 			 	 20, 		  		20,       100,         100,       	100,		100};

//                                              Tail up <-----------------------------------flat ground----------------------------------->Nose up
//Nose Angle:  MININCLINATION to -12.1,   -12 to -10.1,  -10 to -5.1,   -5 to 0,  0 to 5,  5.1 to 9,    9.1 to 1   12.1 to MAXINCLINATION
int  V_Start_Open_Nose[8]=	{   100,           100,               100,        100,         100,      100,          100,		     100};
int V_Open_Nose[8]=				{   100,        100,               100,         100,          100,      90,           80,			80};
int V_End_Open_Nose[8]=		{   100,        100,               100,         100,        	55,      55,         	 55,			55};
int  V_Start_Close_Nose[8]=	{   50,           50,                60,          60,       	100,        100,        	100,			100};
int V_Close_Nose[8]=				{   85,           85,                90,          90,          100,      100,      		100,			100};
//int V_End_Close_Nose[8]=		{   100,         100,             100,        100,         100,    	100,       		100,			100};
int V_End_Close_Nose[8]=		{   50,         50,             50,        50,         50,    	50,       		50,			50};
int Time_Pull_Door_Latch_Nose[8] = {100,        100, 				100,		100, 			5, 		  5,					5, 				5 };             //Pull door back time to unlatch in Nose angle.
//int Time_Pull_Door_Latch_Nose[8] = {100,     100, 				  100, 		  		100, 		100, 	      100,			   100,        100 };               //Pull door back time to unlatch in Nose angle.
int Power_Pull_Door_Latch_Nose[8] ={ 100,      100,          100,     100,          20,   	 20,                20,			  20};



float NoseAngleArray[] = { -12, -10, -8, -5, 0, 5, 8, 10, 12 };                              //Positive: head up;           Negative:Head down
float PitchAngleArray[] = { -12, -10, -8, -5, 0, 5, 8, 10, 12 };//{12,5,-5,-12};       //Positive: passage side up. Negative: drive side up
//V_Start_Open[pitch][nose];
//open_door_speed_1 = V_Start_Open[pitchanglezone][noseanglezone];/
//V_Start_Open[8][8]: ={{(>10),(8 to 0),(4 to 8),(0 to 4),(-4 to0),(-8 to -4),( -10 to -8) ,(-10 to -8), (<-10)},{},{},{},{},....{}
//for example:
//int V_Start_Open[8][8]= {{100,              100,            100,          100,  100,100,               100,           100},{90,100,100,100,100,100,100,100},{80,90,100,100,100,100,100,100},{70,80,90,100,100,100,100,100},........{...}}


//       						                       Pitch Angle:  -12 to -10,   -10 to -8,   -8 to -5,   -5 to 0,      0 to 5,    5 to 8,     8 to 10,     10 to 12
int V_Start_Open[8][8] =
		/* Nose_Angle_N12_N10*/  		         { {   100,           100,               100,        100,         100,      100,          100,            100},
		/* Nose_Angle_N10_N8 */	  					{    90,            100,               100,        100,         100,      100,          100,            100},
		/* Nose_Angle_N8_N5*/	  					    {   80,               90,               100,        100,         100,      100,          100,            100},
		/* Nose_Angle_N5_0*/	  					        {   70,               80,                90,         100,         100,      100,          100,            75},
		/* Nose_Angle_0_5*/    					        {   60,               70,                80,          90,          100,      100,           100,          75},
		/* Nose_Angle_5_8*/	  					        {   50,               60,                70,           80,           90,       100,          100,            100},
		/* Nose_Angle_8_10*/    					        {   40,               50,                60,           70,           80,        90,           100,            100},
		/* Nose_Angle_10_12*/	  					    {   30,               40,                50,           60,           70,        80,            90,             100}};

//       						                  Pitch Angle:  -12 to -10,   -10 to -8,   -8 to -5,   -5 to 0,      0 to 5,    5 to 8,     8 to 10,     10 to 12
int V_Open[8][8] =
		/* Nose_Angle_N12_N10*/   		 		{  {   90,           95,             100,        100,         100,      100,          100,            100},
		/* Nose_Angle_N10_N8 */		  				{    80,          85,               95,        100,         100,      100,          100,            100},
		/* Nose_Angle_N8_N5*/		  					{   75,           85,               90,        100,         100,      100,          100,            100},
		/* Nose_Angle_N5_0*/		  						{   70,           80,               90,         95,          100,      100,          100,            100},
		/* Nose_Angle_0_5*/    	    					{   65,           75,               85,          90,           95,      100,          100,           100},
		/* Nose_Angle_5_8*/	  		  					{   60,           70,               80,          85,           90,       95,           100,            100},
		/* Nose_Angle_8_10*/        						{   55,           65,               75,          85,          90,        95,           100,            100},
		/* Nose_Angle_10_12*/		  					{   50,           60,               70,           80,          85,       90,            95,             100}};

//       						                  Pitch Angle:  -12 to -10,   -10 to -8,   -8 to -5,   -5 to 0,      0 to 5,    5 to 8,     8 to 10,     10 to 12
int V_End_Open[8][8] =
		/* Nose_Angle_N12_N10*/   		 		{  {   80,           85,             90,        95,         100,      100,          100,            100},
		/* Nose_Angle_N10_N8 */		  				{    75,          80,               85,        90,         95,      100,          100,            100},
		/* Nose_Angle_N8_N5*/		  					{   65,           70,               75,        80,         85,        90,            95,            100},
		/* Nose_Angle_N5_0*/		  					    {   65,           70,               75,        80,         85,        90,            95,            100},
		/* Nose_Angle_0_5*/    	    					{   65,           70,               75,        80,         85,        90,             95,           100},
		/* Nose_Angle_5_8*/	  		  					{   60,           65,               70,        75,         80,        85,              90,            95},
		/* Nose_Angle_8_10*/        					    {   55,           60,               70,        75,         80,        85,              90,            95},
		/* Nose_Angle_10_12*/		  					{   50,           55,               60,        65,        70,         75,            80,             85}};

//       						                     Pitch Angle:  -12 to -10,   -10 to -8, |  -8 to -5, |  -5 to 0,   |   0 to 5,  |  5 to 8,  |   8 to 10,   |  10 to 12
int V_Start_Close[8][8] =
		/* Nose_Angle_N12_N10*/  		 		{ {   100,           100,               90,        80,         70,   		   60,          50,            40},
		/* Nose_Angle_N10_N8 */	  					{    100,           90,               85,        80,         75,     		60,          50,            40},
		/* Nose_Angle_N8_N5*/	  						{   95,              85,               80,        75,         70,            65,          55,            40},
		/* Nose_Angle_N5_0*/	  							{   95,              85,               80,        75,         70,            65,          55,            40},
		/* Nose_Angle_0_5*/    	   						{   95,              85,               80,        75,         70,            65,          55,            40},
		/* Nose_Angle_5_8*/	  	  						{   90,              80,               75,        70,         65,            60,          55,            40},
		/* Nose_Angle_8_10*/       						{   90,              80,               75,        70,         65,            60,          55,            40},
		/* Nose_Angle_10_12*/	  						{   90,              80,               75,        70,         65,            60,          55,            40}};

//       						                     Pitch Angle:  -12 to -10,   -10 to -8,   -8 to -5,   -5 to 0,    0 to 5,    5 to 8,    8 to 10,   10 to 12
int V_Close[8][8] =
		/* Nose_Angle_N12_N10*/   		 		{ {   100,           100,               90,        80,         70,      	60,          50,            40},
		/* Nose_Angle_N10_N8 */		  				{    100,           90,               85,        80,         75,     	 60,          50,            40},
		/* Nose_Angle_N8_N5*/		  					{   95,              85,               80,        75,         70,     	 65,          55,            40},
		/* Nose_Angle_N5_0*/		  						{   95,              85,               80,        75,         70,   	 	 65,          55,            40},
		/* Nose_Angle_0_5*/    	    					{   95,              85,               80,        75,         70,   	     65,           55,           40},
		/* Nose_Angle_5_8*/	  		  					{   90,              80,               75,        70,         65,   	     60,          55,            40},
		/* Nose_Angle_8_10*/        						{   90,              80,               75,        70,         65,   		 60,          55,            40},
		/* Nose_Angle_10_12*/		  					{   90,              80,               75,        70,         65,   	     60,          55,             40}};

//       						                     Pitch Angle:  -12 to -10,   -10 to -8,     -8 to -5,    -5 to 0,    0 to 5,     5 to 8,    8 to 10,     10 to 12
int V_End_Close[8][8] =
		/* Nose_Angle_N12_N10*/  		 		{ {   		100,           100,            100,        90,            80,          70,          60,            50},
		/* Nose_Angle_N10_N8 */	  					{  100,           100,            100,        100,          90,          80,          70,            60},
		/* Nose_Angle_N8_N5*/	  						{  100,           100,            100,        100,         100,         90,          80,            70},
		/* Nose_Angle_N5_0*/	  							{  100,           100,            100,        100,         100,         90,          80,            70},
		/* Nose_Angle_0_5*/    	   						{  100,           100,            100,        100,         100,         90,           85,           80},
		/* Nose_Angle_5_8*/	  	  						{  100,           100,            100,        100,         100,       100,          95,            90},
		/* Nose_Angle_8_10*/       						{  100,           100,            100,        100,         100,       100,          100,            95},
		/* Nose_Angle_10_12*/	  						{  100,           100,            100,        100,         100,       100,          100,            100}};

//int V_End_Close[8][8]={{100,100,100,90,80,70,60,50},{100,100,100,100,90,80,70,60},{100,100,100,100,100,90,80,70},{100,100,100,100,100,90,80,70},
//										{100,100,100,100,100,90,85,80},{100,100,100,100,100,100,95,90},{100,100,100,100,100,100,100,95},{100,100,100,100,100,100,100,100}};

float No_Release_Open_Angle=8.0;          //if nose angle over this, no reverse open is required before release latch.


uint8_t	RxData[8]={0,0,0,0,0,0,0,0};
uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t SwitchData[8]={0,0,0,0,0,0,0,0};
//uint16_t ID[8] = {0x01,0x10,0x100,0x101,0x102,0x103,0x104,0x105};  //Andrew
uint16_t ID[10] = {0x20,0x10,0x100,0x101,0x102,0x103,0x104,0x1,0x11,0x51};  //Updated from CAN DB

static FDCAN_RxHeaderTypeDef RxHeader;
static FDCAN_TxHeaderTypeDef SW_Stat_Header;
static FDCAN_TxHeaderTypeDef Dr_Stat_Header;
static FDCAN_TxHeaderTypeDef Dr_Curr_Header;
static FDCAN_TxHeaderTypeDef Dr_Sw_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_En_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_To_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_RevsOpen_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_RevsClose_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_Encode_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_Release_To_Er_Header;
static FDCAN_TxHeaderTypeDef Dr_Unlatch_To_Er_Header;
/////////////////////////////////////////////////////////////////////////////
typedef struct EEPROMCOUNT
 {
	int32_t maint_saved;
	int32_t Saved;
	int32_t open_count;
	int32_t close_count;
	int32_t hndl_count;
	int32_t switch_count;
	int32_t reverse_count;
	int32_t overcurrent_count;
	int32_t learned_enc;
	int32_t full_opened_encode;
	int32_t full_closed_encode;
	int32_t pre_warning_days;
} EEPROMCOUNT_t;
EEPROMCOUNT_t eepromcount_t;
EEPROMCOUNT_t pre_eepromcount_t;

typedef struct LED_Struct
{
	 char ParkingWarning;
	 char AngleWarning;
	 char ErrorWarning;
	 char FlashFreq;             //Flash frequence;  0: solid off, 1: solid on,  2: Fast flash, 3:Slow flash
	 int FlashCount;				//Flash count for each cycle;
	 int ON_Time;
	 int OFF_Time;
	 int flashedcount;
} LED_t;
LED_t WhiteLED;
LED_t RedLED;
LED_t YellowLED;
int LEDTimeCount;        // the times has been flashed;

typedef enum {
	UNKNOWN = 0,
	NOTPARKING,
	CLINATIONOVERRANGE,
	TEMPERATUREOVERRANGE,
	RAMPOUT,
	OP_ALLOW
} DOOR_OP_CONDITION_st;
DOOR_OP_CONDITION_st door_op_cond;


typedef enum
{
	DOOR_MONITOR = 0,
	PREOPEN    = 1,
	START_OPEN  = 2,
	NORMAL_OPEN  = 3,
	ALMOST_OPENED  = 4,
	FULLY_OPENED  =5,

	PRECLOSE =6,
	START_CLOSING =7,
	NORMAL_CLOSE= 8,
	ALMOST_CLOSED =9,
	FULLY_CLOSED= 10,

	AUTOCYCLE_WAIT_CLOSE_DELAY = 11,
	AUTOCYCLE_WAIT_OPEN_DELAY = 12,

	DOORBUMPSTRIPACTIVATE =13,
	DOOROPENSTUCKED =14,							//The door is stuck during opening.
	REVERSEOPEN = 15,
	REVERSCLOSE = 16,
	STEP_AFTER_TO = 17,

	SLOW_OPEN =18,
	SLOW_CLOSE = 19,
	OUTHANDLE_ACTIVATED = 20,
	AFTER_OUTHANDLE_ACTIVATED = 21

} Door_Contol_Step;
Door_Contol_Step m_doorstep;

const char STEP_STRING[][32] = {
    "Door Idle", "Pre-Open", "Start Open", "Normal Open", "Almost Opened", "Full Opened",
	"Pre-Close", "Start Close","Normal Close","Almost Closed", "Fully Closed", "Auto Cycle Wait Close",
	"Auto Cycle Wait Open", "Bump Strip Event", "Unknow", "Revers Open", "Reverse Close","OUTHANDLE_ACTIVATED","AFTER OUTHANDLE_ACTIVATED"};


typedef enum
{
	MANUAL_DOOR_IDLE =0,
	MANUAL_PREOPEN =1,
	MANUAL_OPEN_RELEASE=2,
	MANUAL_OPENING =3,

	MANUAL_PRECLOSE =11,
	MANUAL_CLOSE_RELEASE =12,
	MANUAL_CLOSING =13,

}MANUEL_CONTROL_STEPS;
MANUEL_CONTROL_STEPS m_manual_step;

char flag_Need_Full_Power_Motor;
//float ADC_Cur_SNS1;
//float ADC_Cur_SNS2;
float Bump_Strip_Signal;
uint16_t EncodeBuf[21];
uint16_t Full_Opened_Encode=27000;
uint16_t Full_Closed_Encode=60000;

#define MOTORCUROFFSET1 4080
#define MOTORCUROFFSET2 0.48
//#define ANGLE_CLINATION_MAX 45   //10 degree
//#define ANGLE_CLINATION_MIN -45	 //7 degree.

#define OPEN_DOOR_SPEED_0 1.5           //1.5cm/s
#define OPEN_DOOR_SPEED_1 2			 //2.0cm/s
#define OPEN_DOOR_SPEED_2 1			 //1.5cm/s

#define MAX_SLEEP_COUNT 1500      //15 second to go to sleep mode.

#define USE_PID 0
#define   KP  0.1
#define 	KI  1

typedef struct
{
	int ANGLE_CLINATION_MAX;
	int ANGLE_CLINATION_MIN;

}CONFIG_VALUE_st;
CONFIG_VALUE_st Config_Value;




int open_door_speed_1;
int open_door_speed_2;
int open_door_speed_3;
int close_door_speed_1;
int close_door_speed_2;
int close_door_speed_3;
int open_door_speed_unlatch;
int open_door_time_unlatch;

float CurrentDoorPower;
#define CLOSE_DOOR_SPEED_0 2
#define CLOSE_DOOR_SPEED_1 2
#define CLOSE_DOOR_SPEED_2 4


int door_speed_adjust_array[7]={-30,-20,-10,0,10,20,30};            //speed compensation for the different inclination.
int door_speed_compensation;



typedef struct
{
	char lock_status;
	char kneel_status;
	char park_stat;
	char operator_stat;
	char buckle_stat;
	char ramp_state;
	char door_stat;
	char ramp_motor_state;
	char fob_data;
	char fob_nearby;
	char Motor_data;
	char kneel_switch;
	char ramp_switch;
	char Error_Stow;
	char Error_Deploy;
	char Error_Kneel_SW;
	char Unkneel_Switch;
	char Error_Door_Stat;
	char Year;
	char Month;
	char Day;
	char Hour;
	char Minute;
	char Second;
	char Fob_Op_Cmd;
	float  Battery_Voltage;
	uint16_t Battery_Voltage_int;
//	uint16_t pre_waring_date;
	char Error_Low_Battery;

} CAN_IN;
CAN_IN CAN_Data;
CAN_IN PreCAN_Data;
char unpressed;


#define SWITCH_STEW_ACTIVE  1
#define SWITCH_DEPLOY_ACTIVE 2
#define NO_SWITCH_ACTIVE    3
#define BOTH_SWITCHS_ACTIVE   4

#define NO_MOTOR_RUNNING   0
#define KNEELING  1
#define UNKNEELING 2
#define MOTOR_LEADS_LOCKED    3

#define SWITCH_NOT_ACTIVE  0
#define SWITCH_ACTIVE   1

#define SWITCH_NOT_ACTIVE  0
#define SWITCH_ACTIVE  1


char m_SendIOstatus_Enable;
//
//char ramp_timeout_flag;
//char bump;
//char prev_bump;
//char operate;
//int can_sleep_count;
char bus_sleep;
//char ramp_moving;
//char prev_ramp;
//char bumper;

char sleep_flag;
//char release_timeout_flag;
//char door_timeout_flag;
//char unlatch_timeout_flag;
//int try_release;
//int unlatch_timeout_count=0;
char can_sleep_flag;
//int learned_enc;
//
//char soft_open_flag;
//char soft_close_flag;
int can_sleep_count;
//int close_oc_count=0;
//int open_oc_count=0;
//int ramp_timeout_count=0;
int sleep_count;
//int soft_open_count;
//int soft_close_count;
//int prev_close_count;
//int prev_switch_count;
//int prev_hndl_count;
int unlock_count;
int manual_count;
//int prev_reverse_count;
//int prev_open_count=0;
//
//char oc_reversed;
//char close_oc_flag;
//char open_oc_flag;
//char oc_reversed;
char unlock_timeout_flag;
//char controller_op;
char status_update_flag;
char wake_door_flag;
//char manual_flag;
//char handle_flag;

char maint_reset_flag;
int maint_reset_count;
char ul_prev_num;

char maint_set;
char stat_countup;


int door_timeout_count=0;
int cinch_timeout_count=0;
int release_timeout_count=0;
int unlock_timeout_count=0;
int status_update_count=0;
//int maint_counter;
int learned_enc;           //Ken: Encode reading when door fully closed , Max reading
int encoder_data;
int prev_enc;
int full_open_pos;
int cinch_data;


char r_move_prev;   //ramp move previous.
char prev_door;
char startup;
char sleep;
char was_sleep;

char debounce_flag;
int debounce_count;
char debounce_done;
//
////*******DO variable**************///
//char door_motor_closed;
//char door_motor_open;
//char clutch;
//char unlock_out;
//char handle;
//char operate_sw;
//char unlatch;
//char maint_req;
////*******DI variable**************///
char door_full_closed;       //ajar
//char can_err;
//char braun_sw_pressed;
//char handle_sw_pressed;
/////////////////////////////////
char Gyro_Sensor_Error_Flag;
int testcount;
uint16_t m_doorspeed;   //Motor speed

#define DOOR_OPEN_TIMEOUT   15000
#define DOOR_CLOSE_TIMEOUT   15000
#define MAX_ENCODE_ERROR  60000
#define MIN_ENCODE_ERROR 10000
#define RELEASE_TIMEOUT 200
#define UNLATCH_TIMEOUT 200

void start_OP_Door(DI_st din,uint16_t door_encode);
#endif /* SRC_VARIABLES_H_ */
