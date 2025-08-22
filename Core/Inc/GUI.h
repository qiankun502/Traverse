/*
 * GUI.h
 *
 *  Created on: Apr 19, 2025
 *      Author: Ken.Qian
 */

#ifndef SRC_GUI_H_
#define SRC_GUI_H_
#include "variables.h"
#include "main.h"
//#include "functions.h"

extern UART_HandleTypeDef huart2;
extern uint8_t door_stat;
void AddCharToUartBuf(uint8_t recchar) {
	if (recchar == '#') {
		UartRecDoneFlag = FALSE;
		memset(recvbuffer, 0, 32);
		recindex = 0;
	}
	if ((recindex == 0)
			|| ((recindex > 0) && (recvbuffer[recindex - 1] != '\n'))) {
		recvbuffer[recindex] = recchar;
		recindex++;
	}
//	if((recchar == '\n')||(recchar == '\r'))
	if ((recchar == '\r') || (recchar == '\n')) {
		memcpy(uart_msg, recvbuffer, 32);
		UartRecDoneFlag = TRUE;
	}
}

void processUartMsg(char *msg)
{
	msg_come_in_wait_count = 2;
	if ((msg[1] == 'R') && (msg[2] == 'u') && (msg[3] == 'n')) //"Run" to start Pi monitor.
	{
		Enable_PI_Moniter = TRUE;
		mDebugPrintCount = 0;
	}
	if ((msg[1] == 'S') && (msg[2] == 't') && (msg[3] == 'o')
			&& (msg[4] == 'p'))  //"Stop" to stop Pi monitor.
	{
		Enable_PI_Moniter = FALSE;
		//Set_Lsm6dso_Sleep_Mode(TRUE);//////////////
		//LSM6DSO_ACC_Disable(&MotionSensor);
	}

	if ((msg[1] == 's') && (msg[2] == 't') && (msg[3] == 'a')) //"Start" to start the test.
	{
		PRINTF(" Braun_Op_Pressed\r\n");
		Braun_Op_Pressed();
		Braun_Op_Switch_UART_flag = TRUE;
		//Set_Lsm6dso_Sleep_Mode(FALSE);//////////////////
	}

	if (msg[1] == 'A')   									//auto cycle on/off
			{
		printf("Autocyle=%c.\r\n", msg[3]);
		if ((msg[3] == '0') && (msg[4] == '0') && (msg[5] == '0'))
			AUTOCYCLE = FALSE;
		else {
			AUTOCYCLE = TRUE;
			AutoCycleWaitTime = str_to_Int(msg, strlen(msg)) * 32;  //32x32=1s
		}
	}

	//"#C;col,row,group,value;\r": Download_Test_Page_to_Board
		if (msg[1] == 'C') {
			char *pt;
			pt = strtok(msg, ";");
			printf("%s\n", pt);			//#B
			pt = strtok(NULL, ",");
			int group = atoi(pt);   //0:Start open;  1: opening; 2: end open; 3: star close; 4:closing; 5: end close; 6: unlatch time
			pt = strtok(NULL, ",");
			int angle = atoi(pt); //         angle: 0: >12;   1: 9 to 12;  2:5 to 9;........6: -12 to -9; 7: <-12
			pt = strtok(NULL, ",");
			int phase = atoi(pt); //    8:Pitch;   9:Nose    printf("%s\n", pt);	//0
			pt = strtok(NULL, ",");
			int setvalue = atoi(pt); //       printf("%s\n", pt);	//100
			pt = strtok(NULL, ",");
			if (group ==8)
			{
				switch (phase)
				{
					case 1:
						V_Start_Open_Pitch[angle] = setvalue;
						break;
					case 2:
						V_Open_Pitch[angle] = setvalue;
						break;
					case 3:
						V_End_Open_Pitch[angle] = setvalue;
						break;
					case 5:
						V_Start_Close_Pitch[angle] = setvalue;
						break;
					case 6:
						V_Close_Pitch[angle] = setvalue;
						break;
					case 7:
						V_End_Close_Pitch[angle] = setvalue;
						break;
					case 8:
						Time_Pull_Door_Latch_Pitch[angle] = setvalue;
						break;
					default:
						break;
				}
			}
			if (group == 9)
			{
				switch (phase)
					{
						case 1:
							V_Start_Open_Nose[angle] = setvalue;
							break;
						case 2:
							V_Open_Nose[angle] = setvalue;
							break;
						case 3:
							V_End_Open_Nose[angle] = setvalue;
							break;
						case 5:
							V_Start_Close_Nose[angle] = setvalue;
							break;
						case 6:
							V_Close_Nose[angle] = setvalue;
							break;
						case 7:
							V_End_Close_Nose[angle] = setvalue;
							break;
						case 8:
							Time_Pull_Door_Latch_Nose[angle] = setvalue;
							break;
						default:
							break;
					}

			}
		}

	//"#B;7,5,0,90;\r": download the value from the display
	if (msg[1] == 'B') {
		char *pt;
		pt = strtok(msg, ";");
		printf("%s\n", pt);			//#B
		pt = strtok(NULL, ",");
		int groupname = atoi(pt);
//        char groupname[4];
//        strcpy(groupname,pt);
//        printf("%s\n", groupname);	//VSO
		pt = strtok(NULL, ",");
		int nosegroup = atoi(pt); //        printf("%s\n", pt);	//1
		pt = strtok(NULL, ",");
		int pitchgroup = atoi(pt); //        printf("%s\n", pt);	//0
		pt = strtok(NULL, ",");
		int setvalue = atoi(pt); //       printf("%s\n", pt);	//100
		pt = strtok(NULL, ",");
		switch (groupname) {
		case 1:
			V_Start_Open[pitchgroup][nosegroup] = setvalue;
			break;
		case 2:
			V_Open[pitchgroup][nosegroup] = setvalue;
			break;
		case 3:
			V_End_Open[pitchgroup][nosegroup] = setvalue;
			break;
		case 5:
			V_Start_Close[pitchgroup][nosegroup] = setvalue;
			break;
		case 6:
			V_Close[pitchgroup][nosegroup] = setvalue;
			break;
		case 7:
			V_End_Close[pitchgroup][nosegroup] = setvalue;
			break;
		}
	}

	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '1'))  //upload cmd #b;1;\r
	{
		Upload_Open_0_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '2'))  //upload cmd
	{
		Upload_Open_1_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '3'))  //upload cmd
	{
		Upload_Open_2_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '4'))  //upload cmd
	{
		Upload_Open_3_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '5'))  //upload cmd
	{
		Upload_Open_4_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '6'))  //upload cmd
	{
		Upload_Open_5_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '7'))  //upload cmd
	{
		Upload_Open_6_flag = TRUE;
	}
	if ((msg[1] == 'b') && (msg[2] == ';') && (msg[3] == '8'))  //upload cmd
	{
		Upload_Open_7_flag = TRUE;
	}

	if ((msg[1] == 'A') && (msg[2] == 'T')&& (msg[3] == ';') && (msg[4] == 'P'))  //#AT;P   Pitch Angle Test recipe upload cmd
	{
		Upload_Open_Pitch_Angle_Test_flag = TRUE;
	}
	if ((msg[1] == 'A')&& (msg[2] == 'T')  && (msg[3] == ';') && (msg[4] == 'N'))  //#AT;N   Nose Angle Test recipe upload cmd
	{
		Upload_Open_Nose_Angle_Test_flag = TRUE;
	}
}

int str_to_Int(char *msg, int length) {
	char str_in[16];
	int result;
	for (int i = 3; i < length - 1; i++) {
		if ((msg[i] != '\r') || (msg[i] != '\n'))
			str_in[i - 3] = msg[i];
		str_in[i - 2] = 0;
	}

	result = atoi(str_in);
	return result;
}

/***
 * SendIOstatus: Send the status to the Pi for display.
 */
void SendIOstatus(int msgcount)
{
	if (Enable_PI_Moniter == FALSE)
		return;

	char tempstring[100];
	char *str_Status;
	if (PRINTF_Flag == TRUE)
	{
		HAL_UART_Transmit(&huart2, PIprintstr, strlen(PIprintstr), 200);
		PRINTF_Flag = FALSE;
		return;
	}
	switch (msgcount) {
	case 1:             //IOstatuse
		sprintf(tempstring,
				"IOStatus-1;%d;%d;%d;%d;%d;%d;%d;%d;%.2f;%.2f;%.2f;%d;%s;\r", //%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%d;%d;%d;%d\r",
				//sprintf(tempstring,"IOStatus;%d;%d;%d;%d;%d;%d;%d;%d;%.2f;%.2f;%.2f;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\r",
				DI.Braun_Op_Switch, DI.ConvDisabled, DI.Door_ajar,
				DI.Door_Latched_Switch, DI.Kneel_disabled,
				DI.Lock_status_switch, DI.Out_side_handle, DI.full_door_open,
				adc->Motor_Cur_Open, adc->Motor_Cur_Close, adc->Bump_Strip_Signal,
				adc->Door_Encode_Signal, STEP_STRING[m_doorstep]);
//						SensorReading.LSM6AngleX,SensorReading.LSM6AngleY,SensorReading.LSM6AngleZ, SensorReading.NoseAngle, SensorReading.SideAngle,doorspeed_cal,
//						open_door_speed_1,open_door_speed_2,open_door_speed_3,close_door_speed_1,close_door_speed_2,close_door_speed_3,Normal_Error,door_stat);
		HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);

		break;
	case 2:
		sprintf(tempstring,
				"IOStatus-2;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r",
				SensorReading.LSM6AngleX, SensorReading.LSM6AngleY,
				SensorReading.LSM6AngleZ, SensorReading.NoseAngle,
				SensorReading.SideAngle, doorspeed_cal, open_door_speed_1,
				open_door_speed_2, open_door_speed_3, close_door_speed_1,
				close_door_speed_2, close_door_speed_3, Normal_Error,
				door_stat, SensorReading.LSM6DSOtemperature,open_door_speed_unlatch,open_door_time_unlatch );
		HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		break;

	case 3:			   //
		char version1, version2;
		version1 = VERSION1;
		version2 = VERSION2;
		sprintf(tempstring, "Output;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r",
				DO.Clutch, DO.Latch, DO.ReleaseEnable, DO.LEDWhite,
				DO.LEDYellow, DO.LEDRed, CAN_Data.park_stat,
				CAN_Data.kneel_status, CAN_Data.ramp_state, DO.ReleaseTrig,
				DO.Lock_Coil,version1,version2);
		HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		break;

	case 4:
		if (Upload_Open_0_flag == TRUE) {
			sprintf(tempstring,
					"OpenV1;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
					V_Start_Open[0][0], V_Start_Open[0][1], V_Start_Open[0][2],
					V_Start_Open[0][3], V_Start_Open[0][4], V_Start_Open[0][5],
					V_Start_Open[0][6], V_Start_Open[0][7], V_Open[0][0],
					V_Open[0][1], V_Open[0][2], V_Open[0][3], V_Open[0][4],
					V_Open[0][5], V_Open[0][6], V_Open[0][7], V_End_Open[0][0],
					V_End_Open[0][1], V_End_Open[0][2], V_End_Open[0][3],
					V_End_Open[0][4], V_End_Open[0][5], V_End_Open[0][6],
					V_End_Open[0][7]);
			Upload_Open_0_flag = FALSE;
			Upload_Close_0_flag = TRUE;
			HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		}
		if (Upload_Open_1_flag == TRUE) {
			sprintf(tempstring,
					"OpenV2;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
					V_Start_Open[1][0], V_Start_Open[1][1], V_Start_Open[1][2],
					V_Start_Open[1][3], V_Start_Open[1][4], V_Start_Open[1][5],
					V_Start_Open[1][6], V_Start_Open[1][7], V_Open[1][0],
					V_Open[1][1], V_Open[1][2], V_Open[1][3], V_Open[1][4],
					V_Open[1][5], V_Open[1][6], V_Open[1][7], V_End_Open[1][0],
					V_End_Open[1][1], V_End_Open[1][2], V_End_Open[1][3],
					V_End_Open[1][4], V_End_Open[1][5], V_End_Open[1][6],
					V_End_Open[1][7]);
			Upload_Open_1_flag = FALSE;
			Upload_Close_1_flag = TRUE;
			HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		}
		if (Upload_Open_2_flag == TRUE) {
			sprintf(tempstring,
					"OpenV3;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
					V_Start_Open[2][0], V_Start_Open[2][1], V_Start_Open[2][2],
					V_Start_Open[2][3], V_Start_Open[2][4], V_Start_Open[2][5],
					V_Start_Open[2][6], V_Start_Open[2][7], V_Open[2][0],
					V_Open[2][1], V_Open[2][2], V_Open[2][3], V_Open[2][4],
					V_Open[2][5], V_Open[2][6], V_Open[2][7], V_End_Open[2][0],
					V_End_Open[2][1], V_End_Open[2][2], V_End_Open[2][3],
					V_End_Open[2][4], V_End_Open[2][5], V_End_Open[2][6],
					V_End_Open[2][7]);
			Upload_Open_2_flag = FALSE;
			Upload_Close_2_flag = TRUE;
			HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		}

		if (Upload_Open_3_flag == TRUE) {
			sprintf(tempstring,
					"OpenV4;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
					V_Start_Open[3][0], V_Start_Open[3][1], V_Start_Open[3][2],
					V_Start_Open[3][3], V_Start_Open[3][4], V_Start_Open[3][5],
					V_Start_Open[3][6], V_Start_Open[3][7], V_Open[3][0],
					V_Open[3][1], V_Open[3][2], V_Open[3][3], V_Open[3][4],
					V_Open[3][5], V_Open[3][6], V_Open[3][7], V_End_Open[3][0],
					V_End_Open[3][1], V_End_Open[3][2], V_End_Open[3][3],
					V_End_Open[3][4], V_End_Open[3][5], V_End_Open[3][6],
					V_End_Open[3][7]);
			Upload_Open_3_flag = FALSE;
			Upload_Close_3_flag = TRUE;
			HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		}

		if (Upload_Open_4_flag == TRUE) {
			sprintf(tempstring,
					"OpenV5;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
					V_Start_Open[4][0], V_Start_Open[4][1], V_Start_Open[4][2],
					V_Start_Open[4][3], V_Start_Open[4][4], V_Start_Open[4][5],
					V_Start_Open[4][6], V_Start_Open[4][7], V_Open[4][0],
					V_Open[4][1], V_Open[4][2], V_Open[4][3], V_Open[4][4],
					V_Open[4][5], V_Open[4][6], V_Open[4][7], V_End_Open[4][0],
					V_End_Open[4][1], V_End_Open[4][2], V_End_Open[4][3],
					V_End_Open[4][4], V_End_Open[4][5], V_End_Open[4][6],
					V_End_Open[4][7]);
			Upload_Open_4_flag = FALSE;
			Upload_Close_4_flag = TRUE;
			HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
		}

			if (Upload_Open_5_flag == TRUE) {
				sprintf(tempstring,
						"OpenV6;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Open[5][0], V_Start_Open[5][1],
						V_Start_Open[5][2], V_Start_Open[5][3],
						V_Start_Open[5][4], V_Start_Open[5][5],
						V_Start_Open[5][6], V_Start_Open[5][7], V_Open[5][0],
						V_Open[5][1], V_Open[5][2], V_Open[5][3], V_Open[5][4],
						V_Open[5][5], V_Open[5][6], V_Open[5][7],
						V_End_Open[5][0], V_End_Open[5][1], V_End_Open[5][2],
						V_End_Open[5][3], V_End_Open[5][4], V_End_Open[5][5],
						V_End_Open[5][6], V_End_Open[5][7]);
				Upload_Open_5_flag = FALSE;
				Upload_Close_5_flag = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
			}

			if (Upload_Open_6_flag == TRUE) {
				sprintf(tempstring,
						"OpenV7;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Open[6][0], V_Start_Open[6][1],
						V_Start_Open[6][2], V_Start_Open[6][3],
						V_Start_Open[6][4], V_Start_Open[6][5],
						V_Start_Open[6][6], V_Start_Open[6][7], V_Open[6][0],
						V_Open[6][1], V_Open[6][2], V_Open[6][3], V_Open[6][4],
						V_Open[6][5], V_Open[6][6], V_Open[6][7],
						V_End_Open[6][0], V_End_Open[6][1], V_End_Open[6][2],
						V_End_Open[6][3], V_End_Open[6][4], V_End_Open[6][5],
						V_End_Open[6][6], V_End_Open[6][7]);
				Upload_Open_6_flag = FALSE;
				Upload_Close_6_flag = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
			}

			if (Upload_Open_7_flag == TRUE) {
				sprintf(tempstring,
						"OpenV8;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Open[7][0], V_Start_Open[7][1],
						V_Start_Open[7][2], V_Start_Open[7][3],
						V_Start_Open[7][4], V_Start_Open[7][5],
						V_Start_Open[7][6], V_Start_Open[7][7], V_Open[7][0],
						V_Open[7][1], V_Open[7][2], V_Open[7][3], V_Open[7][4],
						V_Open[7][5], V_Open[7][6], V_Open[7][7],
						V_End_Open[7][0], V_End_Open[7][1], V_End_Open[7][2],
						V_End_Open[7][3], V_End_Open[7][4], V_End_Open[7][5],
						V_End_Open[7][6], V_End_Open[7][7]);
				Upload_Open_7_flag = FALSE;
				Upload_Close_7_flag = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
			}
			break;

			case 5:
			if (Upload_Close_0_flag == TRUE) {
				sprintf(tempstring,
						"CloseV1;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[0][0], V_Start_Close[0][1],
						V_Start_Close[0][2], V_Start_Close[0][3],
						V_Start_Close[0][4], V_Start_Close[0][5],
						V_Start_Close[0][6], V_Start_Close[0][7], V_Close[0][0],
						V_Close[0][1], V_Close[0][2], V_Close[0][3],
						V_Close[0][4], V_Close[0][5], V_Close[0][6],
						V_Close[0][7], V_End_Close[0][0], V_End_Close[0][1],
						V_End_Close[0][2], V_End_Close[0][3], V_End_Close[0][4],
						V_End_Close[0][5], V_End_Close[0][6],
						V_End_Close[0][7]);
				Upload_Close_0_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 100);
			}

			if (Upload_Close_1_flag == TRUE) {
				sprintf(tempstring,
						"CloseV2;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[1][0], V_Start_Close[1][1],
						V_Start_Close[1][2], V_Start_Close[1][3],
						V_Start_Close[1][4], V_Start_Close[1][5],
						V_Start_Close[1][6], V_Start_Close[1][7], V_Close[1][0],
						V_Close[1][1], V_Close[1][2], V_Close[1][3],
						V_Close[1][4], V_Close[1][5], V_Close[1][6],
						V_Close[1][7], V_End_Close[1][0], V_End_Close[1][1],
						V_End_Close[1][2], V_End_Close[1][3], V_End_Close[1][4],
						V_End_Close[1][5], V_End_Close[1][6],
						V_End_Close[1][7]);
				Upload_Close_1_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 200);
			}
			if (Upload_Close_2_flag == TRUE) {
				sprintf(tempstring,
						"CloseV3;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[2][0], V_Start_Close[2][1],
						V_Start_Close[2][2], V_Start_Close[2][3],
						V_Start_Close[2][4], V_Start_Close[2][5],
						V_Start_Close[2][6], V_Start_Close[2][7], V_Close[2][0],
						V_Close[2][1], V_Close[2][2], V_Close[2][3],
						V_Close[2][4], V_Close[2][5], V_Close[2][6],
						V_Close[2][7], V_End_Close[2][0], V_End_Close[2][1],
						V_End_Close[2][2], V_End_Close[2][3], V_End_Close[2][4],
						V_End_Close[2][5], V_End_Close[2][6],
						V_End_Close[2][7]);
				Upload_Close_2_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 200);
			}

			if (Upload_Close_3_flag == TRUE) {
				sprintf(tempstring,
						"CloseV4;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[3][0], V_Start_Close[3][1],
						V_Start_Close[3][2], V_Start_Close[3][3],
						V_Start_Close[3][4], V_Start_Close[3][5],
						V_Start_Close[3][6], V_Start_Close[3][7], V_Close[3][0],
						V_Close[3][1], V_Close[3][2], V_Close[3][3],
						V_Close[3][4], V_Close[3][5], V_Close[3][6],
						V_Close[3][7], V_End_Close[3][0], V_End_Close[3][1],
						V_End_Close[3][2], V_End_Close[3][3], V_End_Close[3][4],
						V_End_Close[3][5], V_End_Close[3][6],
						V_End_Close[3][7]);
				Upload_Close_3_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 300);
			}

			if (Upload_Close_4_flag == TRUE) {
				sprintf(tempstring,
						"CloseV5;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[4][0], V_Start_Close[4][1],
						V_Start_Close[4][2], V_Start_Close[4][3],
						V_Start_Close[4][4], V_Start_Close[4][5],
						V_Start_Close[4][6], V_Start_Close[4][7], V_Close[4][0],
						V_Close[4][1], V_Close[4][2], V_Close[4][3],
						V_Close[4][4], V_Close[4][5], V_Close[4][6],
						V_Close[4][7], V_End_Close[4][0], V_End_Close[4][1],
						V_End_Close[4][2], V_End_Close[4][3], V_End_Close[4][4],
						V_End_Close[4][5], V_End_Close[4][6],
						V_End_Close[4][7]);
				Upload_Close_4_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 400);
			}

			if (Upload_Close_5_flag == TRUE) {
				sprintf(tempstring,
						"CloseV6;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[5][0], V_Start_Close[5][1],
						V_Start_Close[5][2], V_Start_Close[5][3],
						V_Start_Close[5][4], V_Start_Close[5][5],
						V_Start_Close[5][6], V_Start_Close[5][7], V_Close[5][0],
						V_Close[5][1], V_Close[5][2], V_Close[5][3],
						V_Close[5][4], V_Close[5][5], V_Close[5][6],
						V_Close[5][7], V_End_Close[5][0], V_End_Close[5][1],
						V_End_Close[5][2], V_End_Close[5][3], V_End_Close[5][4],
						V_End_Close[5][5], V_End_Close[5][6],
						V_End_Close[5][7]);
				Upload_Close_5_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 500);
			}

			if (Upload_Close_6_flag == TRUE) {
				sprintf(tempstring,
						"CloseV7;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[6][0], V_Start_Close[6][1],
						V_Start_Close[6][2], V_Start_Close[6][3],
						V_Start_Close[6][4], V_Start_Close[6][5],
						V_Start_Close[6][6], V_Start_Close[6][7], V_Close[6][0],
						V_Close[6][1], V_Close[6][2], V_Close[6][3],
						V_Close[6][4], V_Close[6][5], V_Close[6][6],
						V_Close[6][7], V_End_Close[6][0], V_End_Close[6][1],
						V_End_Close[6][2], V_End_Close[6][3], V_End_Close[6][4],
						V_End_Close[6][5], V_End_Close[6][6],
						V_End_Close[6][7]);
				Upload_Close_6_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 600);
			}

			if (Upload_Close_7_flag == TRUE) {
				sprintf(tempstring,
						"CloseV8;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Close[7][0], V_Start_Close[7][1],
						V_Start_Close[7][2], V_Start_Close[7][3],
						V_Start_Close[7][4], V_Start_Close[7][5],
						V_Start_Close[7][6], V_Start_Close[7][7],
						V_Close[7][0],
						V_Close[7][1], V_Close[7][2], V_Close[7][3],
						V_Close[7][4], V_Close[7][5], V_Close[7][6],
						V_Close[7][7], V_End_Close[7][0], V_End_Close[7][1],
						V_End_Close[7][2], V_End_Close[7][3], V_End_Close[7][4],
						V_End_Close[7][5], V_End_Close[7][6],
						V_End_Close[7][7]);
				Upload_Close_7_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}

			//Time_Pull_Door_Latch_Nose[8]
			//Uploading the Unlatch Time before closing door
			if (Upload_Pitch_Unlatch_Time_flag == TRUE) {
				sprintf(tempstring,
						"UnLatchPitch;%d;%d;%d;%d;%d; %d;%d;%d;\r",
						Time_Pull_Door_Latch_Pitch[0], Time_Pull_Door_Latch_Pitch[1],Time_Pull_Door_Latch_Pitch[2], Time_Pull_Door_Latch_Pitch[3],
						Time_Pull_Door_Latch_Pitch[4], Time_Pull_Door_Latch_Pitch[5],Time_Pull_Door_Latch_Pitch[6],Time_Pull_Door_Latch_Pitch[7]);
				Upload_Pitch_Unlatch_Time_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}
			//Uploading the close power table for pitch angle test
			if (Upload_Close_Pitch_Angle_Test_flag == TRUE) {
				sprintf(tempstring,
						"ClosePitch;%d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d; %d;%d;%d; %d;%d;%d;%d;%d;\r",
						V_Start_Close_Pitch[0], V_Start_Close_Pitch[1],V_Start_Close_Pitch[2], V_Start_Close_Pitch[3],
						V_Start_Close_Pitch[4], V_Start_Close_Pitch[5],V_Start_Close_Pitch[6],V_Start_Close_Pitch[7],
						V_Close_Pitch[0],	V_Close_Pitch[1], V_Close_Pitch[2], V_Close_Pitch[3],
						V_Close_Pitch[4], V_Close_Pitch[5], V_Close_Pitch[6],V_Close_Pitch[7],
						V_End_Close_Pitch[0], V_End_Close_Pitch[1],
						V_End_Close_Pitch[2], V_End_Close_Pitch[3],
						V_End_Close_Pitch[4],	V_End_Close_Pitch[5],
						V_End_Close_Pitch[6],V_End_Close_Pitch[7]);
				Upload_Close_Pitch_Angle_Test_flag = FALSE;
				Upload_Pitch_Unlatch_Time_flag = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}


			//Uploading the open power table for pitch angle test
			if (Upload_Open_Pitch_Angle_Test_flag == TRUE) {
				sprintf(tempstring,
						"OpenPitch;%d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d; %d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Open_Pitch[0], V_Start_Open_Pitch[1],
						V_Start_Open_Pitch[2], V_Start_Open_Pitch[3],
						V_Start_Open_Pitch[4], V_Start_Open_Pitch[5],
						V_Start_Open_Pitch[6],V_Start_Open_Pitch[7],
						V_Open_Pitch[0], V_Open_Pitch[1], V_Open_Pitch[2], V_Open_Pitch[3],
						V_Open_Pitch[4], V_Open_Pitch[5], V_Open_Pitch[6], V_Open_Pitch[7],
						V_End_Open_Pitch[0], V_End_Open_Pitch[1],
						V_End_Open_Pitch[2], V_End_Open_Pitch[3], V_End_Open_Pitch[4],
						V_End_Open_Pitch[5], V_End_Open_Pitch[6], V_End_Open_Pitch[7]);
				Upload_Open_Pitch_Angle_Test_flag = FALSE;
				Upload_Close_Pitch_Angle_Test_flag  = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}



			//Uploading the Unlatch Time before closing door for nose test
			if (Upload_Nose_Unlatch_Time_flag == TRUE) {
				sprintf(tempstring,
						"UnlatchNose;%d;%d;%d;%d;%d; %d;%d;%d;\r",
						Time_Pull_Door_Latch_Nose[0], Time_Pull_Door_Latch_Nose[1],Time_Pull_Door_Latch_Nose[2], Time_Pull_Door_Latch_Nose[3],
						Time_Pull_Door_Latch_Nose[4], Time_Pull_Door_Latch_Nose[5],Time_Pull_Door_Latch_Nose[6],Time_Pull_Door_Latch_Nose[7]);
				Upload_Nose_Unlatch_Time_flag = FALSE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}
			//Uploading the close power table for nose angle test
				if (Upload_Close_Nose_Angle_Test_flag == TRUE) {
					sprintf(tempstring,
							"CloseNose;%d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d; %d;%d;%d;%d;%d;%d;%d;%d;\r",
							V_Start_Close_Nose[0], V_Start_Close_Nose[1],
							V_Start_Close_Nose[2], V_Start_Close_Nose[3],
							V_Start_Close_Nose[4], V_Start_Close_Nose[5],
							V_Start_Close_Nose[6], V_Start_Close_Nose[7],
							V_Close_Nose[0], V_Close_Nose[1], V_Close_Nose[2], V_Close_Nose[3],
							V_Close_Nose[4], V_Close_Nose[5], V_Close_Nose[6], V_Close_Nose[7],
							V_End_Close_Nose[0], V_End_Close_Nose[1],
							V_End_Close_Nose[2], V_End_Close_Nose[3],
							V_End_Close_Nose[4], V_End_Close_Nose[5],
							V_End_Close_Nose[6], V_End_Close_Nose[7]);
					Upload_Close_Nose_Angle_Test_flag = FALSE;
					Upload_Nose_Unlatch_Time_flag = TRUE;
					HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
				}
			//Uploading the open power table for nose angle test
			if (Upload_Open_Nose_Angle_Test_flag == TRUE) {
				sprintf(tempstring,
						"OpenNose;%d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d;%d;%d;%d;%d; %d; %d;%d;%d;%d;%d;%d;%d;%d;\r",
						V_Start_Open_Nose[0], V_Start_Open_Nose[1],
						V_Start_Open_Nose[2], V_Start_Open_Nose[3],
						V_Start_Open_Nose[4], V_Start_Open_Nose[5],
						V_Start_Open_Nose[6],V_Start_Open_Nose[7],
						V_Open_Nose[0],	V_Open_Nose[1], V_Open_Nose[2], V_Open_Nose[3],
						V_Open_Nose[4], V_Open_Nose[5], V_Open_Nose[6],V_Open_Nose[7],
						V_End_Open_Nose[0], V_End_Open_Nose[1],
						V_End_Open_Nose[2], V_End_Open_Nose[3],
						V_End_Open_Nose[4],	V_End_Open_Nose[5],
						V_End_Open_Nose[6],V_End_Open_Nose[7]);
				Upload_Open_Nose_Angle_Test_flag = FALSE;
				Upload_Close_Nose_Angle_Test_flag = TRUE;
				HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 700);
			}
			break;

			case 6:
					sprintf(tempstring, "CAN;%d;%d;%d;%d;%d;%d;%d;\r", RxData[0],
							RxData[1], RxData[2], RxData[3], RxData[4], RxData[5],
							RxData[6], RxData[7]);
					HAL_UART_Transmit(&huart2, tempstring, strlen(tempstring), 200);

				break;

			default:
				break;
		}
	}

void PRINTF(const char * str2print)
{
	if (mDebugPrint != TRUE)
		return;

	PRINTF_Flag = TRUE;
	strcpy(PIprintstr,"Log;");
	strcat(PIprintstr,str2print);
	strcat(PIprintstr,"\r\n");

//	HAL_UART_Transmit(&huart2, str2print, strlen(str2print), 100);
}

void PRINTFWithValue(const char * str2print, char** strarray)
{
	if (mDebugPrint != TRUE)
		return;

	int n;
	n=sizeof(strarray);
	printf(str2print);
	PRINTF_Flag = TRUE;
	strcpy(PIprintstr,"Log;");
	strcat(PIprintstr,str2print);
	for (int i =0;i < n;i++)
	{
		strcat(PIprintstr,strarray[i]);
		strcat(PIprintstr,";");
	}
	strcat(PIprintstr,"\r\n");

//	HAL_UART_Transmit(&huart2, str2print, strlen(str2print), 100);
}

#endif /* SRC_GUI_H_ */
