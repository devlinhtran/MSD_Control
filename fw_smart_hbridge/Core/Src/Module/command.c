// Header:
// File Name: 
// Author:		LinhTran
// Date:

#include "command.h"
#include "product_infor.h"
#include "data_struct.h"
#include "stm32f1xx_hal_flash.h"
#include "crc.h"
#include "delay.h"
#include "rs485.h"
#include "board.h"
#include "serial.h"
#include "transfer.h"
#include "EncoderIncremental.h"
#include "user_interface.h"
#include "ExternalCounter.h"
#include "stm32_tm1637.h"


#define TRANSFER_TIME_OUT  50000


extern PAYLOAD_TYPE  asPayloadTx;
extern PAYLOAD_TYPE  asPayloadRx;
extern uint8_t bufferPrint[];

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet;
extern DATA_MOTOR_INFOR asMotorInfor;

extern DATA_MOTOR_CURR_INFOR asMotorCurrInfor;
extern MENU_DISLAY_TYPEDEF sMenuDisplay[];
	
extern void   FLASH_ClearErrorCode(void);
extern void ExternalCounterPulseInit(void);


static int8_t  (*pfCommandProccess[CMD_END_OF_COMMAND])(void);

int8_t CMD_StringMessage(void);
int8_t CMD_ResetMcu(void);
int8_t CMD_ReadHwfwprVersion(void);
int8_t CMD_WriteFwprVersion(void);
int8_t CMD_SetHwInfor(void);
int8_t CMD_GetHwInfor(void);
int8_t CMD_SetControlLoopMode(void);
int8_t CMD_GetControlLoopMode(void);
int8_t CMD_SetRunningInfor(void);
int8_t CMD_GetRunningInforSet(void);
int8_t CMD_SetMotorInfor(void);
int8_t CMD_SetMotorCurrInfor(void);
int8_t CMD_GetMotorInfor(void);
int8_t CMD_TurnOnLoopControl(void);
int8_t CMD_TurnOffLoopControl(void);
int8_t CMD_SaveAllInForToFlash(void);
int8_t CMD_SetCommunicaType(void);
int8_t CMD_GetCommunicaType(void);
int8_t CMD_BootLoaderJum(void);
int8_t CMD_AutoTuning(void);
int8_t UPDATE_RunningInfor(void);
int8_t CMD_GetOnOfLoopControl(void);
int8_t CMD_ClearError(void);
int8_t CMD_GetSourceVoltage(void);
int8_t CMD_GetSmartInfor(void);


////////////////////////////////////////////////////////////////////////Binarry Command//////////////////////////////////////////////////////////////////////
//Funtion Communication with Host:
void COMMAND_FuntionAssign(void){ // linking a funtion to array funtion:
	pfCommandProccess[CMD_STRING_MESSAGE] = 								CMD_StringMessage;
	pfCommandProccess[CMD_RESET_MCU] = 										  CMD_ResetMcu;
	pfCommandProccess[CMD_READ_HWFWPR_VERSION] = 						CMD_ReadHwfwprVersion;
  pfCommandProccess[CMD_WRITE_FWPR_VERSION] =             CMD_WriteFwprVersion;
	pfCommandProccess[CMD_SET_HW_INFOR] = 									CMD_SetHwInfor;
	pfCommandProccess[CMD_GET_HW_INFOR] = 									CMD_GetHwInfor;
	pfCommandProccess[CMD_SET_CONTROL_LOOP_MODE] = 					CMD_SetControlLoopMode;
	pfCommandProccess[CMD_GET_CONTROL_LOOP_MODE] = 					CMD_GetControlLoopMode;
	pfCommandProccess[CMD_SET_RUNNING_INFOR] = 							CMD_SetRunningInfor;
	pfCommandProccess[CMD_GET_RUNNING_INFOR_SET] = 					CMD_GetRunningInforSet;
	pfCommandProccess[CMD_SET_MOTOR_INFOR] = 								CMD_SetMotorInfor;
	pfCommandProccess[CMD_GET_MOTOR_INFOR] = 								CMD_GetMotorInfor;
	pfCommandProccess[CMD_TURN_ON_LOOP_CONTROL] = 					CMD_TurnOnLoopControl;
	pfCommandProccess[CMD_TURN_OFF_LOOP_CONTROL] = 					CMD_TurnOffLoopControl;
	pfCommandProccess[CMD_GET_ON_OFF_LOOP_CONTROL] = 				CMD_GetOnOfLoopControl;
	pfCommandProccess[CMD_SAVE_ALL_INFOR_TO_FLASH] = 				CMD_SaveAllInForToFlash;
	pfCommandProccess[CMD_SET_COMUNICA_TYPE] = 							CMD_SetCommunicaType;
	pfCommandProccess[CMD_GET_COMUNICA_TYPE] = 							CMD_GetCommunicaType;
	pfCommandProccess[CMD_AUTO_TUNING] = 										CMD_AutoTuning;
	pfCommandProccess[CMD_BOOT_LOADER_JUM] = 								CMD_BootLoaderJum;
	pfCommandProccess[CMD_CLEAR_ERROR]  = 									CMD_ClearError;
	pfCommandProccess[CMD_GET_SOURCE_VOLTAGE]	= 					  CMD_GetSourceVoltage;
	pfCommandProccess[CMD_SET_MOTOR_CURR_INFOR]	= 					CMD_SetMotorCurrInfor;
	pfCommandProccess[CMD_GET_SMART_INFOR]	= 					    CMD_GetSmartInfor;

}


int8_t COMMAND_Process(PAYLOAD_TYPE *psPayload){ 
  int8_t ret=0;
		
    if((psPayload->optCode) < CMD_END_OF_COMMAND){
			asPayloadRx = *psPayload;
      (*pfCommandProccess[psPayload->optCode])();
    }
    else{
      ret =1;
    }
  return ret;
}
int8_t CMD_StringMessage(void){
	
  
}
int8_t CMD_ResetMcu(void){
  PAYLOAD_Transmit(CMD_RESET_MCU|CMD_RESPONT_MASK,NULL,0,TRANSFER_TIME_OUT);
	HAL_NVIC_SystemReset(); 
}
int8_t CMD_ReadHwfwprVersion(void){
  void *pData;
  int8_t ret=0;
  uint32_t aAddress;
  uint32_t aSize= sizeof(PRODUCT_INFOR);
  pData = malloc(aSize+5);
  ret = HAL_FLASH_Unlock();
  
  //Status of Command:
  *(uint8_t*)pData = ret;
  
  aAddress = PRODUCT_INFOR_FLASH_ADDR;
  while (aAddress < (aSize+PRODUCT_INFOR_FLASH_ADDR))
  {
      *((uint32_t*)((uint8_t*)pData+1) + (aAddress-PRODUCT_INFOR_FLASH_ADDR)/4) = *(__IO uint32_t *)aAddress;
      aAddress = aAddress + 4;
  }

  if(pData!=NULL){
  
    PAYLOAD_Transmit(CMD_READ_HWFWPR_VERSION,(uint8_t*)pData,sizeof(PRODUCT_INFOR)+1,TRANSFER_TIME_OUT);

  }
  else{
     ret =-1;
  }
  HAL_FLASH_Lock();
  return ret;
}
int8_t CMD_WriteFwprVersion(void){
  
}
int8_t CMD_SetHwInfor(void){
  int8_t ret=0;
	DATA_HW_INFOR asDataHwInforTerm = asDataHwInfor;
	
  asDataHwInfor = *((DATA_HW_INFOR *)&asPayloadRx.data);
	asDataHwInfor.aMotorProtectEnable = asDataHwInforTerm.aMotorProtectEnable;
	asDataHwInfor.aPositionExpectValue = asDataHwInforTerm.aPositionExpectValue;
	asDataHwInfor.aVelocityExpectValue = asDataHwInforTerm.aVelocityExpectValue;
	asDataHwInfor.aAcceleraExpectValue = asDataHwInforTerm.aAcceleraExpectValue;
	asDataHwInfor.aUartBaudrate = asDataHwInforTerm.aUartBaudrate;
	//if(asStateMechine.asCmType == CM_RS485) Khi thay doi PulsePerRound trong dcTurningTool no se thay doi ca encoder line.
	{ //Phan mem dcTurningPro V2.2 su dung 1 bien asDataHwInfor.aPulsePerRound cho 2 loai aPulsePerRound va aUartBaudrate
		asDataHwInfor.aUartBaudrate = asDataHwInfor.aPulsePerRound;
	}
	
#ifdef MSD_E3A
	DCM_Stspin840CurrentSet(asDataHwInfor.aMaxCurrent);
#endif
  ret = PAYLOAD_Transmit(CMD_SET_HW_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR),TRANSFER_TIME_OUT);
  return ret;
}
int8_t CMD_GetHwInfor(void){
  int8_t ret=0;
	DATA_HW_INFOR asDataHwInforTerm = asDataHwInfor;
	if(asStateMechine.asCmType == CM_RS485){ //Phan mem dcTurningPro V2.2 su dung 1 bien asDataHwInfor.aPulsePerRound cho 2 loai aPulsePer
		asDataHwInforTerm.aPulsePerRound = asDataHwInforTerm.aUartBaudrate ; 
	}
  ret = PAYLOAD_Transmit(CMD_GET_HW_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asDataHwInforTerm,sizeof(DATA_HW_INFOR),TRANSFER_TIME_OUT);
  return ret;
}
int8_t TURNING_Respont(void){
	int8_t ret =0;
	ret = PAYLOAD_Transmit(CMD_AUTO_TUNING|CMD_RESPONT_MASK,(uint8_t*)&asMotorInfor,sizeof(asMotorInfor),TRANSFER_TIME_OUT);
}
int8_t CMD_GetSmartInfor(void){
	int8_t ret =0;
	ret = PAYLOAD_Transmit(CMD_GET_SMART_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asMotorCurrInfor,sizeof(asMotorCurrInfor),TRANSFER_TIME_OUT);
}
int8_t CMD_AutoTuning(void){
	int8_t ret =0;
	int timeOut =3000;	//3000(ms)
	asStateMechine.aStatus = STATUS_RUN;
	DC_ENABLE();
	asStateMechine.asRunningMode = MODE_TURNING;	
	asStateMechine.asLoopControlStatus = CLOOP_STARTING;
	asRuningInforCurrent.aListError = 0; //Clear all error
	//reponst a payload when turning finished in main funtion:
	return ret; 
}
int8_t CMD_SetControlLoopMode(void){
	int8_t ret=0;
  uint8_t aCmd = CMD_SET_CONTROL_LOOP_MODE;
  
  asStateMechine.asRunningMode = asPayloadRx.data[0];
  ret = PAYLOAD_Transmit(CMD_SET_CONTROL_LOOP_MODE|CMD_RESPONT_MASK,(uint8_t*)&aCmd,1,TRANSFER_TIME_OUT);
  return ret;
}

int8_t CMD_GetControlLoopMode(void){
  int8_t ret=0;
  
  ret = PAYLOAD_Transmit(CMD_GET_CONTROL_LOOP_MODE|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.asRunningMode,1,TRANSFER_TIME_OUT);
  return ret;
}

int8_t CMD_SetRunningInfor(void){
  int8_t ret =1;
	int64_t termPulseSet;
	
	if(asStateMechine.asLoopControlStatus != CLOOP_PROCESS || asStateMechine.asCmType == CM_PULSE){
		  asRuningInforSet =*(DATA_RUNING_INFOR*)asPayloadRx.data;
	
			termPulseSet = (asRuningInforSet.aPosition*asDataHwInfor.aEncoderLine)/1.570796327;
			asRuningInforSet.aPosition =    (termPulseSet*1.570796327)/(asDataHwInfor.aEncoderLine);
		
			asStateMechine.asLoopControlStatus =CLOOP_STARTING;
		  ret = 0;
	}
  
  ret = PAYLOAD_Transmit(CMD_SET_RUNNING_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetRunningInforSet(void){
  int8_t ret =0;
	
  ret = PAYLOAD_Transmit(CMD_GET_RUNNING_INFOR_SET|CMD_RESPONT_MASK,(uint8_t*)&asRuningInforSet,sizeof(asRuningInforSet),TRANSFER_TIME_OUT);

  return ret; 
}
int8_t UPDATE_RunningInfor(void){
	int8_t ret =0;
		
	ret = PAYLOAD_Transmit(CMD_GET_RUNNING_INFOR_SET,(uint8_t*)&asRuningInforCurrent,sizeof(DATA_RUNING_INFOR),TRANSFER_TIME_OUT);
  
  return ret; 
}
int8_t CMD_SetMotorInfor(void){
  int8_t ret =0;
  
  asMotorInfor =*(DATA_MOTOR_INFOR*)asPayloadRx.data;
  ret = PAYLOAD_Transmit(CMD_SET_MOTOR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_SetMotorCurrInfor(void){
	int8_t ret =0;
	asMotorCurrInfor =*(DATA_MOTOR_CURR_INFOR*)asPayloadRx.data;
  ret = PAYLOAD_Transmit(CMD_SET_MOTOR_CURR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}

int8_t CMD_GetMotorInfor(void){
  int8_t ret =0;
  ret = PAYLOAD_Transmit(CMD_GET_MOTOR_INFOR|CMD_RESPONT_MASK,(uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR),TRANSFER_TIME_OUT);
  return ret; 
}

int8_t CMD_GetSourceVoltage(void){
	
}
int8_t CMD_ClearError(void){
	int8_t ret =0;
	asRuningInforCurrent.aListError =0;
	if(asStateMechine.aStatus == STATUS_RUN){
		DC_ENABLE();
	}
  ret = PAYLOAD_Transmit(CMD_CLEAR_ERROR|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetOnOfLoopControl(void){
	int8_t ret=0;

  PAYLOAD_Transmit(CMD_GET_ON_OFF_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.aStatus,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_TurnOnLoopControl(void){
  int8_t ret=0;
  asStateMechine.aStatus = STATUS_RUN;
	DC_ENABLE();
  PAYLOAD_Transmit(CMD_TURN_ON_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret;  
}
int8_t CMD_TurnOffLoopControl(void){
  int8_t ret=0;
  asStateMechine.aStatus = STATUS_STOOP;
	DC_DISABLE();
  ret = PAYLOAD_Transmit(CMD_TURN_OFF_LOOP_CONTROL|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t Flash_WriteData(uint8_t *pData,uint32_t size,uint32_t *aAddress){
  uint32_t aIndex=0,i=0;
  
  while(i<size+2){
		i+=2;
		
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD ,*aAddress,*(__IO uint16_t *)(pData+aIndex))!=HAL_OK){
      return 1;
      break;
    }
    else{ //Check data again;
      if( *(__IO uint16_t *)(*aAddress) != *(__IO uint16_t *)(pData+aIndex)){   //data wrong:
        return 1;
      }
      else{
        (*aAddress) +=2;
        aIndex+=2;
      }
    }   
  }
  return 0;
}
int8_t CMD_SaveAllInForToFlash(void){
	static FLASH_EraseInitTypeDef EraseInitStruct;
	static uint32_t SectorError;
	uint8_t byte[500],term;
	uint32_t i,j;
  int8_t ret=0;
  uint8_t aCrc=0;
  
  uint32_t aAddCurrent = USER_DATA_BASE_FLASH_ADDR;
	//Erase a flash area:
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = USER_DATA_BASE_FLASH_ADDR;
  EraseInitStruct.NbPages = 1;
	
	asMotorCurrInfor.aCFG_HOLDER = CFG_HOLDER ;
	
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
		ret = 0xff;
	}
	term = sizeof(DATA_HW_INFOR);
	for(i=0;i<term;i++){
		byte[i] = *((uint8_t*)&asDataHwInfor+i);
	}
	term += sizeof(DATA_RUNING_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asRuningInforSet+j++);
	}
	term +=sizeof(DATA_MOTOR_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asMotorInfor+j++);
	}
	term +=sizeof(STATE_MECHINE);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asStateMechine+j++);
	}
	
	term +=sizeof(DATA_MOTOR_CURR_INFOR);
	j=0;
	for(;i<term;i++){
		byte[i] = *((uint8_t*)&asMotorCurrInfor+j++);
	}
	
//	term += sizeof(MENU_DISLAY_TYPEDEF)*MENUE_ITERM_NUMBER ;
//	j=0;
//	for(;i<term;i++){
//		byte[i] = *((uint8_t*)&asMotorCurrInfor+j++);
//	}
  //Save user data Struct:
  //ret += Flash_WriteData((uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asRuningInforSet,sizeof(DATA_RUNING_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR),&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&asStateMechine,sizeof(STATE_MECHINE),&aAddCurrent);
	
  //Create checksum
  aCrc = CRC8_Generate(0,(uint8_t*)&asDataHwInfor,sizeof(DATA_HW_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asRuningInforSet,sizeof(DATA_RUNING_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorInfor,sizeof(DATA_MOTOR_INFOR));
  aCrc = CRC8_Generate(aCrc,(uint8_t*)&asStateMechine,sizeof(STATE_MECHINE));
	aCrc = CRC8_Generate(aCrc,(uint8_t*)&asMotorCurrInfor,sizeof(DATA_MOTOR_CURR_INFOR));
  //Save CRC:
	byte[i] = aCrc;
	ret = Flash_WriteData(byte,i,&aAddCurrent);
  //ret += Flash_WriteData((uint8_t*)&aCrc,1,&aAddCurrent);
  HAL_FLASH_Lock();
  //responst:
  PAYLOAD_Transmit(CMD_SAVE_ALL_INFOR_TO_FLASH|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret;
}
int8_t CMD_SetCommunicaType(void){
	int8_t ret =0;
	if( asPayloadRx.data[0] == CM_PULSE && asStateMechine.asCmType!=CM_PULSE){
		ExternalCounterPulseInit();

		aExternCounterValue = (asRuningInforCurrent.aEstimatesValue*asDataHwInfor.aPulsePerRound)/6.283185307;
	}
	else if(asPayloadRx.data[0] == CM_RS485 && asStateMechine.asCmType!=CM_RS485){	//Init for network:
		asStateMechine.asLoopControlStatus = CLOOP_STOOP;
	}
	
	asStateMechine.asCmType = asPayloadRx.data[0];
  ret = PAYLOAD_Transmit(CMD_SET_COMUNICA_TYPE|CMD_RESPONT_MASK,(uint8_t*)&ret,1,TRANSFER_TIME_OUT);
  return ret; 
}
int8_t CMD_GetCommunicaType(void){
	int8_t ret=0;
	ret =PAYLOAD_Transmit(CMD_GET_COMUNICA_TYPE|CMD_RESPONT_MASK,(uint8_t*)&asStateMechine.asCmType,1,TRANSFER_TIME_OUT);
	return ret;
}
int8_t CMD_BootLoaderJum(void){
  
}

///*
//typedef enum{
//	RS485_CMD_NULL,
//	RS485_CMD_POSITION,
//	RS485_CMD_VELOCITY,
//	RS485_CMD_ACCEL,
//	RS485_CMD_RESET,
//	RS485_CMD_READ,
//	RS485_CMD_ENABLE,
//	RS485_CMD_DISABLE,
//	RS485_CMD_STOP,
//	RS485_CMD_CLEAR_ERR
//}RS485_CMD;
//*/
//RS485_ProcessNewData(RS485_DATA_STRUCT *psRs485Data){
//	switch(psRs485Data->aOptCode){
//		case RS485_CMD_POSITION:
//			asRuningInforSet.aPosition = (double)(*(float*)psRs485Data->aData);
//			break;
//		case RS485_CMD_VELOCITY:
//			asRuningInforSet.aVelocity = (double)(*(float*)psRs485Data->aData);
//			break;
//		case RS485_CMD_ACCEL:
//			asRuningInforSet.aAccel = (double)(*(float*)psRs485Data->aData);
//			break;
//		case RS485_CMD_RESET:
//			HAL_NVIC_SystemReset();
//			break;
//		case RS485_CMD_READ:
//		{
//			float position = (float)asRuningInforCurrent.aPosition;
//			RS485_Transmit(psRs485Data->aOptCode,psRs485Data->aAddSource,(uint8_t*)&position,4,0xffffff);
//			break;
//		}
//		case RS485_CMD_ENABLE:
//		{
//			asStateMechine.aStatus = STATUS_RUN;
//			ENCODER_Reset();
//			asRuningInforSet.aPosition =0;
//			asRuningInforCurrent.aEstimatesValue =0;
//			break;
//		}
//		case RS485_CMD_DISABLE:
//			asStateMechine.aStatus = STATUS_STOOP;
//			break;
//		case RS485_CMD_STOP:
//			asRuningInforSet.aPosition = asRuningInforCurrent.aEstimatesValue;
//			break;
//			case RS485_CMD_CLEAR_ERR:
//				asRuningInforCurrent.aListError =0;
//			break;
//		default:
//			break;
//		
//	}
//}
/////////////////// Command by direction value://///////////////////////
char CMD_GoToP(int p, int velocity, int accelerat){	//round
	if(asStateMechine.asLoopControlStatus != CLOOP_PROCESS) //Runing to P with a V,A config by appp
	{
		asRuningInforSet.aPosition += p*6.28318531;
		asRuningInforSet.aVelocity = velocity;
		asRuningInforSet.aAccel = accelerat;
		asStateMechine.asLoopControlStatus =CLOOP_STARTING;
		asStateMechine.aStatus = STATUS_RUN;
		return 0; //pass;
	}
	return 1;
}
char CMD_GoToFinished(void){
	if( asStateMechine.asLoopControlStatus == CLOOP_FINISH){
		return 0;
	}
	else{
		return 1;
	}
}
////////////////////////////////////////////////////////////////////////End of Binarry Command//////////////////////////////////////////////////////////////////////
/* Acii commmand groutable:
///////////////////////////////////////////////////////// $ (Parametter group)://////////////////////////////////////////////////////////////////
$ 				show all parameter, show help command
$1 = x		Address of the Driver (x: 1-254)
$2 = x		Encoder Line (Encoder resolution per Round)
$3 = x		Baurdrate of the UART (if Control Methode: uart mode), Pulse per round (if Control Methode: Pulse/Dir Mode).
$4 = x		Control Methode (0:PULSE/DIR0, 1:UART Network, 2:CM_CAN, 3:Analog, 4:USB )
$5 = x		Model Close Loop type (0:Turning, 1:None, 2:PID Position, 3:PI Velocity (recommend), 4:Smart Position (recommend), 5: None, 6: PWM mode(Working as H-Bridge))
$6 = xx	  Current Limit (unit mA).
$7 = xx		Select Motor Type, All other parameters (Encoder line, Kp,Kd,...) will autor follow with selected motor model(1: CS_314, 2: CS_315
$8 = xx		(xx don't care) Save all config to Flash.
....
$101 =xx  xx = 0: Runing, 1: Saving and MCU Reset; 2: Factory Reset; 255: MCU Reset;

//////////////////////////////////////////////// N	(Moving Group) ; 
///////////Command format:  Nx [P/p x] [v x] [a x] 
/////////EX: N0 P-000 v200 a500 G	: N0 = network 0, P-000: Abs position 0, velocity 200 , 

//////////////////////////////////////////////// Nx O	(operation Group) ; 
///////////Command format:  Nx O  [l] [u] [r] [R 1607] [G x] [c]
ex: N1 P100 v60 a500 O G1 $0A
*/
char Para_Setting(char commandType, int32_t value){
	int len;

	
	switch(commandType){
		case 0:	//Request a help table.
			if(asStateMechine.aHostCommunicationAddress ==0){ ////Just send address in broadcat :
					len = sprintf(bufferPrint,"\nDeviceID: 0x%x	 - Address: %d - ",UID1, asDataHwInfor.aAdd);
					BOART_TransferDataToHost(bufferPrint,len);
					break;
			}
		
			len = sprintf(bufferPrint," \n\nCC-Smart Technology");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint," \nWWW.CC-Smart.net");
			BOART_TransferDataToHost(bufferPrint,len);
		
#ifdef MSD_E3A
			len = sprintf(bufferPrint," \nMSD_E3A FW_V3.6");
#elif MSD_E10A
		len = sprintf(bufferPrint," \nMSD_E10A FW_V3.6");
#elif MSD_E20A
		len = sprintf(bufferPrint," \nMSD_E20A FW_V3.6");
#endif

			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint," \nDevice ID: 0x%x", UID1);
			BOART_TransferDataToHost(bufferPrint,len);
		
			len = sprintf(bufferPrint,"\n\nNx ? \\n;	 Help?");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\nNote:	 Please Saving and Reset MCU after changing a parameter to have fully correct result.");
			BOART_TransferDataToHost(bufferPrint,len);
//Error table		
		len = sprintf(bufferPrint,"\n\nError Code Table:");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E1: Follow Error (The error betwean estimate value and current value)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E2: Encoder is wrong connection. Wiring (A<->B should be reverse)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E3: Encoder dead (have no encoder signal) or Motor stuck");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E4: Phase Error (The Motor wiring is not good)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E5: Over Current");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E6: The supply is out of range.");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E7: Turning Fail (Can't Autor detect your Motor property)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    E8: Over Termperature");
			BOART_TransferDataToHost(bufferPrint,len);
		
			len = sprintf(bufferPrint,"\n\nNx  $xxx= Parameter_Value\\n;	 Parameter Setting Group;");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $001=%d;	 Address of the Driver is: %d",asDataHwInfor.aAdd, asDataHwInfor.aAdd);
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $002=%d;	 Encoder Line (Encoder resolution per Round)",asDataHwInfor.aEncoderLine);
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $003=%d; The main Motor Saft will run 1/%d circle per One Pulse from External Pin (Pul/Dir).",asDataHwInfor.aPulsePerRound,asDataHwInfor.aPulsePerRound);
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    $004=%d;	 Model Close Loop Type (0: Turning, 1: None, 2: PID Position, 3: PI Velocity (recommend), 4: Smart Position (recommend), 5: None, 6: H-Bridge mode (Working as H-Bridge))",asStateMechine.asRunningMode);
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $005=%d;	 Communicate Methode (0: PULSE/DIR, 1: UART Network, 2: None, 3: Analog (Just for velocity Mode))",asStateMechine.asCmType);
			BOART_TransferDataToHost(bufferPrint,len);
		
		  len = sprintf(bufferPrint,"\n\t    $006=%dmA; Current Limit ",asDataHwInfor.aMaxCurrent);
			BOART_TransferDataToHost(bufferPrint,len);
		
			len = sprintf(bufferPrint,"\n\t    $007=%d;	 Follow Error (rad(PositionModel) or rad/s(VelocityModel)): The Maximum different betwean ExtimateValue vs RealValue is %d",(int16_t)asDataHwInfor.aFollowError,(int16_t)asDataHwInfor.aFollowError);
			BOART_TransferDataToHost(bufferPrint,len);

			len = sprintf(bufferPrint,"\n\t    $008=%d;	 Motor Protection Active (0: Disable, 1: Enable) ",asDataHwInfor.aMotorProtectEnable);
			BOART_TransferDataToHost(bufferPrint,len);
		
			len = sprintf(bufferPrint,"\n\t    $009=%d;	 Uart Baudrate",asDataHwInfor.aUartBaudrate);
			BOART_TransferDataToHost(bufferPrint,len);
	//
			len = sprintf(bufferPrint,"\n\t    $010=%d;	 Delta Position Expect When press the TEST Button (Circle)",asDataHwInfor.aPositionExpectValue);
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    $011=%d;	 Velocity Expect When press the TEST Button (Round/s)",asDataHwInfor.aVelocityExpectValue);
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    $012=%d;	 Acceleration Expect When press the TEST Button (Round/s^2)",asDataHwInfor.aAcceleraExpectValue);
			BOART_TransferDataToHost(bufferPrint,len);
	
		//Kx Position:
			len = sprintf(bufferPrint,"\n\n\t    $020=%d;	 Kp_P=%d",(int32_t)(100*asMotorCurrInfor.aKp_P),(int32_t)(100*asMotorCurrInfor.aKp_P));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $021=%d;	 Ki_P=%d",(int32_t)(100*asMotorCurrInfor.aKi_P),(int32_t)(100*asMotorCurrInfor.aKi_P));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $022=%d;	 Kd_P=%d",(int32_t)(100*asMotorCurrInfor.aKd_P),(int32_t)(100*asMotorCurrInfor.aKd_P));
			BOART_TransferDataToHost(bufferPrint,len);
		//Kx Velocity:	
			len = sprintf(bufferPrint,"\n\t    $023=%d;	 Kp_V=%d",(int32_t)(100*asMotorCurrInfor.aKp_V),(int32_t)(100*asMotorCurrInfor.aKp_V));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $024=%d;	 Ki_V=%d",(int32_t)(100*asMotorCurrInfor.aKi_V),(int32_t)(100*asMotorCurrInfor.aKi_V));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $025=%d;	 Kd_V=%d",(int32_t)(100*asMotorCurrInfor.aKd_V),(int32_t)(100*asMotorCurrInfor.aKd_V));
			BOART_TransferDataToHost(bufferPrint,len);
		//Kx Current:	
			len = sprintf(bufferPrint,"\n\t    $026=%d;	 Kp_I=%d",(int32_t)(100*asMotorCurrInfor.aKp_I),(int32_t)(100*asMotorCurrInfor.aKp_I));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $027=%d;	 Ki_I=%d",(int32_t)(100*asMotorCurrInfor.aKi_I),(int32_t)(100*asMotorCurrInfor.aKi_I));
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    $028=%d;	 Kd_I=%d",(int32_t)(100*asMotorCurrInfor.aKd_I),(int32_t)(100*asMotorCurrInfor.aKd_I));
			BOART_TransferDataToHost(bufferPrint,len);
			
			
			len = sprintf(bufferPrint,"\n\n\t    $101=0;	 MCU(0: Runing, 1: Saving & Reset; 2: Reset; 3: Factory Reset & Reset;)	\n");
			BOART_TransferDataToHost(bufferPrint,len);
		
			//len = sprintf(bufferPrint,"\n\t    Select Motor Type, All other parameters (Encoder line, Kp,Kd,...) will autor follow with selected motor model(1: CS_314, 2: CS_315)");
			//BOART_TransferDataToHost(bufferPrint,len);
			//N	(Moving Group) ; EX: N0 p-000 v200 a500 : network 0, position 0, velocity 200 
			len = sprintf(bufferPrint,"\nNx [p/P value] [v value] [a value]\\n; Moving motor Nx with p/P,v,a parametter");
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    Nx: x Adress Of Driver (0: Broadcast ; 1->99: Unicast)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    p: Absolute Position Value (Option)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    P: Relative Position Value (Option)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    v: Velocity Value(Option)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    a: Acceleration Value (Option)");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    Example: (The Driver 1 go to 100rad with Velocity 50rad/s and Acceleration 600rad/s^2): 	N1 p100 v50 a600 \\n");
			BOART_TransferDataToHost(bufferPrint,len);
			
			
			len = sprintf(bufferPrint,"\n\nNx  [d value] \\n; d: Duty Cycle in H-Bridge Mode ($004 = 6); (Value Range: -900 to 900) \n");
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\nNx O [Kx] [T] [Mx] [Dx] [S] [L] [U] [r] [R101] [Gx] [C] \\n ; (O: Operation Group Command)");
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    [ ] : Option");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    Kx : Ack command respond (K1: Enable (default at start up MCU); K0: Disable");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    T: Turning The Motor");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    Mx: Control Method = M%d (M3: PI Velocity, M4: Smart Position, M5: None, M6: H-Bridge mode (Working as H-Bridge))", asStateMechine.asRunningMode);
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    Dx: Communicate Methode = D%d (D0: PULSE/DIR, D1: UART Network, D2: None, D3: Analog (Just for velocity Mode))",asStateMechine.asCmType);
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    S: Saving All Parameter");
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\t    L: Loock/Pause/Stoop the Motor immediately");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    U: Unlook Motor");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    r: Reset the Current Position to 0");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    R101: Reset the driver");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    C: Clear error list");
			BOART_TransferDataToHost(bufferPrint,len);
			len = sprintf(bufferPrint,"\n\t    G: Get moving infor (G1: One Time; G3: Unitil Receive a New Data With Frequency Respond 5Hz; G255: One time with Randome Delay)");
			BOART_TransferDataToHost(bufferPrint,len);
			
			len = sprintf(bufferPrint,"\n\nParameter End\n");
			BOART_TransferDataToHost(bufferPrint,len);
			break;
		case 1:
			if(asStateMechine.aHostCommunicationAddress !=0){ //Don't accept a from Broadcat
				asDataHwInfor.aAdd = value;
			}
			break;
		case 2:
			asDataHwInfor.aEncoderLine = value;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 3: 
			asDataHwInfor.aPulsePerRound = value;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 4:
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			asStateMechine.asRunningMode  = value;
			
			break;
		case 5:
			if(value<=3){
				asStateMechine.asCmType = value;
			}
			break;
		case 6:
			asDataHwInfor.aMaxCurrent = value;

			break;
		case 7: 
			asDataHwInfor.aFollowError = value;
			break;
		case 8: 
			asDataHwInfor.aMotorProtectEnable = value;
			break;
		case 9:
			asDataHwInfor.aUartBaudrate = value;
			break;
		case 10:
			asDataHwInfor.aPositionExpectValue = value;
			sMenuDisplay[MENUE_POSITION_EXPECT_INDEX].cValue = asDataHwInfor.aPositionExpectValue;
			break;
		case 11: 
			asDataHwInfor.aVelocityExpectValue = value;
			sMenuDisplay[MENUE_VELOCITY_EXPECT_INDEX].cValue = asDataHwInfor.aVelocityExpectValue/10;
			break;
		case 12: 
			asDataHwInfor.aAcceleraExpectValue = value;
			sMenuDisplay[MENUE_ACCELERATION_INDEX].cValue = asDataHwInfor.aAcceleraExpectValue/100;
			break;
		
		case 20:
			asMotorCurrInfor.aKp_P = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 21:
			asMotorCurrInfor.aKi_P = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 22:
			asMotorCurrInfor.aKd_P = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 23:
			asMotorCurrInfor.aKp_V = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			//Copy Kx to Cotrol Loop. 
			asMotorInfor.aK1 = asMotorCurrInfor.aKp_V;
										
			break;
		case 24:
			asMotorCurrInfor.aKi_V = value/100.0;
			asMotorInfor.aK2 = asMotorCurrInfor.aKi_V;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 25:
			asMotorCurrInfor.aKd_V = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 26:
			asMotorCurrInfor.aKp_I = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 27:
			asMotorCurrInfor.aKi_I = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		case 28:
			asMotorCurrInfor.aKd_I = value/100.0;
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			break;
		
		case 101:	//
			asStateMechine.asLoopControlStatus = CLOOP_STOOP;
			if(value == 0x01){			//Save all infor and Reset;
				
				CMD_SaveAllInForToFlash();
				HAL_Delay(20);
				HAL_NVIC_SystemReset();
				
			}
			else if(value ==0x02){	//Reset Mcu;	
				HAL_NVIC_SystemReset(); 
			}
			else if(value == 0x03){	//Factory Reset
				APP_FactoryReset();
				HAL_Delay(20);
				HAL_NVIC_SystemReset();
			}
			break;
		default:
			break;
	}	
}
	
int8_t ASCII_COMMAND_Process(char *aCommand){ 
	char *subString;
	int8_t *pTerm;
	int32_t  value, address ;
	double aPositionSet = 0.0;
	double aVelocitiSet =0.0;
	char commandIndex;
	char modelSetByV = 0; 
	int len; 
	char motionCommand =0;
	
	
	address = 0xffff;
	value = 0;
	//Get Address
	subString = strchr(aCommand,'N');
	if(subString){
		address  = atoi(subString+1);
	}
	asStateMechine.aHostCommunicationAddress = address;
	
	//Analy data:
	if(subString && (address == 0 || address == asDataHwInfor.aAdd)){ //The command for me. 
		
//Para command group:
			subString = strchr(aCommand,'$');
			if(subString){
				commandIndex = atoi((subString+1));
				value =0;
				if(commandIndex >0){
					subString = strchr(aCommand,'=');
					if(subString){
						value = atoi((subString+1));
					}
				}
				Para_Setting(commandIndex, value);
			}

			//Help command;
			subString = strchr(aCommand,'?');
			if(subString){ 
				Para_Setting(0, address);
			}

//Moving Command Group:
			//v:
			subString = strchr(aCommand,'v');
			if( subString){
				motionCommand = 1;
				if( asStateMechine.asRunningMode == MODE_TURNING ){ //Don't process P,V when Turning Mode.
					return 0;
				}
				
				if(asStateMechine.asCmType == CM_ANALOG){
						len = sprintf(bufferPrint,"\n\aANALOG Mode, Can't process this command");
						BOART_TransferDataToHost(bufferPrint,len);
				}
				else{
						aVelocitiSet = strtod((subString+1),&pTerm);
						if(asStateMechine.asRunningMode == MODE_PI_VELOCITY){
							asRuningInforSet.aVelocity = aVelocitiSet;
							asStateMechine.asLoopControlStatus = CLOOP_STARTING;
							asStateMechine.aStatus = STATUS_RUN;
						}
						else{
							if(aVelocitiSet <0){
								asRuningInforSet.aVelocity =  -1.0*aVelocitiSet;
							}
							else{
								asRuningInforSet.aVelocity = aVelocitiSet;
							}
							//asRuningInforSet.aVelocity = absD(aVelocitiSet); Chua hieu tai sao goi ham absD o day ra sai
						}
				 }
			}
			//a:
			subString = strchr(aCommand,'a');
			if( subString){
					asRuningInforSet.aAccel = strtod((subString+1),&pTerm);
			}
			//p: vi tri tuyet doi
			subString = strchr(aCommand,'p');
			if( subString){
				motionCommand =1;
				if( asStateMechine.asRunningMode == MODE_TURNING ){ //Don't process P,V when Turning Mode.
					return 0;
				}
				
				aPositionSet  = strtod((subString+1),&pTerm);
				if(asRuningInforSet.aPosition != aPositionSet || asStateMechine.aStatus == STATUS_STOOP || asStateMechine.asLoopControlStatus == CLOOP_STOOP){
					asRuningInforSet.aPosition = aPositionSet;//CLOOP_PROCESS		
					asStateMechine.aStatus = STATUS_RUN;
					asStateMechine.asLoopControlStatus = CLOOP_STARTING;
				}
			}
			else{//P: Vi tri tuong doi:
				subString = strchr(aCommand,'P');
				if(subString){
					motionCommand  = 1; 
					if( asStateMechine.asRunningMode == MODE_TURNING ){ //Don't process P,V when Turning Mode.
						return 0;
					}
					aPositionSet  = strtod((subString+1),&pTerm);
					asRuningInforSet.aPosition += aPositionSet ; //CLOOP_PROCESS
					asStateMechine.asLoopControlStatus = CLOOP_STARTING;
					asStateMechine.aStatus = STATUS_RUN;
				}
			}
			//d: Duty in H-bridge mode. 
			subString = strchr(aCommand,'d');
			if( subString){
					asRuningInforSet.aVelocity = strtod((subString+1),&pTerm);
					asStateMechine.aStatus = STATUS_RUN;
					asStateMechine.asLoopControlStatus = CLOOP_STARTING;
			}
//Operation command:			
			subString = strchr(aCommand,'O');
			if(subString)
			{ 
					//Ack respond: 
					subString = strchr(aCommand,'K');
					if(subString){//Set ack flag
						if(atoi(subString+1) == 0){
							asStateMechine.aAckCommandFlag = 0;
						}
						else{
							asStateMechine.aAckCommandFlag = 1;
						}
					}
					//T:	Turning the motor
					subString = strchr(aCommand,'T');
					if( subString){
						user_MenueActiveViewSet(MENUE_CONTROL_MODE_INDEX);
						CMD_AutoTuning();
					}
					//M: Model Cloose Loop Control
					subString = strchr(aCommand,'M');
					if( subString){
						if(atoi(subString+1) > 0 && atoi(subString+1)<7){
							asStateMechine.asLoopControlStatus = CLOOP_STOOP;
							asStateMechine.asRunningMode = atoi(subString+1);
						
						}
					}
					//Dx: Control Methode (D0: PULSE/DIR, D1: UART Network, D2: None, D3: Analog (Just for velocity Mode))");
					subString = strchr(aCommand,'D');
					if( subString){
						value =atoi(subString+1); 
						if(value<=3 && value>=0){
							asStateMechine.asLoopControlStatus = CLOOP_STOOP;
							asStateMechine.asCmType = value;
						}
					}
					//S: Saving All Para
					subString = strchr(aCommand,'S');
					if( subString){
						asStateMechine.asLoopControlStatus = CLOOP_STOOP;
						CMD_SaveAllInForToFlash();
						//HAL_Delay(20);
					}
					//U:	Unlock the Motor
					subString = strchr(aCommand,'U');
					if( subString){
						motionCommand = 1;
						asStateMechine.asLoopControlStatus = CLOOP_STOOP;
						asStateMechine.aStatus = STATUS_STOOP;
					}
					
					//L:	Lock/Pause the Motor: con bug o mode velocity
					subString = strchr(aCommand,'L');
					if( subString){ 	//dung tai cho 
							motionCommand = 1;
							if(asStateMechine.asRunningMode == MODE_PI_VELOCITY){ //Velocity Model
								asRuningInforSet.aVelocity = 0;
								asStateMechine.asLoopControlStatus = CLOOP_STARTING;
								asStateMechine.aStatus = STATUS_RUN;
							}
							else{ //Position model:
								asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;
								asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
								aExternCounterValue = asDataHwInfor.aPulsePerRound*asRuningInforCurrent.aEstimatesValue/6.283185307;
								asStateMechine.asLoopControlStatus = CLOOP_STARTING;
								asStateMechine.aStatus = STATUS_RUN;
							}
					}
					//C:	clear error list:
					subString = strchr(aCommand,'C');
					if( subString){
						asRuningInforCurrent.aListError = 0;
						asStateMechine.aStatus = STATUS_STOOP;
						asStateMechine.asLoopControlStatus = CLOOP_STOOP;
						user_MenueActiveViewSet(MENUE_ERROR_INDEX);
					}
					//r: reset the current position to 0:
					subString = strchr(aCommand,'r');
					if( subString){
						asRuningInforSet.aPosition = 0 ;
						asRuningInforCurrent.aPosition = 0;
						asRuningInforCurrent.aEstimatesValue = 0 ;
						EncoderCounterValue =0;
					}
					//R: reset the Driver
					subString = strchr(aCommand,'R');
					if( subString){
						if(atoi(subString+1) == 101){
							asStateMechine.asLoopControlStatus = CLOOP_STOOP;
							HAL_NVIC_SystemReset(); 
						}
					}
					//'G':	//get moving information
					subString = strchr(aCommand,'G');
					if( subString){
						asStateMechine.aHostRequestMovingInfor = atoi((subString+1));
						if(address ==0){//Have to send with Random delay;
							asStateMechine.aHostRequestMovingInfor = 0xff;
						}
					}
			}
//The Driver beeing Error:
			if(motionCommand == 1 && asRuningInforCurrent.aListError !=0){
				len = sprintf(bufferPrint,"\n\n\aERR:%d, Please Clear Error First And Try Again This Command \n", sMenuDisplay[MENUE_ERROR_INDEX].cValue );
				BOART_TransferDataToHost(bufferPrint,len);
			}
//ACK respond for a new Line commmand:
			if(asStateMechine.aAckCommandFlag)
			{	
				BOART_TransferDataToHost(aCommand,strlen(aCommand));
			}
	}
}




