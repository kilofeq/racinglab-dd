/*
 * global_callbacks.cpp
 *
 *  Created on: 23.01.2020
 *      Author: Yannick
 */

#include "vector"
#include <global_callbacks.h>
#include "main.h"
#include "cppmain.h"
#include "FFBoardMain.h"
#include "ledEffects.h"
#include "constants.h"

#include "UsbHidHandler.h"
#include "PersistentStorage.h"
#include "ExtiHandler.h"
#include "TimerHandler.h"
#include "CommandHandler.h"
#include "EffectsCalculator.h"
#include "HidCommandInterface.h"

#ifdef CANBUS
#include "CanHandler.h"
#endif

#ifdef MIDI
#include "MidiHandler.h"
#include "midi_device.h"
#endif

#include "cdc_device.h"
#include "CDCcomm.h"

extern FFBoardMain* mainclass;

/**
 * Note: this is normally generated in the main.c
 * A call to HAL_TIM_PeriodElapsedCallback_CPP must be added there instead!
 */
__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	HAL_TIM_PeriodElapsedCallback_CPP(htim);
}

void HAL_TIM_PeriodElapsedCallback_CPP(TIM_HandleTypeDef* htim) {
	for(TimerHandler* c : TimerHandler::timerHandlers){
		c->timerElapsed(htim);
	}
}

/**
 * Callback for GPIO interrupts
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	for(ExtiHandler* c : ExtiHandler::extiHandlers){
		c->exti(GPIO_Pin);
	}
}

#ifdef CANBUS
// CAN

uint8_t canRxBuf0[8];
CAN_RxHeaderTypeDef canRxHeader0; // Receive header 0
uint8_t canRxBuf1[8];
CAN_RxHeaderTypeDef canRxHeader1; // Receive header 1
// RX
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	for(uint8_t i = 0; i < HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); i++){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader0, canRxBuf0) == HAL_OK){
			for(CanHandler* c : CanHandler::getCANHandlers()){
				c->canRxPendCallback(hcan,canRxBuf0,&canRxHeader0,CAN_RX_FIFO0);
			}
		}
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	for(uint8_t i = 0; i < HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1); i++){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &canRxHeader1, canRxBuf1) == HAL_OK){
			for(CanHandler* c : CanHandler::getCANHandlers()){
				c->canRxPendCallback(hcan,canRxBuf1,&canRxHeader1,CAN_RX_FIFO1);
			}
		}
	}
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canRxFullCallback(hcan,CAN_RX_FIFO0);
	}
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canRxFullCallback(hcan,CAN_RX_FIFO1);
	}
}
// TX
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxCpltCallback(hcan,CAN_TX_MAILBOX0);
	}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxCpltCallback(hcan,CAN_TX_MAILBOX1);
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxCpltCallback(hcan,CAN_TX_MAILBOX2);
	}
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxAbortCallback(hcan,CAN_TX_MAILBOX0);
	}
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxAbortCallback(hcan,CAN_TX_MAILBOX1);
	}
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canTxAbortCallback(hcan,CAN_TX_MAILBOX2);
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	for(CanHandler* c : CanHandler::getCANHandlers()){
		c->canErrorCallback(hcan);
	}
	hcan->ErrorCode = 0; // Clear errors
}
#endif


//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

// USB Callbacks
USBdevice* usb_device;
uint8_t const * tud_descriptor_device_cb(void){
  return usb_device->getUsbDeviceDesc();
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index){
	return usb_device->getUsbConfigurationDesc(index);
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid){
	return usb_device->getUsbStringDesc(index, langid);
}

uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf){
	return UsbHidHandler::getHidDesc();
}

void tud_cdc_rx_cb(uint8_t itf){
	pulseSysLed();
	if(mainclass!=nullptr){
		mainclass->cdcRcvReady(itf);
	}
}

void tud_cdc_tx_complete_cb(uint8_t itf){

	CDCcomm::cdcFinished(itf);
}



/**
 * USB Out Endpoint callback
 * HID Out and Set Feature
 */
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
	if(report_type == HID_REPORT_TYPE_INVALID && report_id == 0){
		report_id = *buffer;
	}

	if(UsbHidHandler::globalHidHandler!=nullptr)
		UsbHidHandler::globalHidHandler->hidOut(report_id,report_type,buffer,bufsize);

	if(report_id == HID_ID_HIDCMD){
		if(HID_CommandInterface::globalInterface != nullptr)
			HID_CommandInterface::globalInterface->hidCmdCallback((HID_CMD_Data_t*)(buffer));
	}


}

/**
 * HID Get Feature
 */
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
	if(UsbHidHandler::globalHidHandler != nullptr)
		return UsbHidHandler::globalHidHandler->hidGet(report_id, report_type, buffer,reqlen); // reply buffer should not contain report ID in first byte
	return 0;
}

/**
 * HID transfer complete
 */
void tud_hid_report_complete_cb(uint8_t itf, uint8_t const* report, uint8_t len){
	if(HID_CommandInterface::globalInterface != nullptr){
		HID_CommandInterface::globalInterface->transferComplete(itf, report, len);
	}
	if(UsbHidHandler::globalHidHandler != nullptr){
		UsbHidHandler::globalHidHandler->transferComplete(itf, report, len);
	}
}

#ifdef MIDI
MidiHandler* midihandler = nullptr;
/**
 * Midi receive callback
 */
void tud_midi_rx_cb(uint8_t itf){
	if(!midihandler) return;

	if(tud_midi_n_packet_read(itf,MidiHandler::buf)){
		midihandler->midiRx(itf, MidiHandler::buf);
	}
}
#endif

/**
 * Called on usb disconnect and suspend
 */
void tud_suspend_cb(){
	mainclass->usbSuspend();
}
void tud_umount_cb(){
	mainclass->usbSuspend();
}

/**
 * Called on usb mount
 */
void tud_mount_cb(){
	mainclass->usbResume();
}
void tud_resume_cb(){
	mainclass->usbResume();
}

