/**
 * \file WSNDemo.c
 *
 * \brief WSNDemo application implementation
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *
 */

/*
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the WSN Demo Application Application
 * The WSNDemo application implements a typical wireless sensor network
 * scenario,
 * in which one central node collects the data from a network of sensors and
 * passes this data over a serial connection for further processing.
 * In the case of the WSNDemo this processing is performed by the WSNMonitor PC
 * application. The BitCloud&reg; Quick Start Guide  provides a detailed
 *description
 * of the WSNDemo application scenario, and instructions on how to use
 * WSNMonitor.
 *  However since BitCloud is a ZigBee&reg; PRO stack, there are a few
 *differences
 * in the protocol:
 * • Device types (Coordinator, Router and End Device) are simulated on the
 * application level; there is no such separation in Lightweight Mesh on the
 * stack level
 * • The value of the extended address field is set equal to the value of the
 * short address field
 * • For all frames, the LQI and RSSI fields are filled in by the coordinator
 * with the values of LQI and RSSI from the received frame. This means that
 *nodes
 * that are not connected to the coordinator directly will have the same values
 * as the last node on the route to the coordinator
 * • Sensor data values are generated randomly on all platforms
 * • Sending data to the nodes on the network is not implemented and not
 * supported in this demo application
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#if APP_ENDDEVICE
#include "sleep_mgr.h"
#endif
#include "commands.h"
#if APP_COORDINATOR
#include "sio2host.h"
#endif
#if SAMD || SAMR21 || SAML21
#include "system.h"
#else
#include "sysclk.h"
#if (LED_COUNT > 0)
#include "led.h"
#endif
#endif
#include "asf.h"
#include "board.h"
#include "wsndemo.h"

/*****************************************************************************
*****************************************************************************/

#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1)
bool isSerialCommand = false;
int confidence_level = 0;
int threshhold = 2500;
uint8_t dat[7];
int byteCount = 0;
static void appSendData(void);
#if APP_ENDDEVICE || APP_ROUTER

uint8_t ledPin[4] = {PIN_PA06,PIN_PB02,PIN_PA18,PIN_PA22};
uint8_t buttonPin[4] = {PIN_PA07,PIN_PB03,PIN_PA13,PIN_PA23};
uint8_t irledPin[4] = {PIN_PA28,PIN_PB23,PIN_PA16,PIN_PA17};
uint8_t ledStatus[4] = {0,0,0,0};
int chairTime[4] = {0,0,0,0};
int seconds = 0;
int occupiedTime = 40;
int reserveTime = 20;

	/* STATUS DESCRIPTION
	LED
	0 - Vacant - LED_ON
	1 - Occupied - LED_OFF
	2 - Reserved - LED_TOGGLE
	
	IRLED
	0 - ON
	1 - OFF
	*/
	
void change_status(int pin);		//change the status of chair based on previous
void update_led(void);				//update the leds corresponding to current status



#endif
/*- Types ------------------------------------------------------------------*/
COMPILER_PACK_SET(1)
typedef struct  AppMessage_t {
	uint8_t commandId;
	uint8_t nodeType;
	uint64_t extAddr;
	uint16_t shortAddr;
	uint32_t softVersion;
	uint32_t channelMask;
	uint16_t panId;
	uint8_t workingChannel;
	uint16_t parentShortAddr;
	uint8_t lqi;
	int8_t rssi;

	struct {
		uint8_t addr1;		//MSB
		uint8_t addr2;		//LSB
		uint8_t type;	//change led or irled
		uint8_t pin0;
		uint8_t pin1;
		uint8_t pin2;
		uint8_t pin3;
		
	} command;

	struct {
		uint8_t type;
		uint8_t size;
		char text[APP_CAPTION_SIZE];
	} caption;
} AppMessage_t;

typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_SEND,
	APP_STATE_WAIT_CONF,
	APP_STATE_SENDING_DONE,
	APP_STATE_WAIT_SEND_TIMER,
	APP_STATE_WAIT_COMMAND_TIMER,
	APP_STATE_PREPARE_TO_SLEEP,
	APP_STATE_SLEEP,
	APP_STATE_WAKEUP,
	APP_STATE_IDLE,
} AppState_t;
COMPILER_PACK_RESET()
/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;

#if APP_ROUTER || APP_ENDDEVICE || APP_COORDINATOR
static NWK_DataReq_t appNwkDataReq;
static SYS_Timer_t appNetworkStatusTimer;
static SYS_Timer_t appCommandWaitTimer;
static bool appNetworkStatus;
#endif

// #if APP_COORDINATOR
// static uint8_t rx_data[APP_RX_BUF_SIZE];
// #endif

static AppMessage_t appMsg;
static SYS_Timer_t appDataSendingTimer;
#define APP_COMMAND_PENDING 0x01
void UartBytesReceived(uint8_t byte[7]);

/*- Implementations --------------------------------------------------------*/

#if APP_COORDINATOR

/*****************************************************************************
*****************************************************************************/
void UartBytesReceived(uint8_t byte[7])
{
	
	isSerialCommand = true;
	appSendData();
// 	for (uint16_t i = 0; i < bytes; i++) {
// 		APP_CommandsByteReceived(byte[i]);
// 	}
}

static void appUartSendMessage(AppMessage_t *msg)
{
	sio2host_putchar(msg->command.addr1+48);
	sio2host_putchar(msg->command.addr2+48);
	sio2host_putchar(msg->command.type+48);
	sio2host_putchar(msg->command.pin0+48);
	sio2host_putchar(msg->command.pin1+48);
	sio2host_putchar(msg->command.pin2+48);
	sio2host_putchar(msg->command.pin3+48);
	//sio2host_putchar(0x0A);
}

#endif

/*****************************************************************************
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	AppMessage_t *msg = (AppMessage_t *)ind->data;
#if (LED_COUNT > 0)
	LED_Toggle(LED_DATA);
#endif
	msg->lqi = ind->lqi;
	msg->rssi = ind->rssi;
#if APP_ENDDEVICE || APP_ROUTER
	if(msg->command.type == 1)
	{
		ledStatus[0] = msg->command.pin0;
		ledStatus[1] = msg->command.pin1;
		ledStatus[2] = msg->command.pin2;
		ledStatus[3] = msg->command.pin3;
		update_led();
		for(int i=0;i<4;i++)
		{
			if(ledStatus[i]==2)
			{
				chairTime[i]=(seconds+reserveTime)%3600;
				if(chairTime[i]==0)
					chairTime[i]++;
			}
			else if(ledStatus[i]==0)
			{
				chairTime[i]=0;
			}
			
		}
	}
	if(msg->command.type == 2)
	{
		port_pin_set_output_level(irledPin[0], (msg->command.pin0 == 0));
		port_pin_set_output_level(irledPin[1], (msg->command.pin1 == 0));
		port_pin_set_output_level(irledPin[2], (msg->command.pin2 == 0));
		port_pin_set_output_level(irledPin[3], (msg->command.pin3 == 0));
	}
#endif
#if APP_COORDINATOR
	appUartSendMessage(msg);

	if (APP_CommandsPending(ind->srcAddr)) {
		NWK_SetAckControl(APP_COMMAND_PENDING);
	}
#endif
	return true;
}

/*****************************************************************************
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	if (APP_STATE_WAIT_SEND_TIMER == appState) {
		appState = APP_STATE_IDLE;
	} else {
		SYS_TimerStart(&appDataSendingTimer);
	}

	(void)timer;
}

#if APP_ROUTER || APP_ENDDEVICE

/*****************************************************************************
*****************************************************************************/

static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
#if (LED_COUNT > 0)
	LED_Toggle(LED_NETWORK);
#endif
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static void appCommandWaitTimerHandler(SYS_Timer_t *timer)
{
	appState = APP_STATE_SENDING_DONE;
	(void)timer;
}

#endif

/*****************************************************************************
*****************************************************************************/
#if APP_ROUTER || APP_ENDDEVICE || APP_COORDINATOR
static void appDataConf(NWK_DataReq_t *req)
{
#if (LED_COUNT > 0)
	LED_Off(LED_DATA);
#endif

	if (NWK_SUCCESS_STATUS == req->status) {
		if (!appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_On(LED_NETWORK);
#endif
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	} else {
		if (appNetworkStatus) {
#if (LED_COUNT > 0)
			LED_Off(LED_NETWORK);
#endif
			SYS_TimerStart(&appNetworkStatusTimer);
			appNetworkStatus = false;
		}
	}

	if (APP_COMMAND_PENDING == req->control) {
		SYS_TimerStart(&appCommandWaitTimer);
#if (LED_COUNT > 0)
		LED_Toggle(LED_NETWORK);
#endif
		appState = APP_STATE_WAIT_COMMAND_TIMER;
	} else {
		appState = APP_STATE_SENDING_DONE;
	}
}

#endif

/*****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
#ifdef NWK_ENABLE_ROUTING
	appMsg.parentShortAddr = NWK_RouteNextHop(0, 0);
#else
	appMsg.parentShortAddr = 0;
#endif

#if APP_COORDINATOR
	//appUartSendMessage((uint8_t *)&appMsg, sizeof(appMsg));
	if(isSerialCommand)
	{
			appMsg.command.addr1 = dat[0]-48;
			appMsg.command.addr2 = dat[1]-48;
			appMsg.command.type = dat[2]-48;
			appMsg.command.pin0 = dat[3]-48;
			appMsg.command.pin1 = dat[4]-48;
			appMsg.command.pin2 = dat[5]-48;
			appMsg.command.pin3 = dat[6]-48;
			appNwkDataReq.dstAddr = ((uint16_t)(dat[0]-48)<<8) + (uint16_t)(dat[1]-48);
			appNwkDataReq.dstEndpoint = APP_ENDPOINT;
			appNwkDataReq.srcEndpoint = APP_ENDPOINT;
			appNwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
			appNwkDataReq.data = (uint8_t *)&appMsg;
			appNwkDataReq.size = sizeof(appMsg);
			appNwkDataReq.confirm = appDataConf;
			#if (LED_COUNT > 0)
			LED_On(LED_DATA);
			#endif
			NWK_DataReq(&appNwkDataReq);
			isSerialCommand = false;
			appState = APP_STATE_WAIT_CONF;
	}
	SYS_TimerStart(&appDataSendingTimer);
	appState = APP_STATE_WAIT_SEND_TIMER;
#else
	
	appMsg.command.addr1 = (uint8_t) APP_ADDR>>8;
	appMsg.command.addr2 = (uint8_t) APP_ADDR & 0xff;
	appMsg.command.type = 1;
	appMsg.command.pin0 = ledStatus[0];
	appMsg.command.pin1 = ledStatus[1];
	appMsg.command.pin2 = ledStatus[2];
	appMsg.command.pin3 = ledStatus[3];
	
	appNwkDataReq.dstAddr = 0;
	appNwkDataReq.dstEndpoint = APP_ENDPOINT;
	appNwkDataReq.srcEndpoint = APP_ENDPOINT;
	appNwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	appNwkDataReq.data = (uint8_t *)&appMsg;
	appNwkDataReq.size = sizeof(appMsg);
	appNwkDataReq.confirm = appDataConf;
#if (LED_COUNT > 0)
	LED_On(LED_DATA);
#endif
	NWK_DataReq(&appNwkDataReq);

	appState = APP_STATE_WAIT_CONF;
#endif
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	appMsg.commandId            = APP_COMMAND_ID_NETWORK_INFO;
	appMsg.nodeType             = APP_NODE_TYPE;
	appMsg.extAddr              = APP_ADDR;
	appMsg.shortAddr            = APP_ADDR;
	appMsg.softVersion          = 0x01010100;
	appMsg.channelMask          = (1L << APP_CHANNEL);
	appMsg.panId                = APP_PANID;
	appMsg.workingChannel       = APP_CHANNEL;
	appMsg.parentShortAddr      = 0;
	appMsg.lqi                  = 0;
	appMsg.rssi                 = 0;

	appMsg.command.pin0			= 0;
	appMsg.command.pin1			= 0;
	appMsg.command.pin2			= 0;
	appMsg.command.pin3			= 0;

	appMsg.caption.type         = 32;
	appMsg.caption.size         = APP_CAPTION_SIZE;
	memcpy(appMsg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);

	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
#if (defined(PHY_AT86RF212B) || defined(PHY_AT86RF212))
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
#endif
	PHY_SetRxState(true);

#ifdef NWK_ENABLE_SECURITY
	NWK_SetSecurityKey((uint8_t *)APP_SECURITY_KEY);
#endif

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

#if APP_ROUTER || APP_ENDDEVICE
	appNetworkStatus = false;
	appNetworkStatusTimer.interval = 500;
	appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
	SYS_TimerStart(&appNetworkStatusTimer);

	appCommandWaitTimer.interval = NWK_ACK_WAIT_TIME;
	appCommandWaitTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appCommandWaitTimer.handler = appCommandWaitTimerHandler;
	
	// **** This is config of GPIO pins **** //
	struct port_config pin_config_led;
	struct port_config pin_config_button;
	port_get_config_defaults(&pin_config_led);
	port_get_config_defaults(&pin_config_button);
	pin_config_led.direction = PORT_PIN_DIR_OUTPUT;
	pin_config_button.direction = PORT_PIN_DIR_INPUT;
	pin_config_button.input_pull = PORT_PIN_PULL_NONE;
	for(int i=0; i<4;i++){
		port_pin_set_config(ledPin[i], &pin_config_led);
		port_pin_set_output_level(ledPin[i],true);
		port_pin_set_config(irledPin[i], &pin_config_led);
		port_pin_set_output_level(irledPin[i],false);
		port_pin_set_config(buttonPin[i], &pin_config_button);
	}
#else
#if (LED_COUNT > 0)
	LED_On(LED_NETWORK);
#endif
#endif

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
	srand(PHY_RandomReq());
#endif

	APP_CommandsInit();

	appState = APP_STATE_SEND;
}
#if APP_ROUTER
static SYS_Timer_t blinkTimer;
static void blinkTimerHandler(SYS_Timer_t *timer)
{
	seconds++;
	if(seconds > 3601) 
	 {
	 seconds = 1;
	 }
	for(int i =0; i<4;i++)
	{
		if(ledStatus[i]==2)
			port_pin_toggle_output_level(ledPin[i]);
	}
	
}

static void startBlinkTimer(void)
{
	blinkTimer.interval = 1000;
	blinkTimer.mode = SYS_TIMER_PERIODIC_MODE;
	blinkTimer.handler = blinkTimerHandler;
	SYS_TimerStart(&blinkTimer);
	appSendData();
}

#endif

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	{
		appInit();
		#if APP_ENDDEVICE || APP_ROUTER
		startBlinkTimer();
		seconds = 0;
		appState = APP_STATE_IDLE;
		#endif
	}
	break;

#if APP_ENDDEVICE || APP_ROUTER
	case APP_STATE_IDLE:
	{
		for(int i=0; i<4; i++)
		{
			if(port_pin_get_input_level(buttonPin[i]) == false)
			{
				confidence_level++;				
				if(confidence_level>=threshhold)
				{	
					confidence_level =0;
					change_status(i);
					update_led();
					appState = APP_STATE_SEND;
				}
			}
			else if(chairTime[i]==seconds)
			{
				if(ledStatus[i]==1)
				{
					ledStatus[i]++;
//					update_led();
					chairTime[i]=(seconds+reserveTime)%3600;
					if(chairTime[i]==0) chairTime[i]++;
				}
 				else if(ledStatus[i]==2)
				{
					ledStatus[i]=0;
					update_led();
					appSendData();
					chairTime[i] = 0;
					
				}
			}
		}
	}
	break;
#endif

	case APP_STATE_SEND:
	{
		appSendData();
	}
	break;

	case APP_STATE_SENDING_DONE:
	{
#if APP_ENDDEVICE
		appState = APP_STATE_PREPARE_TO_SLEEP;
#else
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_WAIT_SEND_TIMER;
#endif
	}
	break;

#if APP_ENDDEVICE
	case APP_STATE_PREPARE_TO_SLEEP:
	{
		if (!NWK_Busy()) {
			NWK_SleepReq();
			appState = APP_STATE_SLEEP;
		}
	}
	break;

	case APP_STATE_SLEEP:
	{
		sm_sleep(APP_SENDING_INTERVAL / 1000);
		appState = APP_STATE_WAKEUP;
	}
	break;

	case APP_STATE_WAKEUP:
	{
		NWK_WakeupReq();

		/*
		 * #if (LED_COUNT > 0)
		 *    LED_On(LED_NETWORK);
		 #endif*/

		appState = APP_STATE_IDLE;
	}
	break;
#endif
	default:
		break;
	}

#if (APP_COORDINATOR)
	
	int incoming = sio2host_getchar_nowait();
	while(incoming >= 0)
	{
		dat[byteCount] = (uint8_t) incoming;
		byteCount++;
		incoming = sio2host_getchar_nowait();
	}
	if(byteCount >= 7)
	{
		byteCount = 0;
		UartBytesReceived(dat);
		
	}

// 	uint16_t bytes;
// 	if ((bytes = sio2host_rx(rx_data, APP_RX_BUF_SIZE)) > 0) {
// 		UartBytesReceived(bytes, (uint8_t *)&rx_data);
// 	}
#endif
}

/*****************************************************************************
*****************************************************************************/

/**
 * Init function of the WSNDemo application
 */
void wsndemo_init(void)
{
	SYS_Init();
#if APP_ENDDEVICE
	sm_init();
#endif
#if APP_COORDINATOR
	sio2host_init();
#endif
}

/**
 * Task of the WSNDemo application
 * This task should be called in a while(1)
 */
void wsndemo_task(void)
{
	SYS_TaskHandler();
	APP_TaskHandler();
}

#if APP_ENDDEVICE || APP_ROUTER
void change_status(int pin)
{
	if(ledStatus[pin] == 0)
	{
		 ledStatus[pin]++;
		 chairTime[pin]=(seconds+occupiedTime)%3600;
		 if(chairTime[pin]==0) chairTime[pin]++;
	}
	else if(ledStatus[pin] == 1)
	{
		ledStatus[pin]--;
		chairTime[pin]=0;
	}
	else if(ledStatus[pin] == 2)
	{
		ledStatus[pin]--;
		chairTime[pin]=(seconds+occupiedTime)%3600;
		if(chairTime[pin]==0) chairTime[pin]++;
	}
}

void update_led(void)
{
	for(int i=0; i<4; i++)
	{
		if(ledStatus[i] == 1) port_pin_set_output_level(ledPin[i],false);
		if(ledStatus[i] == 0) port_pin_set_output_level(ledPin[i],true);
	}
}

#endif