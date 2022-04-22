/* Apollo ATU firmware
	 Copyright (C) 2016  Espen S. Johnsen, LA7DJA

	 This program is free software: you can redistribute it and/or modify
	 it under the terms of the GNU General Public License as published by
	 the Free Software Foundation, either version 3 of the License, or
	 (at your option) any later version.
	 
	 This program is distributed in the hope that it will be useful,
	 but WITHOUT ANY WARRANTY; without even the implied warranty of
	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	 GNU General Public License for more details.
	 
	 You should have received a copy of the GNU General Public License
	 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "kernel/include/spi.h"

//#undef USB_DEBUG // uncomment to disable debugging output from this file
#include "apollo.h"

#include "VirtualSerial.h"
#include "apollo_diagnostic.h"
#define ATU_CNT_TO_ERASE_EEPROM		10
#define ATU_TIME_TO_ERASE_EEPROM	20

/* Message constants */

#define MSG_SET_FREQ 1
#define MSG_ENABLE_PTT 2
#define MSG_DISABLE_PTT 3
#define MSG_START_TUNING 4
#define MSG_ABORT_TUNING 5
#define MSG_GET_STATUS 6
#define MSG_GET_VERSION 7

spi_device_t spi = {
	.mode = SPI_MODE_SLAVE,
	.output = true,
	.clock_polarity = SPI_CLOCK_IDLE_LOW,
	.clock_phase = SPI_SAMPLE_LEADING_EDGE,
	.bit_order = SPI_LSB_FIRST
};

uint32_t frequency;

struct eeprom EEMEM settings;
lock_t eraselock;
volatile uint16_t erasetime;
volatile uint8_t erasecnt;


TASK(heartbeat, TASK_PRIORITY_MEDIUM, TASK_DEFAULT_STACK_SIZE)
{	
	make_output (LED2);  
	while (true) {
		lock_acquire(&eraselock);
		if (erasetime)
		{		 
			erasetime--; 		
		}
		else
		{
			erasecnt = 0;
		}		
		lock_release(&eraselock);
		
		toggle_port (LED2);	
		usleep (100000);		
	}
}


TASK(usbdebug, TASK_PRIORITY_MEDIUM + 3, 1024)
{	
	int16_t USBrecieved_char;
	usleep (1000);
	VirtualSerialInit();	
	while (true) 
	{					
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			USBrecieved_char = VirtualSerialProcess();	
			if (USBrecieved_char > 0)
			{
				DiagnosticRecieveUSBchar((uint8_t) (USBrecieved_char & 0xFF));		
			}
		}
		usleep (20000);//20ms sleep
	}
}


lock_t ptt_lock;
condition_t ptt_change;
time_t ptt_timeout = 0;
volatile bool ptt_enabled = false;

TASK (ptt, TASK_PRIORITY_HIGH, TASK_DEFAULT_STACK_SIZE)
{	
	pull_down (PTT_LED);
	pull_down (PTT_OUT);
	lock_acquire (&ptt_lock);
	while (true) {
		
		tick_t time = get_system_ticks ();
		if (time_after(ptt_timeout, time)) {
			set_port (PTT_LED);
			set_port (PTT_OUT);
			ptt_enabled = true;
			condition_wait_with_wakeup (&ptt_change, &ptt_lock, ptt_timeout);
			} else {
			clear_port (PTT_OUT);
			clear_port (PTT_LED);
			ptt_enabled = false;
			condition_wait (&ptt_change, &ptt_lock);
		}
	}
}

void enable_ptt (uint16_t timeout)
{	
  lock_acquire (&ptt_lock);  
  ptt_timeout = get_system_ticks () + ms_to_ticks (timeout);
  condition_signal (&ptt_change);
  lock_release (&ptt_lock);
}

void disable_ptt ()
{
  lock_acquire (&ptt_lock);
  ptt_timeout = get_system_ticks();
  condition_signal (&ptt_change);
  lock_release (&ptt_lock);
}

static void read_msg_data (uint32_t *msg)
{
	uint8_t byte = (uint8_t)*msg;
	for (uint8_t n = 0; n < 3; n++) {
		byte = spi_transfer_byte(&spi, ~byte);
		*msg = *msg << 8 | byte;
	}
	spi_transfer_byte(&spi, ~byte);
}

static void write_msg_data (uint32_t msg)
{
	for (uint8_t n = 0; n < 4; n++) {
		uint8_t byte = (uint8_t)(msg >> 24);
		spi_transfer_byte(&spi, ~byte);
		msg = msg << 8;
	}
}

#define SPI_QUE_SIZE 20
volatile uint32_t g_SPI_Rx_Que[SPI_QUE_SIZE];
volatile uint8_t g_Rx_Que_cnt = 0;
lock_t spi_que_lock;
volatile bool QueOverFlow = false;

bool SPI_AddTOQue(uint32_t message)
{
	lock_acquire (&spi_que_lock);
	if(g_Rx_Que_cnt < SPI_QUE_SIZE)
	{
		g_SPI_Rx_Que[g_Rx_Que_cnt] = message;
		g_Rx_Que_cnt++;
		lock_release (&spi_que_lock);
		return true;
	}
	else
	{
		QueOverFlow = true;
	}
	lock_release (&spi_que_lock);
	return false;
}
bool SPI_PopQue(uint32_t* message)
{
	lock_acquire (&spi_que_lock);
	if (QueOverFlow)
	{
		INFO("SPI Que Overflow!")
		QueOverFlow = false;
	}
	if(g_Rx_Que_cnt > 0)
	{
		g_Rx_Que_cnt--;
		*message = g_SPI_Rx_Que[g_Rx_Que_cnt];
		
		lock_release (&spi_que_lock);
		return true;
	}
	lock_release (&spi_que_lock);
	return false;
}
TASK (spi_comm, 9, TASK_DEFAULT_STACK_SIZE)
{	
	//Critical task to exchange data with Hermes or other radios using the SPI.
	//Do not use print,INFO in this thread or any blocking functions
	//Received messages are shoved into a que then processed by the main thread.
	 uint32_t msg;
	 spi_init ();
	 spi_acquire (&spi);
	 while(true)
	 {
		 //toggle_port (LED2);	
		msg = spi_transfer_byte(&spi, 0xff);		
		switch (msg) {
		case MSG_SET_FREQ: 
				read_msg_data(&msg); //
				//Set frequecny message add to que
				SPI_AddTOQue(msg);				
				break;
			
      case MSG_ENABLE_PTT: 
				read_msg_data(&msg);
				//PTT message add to que
				SPI_AddTOQue(msg);	
				break;			
      case MSG_DISABLE_PTT:
				read_msg_data(&msg);
				//PTT message add to que
				SPI_AddTOQue(msg);	
				break;
				
      case MSG_START_TUNING:
				read_msg_data(&msg);
				//Tune message add to que
				SPI_AddTOQue(msg);	
				break;

      case MSG_ABORT_TUNING:
				read_msg_data(&msg);
				SPI_AddTOQue(msg);	
				break;

			case MSG_GET_STATUS:
				msg = msg << 24 | atu_get_status ();
				write_msg_data (msg);
				break;

			case MSG_GET_VERSION:
				msg = msg << 8 | VERSION_MAJOR;
				msg = msg << 8 | VERSION_MINOR;
				msg = msg << 8 | VERSION_MICRO;
				write_msg_data(msg);				
				break;
					
			default:
				/* Unknown message, read error or out of sync. */
				break;
		}
	 }
}

int main ()
{	
	INFO ("Initializing");
	init_lpf ();
	init_atu ();   
	INFO ("Starting main loop");
	pull_up (STATUS);
	uint32_t spimsg;
	
	while (true) 
	{
		if (SPI_PopQue(&spimsg))
		{
			//INFO("SPIH:%X",spimsg>>16);
			
			switch (spimsg >> 24) 
			{
				case MSG_SET_FREQ:
				{
					if ((atu_get_status() >> 20) != ATU_STATUS_IN_PROGRESS)
					{						
						frequency = (spimsg & 0x003fffff) << 4;
						bool lpf = spimsg & 0x00800000;
						bool atu = spimsg & 0x00400000;
						INFO ("Setting frequency to %lu Hz", frequency);
						ATU_CalculateLimits(frequency);						
						if (lpf) {						
							enable_lpf ();
							update_lpf ();						
							INFO("Band %d",lpf_get_newband ());
						} else					
							disable_lpf ();	
						if (atu)
						{
							TuneFromMemory(frequency);
						}				
						else
						{
							latch_inductors (0); //Tuner is on bypass
							latch_capacitors_signed (0);
						}
					}
				break;			
				}
				case MSG_ENABLE_PTT:
				{
					uint16_t timeout = spimsg & 0xffff;
					if (!ptt_enabled)
						INFO ("PTT enabled with timeout %u ms\n\r", timeout);
					enable_ptt (timeout);				
				break;			
				}
      case MSG_DISABLE_PTT:				
				/* This message seems to create some instability when tuning,
					 so for now it is disabled. PTT will be disabled after a
					 short timeout when Hermes stops sendig PTT enable updates */
				//This may be related to the block with wakeup problem in the kernel.
				disable_ptt ();
			    INFO ("PTT disabled\n\r");
				break;				
      case MSG_START_TUNING:				
				//If tune button is pressed 5 times in a short period. all tune storage erased. Ex antenna change
				lock_acquire(&eraselock);
				erasetime = ATU_TIME_TO_ERASE_EEPROM;
				if (erasecnt % 2 == 0)
					erasecnt++;
				if (erasecnt >= ATU_CNT_TO_ERASE_EEPROM)
				{
					INFO("Erasing EEPROM!");
					TriggerEEpromErase();
				}
				lock_release(&eraselock);
				atu_tune (frequency);
				break;

      case MSG_ABORT_TUNING:
				INFO("Abort tuning message");
				if (erasecnt % 2 == 1)
					erasecnt++;
				// TODO
				break;

			case MSG_GET_STATUS:
				INFO("Status read");
				break;

			case MSG_GET_VERSION:
				INFO("Version read");
				break;					
			default:
				/* Unknown message, read error or out of sync. */
				INFO ("Unknown\n\r");
		}		   
	  }
	  usleep (10000);	
  }
}
