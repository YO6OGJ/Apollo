/*
 * apollo_diagnostic.c
 *
 * Created: 05/12/2019 21:38:35
 *  Author: Nagy Csaba YO6PVH
 */ 
#include "apollo_diagnostic.h"
#include "kernel.h"
#include "apollo.h"
static char ser_buffer[20];
static uint8_t buffer_wr_cnt;

void DiagnosticRecieveUSBchar(uint8_t c)
{	
	int16_t i;
	int32_t freq;
	double vswr;
	ser_buffer[buffer_wr_cnt] = c;
	if (buffer_wr_cnt < sizeof(ser_buffer) - 1)
	{
		buffer_wr_cnt++;
	}
	else
	{
		buffer_wr_cnt = 0;	
	}
	
	if ((buffer_wr_cnt >= 2)&&(ser_buffer[buffer_wr_cnt - 1] == 10))//Read characters until linefeed
	{
		switch(ser_buffer[0])
		{
			case 'E':			
				INFO("Erasing EEPROM!");
				EraseEEpromStorage();
				INFO("DONE");
			break;
			case 'M':
				INFO("Working");
				ATU_EEPROM_SearchClosestTune(5,CLOSE_FREQUENCY_TUNE);
			break;
			case 'F':				
					i = atoi(&ser_buffer[1]);
					if (i == 0)
					{
						INFO("Disalbe LPF");
						disable_lpf ();
					}
					else
					{
						INFO("Setting band to %u Mhz",lpf_get_FreqForBand(i-1)/1000000);
						frequency = lpf_get_FreqForBand(i-1);
						enable_lpf ();
						update_lpf ();
					}
			break;
			case 'L':
				i = atoi(&ser_buffer[1]);
				if (i == 0)
				{
					INFO("Short All inductors");
					latch_inductors(0);					
				}
				else
				{
					INFO("Connected L%d",i);
					latch_inductors(1 << (i - 1));
				}
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				vswr = get_vswr();
				INFO ("VSWR=%.2f", vswr);
			break;
			case 'l':
			i = atoi(&ser_buffer[1]);
			
			latch_inductors(i);
			usleep (RELAY_SETTLE_TIME);
			usleep (RELAY_SETTLE_TIME);
			usleep (RELAY_SETTLE_TIME);
			vswr = get_vswr();
			INFO ("VSWR=%.2f", vswr);
			break;
			case 'C':
				i = atoi(&ser_buffer[1]);
				if (i == 0)
				{
					INFO("Disconenct all capacitors");
					latch_capacitors_signed(0);
				}
				else
				{
					if (i < 0)
					{
						INFO ("LC configuration C%d", -i)
						latch_capacitors_signed(-(1 << (-i-1)));
					}
					else
					{
						INFO("CL configuration C%d", i)
						latch_capacitors_signed((1 << (i-1)));
					}
					
				}
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				vswr = get_vswr();
				INFO ("VSWR=%.2f", vswr);
			break;
			case 'c':
				i = atoi(&ser_buffer[1]);
				latch_capacitors_signed(i);	
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				usleep (RELAY_SETTLE_TIME);
				vswr = get_vswr();
				INFO ("VSWR=%.2f", vswr);	
			break;
			case 'T':
				freq = atol(&ser_buffer[1]);
				atu_tune(freq);				
			break;	
			case 'P':
				i = atoi(&ser_buffer[1]);
				if (!ptt_enabled)
				INFO ("PTT enabled with timeout %u ms\n\r", i);
				enable_ptt (i);
				//usleep (500000);
				//INFO ("SWR:%f",get_vswr ());
				//usleep (500000);
			
				
			break;	
			case 'S':
				INFO ("SWR:%f",get_vswr ());
			break;
			case 's':
				get_phi_filtered ();
			break;
			default:
				DiagnosticPrintHelp();
			break;
		}
		buffer_wr_cnt = 0;		
	}	
}

void DiagnosticPrintHelp()
{
	INFO("Test ATU functionality use following commands:");
	INFO("To disable filter: F0 to enable filter step Fn n=1->6");
	INFO("To short inductors: L0 to enable one inductor Ln n=1->8");
	INFO("To doconnect capacitors: C0 to connect one capacitor in Cn n=-8->8");	
	INFO("To connect Capacitor value use cn n=-255->255");
	INFO("To connect Inductor value use ln n=0->255");
	INFO("Ptt with timeout Pn n=ms");
	INFO("Tune Tn n=freq");
	INFO("Erase EEPROM E")
}