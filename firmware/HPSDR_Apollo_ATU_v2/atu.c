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

#include <math.h>
#include <avr/power.h>
#include "kernel/include/analog.h"
#include "kernel/include/kernel.h"

//#undef USB_DEBUG // uncomment to disable debugging output from this file
#include "apollo.h"

//Inductor values in nH
//100,175,350,600,1250,2500,5000,10000
const uint16_t L_VALUES_nH[] = {100,175,350,600,1250,2500,5000,10000};
	
//Capacitor values in pF
//10,22,39,82,150,330,680,1360
const uint16_t C_VALUES_pF[] = {10,22,39,82,150,330,680,1360};
#define FWD_CHANNEL 0
#define REV_CHANNEL 1

lock_t atu_lock;
condition_t atu_tune_cond;
uint32_t tunig_frequency;
uint8_t atu_status = ATU_STATUS_RESET;

uint8_t active_inductor_relays;
int16_t active_capacitor_relays;
uint8_t EEpromEraseTriggerd = 0;
bool FirstTune = true;

#define L1S E, 7
#define L1R E, 6
#define L2S D, 5
#define L2R D, 4
#define L3S E, 1
#define L3R E, 0
#define L4S E, 5
#define L4R E, 4
#define L5S B, 6
#define L5R B, 7
#define L6S B, 5
#define L6R B, 4
#define L7S E, 3
#define L7R F, 7
#define L8S D, 3
#define L8R D, 2

#define C1S A, 0
#define C1R A, 1
#define C2S A, 2
#define C2R A, 3
#define C3S A, 4
#define C3R A, 5
#define C4S A, 6
#define C4R A, 7
#define C5S C, 7
#define C5R C, 6
#define C6S C, 5
#define C6R C, 4
#define C7S C, 3
#define C7R C, 2
#define C8S C, 1
#define C8R C, 0

#define CBANKS D, 7
#define CBANKR D, 6


bool latch_inductors (int value)
{
  if (value != active_inductor_relays) {
    uint8_t set = (value ^ active_inductor_relays) & value;
    uint8_t reset = (value ^ active_inductor_relays) & ~value;
    active_inductor_relays = value;
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); set_port (L1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); set_port (L2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); set_port (L3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); set_port (L4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); set_port (L5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); set_port (L6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); set_port (L7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); set_port (L8S); }
        
    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); set_port (L1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); set_port (L2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); set_port (L3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); set_port (L4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); set_port (L5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); set_port (L6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); set_port (L7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); set_port (L8R); }
        
    usleep (RELAY_LATCH_TIME);
        
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (L1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (L2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (L3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (L4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (L5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (L6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (L7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (L8S); }
                
    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (L1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (L2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (L3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (L4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (L5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (L6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (L7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (L8R); }

    return true;
  }
        
  return false;
}


bool latch_capacitors (int value)
{
  if (value != active_capacitor_relays) {
    uint16_t set = (value ^ active_capacitor_relays) & value;
    uint16_t reset = (value ^ active_capacitor_relays) & ~value;
    active_capacitor_relays = value;
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); set_port (C1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); set_port (C2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); set_port (C3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); set_port (C4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); set_port (C5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); set_port (C6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); set_port (C7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); set_port (C8S); }
    if (set & 0x100) { usleep (RELAY_LATCH_DELAY); set_port (CBANKS); }

    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); set_port (C1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); set_port (C2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); set_port (C3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); set_port (C4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); set_port (C5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); set_port (C6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); set_port (C7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); set_port (C8R); }
    if (reset & 0x100) { usleep (RELAY_LATCH_DELAY); set_port (CBANKR); }
                
    usleep (RELAY_LATCH_TIME);
                
    if (bit_is_set (set, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (C1S); }
    if (bit_is_set (set, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (C2S); }
    if (bit_is_set (set, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (C3S); }
    if (bit_is_set (set, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (C4S); }
    if (bit_is_set (set, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (C5S); }
    if (bit_is_set (set, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (C6S); }
    if (bit_is_set (set, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (C7S); }
    if (bit_is_set (set, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (C8S); }
    if (set & 0x100) { usleep (RELAY_LATCH_DELAY); clear_port (CBANKS); }

    if (bit_is_set (reset, 0)) { usleep (RELAY_LATCH_DELAY); clear_port (C1R); }
    if (bit_is_set (reset, 1)) { usleep (RELAY_LATCH_DELAY); clear_port (C2R); }
    if (bit_is_set (reset, 2)) { usleep (RELAY_LATCH_DELAY); clear_port (C3R); }
    if (bit_is_set (reset, 3)) { usleep (RELAY_LATCH_DELAY); clear_port (C4R); }
    if (bit_is_set (reset, 4)) { usleep (RELAY_LATCH_DELAY); clear_port (C5R); }
    if (bit_is_set (reset, 5)) { usleep (RELAY_LATCH_DELAY); clear_port (C6R); }
    if (bit_is_set (reset, 6)) { usleep (RELAY_LATCH_DELAY); clear_port (C7R); }
    if (bit_is_set (reset, 7)) { usleep (RELAY_LATCH_DELAY); clear_port (C8R); }
    if (reset & 0x100) { usleep (RELAY_LATCH_DELAY); clear_port (CBANKR); }

    return true;
  }

  return false;
}


int calculate_signed_capacitor_value(uint16_t unsigned_value)
{
	if ((unsigned_value & 0x100) == 0)
		return unsigned_value;
	return -(unsigned_value & 0x00FF);
}

uint16_t calculate_unsigned_capacitor_value(int value)
{
	return value < 0 ? 0x100 - value : value;
}

bool latch_capacitors_signed (int value)
{
  uint16_t unsigned_value = value < 0 ? 0x100 - value : value;
  return latch_capacitors (unsigned_value);
}
        
 int32_t fwd_offset = 0;
 int32_t rev_offset = 0;
void init_atu ()
{
  make_output (L1S);
  make_output (L2S);
  make_output (L3S);
  make_output (L4S);
  make_output (L5S);
  make_output (L6S);
  make_output (L7S);
  make_output (L8S);
  make_output (L1R);
  make_output (L2R);
  make_output (L3R);
  make_output (L4R);
  make_output (L5R);
  make_output (L6R);
  make_output (L7R);
  make_output (L8R);

  make_output (C1S);
  make_output (C2S);
  make_output (C3S);
  make_output (C4S);
  make_output (C5S);
  make_output (C6S);
  make_output (C7S);
  make_output (C8S);
  make_output (C1R);
  make_output (C2R);
  make_output (C3R);
  make_output (C4R);
  make_output (C5R);
  make_output (C6R);
  make_output (C7R);
  make_output (C8R);
  
  make_output (CBANKS);
  make_output (CBANKR);
        
  //uint8_t inductors = eeprom_read_byte(&settings.atu_inductors);
  //uint16_t capacitors = eeprom_read_word(&settings.atu_capacitors);

  /* Make sure the relays are in expected states. */
  //active_inductor_relays = ~inductors;  
  //active_capacitor_relays = ~capacitors;        
  //latch_inductors (inductors);
  //latch_capacitors (capacitors);
        
  power_adc_enable ();
  /* adc_set_reference (ADC_REFERENCE_VCC); */
  adc_set_reference (ADC_REFERENCE_INTERNAL);
  adc_enable ();
  usleep (100000);	//Wait for ADC to settle
  fwd_offset = 0;
  rev_offset = 0;
  for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
	  fwd_offset += adc_read_channel (FWD_CHANNEL, NULL);
	  rev_offset += adc_read_channel (REV_CHANNEL, NULL);
  }
}
int32_t sumfwd = 0;
double get_phi_filtered ()
{
	//When the radio is in limiting mode. The signal level fluctuates greatly
	//We fill try to filter the measured values for a reliable measurement
	int16_t fwd[ADC_SAMPLE_COUNT / 10];
	int16_t rev[ADC_SAMPLE_COUNT / 10];
	uint16_t index = 0;
	
	int32_t sumrev = 0;	
	int32_t dev = 0;
	while(1)
	{
		fwd[index % 10] = 0;
		rev[index % 10] = 0;
		for (int i = 0; i < 10; i++) {
			fwd[index % 10] += adc_read_channel (FWD_CHANNEL, NULL);
			rev[index % 10] += adc_read_channel (REV_CHANNEL, NULL);
		}
		if (index *10 > ADC_SAMPLE_COUNT)
		{
			sumfwd = 0;
			//We have enough samples. Calculate deviation
			for (int i = 0; i < ADC_SAMPLE_COUNT / 10; i++) 
			{
				sumfwd += fwd[i];
			}
			sumfwd /= (ADC_SAMPLE_COUNT / 10);			
			dev = 0;	
			for (int i = 0; i < ADC_SAMPLE_COUNT / 10; i++) 
			{
				dev += abs(fwd[i] - sumfwd);
			}
			dev /= (ADC_SAMPLE_COUNT / 10);
			if (dev < 2)
			{
				//INFO("dev=%d", dev);
				break;
			}
		}
		index++;	
	}	
	sumfwd = 0;
	sumrev = 0;
	for (int i = 0; i < ADC_SAMPLE_COUNT / 10; i++) 
	{
			sumfwd += fwd[i];
			sumrev += rev[i];
	}
	//INFO("Fwd=%.1f", (double)sumfwd / ADC_SAMPLE_COUNT);
	//INFO("Rev=%.1f", (double)sumrev / ADC_SAMPLE_COUNT);	
	double phi = (double)sumrev / (double)sumfwd;
	sumfwd /= ADC_SAMPLE_COUNT;
	//INFO("%.4f",phi);
	//INFO("swr=%.1f",(1 + phi) / (1 - phi));
	return phi;
}

double get_phi ()
{
  int32_t fwd = 0;
  int32_t rev = 0;  
        
  for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
    fwd += adc_read_channel (FWD_CHANNEL, NULL);
    rev += adc_read_channel (REV_CHANNEL, NULL);
  }
  //fwd -= fwd_offset;
  //if (fwd < 0)
	//fwd = 0;
  //rev -= rev_offset;
  //if (rev < 0)
	//rev = 0; 	
  
        
  /* Assuming readout is linear, which may not be correct */
  double phi = (double)rev / (double)fwd;
   int16_t ifwd = fwd / ADC_SAMPLE_COUNT;
   int16_t irev = rev / ADC_SAMPLE_COUNT;
INFO("Fwd=%d Rev=%d Phi=%d", ifwd, irev,(int)(phi*1000));
  return phi;
}

double get_vswr ()
{
  double phi = get_phi ();
  return (1 + phi) / (1 - phi);
}

double compute_vswr (double phi)
{
  return (1 + phi) / (1 - phi);
}


static double latch_relays (bool (*latch)(int), int index)
{
  if (latch (index))
    usleep (RELAY_SETTLE_TIME); // TODO: adaptive waiting
	INFO("I:%d",index);
  return get_phi ();
}


static int binary_search (bool (*latch)(int), int min, int max)
{
  if (!ptt_enabled) return 0;
        
  double phi_min;
  double phi_max;

  /* If the order is reversed when min < max, we will avoid an extra
     latching step (ie. the first call to latch_relays will not change
     the setting of any relays) when the capacitor range is extended
     in negative direction */
  if (min < max) {
    phi_min = latch_relays (latch, min);
    phi_max = latch_relays (latch, max);
  } else {
    phi_max = latch_relays (latch, max);
    phi_min = latch_relays (latch, min);
  }

  while (ptt_enabled && min != max) {
		//    INFO("min=%d max=%d", min, max);
    if (phi_min < phi_max) {
      /* Select lower half */
      max = min + (max - min) / 2;
      phi_max = latch_relays (latch, max);                      
    } else {
      /* Select upper half */
      min = max - (max - min) / 2;
      phi_min = latch_relays (latch, min);
    }           
  }
  INFO("val=%d", min);
  return min;
}

void atu_tune(uint32_t frequency)
{
  lock_acquire (&atu_lock);
	if (atu_status != ATU_STATUS_IN_PROGRESS) {
		tunig_frequency = frequency;
		condition_signal (&atu_tune_cond);
	}
  lock_release (&atu_lock);
}


void reset_atu()
{
  lock_acquire (&atu_lock);
  tunig_frequency = 0;
  condition_signal (&atu_tune_cond);
  lock_release (&atu_lock);
}

void disable_atu()
{
  reset_atu ();
}

void enable_atu()
{
  /* Currently a no-op, should restore settings from EEPROM */
}

uint32_t atu_get_status ()
{
  lock_acquire (&atu_lock);
  pull_down (STATUS);
  uint32_t status = 0;
  if (atu_status != ATU_STATUS_IN_PROGRESS) {
		status = active_capacitor_relays;
		status = (status & 0x1ff) << 8;			
    status |= active_inductor_relays;
	}
  status |=  (uint32_t)atu_status << 20;
  lock_release (&atu_lock);
  return status;
}





void ATU_LthenCtune()
{
	//This tuning method is based on the AT-100ProII manual
	//First crose tunes the L value
	//Then corse tunes the C value
	//Decides if positive or negative C is needed
	//Fine tunes with the same strategy but using the corse values.
	//Should be faster then the matrix tune.
	//This tune is fast. But sometimes when the capacitor is croase tuned gets stuck in a local minium with the wrong value.
	//Step size should be optimized.
	
	int16_t inductors_max, capacitors_max,inductors_min, capacitors_min;
	double Phi_min = 10;
	int16_t L_AtMin = 0,C_AtMin = 0;
	uint8_t Lsteps = 8; //We scan 8 inductor values
	uint8_t Csteps = 9;//We scan 9 capacitor values
	double Cstep,Lstep;
	capacitors_min = -255;
	capacitors_max = 255;
	inductors_min = 0;
	inductors_max = 255;
	uint8_t tuning = 1;
	int16_t l,c,tempL,tempC;
	
	tempC = 0;
	while (1)
	{
		//First croase tune the L
		latch_capacitors_signed(tempC);
		Lstep = (double)(inductors_max - inductors_min)/(Lsteps - 1);
		for (l = 0; l < Lsteps; l++)
		{
			tempL = inductors_min + Lstep*l;
			latch_inductors(tempL);
			usleep (RELAY_SETTLE_TIME);			
			double phi = get_phi_filtered ();
			INFO("%d, %d, %d",tempC,tempL,(int)(phi * 1000));
			if (phi < Phi_min)
			{
				Phi_min = phi;
				L_AtMin = tempL;
				C_AtMin = tempC;
			}
		}	
		
		tempL = L_AtMin;
		latch_inductors(tempL);
		Cstep = (double)(capacitors_max - capacitors_min)/(Csteps - 1);
		for (c = 0; c < Csteps; c++)
		{
			tempC = capacitors_min + Cstep*c;
			latch_capacitors_signed(tempC);
			usleep (RELAY_SETTLE_TIME);
			//INFO("%d, %d",tempC,tempL);
			double phi = get_phi_filtered ();
			INFO("%d, %d, %d",tempC,tempL,(int)(phi * 1000));
			if (phi < Phi_min)
			{
				Phi_min = phi;
				L_AtMin = tempL;
				C_AtMin = tempC;
			}
		}
		
		tempC = C_AtMin;
		INFO("M %d, %d",L_AtMin,C_AtMin);
		if (tuning == 0)
		{
			latch_inductors(L_AtMin);
			latch_capacitors_signed(C_AtMin);
			break;
		}
		inductors_min = max(0, L_AtMin - Lstep);
		inductors_max = min(255, L_AtMin + Lstep);
		capacitors_min = max(-255, C_AtMin - Cstep);
		capacitors_max = min(255, C_AtMin + Cstep);
		if (capacitors_max - capacitors_min < Csteps)
		{
			Csteps = capacitors_max - capacitors_min;
			tuning = 0;
		}
		if (inductors_max - inductors_min < Lsteps)
		{
			Lsteps = inductors_max - inductors_min;
			tuning = 0;
		}
	}
}


////////////////////Global variables used by this method.
int16_t g_currentC;
uint8_t g_currentL;
uint8_t g_Lsteps = 10; //We scan 8 inductor values
uint8_t g_Csteps = 10;//We scan 9 capacitor valuesuint8_t g_currentL;
int16_t g_CapacitorSign = 1;
int16_t g_capacitors_min = 0;
int16_t g_capacitors_max = 255;
int16_t g_inductors_min = 0;
int16_t g_inductors_max = 255;
double g_Cstep;// = (double)(g_capacitors_max - g_capacitors_min)/(g_Csteps - 1);
double g_Lstep;// = (double)(g_inductors_max - g_inductors_min)/(g_Lsteps - 1);

void ATU_CalculateLimits(uint32_t tunig_frequency)
{	
	//XC = 1/(2*pi*f*C) => 
	if (tunig_frequency == 0)
	{
		
		return;
	}
	uint16_t C_limPf = ((double)1e12 / (2.0*3.1415*(double)tunig_frequency*XC_MIN_LIMIT));
	uint16_t L_limnH = ((double)XL_MAX_LIMIT * 1e9) / (2.0*3.1415*(double)tunig_frequency);
	INFO("C limit:%d pF",C_limPf);
	INFO("L limit:%d nH",L_limnH);
	uint16_t sum;
	sum = 0;
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		sum += C_VALUES_pF[i];
		if (sum > C_limPf)
		{			
			break;
		}
	}
	if (i >= 8)
		i  = 7;
	g_capacitors_max = (1 << (i + 1)) - 1; 
	INFO("C %d-%d",i,g_capacitors_max);
	sum = 0;
	for (i = 0; i < 8; i++)
	{
		sum += L_VALUES_nH[i];
		if (sum > L_limnH)
		{			
			break;
		}
	}
	if (i >= 8)
		i  = 7;
	g_inductors_max = (1 << (i+1)) - 1;
	INFO("L %d-%d",i,g_inductors_max);
	g_capacitors_min = 0;
	g_inductors_min = 0;
}

void ATU_CroaseCapacitor()
{
	double Phi;
	double Phimin;	
	int16_t c,tempC;
	tempC = 0;
	g_currentC = 0;
	//latch_capacitors_signed(0);
	//usleep (RELAY_SETTLE_TIME);
	//Phi = get_phi_filtered ();
	Phimin = 1; 
	for (c = 0; c < g_Csteps; c++)
	{
		tempC = g_capacitors_min + g_Cstep*c;
		latch_capacitors_signed(tempC * g_CapacitorSign);
		usleep (RELAY_SETTLE_TIME);		
		INFO("C%d",tempC);
		Phi = get_phi_filtered ();		
		if (Phi < Phimin)
		{
			INFO("Store%.4f",Phi);
			Phimin = Phi * (double)1.02; //We leave a 2% headroom original source used the SWR as value
			INFO("Smin%.4f",Phimin);
			g_currentC = tempC;
			//Maybe we check Phi if SWR is smaller than 1.2 and break
		}
		else
		{
			INFO("BREAK");
			break;
		}
	}
	latch_capacitors_signed(g_currentC * g_CapacitorSign);
	usleep (RELAY_SETTLE_TIME);
}
	
void ATU_Crouse()
{
	double Phi;
	double Phimin = 1;
	int8_t l,tempL;
	int16_t store_cap = g_currentC;
	for (l = 0; l < g_Lsteps; l++)
	{
		tempL = g_inductors_min + g_Lstep*l;
		latch_inductors(tempL);
		usleep (RELAY_SETTLE_TIME);
		INFO("L%d",tempL);
		ATU_CroaseCapacitor();
		Phi = get_phi_filtered ();
		//INFO("%d, %d, %d",tempC,tempL,(int)(phi * 1000));
		if (Phi < Phimin)
		{
			INFO("Store%.4f",Phi);
			Phimin = Phi * (double)1.02; //We original source used the SWR as value
			g_currentL = tempL;
			store_cap = g_currentC;	//Also save mem capacitor
			//Maybe we check Phi if SWR is smaller than 1.2 and break
		}
		else
		{
			INFO("BREAK");
			break;
		}
	}
	latch_inductors(g_currentL);
	g_currentC = store_cap;
	latch_capacitors_signed(g_currentC * g_CapacitorSign);
	INFO("L%d,C%d",g_currentL,g_currentC);
	usleep (RELAY_SETTLE_TIME);
}

void ATU_SharpCapacitor()
{
	int16_t minrange,maxrange,c;
	double Phi;
	double Phimin = 1000;
	minrange = floor(g_currentC - g_Cstep);
	if (minrange < g_capacitors_min)
		minrange = g_capacitors_min;
	maxrange = ceil(g_currentC + g_Cstep);
	if (maxrange > g_capacitors_max)
		maxrange = g_capacitors_max;
	
	for (c = minrange + 1; c<= maxrange; c++)
	{
		latch_capacitors_signed(c * g_CapacitorSign);
		usleep (RELAY_SETTLE_TIME);
		Phi = get_phi_filtered ();
		if (Phi >= Phimin){usleep(10000);Phi = get_phi_filtered ();}
		if (Phi >= Phimin){usleep(10000);Phi = get_phi_filtered ();}
		if (Phi <= Phimin)
		{
			Phimin = Phi;
			g_currentC = c;
		}
		else
		{
			INFO("BREAK");
			break;
		}
	}
	latch_capacitors_signed(g_currentC * g_CapacitorSign);
	usleep (RELAY_SETTLE_TIME);	
}

void ATU_SharpInductor()
{	
	int16_t minrange,maxrange,l;
	double Phi;
	double Phimin = 1000;
	minrange = floor(g_currentL - g_Lstep);
	if (minrange < g_inductors_min)
		minrange = g_inductors_min;
	maxrange = ceil(g_currentL + g_Lstep);
	if (maxrange > g_inductors_max)
		maxrange = g_inductors_max;
	for (l = minrange + 1; l<= maxrange; l++)
	{
		latch_inductors(l);
		usleep (RELAY_SETTLE_TIME);
		Phi = get_phi_filtered ();
		if (Phi >= Phimin){usleep(10000);Phi = get_phi_filtered ();}
		if (Phi >= Phimin){usleep(10000);Phi = get_phi_filtered ();}
		if (Phi <= Phimin)
		{
			Phimin = Phi;
			g_currentL = l;
		}
		else
		{
			INFO("BREAK");
			break;
		}
	}
	latch_inductors(g_currentL);
	usleep (RELAY_SETTLE_TIME);	
}


void ATU_GridTune()
{
	//Tunig method based on Diver Martin <diver.martin@...>1/01/18   #37958  based on K3NG
	//https://groups.io/g/BITX20/topic/tuner_well_sure/7723927?p=Created%2C%2C%2C20%2C1%2C60%2C0&fbclid=IwAR27BOTw2lt75u3eKD9OsFtU7tDQqFLl9YhJ5qq7qQ0o9A3nrusJRIy0Am4
	//This tuning method gives excellent antenna matching but is slow at about 200-300 measurements,  Tunig time about 13 seconds.
	//corse tune using CxL grid.
	//int16_t inductors_max, capacitors_max,inductors_min, capacitors_min;
	double Phi_min = 10;
	int16_t L_AtMin = 0,C_AtMin = 0;
	uint8_t Lsteps = 10; //We scan 8 inductor values
	uint8_t Csteps = 10;//We scan 9 capacitor values
	double Cstep,Lstep;
	//capacitors_min = -255;
	//capacitors_max = 255;
	//inductors_min = 0;
	//inductors_max = 255;
	uint8_t tuning = 1;
	int16_t l,c,tempL,tempC;
	while (1)
	{
		Cstep = (double)(g_capacitors_max - g_capacitors_min)/(Csteps - 1);
		Lstep = (double)(g_inductors_max - g_inductors_min)/(Lsteps - 1);
		for (c = 0; c < Csteps; c++)
		{
			tempC = g_capacitors_min + Cstep*c;
			latch_capacitors_signed(tempC);
			for (l = 0; l < Lsteps; l++)
			{
				tempL = g_inductors_min + Lstep*l;
				latch_inductors(tempL);
				usleep (RELAY_SETTLE_TIME);
				INFO("%d, %d",tempL,tempC);
				double phi = get_phi_filtered ();
				if (phi < Phi_min)
				{
					Phi_min = phi;
					L_AtMin = tempL;
					C_AtMin = tempC;
				}
			}
		}
		INFO("M %d, %d",L_AtMin,C_AtMin);
		if (tuning == 0)
		{
			break;
		}
		g_inductors_min = max(0, L_AtMin - Lstep/2);
		g_inductors_max = min(255, L_AtMin + Lstep/2);
		g_capacitors_min = max(-255, C_AtMin - Cstep/2);
		g_capacitors_max = min(255, C_AtMin + Cstep/2);
		if (g_capacitors_max - g_capacitors_min < Csteps)
		{
			Csteps = g_capacitors_max - g_capacitors_min;
			tuning = 0;
		}
		if (g_inductors_max - g_inductors_min < Lsteps)
		{
			Lsteps = g_inductors_max - g_inductors_min;
			tuning = 0;
		}
	}
	latch_inductors(L_AtMin);
	latch_capacitors_signed(C_AtMin);
	usleep (RELAY_SETTLE_TIME);
}

void ATU_OptimizedGridTune(int16_t Csign)
{
	//Tunig method based on Diver Martin <diver.martin@...>1/01/18   #37958  based on K3NG
	//https://groups.io/g/BITX20/topic/tuner_well_sure/7723927?p=Created%2C%2C%2C20%2C1%2C60%2C0&fbclid=IwAR27BOTw2lt75u3eKD9OsFtU7tDQqFLl9YhJ5qq7qQ0o9A3nrusJRIy0Am4
	//This tuning method gives excellent antenna matching but is slow at about 200-300 measurements,  Tunig time about 13 seconds.
	//Added optimizations inspired by David Fainitski's	N7DDC ATU-100 project 2016 tuner implementation using Branch cutting
	//corse tune using CxL grid.
	//int16_t inductors_max, capacitors_max,inductors_min, capacitors_min;
	double Phi_min = 10;
	double Phi_min_branch = 10;
	int16_t L_AtMin = 0,C_AtMin = 0;
	uint8_t Lsteps = 10; //We scan 8 inductor values
	uint8_t Csteps = 10;//We scan 9 capacitor values
	double Cstep,Lstep;
	//capacitors_min = -255;
	//capacitors_max = 255;
	//inductors_min = 0;
	//inductors_max = 255;
	uint8_t tuning = 1;
	int16_t l,c,tempL,tempC;
	
	//Force ATU relays open. To prevent any surprises;
	active_inductor_relays = 0xFF;
	active_capacitor_relays = 0x1FF;	
	latch_inductors(0);
	latch_capacitors_signed(0);
	usleep (RELAY_SETTLE_TIME);
	while (1)
	{
		if (tuning)
		{
			Cstep = (double)(g_capacitors_max - g_capacitors_min)/(Csteps - 1);
			Lstep = (double)(g_inductors_max - g_inductors_min)/(Lsteps - 1);	
		}
		else
		{
			Cstep = 1;
			Lstep = 1;			
		}
		
		for (c = 0; c < Csteps; c++)
		{
			tempC = g_capacitors_min + Cstep*c;
			latch_capacitors_signed(tempC*Csign);
			Phi_min_branch = 10;
			for (l = 0; l < Lsteps; l++)
			{
				tempL = g_inductors_min + Lstep*l;
				latch_inductors(tempL);
				usleep (RELAY_SETTLE_TIME);
				
				//INFO("%d, %d",tempL,tempC);
				
				double phi = get_phi_filtered ();
				if (sumfwd < 15) return;
				if (phi > 1.0) //limit phi
					phi = 1.0;
				if (phi < Phi_min)
				{
					Phi_min = phi;
					L_AtMin = tempL;
					C_AtMin = tempC;
				}
				if (phi < Phi_min_branch)
				{
					Phi_min_branch = phi * (double)1.02; //2% headroom
				}
				else
				{
				//	INFO("Break");
					break;//This branch is rotten swr creeped back up
				}
			}
		}
		//INFO("M %d, %d",L_AtMin,C_AtMin);
		if (tuning == 0)
		{
			break;
		}
		g_inductors_min = max(0, L_AtMin - Lstep);
		g_inductors_max = min(255, L_AtMin + Lstep);
		g_capacitors_min = max(0, C_AtMin - Cstep);
		g_capacitors_max = min(255, C_AtMin + Cstep);
		if (g_capacitors_max - g_capacitors_min < Csteps)
		{
			Csteps = g_capacitors_max - g_capacitors_min;
			Csteps ++;
			g_capacitors_max++;
			Lsteps++;
			g_inductors_max ++;
			tuning = 0;
		}
		if (g_inductors_max - g_inductors_min < Lsteps)
		{
			Lsteps = g_inductors_max - g_inductors_min;
			Lsteps++;
			g_inductors_max ++;
			Csteps ++;
			g_capacitors_max++;
			tuning = 0;
		}
	}
	latch_inductors(L_AtMin);
	latch_capacitors_signed(C_AtMin*Csign);
	usleep (RELAY_SETTLE_TIME);
}

void ATU_OptimizedGridTuneRev(int16_t Csign)
{
	//Tunig method based on Diver Martin <diver.martin@...>1/01/18   #37958  based on K3NG
	//https://groups.io/g/BITX20/topic/tuner_well_sure/7723927?p=Created%2C%2C%2C20%2C1%2C60%2C0&fbclid=IwAR27BOTw2lt75u3eKD9OsFtU7tDQqFLl9YhJ5qq7qQ0o9A3nrusJRIy0Am4
	//This tuning method gives excellent antenna matching but is slow at about 200-300 measurements,  Tunig time about 13 seconds.
	//Added optimizations inspired by David Fainitski's	N7DDC ATU-100 project 2016 tuner implementation using Branch cutting
	//corse tune using CxL grid.
	//int16_t inductors_max, capacitors_max,inductors_min, capacitors_min;
	double Phi_min = 10;
	double Phi_min_branch = 10;
	int16_t L_AtMin = 0,C_AtMin = 0;
	uint8_t Lsteps = 10; //We scan 8 inductor values
	uint8_t Csteps = 10;//We scan 9 capacitor values
	double Cstep,Lstep;
	//capacitors_min = -255;
	//capacitors_max = 255;
	//inductors_min = 0;
	//inductors_max = 255;
	uint8_t tuning = 1;
	int16_t l,c,tempL,tempC;
	
	//Force ATU relays open. To prevent any surprises;
	active_inductor_relays = 0xFF;
	active_capacitor_relays = 0x1FF;
	latch_inductors(0);
	latch_capacitors_signed(0);
	usleep (RELAY_SETTLE_TIME);
	while (1)
	{
		if (tuning)
		{
			Cstep = (double)(g_capacitors_max - g_capacitors_min)/(Csteps - 1);
			Lstep = (double)(g_inductors_max - g_inductors_min)/(Lsteps - 1);
		}
		else
		{
			Cstep = 1;
			Lstep = 1;
		}
		for (l = 0; l < Lsteps; l++)		
		{
			tempL = g_inductors_min + Lstep*l;
			latch_inductors(tempL);
			
			Phi_min_branch = 10;
			for (c = 0; c < Csteps; c++)
			{				
				tempC = g_capacitors_min + Cstep*c;
				latch_capacitors_signed(tempC*Csign);	
				usleep (RELAY_SETTLE_TIME);
				
				//INFO("%d, %d",tempL,tempC);
				
				double phi = get_phi_filtered ();
				if (phi > 1.0) //limit phi
				phi = 1.0;
				if (phi < Phi_min)
				{
					Phi_min = phi;
					L_AtMin = tempL;
					C_AtMin = tempC;
				}
				if (phi < Phi_min_branch)
				{
					Phi_min_branch = phi * (double)1.02; //2% headroom
				}
				else
				{
					INFO("Break");
					break;//This branch is rotten swr creeped back up
				}
			}
		}
		//INFO("M %d, %d",L_AtMin,C_AtMin);
		if (tuning == 0)
		{
			break;
		}
		g_inductors_min = max(0, L_AtMin - Lstep);
		g_inductors_max = min(255, L_AtMin + Lstep);
		g_capacitors_min = max(0, C_AtMin - Cstep);
		g_capacitors_max = min(255, C_AtMin + Cstep);
		if (g_capacitors_max - g_capacitors_min < Csteps)
		{
			Csteps = g_capacitors_max - g_capacitors_min;
			Csteps ++;
			g_capacitors_max++;
			Lsteps++;
			g_inductors_max ++;
			tuning = 0;
		}
		if (g_inductors_max - g_inductors_min < Lsteps)
		{
			Lsteps = g_inductors_max - g_inductors_min;
			Lsteps++;
			g_inductors_max ++;
			Csteps ++;
			g_capacitors_max++;
			tuning = 0;
		}
	}
	latch_inductors(L_AtMin);
	latch_capacitors_signed(C_AtMin*Csign);
	usleep (RELAY_SETTLE_TIME);
}
void ATU_CroaseAndFineTuneWithSWRderivation()
{
	//This method is based on David Fainitski's	N7DDC ATU-100 project 2016 tuner implementation
	//Uses 3 step tuning method.
	//First steps trough the inductors, and for each inductor step the capacitors, but uses variable step size based on value.
	//This step size is later used at fine tuning.
	//Also if the swr starts to increase the branch is cut off.
	//If is steady high the steps continue.
	//Performance to be determined by tests.
	//We have to also include Value limit.
	//coarse_tune(); if(SWR==0) {atu_reset(); return;}
	//get_swr(); if(SWR<120) return;
	//sharp_ind();  if(SWR==0) {atu_reset(); return;}
	//get_swr(); if(SWR<120) return;
	//sharp_cap();
	
	//
	g_Cstep = (double)(g_capacitors_max - g_capacitors_min)/(g_Csteps - 1);
	g_Lstep = (double)(g_inductors_max - g_inductors_min)/(g_Lsteps - 1);
	INFO("%f",g_Cstep);
	INFO("%f",g_Lstep);
	 ATU_Crouse();	 
	 ATU_SharpInductor();
	 ATU_SharpCapacitor();
	 double vswr = get_vswr();
	 if (vswr < 2)
		return;
	g_CapacitorSign *= -1;
	 ATU_Crouse();
	 ATU_SharpInductor();
	 ATU_SharpCapacitor();
	
}

//This function was modified to use a different tuning strategy
//First we do a rough brute force tune
//Then at the minimum SWR we do a binary search for the fine tune
//Binary search is not working properly because in most cases we have a sharper tune 

TASK (atu, TASK_PRIORITY_MEDIUM, 512)
{
	uint8_t PrevL;
	int16_t PrevC;
	double prevPhi;
	lock_acquire (&atu_lock);
	while (true) {
		condition_wait (&atu_tune_cond, &atu_lock);
		//int16_t inductors_max, capacitors_max,inductors_min, capacitors_min;
		if (tunig_frequency == 0) 
		{
			atu_status = ATU_STATUS_IN_PROGRESS;
			lock_release (&atu_lock);
			latch_inductors (0);
			latch_capacitors (0); 				
			lock_acquire (&atu_lock);
			atu_status = ATU_STATUS_RESET;      
			pull_up (STATUS);
			continue;
		} 		
		//inductors_max = capacitors_max = 255;
  
		atu_status = ATU_STATUS_IN_PROGRESS;
		pull_up (STATUS);
		lock_release (&atu_lock);
		INFO("ATU");
		if (ptt_enabled) 
		{
			usleep (PTT_DELAY);
			// latch_capacitors (0);
			ATU_CalculateLimits(tunig_frequency);						
			ATU_OptimizedGridTune(1);
			INFO("L%d C%d",active_inductor_relays,calculate_signed_capacitor_value(active_capacitor_relays));			
			double vswr = get_vswr();
			//INFO ("VSWR=%.2f", vswr);
			//ATU_CalculateLimits(tunig_frequency);
			//ATU_OptimizedGridTuneRev(1);
			//INFO("L%d C%d",active_inductor_relays,calculate_signed_capacitor_value(active_capacitor_relays));
			//vswr = get_vswr();
			//INFO ("VSWR=%.2f", vswr);
			if ((vswr > 1.5) && (sumfwd > 15))
			{
				prevPhi = get_phi_filtered();
				PrevL = active_inductor_relays;
				PrevC = calculate_signed_capacitor_value(active_capacitor_relays);
				ATU_CalculateLimits(tunig_frequency);		
				ATU_OptimizedGridTune(-1);
				INFO("L%d C%d",active_inductor_relays,calculate_signed_capacitor_value(active_capacitor_relays));  
				vswr = get_vswr();
				INFO ("VSWR=%.2f", vswr);
				if (prevPhi < get_phi_filtered())
				{
					latch_capacitors_signed(PrevC);
					latch_inductors(PrevL);
					usleep (RELAY_SETTLE_TIME);
					vswr = get_vswr();
					INFO ("Backstep VSWR=%.2f", vswr);
				}
			}
			
			//ATU_CalculateLimits(tunig_frequency);
			//ATU_OptimizedGridTuneRev(-1);
			//INFO("L%d C%d",active_inductor_relays,calculate_signed_capacitor_value(active_capacitor_relays));
			//vswr = get_vswr();
			//INFO ("VSWR=%.2f", vswr);
			
			//ATU_LthenCtune();
			//ATU_CroaseAndFineTuneWithSWRderivation();
			lock_acquire (&atu_lock);
			if (!ptt_enabled || vswr > ATU_MAX_VSWR)
				atu_status = ATU_STATUS_FAILED;
			else 
				atu_status = ATU_STATUS_SUCCEEDED;                      

			//eeprom_write_byte(&settings.atu_inductors, active_inductor_relays);
			//eeprom_write_word(&settings.atu_capacitors, active_capacitor_relays);
			if (EEpromEraseTriggerd)
			{
				EraseEEpromStorage();
				EEpromEraseTriggerd = 0;
			}
			StoreCurrentTune();
			pull_up (STATUS);
			} else {
				lock_acquire (&atu_lock);
				atu_status = ATU_STATUS_FAILED;
				pull_up (STATUS);
			}
  }
}


////Data storage related functions
int16_t ATU_EEPROM_SearchClosestTune(uint32_t frequency, uint32_t maxDelta)
{
	uint16_t i;
	atu_entry CurrentEntry;
	uint32_t min_Fdelta = 0xFFFFFFFF;
	uint32_t delta;
	int16_t minIndex = -1;
	for (i=0; i < EEPROM_ENTRY_CNT; i++)
	{
		eeprom_read_block(&CurrentEntry,&settings.atu_entries[i],sizeof(atu_entry));
		if (CurrentEntry.frequency == 0 || CurrentEntry.frequency == 0xFFFFFFFF)
			continue;
		if (CurrentEntry.frequency < frequency)
			delta = frequency - CurrentEntry.frequency;
		else
			delta = CurrentEntry.frequency - frequency;
		
		if ((delta < min_Fdelta)&&(delta < maxDelta))
		{
			min_Fdelta = delta;
			minIndex = i;		
		}
	}
	INFO("ClosestTune index: %d", minIndex);
	return minIndex;
}

int16_t ATU_EEPROM_SearchSaveLocation(uint32_t frequency)
{
	uint16_t i;
	atu_entry CurrentEntry;	
	int16_t minIndex = -1;
	uint8_t minWrites = 0xFF;
	minIndex = ATU_EEPROM_SearchClosestTune(frequency, CLOSE_FREQUENCY);
	if (minIndex >= 0)
		return minIndex; //Found a close frequency to overwrite
	minIndex = 0;
	for (i = 0; i < EEPROM_ENTRY_CNT; i++)
	{
		eeprom_read_block(&CurrentEntry,&settings.atu_entries[i],sizeof(atu_entry));
		if (CurrentEntry.frequency == 0 || CurrentEntry.frequency == 0xFFFFFFFF)
			return i;//Found an empty spot
		if (CurrentEntry.cell_age < minWrites)
		{
			minIndex = i;
			minWrites = CurrentEntry.cell_age;
		}
	}
	return minIndex; //Use the oldest spot.	
}

void TuneFromMemory(uint32_t frequency)
{
	int16_t tuneLocation;
	atu_entry CurrentEntry;
	tuneLocation = ATU_EEPROM_SearchClosestTune(frequency,CLOSE_FREQUENCY_TUNE);
	if (tuneLocation >= 0)
	{
		eeprom_read_block(&CurrentEntry,&settings.atu_entries[tuneLocation],sizeof(atu_entry));
		INFO("Tuning from memory %d",tuneLocation);
		INFO ("Req    %lu Hz", frequency);
		INFO ("Stored %lu Hz", CurrentEntry.frequency);
		INFO("L %d, C %d",CurrentEntry.atu_inductors, CurrentEntry.atu_capacitors);
		
		if (FirstTune)
		{
			/* Make sure the relays are in expected states. */
			active_inductor_relays = ~CurrentEntry.atu_inductors;
			active_capacitor_relays = ~calculate_unsigned_capacitor_value(CurrentEntry.atu_capacitors);
			FirstTune = 0;
		}
		latch_inductors(CurrentEntry.atu_inductors);
		latch_capacitors_signed(CurrentEntry.atu_capacitors);
	}
	else
	{
		INFO("No tune for this freq");
		if (FirstTune)
		{
			/* Make sure the relays are in expected states. */
			active_inductor_relays = 0xFF;
			active_capacitor_relays = 0xFF;
			FirstTune = 0;
		}
		latch_inductors(0);
		latch_capacitors_signed(0);
	}	
}

void StoreCurrentTune()
{
	int16_t tuneLocation;
	atu_entry CurrentEntry;	
	tuneLocation = ATU_EEPROM_SearchSaveLocation(tunig_frequency);
	eeprom_read_block(&CurrentEntry,&settings.atu_entries[tuneLocation],sizeof(atu_entry));
	CurrentEntry.frequency = tunig_frequency;
	CurrentEntry.atu_inductors = active_inductor_relays;
	CurrentEntry.atu_capacitors = calculate_signed_capacitor_value(active_capacitor_relays);
	CurrentEntry.cell_age ++;
	eeprom_write_block(&CurrentEntry,&settings.atu_entries[tuneLocation],sizeof(atu_entry));	
	INFO("Saved tune to: %d age:%d",tuneLocation, CurrentEntry.cell_age);
}

void EraseEEpromStorage()
{
	atu_entry CurrentEntry;
	uint16_t i;
	memset(&CurrentEntry,0,sizeof(atu_entry));	
	for (i = 0; i < EEPROM_ENTRY_CNT; i++)
	{
		eeprom_write_block(&CurrentEntry,&settings.atu_entries[i],sizeof(atu_entry));
	}	
}

void TriggerEEpromErase()
{	
	EEpromEraseTriggerd = 1;
}