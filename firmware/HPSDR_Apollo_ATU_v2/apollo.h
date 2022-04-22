#include <stdint.h>
#include <avr/eeprom.h>

#ifndef USB_DEBUG
#undef DEBUG
#endif
#include "kernel/include/kernel.h"
#include "kernel/include/ports.h"

#define VERSION_MAJOR 2
#define VERSION_MINOR 0
#define VERSION_MICRO 0

#define ATU_STATUS_RESET 0
#define ATU_STATUS_IN_PROGRESS 1
#define ATU_STATUS_FAILED 2
#define ATU_STATUS_SUCCEEDED 3
#define ATU_STATUS_ABORTED 4

#define ATU_MAX_VSWR 2.0 // Max VSWR considered a success. TBD

#define RELAY_LATCH_TIME 10000 // µs (10ms)
#define RELAY_LATCH_DELAY 500 // µs
#define RELAY_SETTLE_TIME 30000 // µs (50ms) // This can be faster.! Relay settle time is 10ms
#define PTT_DELAY 100000 // µs (100ms)

#define ADC_SAMPLE_COUNT 100


/* Port definitions */
#define LED1 F, 3
#define LED2 F, 2
#define STATUS D, 0 
#define PTT_OUT D, 1
#define PTT_LED LED1

#define RCK F, 4
#define SRCK F, 5
#define SER_IN F, 6


extern uint32_t frequency;

#define CLOSE_FREQUENCY			10000 //We define close frequency as 10khz May be changed
#define CLOSE_FREQUENCY_TUNE	1000000 //We define close frequency for tuning as 1Mhz
#define EEPROM_ENTRY_CNT 500

#define XL_MAX_LIMIT 250//OHMS
#define XC_MIN_LIMIT 25//OHMS

typedef struct atu_entry {
	int16_t atu_capacitors;
	uint8_t atu_inductors;
	uint32_t frequency;
	uint8_t cell_age;		//This value is incremented every time an entry is updated. Old entries will be overwritten
}atu_entry;

struct eeprom {
	uint8_t lpf_current_band;
	uint8_t lpf_enabled;
	uint8_t atu_inductors;
	uint16_t atu_capacitors;
	uint8_t atu_enabled;
	atu_entry atu_entries[EEPROM_ENTRY_CNT]; //We should have 4k EEPROM entry size is 8
};

extern struct eeprom EEMEM settings;
extern volatile bool ptt_enabled;


void init_lpf ();
void update_lpf ();
void enable_lpf ();
void disable_lpf ();
uint8_t lpf_get_status ();
void enable_ptt (uint16_t timeout);
void disable_ptt ();
void init_atu();
void enable_atu();
void disable_atu();
void reset_atu();
void atu_tune(uint32_t frequency);
uint32_t atu_get_satus ();

//Functions needed for testing
uint32_t lpf_get_FreqForBand(uint8_t band);
bool latch_capacitors_signed (int value);
bool latch_inductors (int value);
double get_vswr ();
uint32_t atu_get_status ();
uint8_t lpf_get_newband ();

int16_t ATU_EEPROM_SearchClosestTune(uint32_t frequency, uint32_t maxDelta);
void StoreCurrentTune();
void TuneFromMemory(uint32_t frequency);
void EraseEEpromStorage();
void TriggerEEpromErase();

//static inline int min (int a, int b) { return a < b ? a : b; };
#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

double get_phi_filtered ();
void ATU_CalculateLimits(uint32_t tunig_frequency);