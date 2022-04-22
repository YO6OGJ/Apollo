/*
 * apollo_diagnostic.h
 *
 * Created: 05/12/2019 21:39:19
 *  Author: Nagy Csaba YO6PVH
 */ 


#ifndef APOLLO_DIAGNOSTIC_H_
#define APOLLO_DIAGNOSTIC_H_
#include <stdint.h>
void DiagnosticRecieveUSBchar(uint8_t c);
void DiagnosticPrintHelp();


#endif /* APOLLO_DIAGNOSTIC_H_ */