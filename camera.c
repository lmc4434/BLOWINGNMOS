/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* LJBeato
* 1/14/2021
*
* Filename: main_timer_template.c
*/
#include <stdio.h>
#include <stdlib.h>

#include "msp.h"
#include "uart.h"
#include "led.h"
#include "switches.h"
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
#include "ADC.h"
#include "ControlPins.h"
#include "camera.h"

//Set Variables



int bin_threshold = 15000;
uint16_t SmoothData[128];
uint16_t BinaryData[128];
extern uint16_t line[128];

void INIT_Camera(void) {
    DisableInterrupts();
    g_sendData = FALSE;
    ControlPin_SI_Init();
    ControlPin_CLK_Init();
    ADC0_InitSWTriggerCh6();
    EnableInterrupts();
}

void camsmooth(void) {
       // Function that brings Fresh Data into RawData
	for (int i = 2; i < 125; i++){
		SmoothData[i] = (int)(line[i-2] + line[i-1] + line[i] + line[i+1]+ line[i+2])/5;	
    }	
}

void bin_enc(void){
	camsmooth();
	for(int i = 0; i < 127; i++){
		if(SmoothData[i] > bin_threshold){
			BinaryData[i] = 1;
		} else {
			BinaryData[i] = 0;
		}
	}
}
