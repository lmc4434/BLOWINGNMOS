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


float SmoothData[128];
int BinaryData[128];
int bin_threshold = 2000;

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
		SmoothData[i] = (line[i-2] + line[i-1] + line[i] + line[i+1] line[i+2])/5;	
    }	
}

void bin_enc(void){
	for(int i = 0; i < 127; i++){
		if(SmoothData[i] > bin_threshold){
			BinaryData[i] = 1;
		} else {
			BinaryData[i] = 0;
		}
	}
}
   























