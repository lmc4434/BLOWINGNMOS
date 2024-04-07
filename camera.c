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


int rawData;
float SmoothData[128];
float LPF_Beta = 0.025; // 0<ß<1


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
    // LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)));
    while(1){
       // Function that brings Fresh Data into RawData
			for (int i = 0; i < 127; i++){
       rawData = line[i];
       SmoothData[i] = SmoothData[i] - (LPF_Beta * (SmoothData[i] - rawData));
			}
    }
		
}
   























