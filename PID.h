#include "msp.h"
#include "uart.h"
#include "timerA.h"
#include "motors.h"
#include "servo.h"
#include "led.h"
#include "controlpins.h"
#include "adc.h"
#include "timer32.h"
#include "cortexm.h"
#include "switches.h"
#include "systicktimer.h"
#include "camera.h"
#include "oled.h"
#include "TI_Main.h"
#include <stdio.h>
 

#ifndef PID_H
#define PID_H

#include "stdbool.h" // Include standard boolean library

// Define constants
#define Kp 0.045
#define Ki 0.0005
#define Kd 0.02

// Function declarations
int find_center(void);
float updateServoPosition(float control_signal);
float PID(void);


#endif /* PID_H */
