#include "led.h"
#include "controlpins.h"
#include "adc.h"
#include "timer32.h"
#include "cortexm.h"
#include "switches.h"
#include "systicktimer.h"
#include "camera.h"
#include "oled.h"
#include <stdio.h>
 

#ifndef PID_H
#define PID_H

#include "stdbool.h" // Include standard boolean library

// Define constants


// Function declarations
int find_center(void);
float updateServoPosition(float control_signal);
float PID(void);
float find_angle(float);


#endif /* PID_H */
