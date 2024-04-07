#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include "Common.h" // Assuming BOOLEAN is defined in Common.h
#define STR_SIZE 100
#define CAMERA_DATA_SIZE 128

// Declare variables related to camera functionality
extern uint16_t line[CAMERA_DATA_SIZE];
static char str[STR_SIZE];

// Function prototypes for camera functionality
void INIT_Camera(void);
void camsmooth(void);
void bin_enc(void);

#endif /* CAMERA_H */
