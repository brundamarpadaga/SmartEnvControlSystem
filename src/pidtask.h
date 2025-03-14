/*
 * pidtask.h - Header file for PID task
 *
 * Purpose: Contain all of the structures and defines for implementing the
 *          PID algorythm in the PID task that will be launched in main.c
 *
 *
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Authors: Nikolay Nikolov, Ashten Bontrager
 */

#include "stdint.h"
#include "stdio.h"

<<<<<<< HEAD
/* PID structure definitions, temp/humidity/lux */
=======
/* PID structure definition */
>>>>>>> prototype
typedef struct
{
    float    Kp;          // proportional gain
    float    Ki;          // integral gain
    float    Kd;          // derivative gain
    uint16_t setpoint;    // intensity setpoint
    float    integral;    // error acccumulator, sum of errors over time
    float    prev_error;  // previous error, used for (d_error/dt)
    float    delta_t;     // change in time for derivative
    uint16_t max_lim;     // value to set upper rail for output
    uint8_t  min_lim;     // value to set lower rail for output
} PID_t;

// prototype for function that initializes PID structure
// returns byte used for checking if init has happened or not
bool pid_init ( PID_t* pid );

// prototype of pid function that returns the controlled signal as a float
// takes lux_value, returned from TSL2561 sensor returns float vlaue
// is used to determine what value to write for LED PWM
float pid_funct ( PID_t* pid, uint16_t lux_value, uint8_t switches );

/*********************PID Task Prototype*************************************
 *   Task Handles the Following:
 *   Reads perameter message from MsgQ
 *   Update new control/setpoint parameters
 *   Get Current lux readig from TSL2561 sensor
 *       done using the TSL2561 driver in implemented
 *       in the C file of the same name
 *   Execute PID algo function
 *   Drive PWM signal for LED, use RGB writ commands
 *   write to display thread MsgQ to update
 *   setpoint and current lux
 *****************************************************************************/
void PID_Task ( void* p );

/************************Display Task****************************************
 *   Gets lux and setpoint values from Q and updates 7-seg display
 *****************************************************************************/
void Display_Task ( void* p );

void print_pid ( PID_t* pid );
