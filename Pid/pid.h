/*
 * pid.h
 *
 *  Created on: 2018Äê7ÔÂ10ÈÕ
 *      Author: Sw Young
 */

#ifndef PID_PID_H_
#define PID_PID_H_

#include <stdint.h>
#include <stdbool.h>

#define CAMERA_W    160
#define CAMERA_H    120
//#define CAMERA_STX  0xFF
//#define ALARM_STX   0xFD
//#define ALARM_END   0xFE

#define CAMERA_MID_X    80
#define CAMERA_MID_Y    60

//#define DEFAULT_KP_X        3.0f
//#define DEFAULT_KI_X        0.00f
//#define DEFAULT_KD_X        0.35f
//
//#define DEFAULT_KP_Y        2.5f
//#define DEFAULT_KI_Y        0.00f
//#define DEFAULT_KD_Y        0.30f

#define DEFAULT_KP_H        3.0f
#define DEFAULT_KI_H        0.0f
#define DEFAULT_KD_H        0.0f

#define DEFAULT_I_MAX       50.0f
#define DEFAULT_OUT_MAX     80.0f
#define DEFAULT_PID_FREQ    50.0f

#define DEFAULT_LPFITER         3.1830e-3
// low pass filter:           0.079577472903393
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 2 Hz -> _filter = 79.5774e-3
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
// f_cut = 50 Hz -> _filter =  3.1830e-3

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float I_MAX;
    float d_LPF;
    float OUT_MAX;
} PID_K;

typedef struct {
    float Error;
    float LastError;
    float Proportion;
    float Integrator;
    float Last_Derivative;
    float Derivative;
    float PID_OUT;
}PID_Data;

extern PID_K PID_X;
extern PID_K PID_Y;
extern PID_Data PID_Data_X;
extern PID_Data PID_Data_Y;

extern bool start_PID_X;
extern bool start_PID_Y;
extern volatile bool start_PID_H;


extern void PID_Init(void);
extern void Position_PID(void);

//extern void CMT2_IntHandler(void);


#endif /* PID_PID_H_ */
