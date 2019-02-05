/*
 * Control.h
 *
 *  Created on: 2018Äê7ÔÂ10ÈÕ
 *      Author: Sw Young
 */

#ifndef CONTROL_CONTROL_H_
#define CONTROL_CONTROL_H_
void UnlockPixhawk(void);
void LockPixhawk(void);
void LandMode(void);
void Get_Coordinate(void);
void Get_Distance(void);
void OledDisplayInit(void);
void Display(void);
void AttitudeProtection(void);
void Point_Filter(int* pre_err,int* last_err,int* err);


#define channel_val_MIN 1100
#define channel_val_MID 1520
#define channel_val_MAX 1950
#define channel_val_RANGE 850

#endif /* CONTROL_CONTROL_H_ */
