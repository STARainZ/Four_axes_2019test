#ifndef _SONAR_H
#define _SONAR_H

#include "head.h"
#include "delay/delay.h"
//V=331¡Á¸ùºÅ(1+T/273)(m/S)

extern uint8_t temperature;


extern volatile bool DataIsReady;

void ShellSort(double A[], int n); //Ï£¶ûÅÅÐò
double GetAverageDistance();
void Sonar_Configure(void);
void SonarTrig(void);
void Sonar_GPIOA_Configure(void);
void Sonar_GPIOA_Interrupt(void);



#endif
