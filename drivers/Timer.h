/*
 * Timer.h
 *
 *  Created on: 2019��3��11��
 *      Author: dell
 */

#ifndef TIMER_H_
#define TIMER_H_

extern bool AltHold_flag;
extern int Ns;

extern void ConfigureTimer1 (void);
extern void ConfigureTimer3 (void);
void delayNs(int n);

#endif /* TIMER_H_ */
