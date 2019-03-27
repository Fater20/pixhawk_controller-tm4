/*
 * Control Command.h
 *
 *  Created on: 2019Äê3ÔÂ9ÈÕ
 *      Author: dell
 */

#ifndef DRIVERS_CONTROL_COMMAND_H_
#define DRIVERS_CONTROL_COMMAND_H_

extern int dir_Distance;

void channel_calibration(void);
extern void Flight_Initial(void);
extern void Flight_Armed(void);
extern void Flight_Disarmed(void);
void AltHold_Rise_ready(void);
extern void AltHold_Rise1(void);
extern void AltHold_Rise2(void);
void AltHold_Rise_Control(void);
extern void AltHold_Fall(void);
void AltHold_Fall_Continue(void);
extern void AltHold_Maintain(void);
void AltHold_Control(void);
void Position_Control(void);
void AltHold_Rise_Control(void);
void STOP(void);
int Throttle_limit(int throttle);

#endif /* DRIVERS_CONTROL_COMMAND_H_ */
