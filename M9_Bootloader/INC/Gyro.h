#ifndef __GYRO_H__
#define __GYRO_H__

#include "SysInitialize.h"

void Gyro_Reset_Cmd(void);

uint16_t Gyro_GetAngle(uint8_t id);
uint16_t Gyro_GetAngleRaw(uint8_t id);
int16_t Gyro_GetRaw(void);

int16_t Gyro_GetRate(uint8_t id);
int16_t Gyro_GetXAcc(void);
int16_t Gyro_GetXAccDiff(void);
int16_t Gyro_GetYAcc(void);
int16_t Gyro_GetYAccDiff(void);
int16_t Gyro_GetZAcc(void);
int16_t Gyro_GetZAccDiff(void);

uint8_t Gyro_IsUpdated(void);
uint8_t Gryo_GetUpdateFlag(void);
void Gyro_ResetUpdateFlag(uint8_t flag);

uint8_t Gyro_ParseMsg(uint8_t * msg);

void Gyro_SetOffset(int16_t offset);
void Gyro_SetAngle(int16_t);
void Gyro_Reset(void);
void Gyro_Sleep_Cmd(void);
void Gyro_Reset_With_Offset(int16_t offset);

uint8_t Gyro_GetCalibration(void);
void Gyro_Cmd(EventStatus NewState);
void Gyro_ReceiveCharacter(uint8_t c);

void Gyro_Debug_Cmd(void);

void Gyro_Calibration_EnableCmd(void);
void Gyro_Calibration_Cmd(EventStatus NewState);

uint8_t Is_Gyro_Calibration(void);
void Set_Gyro_Calibration(uint8_t code);

#endif /* __GYRO_H */



