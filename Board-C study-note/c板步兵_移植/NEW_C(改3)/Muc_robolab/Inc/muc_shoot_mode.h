#ifndef _MUC_SHOOT_MODE_H__
#define _MUC_SHOOT_MODE_H__




typedef enum{
	single = 1,
	running = 2,
	ten_running = 3,
}st_mode;


void TenRunning(void);
void Shooting(void);
void One_shoot(void);
void Single(void);
void Running(void);

void Rotation(void);
extern void change_st_mode(void);
extern st_mode St_Mode;

#endif
