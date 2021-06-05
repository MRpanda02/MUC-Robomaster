#ifndef _MUC_SHOOT_MODE_H__
#define _MUC_SHOOT_MODE_H__




typedef enum{
	single = 1,
	running = 2,
	dafu = 3,
}st_mode;



void Shooting(void);
void One_shoot(void);
void Single(void);
void Running(void);

void Rotation(void);
extern void change_st_mode(void);

#endif
