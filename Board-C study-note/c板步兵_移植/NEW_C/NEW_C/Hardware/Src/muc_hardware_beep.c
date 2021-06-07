#include "muc_hardware_beep.h"
#include "tim.h"

void BeepInit( void )
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); 
    Sing(Silent);
}

const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

const Sound_tone_e Mavic_Startup_music[Startup_Success_music_len] = 
{
  So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
};

//
void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}

//play the start up music
void Sing_Startup_music( void )
{
    int i;
    //≤•∑≈∆Ù∂Ø“Ù¿÷
	for(i=0;i<Startup_Success_music_len;i++)
	{
		if(i < Startup_Success_music_len)
            Sing(Mavic_Startup_music[i]);
		HAL_Delay(60);
	}
}

void bibiSing( int count )
{
	while(count--)
	{
		Sing(Si7H);
		HAL_Delay(100);
		Sing(Silent);
		HAL_Delay(100);
	}
	Sing(Silent);
}
