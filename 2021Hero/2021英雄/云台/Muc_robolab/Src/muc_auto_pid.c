#include <math.h>
#include "muc_auto_pid.h"

double PID(double err){
    static double _Kp = Kp;
    static double _Ki = Ki;
    static double _Kd = Kd;
    static double Perror = 0;
    static double Ierror = 0;
    static double Derror = 0;
    static double Ierror_abs = 0;
    static char needsTraining = 1;
    static double currentError = 0;
    static double previousError = 0;
    static double cumulativeError = 0;
    static int count = 0;
    static double learningRate = learnRate;

    // Calcular el PID y obtener el valor para doblar (steer)
    Derror = err - Perror;
    Perror = err;
    Ierror += err;
    Ierror_abs += fabs(err);
    cumulativeError += (err * err);
    double steer = _Kp * Perror + _Kd * Derror + _Ki * Ierror;

    // Contador de epoch
    count++;

    // Cada epoch:
    if(count == epochLength){
        count = 0;

        // Si hace falta seguir tuneando:
        if(needsTraining){

            // Calcular el error del epoch, siendo el promedio de la
            // integraci�n de todos los errores.
            currentError = sqrt(cumulativeError / epochLength);

            // Evaluar si el error del epoch est� dentro de un threshold.
            // Cuando el error es menor al threshold es porque est� lo
            // suficientemente acercado a 0 como para seguir tuneando.
            // El threshold debe ser bajo pero no tanto, ya que si es
            // muy bajo podr�a nunca llegarse a lograr un PID "�ptimo".
            needsTraining = currentError > errorThreshold;

            // NEEDS TESTING
            // Ajusta el learning rate para que decaiga en base al error,
            // herramienta clásica de optimización de redes neuronale;
            // ya que al acercarse al mínimo error posible se necesita
            // ser linealmente más preciso.
            learningRate *= fabs(currentError) / 100;

            // Si es necesario despu�s de hacer el c�lculo,
            // se hace el entrenamiento (backpropagation).
            if(needsTraining){

                // Calcular los errores.
                double deltaError = previousError - currentError;
                previousError = currentError;

                // Tunear las constantes.
                _Kp -= _Kp * Perror * deltaError * learningRate;
                _Ki -= _Ki * Ierror_abs * deltaError * learningRate;
                _Kd -= _Kd * Derror * deltaError * learningRate;
            }

            // Resetear los contadores de errores, para el pr�ximo epoch.
            Ierror_abs = 0;
            cumulativeError = 0;
        }
    }

    // Devolver el valor de steer, para poder aplicarlo
    // a los motores o servos, dependiendo del uso.
    return steer;
}











/*-------------------模糊自适应pid-------------------*/
//#define IS_Kp 1
//#define IS_Ki 2
//#define IS_Kd 3

//#define NL   -3
//#define NM   -2
//#define NS         -1
//#define ZE         0
//#define PS         1
//#define PM         2
//#define PL         3



//static const float fuzzyRuleKp[7][7]={
//        PL,        PL,        PM,        PM,        PS,        PS,        ZE,
//        PL,        PL,        PM,        PM,        PS,        ZE,        ZE,
//        PM,        PM,        PM,        PS,        ZE,        NS,        NM,
//        PM,        PS,        PS,        ZE,        NS,        NM,        NM,
//        PS,        PS,        ZE,        NS,        NS,        NM,        NM,
//        ZE,        ZE,        NS,        NM,        NM,        NM,        NL,
//        ZE,        NS,        NS,        NM,        NM,        NL,        NL
//};

//static const float fuzzyRuleKi[7][7]={
//        NL,        NL,        NL,        NM,        NM,        ZE,        ZE,
//        NL,        NL,        NM,        NM,        NS,        ZE,        ZE,
//        NM,        NM,        NS,        NS,        ZE,        PS,        PS,
//        NM,        NS,        NS,        ZE,        PS,        PS,        PM,
//        NS,        NS,        ZE,        PS,        PS,        PM,        PM,
//        ZE,        ZE,        PS,        PM,        PM,        PL,        PL,
//        ZE,        ZE,        PS,        PM,        PL,        PL,        PL
//};

//static const float fuzzyRuleKd[7][7]={
//        PS,        PS,        ZE,        ZE,        ZE,        PL,        PL,
//        NS,        NS,        NS,        NS,        ZE,        NS,        PM,
//        NL,        NL,        NM,        NS,        ZE,        PS,        PM,
//        NL,        NM,        NM,        NS,        ZE,        PS,        PM,

//        NL,        NM,        NS,        NS,        ZE,        PS,        PS,
//        NM,        NS,        NS,        NS,        ZE,        PS,        PS,
//        PS,        ZE,        ZE,        ZE,        ZE,        PL,        PL
//};

//typedef struct{
//        float Kp;
//        float Ki;
//        float Kd;
//}PID;

//PID fuzzy(float e,float ec);//e 误差，ec误差变化率
//float speed_pid(float s_tar,float s_cur);//在目标值会多次改变的情况下，建议在函数内部初始pid参数而不是作为形参

//PID fuzzy(float e,float ec)//e 误差，ec误差变化率
//{

//     float etemp,ectemp;
//     float eLefttemp,ecLefttemp;
//     float eRighttemp ,ecRighttemp;

//     int eLeftIndex,ecLeftIndex;
//     int eRightIndex,ecRightIndex;
//     PID      fuzzy_PID;
//     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));

//     eLeftIndex = (int)e;
//     eRightIndex = eLeftIndex;
//     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
//     eRightIndex = (int)((etemp+0.5) + 3);

//     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e);
//     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));

//     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));

//     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
//     ecRightIndex = (int)((ectemp+0.5) + 3);

//     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
//     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));

/////////*************************************反模糊*************************************//////


//        fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[ecLeftIndex][eLeftIndex]
//                                        + eLefttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eLeftIndex]
//                                        + eRighttemp * ecLefttemp * fuzzyRuleKp[ecLeftIndex][eRightIndex]
//                                        + eRighttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eRightIndex]);

//        fuzzy_PID.Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eLeftIndex]
//                                        + eLefttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eLeftIndex]
//                                        + eRighttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eRightIndex]
//                                        + eRighttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eRightIndex]);

//        fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[ecLeftIndex][eLeftIndex]
//                                        + eLefttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eLeftIndex]
//                                        + eRighttemp * ecLefttemp * fuzzyRuleKd[ecLeftIndex][eRightIndex]
//                                        + eRighttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eRightIndex]);
//return fuzzy_PID;

//}


//float speed_pid(float s_tar,float s_cur)//在目标值会多次改变的情况下，建议在函数内部初始pid参数而不是作为形参
//{
//        float tar = 0,cur = 0;      //目标值 , 实际值
//        tar=s_tar;s_cur=cur;
//    static PID pid= {1, 0, 0};      //赋予初值kp，ki，kd
//        static float sumE = 0;                   //累加偏差
//        static float lastE = 0;

//        PID OUT = {0, 0, 0};
//        float e = -1,ec = -2.6;

//        e = tar - cur;             //目标值 - 实际值
//        ec = e - lastE;            //误差变化率
//        sumE += e;
//        lastE = e;
//        OUT = fuzzy(e, ec);      //模糊控制调整  kp，ki，kd

//        return (pid.Kp+OUT.Kp)*e + (pid.Kd+OUT.Kd)*ec + (pid.Ki+OUT.Ki)*sumE;
//}

//void controlpwm_fruzy_pid(float expecte_voltage,float expecte_current)//模糊自适应pid
//{

//	
//}















//注1：自适应模糊pid最重要的就是论域的选择，要和你应该控制的对象相切合
//注2：以下各阀值、限幅值、输出值均需要根据具体的使用情况进行更改
//注3：因为我的控制对象惯性比较大，所以以下各部分取值较小
//论域e:[-5,5]  ec:[-0.5,0.5]

//误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
#define Emin 0.0
#define Emid 0.08
#define Emax 0.6
//调整值限幅，防止积分饱和
#define Umax 5
#define Umin -5

//输出值限幅
#define Pmax 7200
#define Pmin 0

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

int kp[7][7]={	{PB,PB,PM,PM,PS,ZO,ZO},
				{PB,PB,PM,PS,PS,ZO,ZO},
				{PM,PM,PM,PS,ZO,NS,NS},
				{PM,PM,PS,ZO,NS,NM,NM},
				{PS,PS,ZO,NS,NS,NM,NM},
				{PS,ZO,NS,NM,NM,NM,NB},
				{ZO,ZO,NM,NM,NM,NB,NB}    };

int kd[7][7]={	{PS,NS,NB,NB,NB,NM,PS},
				{PS,NS,NB,NM,NM,NS,ZO},
				{ZO,NS,NM,NM,NS,NS,ZO},
				{ZO,NS,NS,NS,NS,NS,ZO},
				{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
				{PB,NS,PS,PS,PS,PS,PB},
				{PB,PM,PM,PM,PS,PS,PB}    };

int ki[7][7]={	{NB,NB,NM,NM,NS,ZO,ZO},
				{NB,NB,NM,NS,NS,ZO,ZO},
				{NB,NM,NS,NS,ZO,PS,PS},
				{NM,NM,NS,ZO,PS,PM,PM},
				{NM,NS,ZO,PS,PS,PM,PB},
				{ZO,ZO,PS,PS,PM,PB,PB},
				{ZO,ZO,PS,PM,PM,PB,PB}    };

/**************求隶属度（三角形）***************/
float FTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 0;
	else if((a<x)&&(x<=b))
		return (x-a)/(b-a);
	else if((b<x)&&(x<=c))
		return (c-x)/(c-b);
	else if(x>c)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形左）*******************/
float FTraL(float x,float a,float b)//FuzzyTrapezoidLeft
{
	if(x<=a)  
		return 1;
	else if((a<x)&&(x<=b))
		return (b-x)/(b-a);
	else if(x>b)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形右）*******************/
float FTraR(float x,float a,float b)//FuzzyTrapezoidRight
{
	if(x<=a)
		return 0;
	if((a<x)&&(x<b))
		return (x-a)/(b-a);
	if(x>=b)
		return 1;
	else
		return 1;
}
/****************三角形反模糊化处理**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************梯形（左）反模糊化***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************梯形（右）反模糊化***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************求交集****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************求并集****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}

float ec;
/*==========   PID计算部分   ======================*/   
//double Auto_PID_realize(PID *structpid,uint16_t s,uint16_t in)
//{
//	double pwm_var;//pwm调整量
//	double iError;//当前误差
//	double set,input;
//	
//	//计算隶属度表
//	double es[7],ecs[7],e;
//	double form[7][7];
//	int i=0,j=0;
//	int MaxX=0,MaxY=0;
//	
//	//记录隶属度最大项及相应推理表的p、i、d值
//	double lsd;
//	int temp_p,temp_d,temp_i;
//	double detkp,detkd,detki;//推理后的结果
//	
//	//输入格式的转化及偏差计算
//	set=(double)s/100.0;
//	input=(double)in/100.0;
//	iError = set - input; // 偏差
//	
//	e=iError;
//	ec=iError-structpid->LastError;
//	
//	//当温度差的绝对值小于Emax时，对pid的参数进行调整
//	if(fabs(iError)<=Emax)
//	{
//	//计算iError在es与ecs中各项的隶属度
//	es[NB]=FTraL(e*5,-3,-1);  //e 
//	es[NM]=FTri(e*5,-3,-2,0);
//	es[NS]=FTri(e*5,-3,-1,1);
//	es[ZO]=FTri(e*5,-2,0,2);
//	es[PS]=FTri(e*5,-1,1,3);
//	es[PM]=FTri(e*5,0,2,3);
//	es[PB]=FTraR(e*5,1,3);

//	ecs[NB]=FTraL(ec*30,-3,-1);//ec
//	ecs[NM]=FTri(ec*30,-3,-2,0);
//	ecs[NS]=FTri(ec*30,-3,-1,1);
//	ecs[ZO]=FTri(ec*30,-2,0,2);
//	ecs[PS]=FTri(ec*30,-1,1,3);
//	ecs[PM]=FTri(ec*30,0,2,3);
//	ecs[PB]=FTraR(ec*30,1,3);
//	
//	//计算隶属度表，确定e和ec相关联后表格各项隶属度的值
//	for(i=0;i<7;i++)
//	{
//		for(j=0;j<7;j++)
//		{
//			form[i][j]=fand(es[i],ecs[j]);
//		}
//	}
//	
//	//取出具有最大隶属度的那一项
//	for(i=0;i<7;i++)
//	{
//		for(j=0;j<7;j++)
//		{
//			if(form[MaxX][MaxY]<form[i][j]) 
//			{
//				MaxX=i;
//				MaxY=j;
//			}
//		}
//	}
//	//进行模糊推理，并去模糊
//	lsd=form[MaxX][MaxY];
//	temp_p=kp[MaxX][MaxY];
//	temp_d=kd[MaxX][MaxY];   
//	temp_i=ki[MaxX][MaxY];
//	
//	if(temp_p==NB)
//		detkp=uFTraL(lsd,-0.3,-0.1);
//	else if(temp_p==NM)
//		detkp=uFTri(lsd,-0.3,-0.2,0);
//	else if(temp_p==NS)
//		detkp=uFTri(lsd,-0.3,-0.1,0.1);
//	else if(temp_p==ZO)
//		detkp=uFTri(lsd,-0.2,0,0.2);
//	else if(temp_p==PS)
//		detkp=uFTri(lsd,-0.1,0.1,0.3);
//	else if(temp_p==PM)
//		detkp=uFTri(lsd,0,0.2,0.3);
//	else if(temp_p==PB)
//		detkp=uFTraR(lsd,0.1,0.3);

//	if(temp_d==NB)
//		detkd=uFTraL(lsd,-3,-1);
//	else if(temp_d==NM)
//		detkd=uFTri(lsd,-3,-2,0);
//	else if(temp_d==NS)
//		detkd=uFTri(lsd,-3,1,1);
//	else if(temp_d==ZO)
//		detkd=uFTri(lsd,-2,0,2);
//	else if(temp_d==PS)
//		detkd=uFTri(lsd,-1,1,3);
//	else if(temp_d==PM)
//		detkd=uFTri(lsd,0,2,3);
//	else if(temp_d==PB)
//		detkd=uFTraR(lsd,1,3);

//	if(temp_i==NB)
//		detki=uFTraL(lsd,-0.06,-0.02);
//	else if(temp_i==NM)
//		detki=uFTri(lsd,-0.06,-0.04,0);
//	else if(temp_i==NS)
//		detki=uFTri(lsd,-0.06,-0.02,0.02);
//	else if(temp_i==ZO)
//		detki=uFTri(lsd,-0.04,0,0.04);
//	else if(temp_i==PS)
//		detki=uFTri(lsd,-0.02,0.02,0.06);
//	else if(temp_i==PM)
//		detki=uFTri(lsd,0,0.04,0.06);
//	else if (temp_i==PB)
//		detki=uFTraR(lsd,0.02,0.06);

//	//pid三项系数的修改
//	structpid->Kp+=detkp;
//	structpid->Ki+=detki;
//	//structpid->Kd+=detkd;
//	structpid->Kd=0;//取消微分作用
//	
//	//对Kp,Ki进行限幅
//	if(structpid->Kp<0)
//	{structpid->Kp=0;}
//	if(structpid->Ki<0)
//	{structpid->Ki=0;}
//	
//	//计算新的K1,K2,K3
//	structpid->K1=structpid->Kp+structpid->Ki+structpid->Kd;
//	structpid->K2=-(structpid->Kp+2*structpid->Kd);
//	structpid->K3=structpid->Kd;
//	
//	}
//	
//	if(iError>Emax)
//	{
//		structpid->pwm_out=7200;
//		pwm_var = 0;
//		structpid->flag=1;//设定标志位，如果误差超过了门限值，则认为当控制量第一次到达给定值时，应该采取下面的 抑制超调 的措施
//	}
//	else if(iError<-Emax)
//	{
//		structpid->pwm_out=0;
//		pwm_var = 0;
//	}
//	else if( fabs(iError) < Emin ) //误差的阀值(死区控制??)
//	{
//		pwm_var = 0;
//	}
//	else
//	{
//		if( iError<Emid && structpid->flag==1 )//第一次超过(设定值-Emid(-0.08)摄氏度)，是输出为零，防止超调，也可以输出其他值，不至于太小而引起震荡
//		{
//			structpid->pwm_out=0;
//			structpid->flag=0;
//		}
//		else if( -iError>Emid)//超过(设定+Emid(+0.08)摄氏度)
//		{
//			pwm_var=-1;
//		}
//		else
//		{
//			//增量计算
//			pwm_var=(structpid->K1 * iError  //e[k]
//			+ structpid->K2 * structpid->LastError	//e[k-1]
//			+ structpid->K3 * structpid->PrevError);	//e[k-2]
//		}
//		if(pwm_var >= Umax)pwm_var = Umax;      //调整值限幅，防止积分饱和
//		if(pwm_var <= Umin)pwm_var = Umin;    	//调整值限幅，防止积分饱和

//	}
//	structpid->PrevError=structpid->LastError;
//	structpid->LastError=iError;
//	
//	structpid->pwm_out += 360*pwm_var;        //调整PWM输出
//  
//	if(structpid->pwm_out > Pmax)structpid->pwm_out = Pmax;    //输出值限幅
//	if(structpid->pwm_out < Pmin)structpid->pwm_out = Pmin;    //输出值限幅
//	
//	return (double)(structpid->pwm_out); // 微分项
//}

//void Auto_PID_Set(PID *structpid,double Kp,double Ki,double Kd,double T)
//{
//	(*structpid).Kp=Kp;//Kp*(1+(Td/T));
//	(*structpid).Ki=Ki;
//	(*structpid).Kd=Kd;
//	(*structpid).T=T;
//	
//	structpid->K1=structpid->Kp*(1+structpid->Ki+structpid->Kd);
//	structpid->K2=-(structpid->Kp+2*structpid->Kp*structpid->Kd);
//	structpid->K3=structpid->Kp*structpid->Kd;
//}

//void Auto_PID_Init(PID *structpid)
//{
//	Auto_PID_Set(structpid,8.3,1.2,0,1);
//	structpid->flag=0;
//	structpid->pwm_out=0;
//}
