#ifndef _MUC_AUTO_PID_H_
#define _MUC_AUTO_PID_H_


/* AutoPID - Ianick Noejovich
 *
 * Basado en modelo ajeno, refactorizado y adaptado para el Club de Rob�tica
 *
 * Esto es una red neuronal, un PID que se ajusta a si mismo. Actua
 * teniendo en cuenta al controlador PID como una red neuronal de
 * una capa, con tres inputs y un output. Cada epoch (iteracion) se
 * hace backpropagation para poder ajustar los pesos (las constantes)
 * del PID y as� obtener el error m�s cercano a 0 posible. Una vez
 * que el error est� dentro del threshold, se deja de tunear.
 *
 * En lo posible, este programa se debe instalar con un PID inicial que
 * funcione, ya que caso contrario el robot se ir�a de la pista antes de
 * poder aprender y auto-tunearse. Los pesos para el PID inicial se
 * pueden modificar en las lineas 28 a 32 de este mismo archivo.
 */


// CONSTANTES (TOCAR PARA CAMBIAR LA PRECISION) (ADAPTAR A CADA ROBOT) //
const int epochLength = 200;     // CANTIDAD DE ITERACIONES POR EPOCH
const double errorThreshold = 5; // THRESHOLD DE ERROR
const double learnRate = 0.01;   // LEARNING RATE
/////////////////////////////////////////////////////////////////////////

// PESOS INICIALES //
const double Kp = 1;
const double Ki = 1;
const double Kd = 1;
/////////////////////



//typedef struct{
//	double Kp; // 增量式积分系数
//	double Ki; 
//	double Kd;
//	double T;
//	
//	double K1; // 增量式积分系数
//	double K2; 
//	double K3; 
//	double LastError; //Error[-1]
//	double PrevError; // Error[-2]
//	double pwm_out;
//	
//	uint16_t flag;//温度状态标志位
//}PID;

//void PID_init(PID *structpid);
//void Auto_PID_Set(PID *structpid,double Kp,double Ki,double Kd,double T);
//double Auto_PID_realize(PID *structpid,uint16_t s,uint16_t in);
//void Auto_PID_Init(PID *structpid);


#endif
