#ifndef _MUC_CONFIG_GLOBAL_H__
#define _MUC_CONFIG_GLOBAL_H__



#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


#define MyAbs(x)  ( (x)>0?(x):-(x) )

double DataSlopeProcessing( double RealData, double TargetData, float IntervalData );

#endif //_MUC_CONFIG_GLOBAL_H__
