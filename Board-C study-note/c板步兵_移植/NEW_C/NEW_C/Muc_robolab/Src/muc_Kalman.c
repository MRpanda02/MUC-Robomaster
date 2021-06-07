#include "muc_Kalman.h"
#include "main.h"
#include "math.h"



/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state)
{
    state->x[0]    = 1;
    state->x[1]    = 1;
    state->p[0][0] = 1;
    state->p[0][1] = 1;
    state->p[1][0] = 1;
    state->p[1][1] = 1;
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}



/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - ����״ֵ̬����Ƕȡ��ٶ�
 *   state->x[1] - ���µ�״ֵ̬�����ֽǡ����ٶ�
 *   state->p    - ���µĹ�������������
 * @retval  
 *   ����ֵ����״̬->x[0]�����Կ����ǽǶȻ��ٶ�
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;
 
    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];
 
    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);
 
    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];
 
    return state->x[0];
}



/**
 *@function: - �������˲�����ʼ��
 *@kalmanFilter���������˲����ṹ��
 *@init_x���������ĳ�ʼֵ
 *@init_p������״̬����ֵ���Э����ĳ�ʼֵ
 */
void kalmanFilter_init(KalmanStructTypedef *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q)
{
    kalmanFilter->x = init_x;//�������ĳ�ʼֵ��������ֵһ�������ֵ
    kalmanFilter->p = init_p;//����״̬����ֵ���Э����ĳ�ʼֵ����ҪΪ0���ⲻ��
    kalmanFilter->A = 1;
    kalmanFilter->H = 1;
    kalmanFilter->q = predict_q;//Ԥ�⣨���̣��������� Ӱ���������ʣ����Ը���ʵ���������
    kalmanFilter->r = newMeasured_q;//�������۲⣩��������R������ͨ��ʵ���ֶλ��
}

/**
 *@function: - �������˲���
 *@kalmanFilter:�������ṹ��
 *@newMeasured������ֵ
 *�����˲����ֵ
 */
float kalmanFilter_filter(KalmanStructTypedef *kalmanFilter, float newMeasured)
{
    /* Predict */
    kalmanFilter->x = kalmanFilter->A * kalmanFilter->x;//%x�������������һ��ʱ���ĺ������ֵ��������Ϣ����
    kalmanFilter->p = kalmanFilter->A * kalmanFilter->A * kalmanFilter->p + kalmanFilter->q;  /*������������� p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Correct */
    kalmanFilter->gain = kalmanFilter->p * kalmanFilter->H / (kalmanFilter->p * kalmanFilter->H * kalmanFilter->H + kalmanFilter->r);
    kalmanFilter->x = kalmanFilter->x + kalmanFilter->gain * (newMeasured - kalmanFilter->H * kalmanFilter->x);//���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������
    kalmanFilter->p = (1 - kalmanFilter->gain * kalmanFilter->H) * kalmanFilter->p;//%������������

    return kalmanFilter->x;//�õ���ʱ�̵����Ź���
}




/*       
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��       
*/

/* �������˲����� */



