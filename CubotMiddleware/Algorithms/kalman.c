#include "kalman.h"
	/********************************************
	��������static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
  ���ܣ������ǵ����ݿ������˲�
	���룺��
	���أ���
	ע�⣺
	*********************************************/
 double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   static double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
 double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   static double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
 double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   static  double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
}
 double KalmanFilter_yaw(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   static double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }
 double KalmanFilter_pitch(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   static double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;                
 }

 //*/

float F_ar5_filter(float v,float* v_pi,int8_t* cnt)     //float ���͵������˲�
{
	
	float ret=0;
	if((*cnt)<5)
	{
		v_pi[*cnt]=v;
		(*cnt)=(*cnt)+1;
		if((*cnt)>=5)
		{
			(*cnt)=0;
		
		}
		
	
	
	}
	
	 for(int j=0;j<5;j++)
 {
   ret=v_pi[j]+ret;
 
 }
  
 
 
 ret=ret/5.0f;
 
 
 return ret;



}

/*
������������Ϊ0

����ʹ����Щ����


*/

int16_t in16_ar5_filter(int16_t v,int16_t* v_pi,int8_t* cnt)     //float ���͵������˲�
{
	
	int16_t ret=0;
	if((*cnt)<5)
	{
		v_pi[*cnt]=v;
		(*cnt)=(*cnt)+1;
		 if((*cnt)>=5)
		{
			(*cnt)=0;
		
		}
		

	}
	
	 for(int j=0;j<5;j++)
 {
   ret=v_pi[j]+ret;
 
 }
 
 ret=ret/5;
 
 return ret;


}


