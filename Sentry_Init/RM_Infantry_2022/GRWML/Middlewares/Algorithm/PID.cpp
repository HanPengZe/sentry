/* Includes ------------------------------------------------------------------*/
#include "PID.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#include "math.h"
//#include "arm_math.h"

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
template<typename Type> 
Type _pid_Abs(Type x) {return ((x > 0) ? x : -x);}

template<typename Type> 
void _pid_Constrain(Type *x, Type Min, Type Max) 
{
  if(*x < Min) *x = Min;
  else if(*x > Max) *x = Max;
  else{;}
}
/* function prototypes -------------------------------------------------------*/


/* IncrementPID---------------------------------------------------*/
float IncrementPID::Cal()
{
	Error = Target - Current;

  if (_pid_Abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }

  P_Term = Kp * (Error - pre_error);

	/* PID with Changing integration rate */
	float I_VarSpeedf = 0;
	
	if (_pid_Abs(Error) <= VarSpeed_I_B)
  {
	  I_VarSpeedf = 1;
  }
	else if (_pid_Abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
  {
	  I_VarSpeedf = (VarSpeed_I_A - (_pid_Abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
  }
  
  if(Ki != 0)
  {
    integral_e = I_VarSpeedf * Error * dt;
    /*Constrain*/
    _pid_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
  }
  else
  {
    integral_e = 0;
  }
  
	
  /* Using Integral separation */
  if (_pid_Abs(Error) < I_SeparThresh)
  { 
    I_Term = Ki * integral_e;
    /*Constarin*/
    _pid_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    I_Term = 0;
  }
  
	
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current);
  else
    d_err = (Error - 2.0f*pre_error + prev_error);

	
  D_Term = Kd * d_err;

  prev_error = pre_error;
  pre_error = Error;

  Out += (P_Term + I_Term + D_Term);
  
  /* Constarin */
  _pid_Constrain(&Out, -Out_Max, Out_Max);
  
  return Out;
	
}

void IncrementPID::Reset()
{
	Error = 0;
	Current = 0;
	pre_Current = 0;
	P_Term = 0;
	I_Term = 0;
	D_Term = 0;
	Out = 0;
	integral_e = 0;
	prev_error = 0;
	
}



/* PositionPID---------------------------------------------------*/
float PositionPID::Cal()
{
  Error = Target - Current;

  if (_pid_Abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }

	//变结构---------------------------------------------------------------STAR
	if(a_p != 0)
	{
		//K_p = a_p + b_p * (1 - 1/exp(c_p*|e(t)|)) 
		Kp=a_p+b_p*(1-1/exp(c_p*abs(Error)));
	}
	if(a_i != 0)
	{
		//K_i = a_i * (1 / exp(c_i*|e(t)|))
		Ki=a_i*(1/exp(c_i*abs(Error)));
	}
	//变结构---------------------------------------------------------------OVER
	
  P_Term = Kp * Error;

	/* PID with Changing integration rate */
	float I_VarSpeedf = 0;
	if (_pid_Abs(Error) <= VarSpeed_I_B)
	  I_VarSpeedf = 1;
	else if (_pid_Abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
	  I_VarSpeedf = (VarSpeed_I_A - (_pid_Abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
  
  if(Ki != 0){
    integral_e += I_VarSpeedf * Error * dt;
    /*Constrain*/
    _pid_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
  }
  else{
    integral_e = 0;
  }
  
  /* Using Integral separation */
  if (_pid_Abs(Error) < I_SeparThresh)
  { 
    I_Term = Ki * integral_e;
    /*Constarin*/
    _pid_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    I_Term = 0;
  }
  
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current) / dt;
  else
    d_err = (Error - pre_error) / dt;

	
  D_Term = Kd * d_err;

  pre_error = Error;

  Out = P_Term + I_Term + D_Term;
  
  /* Constarin */
  _pid_Constrain(&Out, -Out_Max, Out_Max);
  
  return Out;
}


void PositionPID::Reset()
{
	Error = 0;
	Current = 0;
	pre_Current = 0;
	P_Term = 0;
	I_Term = 0;
	D_Term = 0;
	Out = 0;
	integral_e = 0;
}


