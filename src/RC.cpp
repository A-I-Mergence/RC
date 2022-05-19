#include "RC.h"
#include "mbed.h"
#include <cstdio>
 
RC::RC(volatile double* Input, volatile double* Output, volatile double* Setpoint){

    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = true;
    RC::SetOutputLimits(-1, 1); //default output limit corresponds to the arduino pwm limits
                                                
    SampleTime = .01;                           
    int k=1;
    e[k-1] = 0;
    e[k] = 0;
    Reg_RC[k-1] = 0;
    
    RCtimer.start();
    RCtimer.reset();
    lastTime = RCtimer.read()-SampleTime;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void RC::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {
        RC::Initialize();
    }
    inAuto = newAuto;
}

void RC::ParaMotor(double a0, double a1, double a2, double b0){
    _a0 = a0;
    _a1 = a1;
    _a2 = a2;
    _b0 = b0;
    CalculRC();
}

void RC::CalculRC(){  
    _A2 = _a1;
    _A1 = _a0;
    _A0 = 0;
    double _d0=pow(_omega,3);
    double _d1=3*pow(_omega,2);
    double _d2=3*_omega;
    double _d3=1;
    double _k=(_r0*_b0)/(_c0*_A0+_r0*_b0);
    _c1=_d3/_A2;
    _c0=(_d2-_A1*_c1)/_A2;
    _r1=(_d1-_A1*_c0-_A0*_c1)/_b0;
    _r0=(_d0-_A0*_c0)/_b0;
}

bool RC::Regule(float dt)
{
   int k=1;
   
   SampleTime = dt;
    //    Declaration des variables   
    double input = *myInput;

    //    Calcul intup2
    PreCommande[k] = (_r0*SampleTime*(*mySetpoint)+_k*_r1*PreCommande[k-1])/(_k*_r1+_k*_r0*SampleTime);
    e[k] = PreCommande[k] - input;

    //    Calcul Regulateur RC et 1/s
    Reg_RC[k] = ((_r1+_r0*SampleTime)*e[k]-_r1*e[k-1]+_c1*Reg_RC[k-1])/(_c1+_c0*SampleTime);
    Reg_1s[k] = SampleTime*Reg_RC[k] + Reg_1s[k-1];

    //    Mise à jour des données à t-1  
    PreCommande[k-1] = PreCommande[k];
    e[k-1] = e[k];
    Reg_RC[k-1] = Reg_RC[k];
    Reg_1s[k-1] = Reg_1s[k];
       
    //    Filtrage du regulateur en sortie
    if (Reg_1s[k] > 1){
        *myOutput = 1;
    }
    else if (Reg_1s[k] < -1){
        *myOutput = -1;
    }
    else {
        *myOutput = Reg_1s[k];
    }
    return true; 
}
 
void RC::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
       if(*myOutput > outMax) *myOutput = outMax;
       else if(*myOutput < outMin) *myOutput = outMin;
 
       if(outputSum > outMax) outputSum= outMax;
       else if(outputSum < outMin) outputSum= outMin;
   }
}
 
/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void RC::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}