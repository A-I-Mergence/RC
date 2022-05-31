#include "RC.h"
#include "mbed.h"
#include <cstdio>
 
RC::RC(float* Input, float* Output, float* Setpoint){

    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = true;
    RC::SetOutputLimits(-1, 1); //default output limit corresponds to the arduino pwm limits

    SampleTime = .01; 
    reset();                          
    
    
    RCtimer.start();
    RCtimer.reset();
    lastTime = RCtimer.read()-SampleTime;
}

void RC::reset(){
    int k=1;
    PreCommande[k] = 0;
    PreCommande[k-1] = 0;
    e[k-1] = 0;
    e[k] = 0;
    Reg_RC[k] = 0;
    Reg_RC[k-1] = 0;
    Reg_1s[k] = 0;
    Reg_1s[k-1] = 0;
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

void RC::ParaMotor(float a0, float a1, float a2, float b0){
    _a0 = a0;
    _a1 = a1;
    _a2 = a2;
    _b0 = b0;
    CalculRC();
}

void RC::setParamRC(float p){
    _T = p;
    _omega=(_w+2*sqrt(_w-1))/_T;
    CalculRC();
}

void RC::CalculRC(){  
    _A2 = _a1;
    _A1 = _a0;
    _A0 = 0;
    float _d0=pow(_omega,3);
    float _d1=3*pow(_omega,2);
    float _d2=3*_omega;
    float _d3=1;
    _c1=_d3/_A2;
    _c0=(_d2-_A1*_c1)/_A2;
    _r1=(_d1-_A1*_c0-_A0*_c1)/_b0;
    _r0=(_d0-_A0*_c0)/_b0;
    _k=(_r0*_b0)/(_c0*_A0+_r0*_b0);
}

//                                  RC Regulator
//            __________________________________________________________            Integrator
//           |   -----------                             -----------    |            -------
// Setpoint  |  |     1     | PreCommande +   e         | r1*s + r0 |   | Reg_RC    |   1   | *myOutput
// ------------>|-----------|-------------->O---------->|-----------|-------------->|-------|----------->
//           |  | r1*s + r0 |               ^ -         | c1*s + c0 |   |           |   s   |
//           |   -----------                |            -----------    |            -------
//           |                              |                           |                     *myInput
//           |                              -------------------------------------------------------------
//           |__________________________________________________________|

bool RC::Regule(float dt)
{
    int k=1;
    //  Update of sample time
    SampleTime = dt;
    // Update of measurement
    float input = *myInput;

    // Compute preorder
    PreCommande[k] = (_r0*SampleTime*(*mySetpoint)+_r1*PreCommande[k-1])/(_r1+_r0*SampleTime);
    // Compute error
    e[k] = PreCommande[k] - input;
    // Compute RC regulator output
    Reg_RC[k] = ((_r1+_r0*SampleTime)*e[k]-_r1*e[k-1]+_c1*Reg_RC[k-1])/(_c1+_c0*SampleTime);    // OK

    // Compute result of integrator
    Reg_1s[k] = SampleTime*Reg_RC[k] + Reg_1s[k-1];

    // Update RC output
    *myOutput = Reg_1s[k];

    // Update of state
    PreCommande[k-1]    = PreCommande[k];
    e[k-1]              = e[k];
    Reg_RC[k-1]         = Reg_RC[k];
    Reg_1s[k-1]         = Reg_1s[k];

    return true; 
}

float RC::computeCommande(float dt, float measurement){
    float result;

    int k=1;

    // Compute preorder
    PreCommande[k] = (_r0*dt*(*mySetpoint)+_r1*PreCommande[k-1])/(_r1+_r0*dt);
    // Compute error
    e[k] = PreCommande[k] - measurement;
    // Compute RC regulator output
    Reg_RC[k] = ((_r1+_r0*dt)*e[k]-_r1*e[k-1]+_c1*Reg_RC[k-1])/(_c1+_c0*dt);    // OK

    // Compute result of integrator
    Reg_1s[k] = dt*Reg_RC[k] + Reg_1s[k-1];

    // Update RC output
    result = Reg_1s[k];

    // Update of state
    PreCommande[k-1]    = PreCommande[k];
    e[k-1]              = e[k];
    Reg_RC[k-1]         = Reg_RC[k];
    Reg_1s[k-1]         = Reg_1s[k];

    return result;
}
 
void RC::SetOutputLimits(float Min, float Max)
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