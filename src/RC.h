#ifndef RC_H
#define RC_H
#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT  0
#define REVERSE  1
#include "mbed.h"
 
class RC 
{ 
public: 
    Timer RCtimer;
    RC(volatile double* Input, volatile double* Output, volatile double* Setpoint); 
    void SetMode(int Mode);                             //   sets PID to either Manual (0) or Auto (non-0)
    
    void ParaMotor(double, double, double, double);     //   paramètres spécifique a chaque moteur

    void CalculRC();                                    //   Calcul des différents coéfficients du regulateur RC

    bool Regule(float);                                 //   calcul de la régulation de mon moteur
 
    void SetOutputLimits(double, double);               //   clamps the output to a specific range. 0-255 by 
                                                        //   default, but it's likely the user will want 
                                                        //   to change this depending on the application   

  private:
    void Initialize();
 
    //entrée sortie et setpoint pour la régulation
    volatile double *myInput;
    volatile double *myOutput;
    volatile double *mySetpoint;

    //variables pour SetMode()  Initialize() et SetOutputLimits()     
    double outMin, outMax;
    bool inAuto;
    double lastTime;
    double outputSum, lastInput;
    
    //Fréquence d'échantillonnage
    double SampleTime;

    //variables pour les calculs des précommandes des intégrateur et du RC
    double e[2];                // e[0] - last, e[1] - actuelle
    double PreCommande[2];      // PreCommande[0] - last, PreCommande[1] - actuelle
    double Reg_RC[2];           // Reg_RC[0] - last, Reg_RC[1] - actuelle
    double Reg_1s[2];           // Reg_1s[0] - last, Reg_1s[1] - actuelle

    //variables paramètres du moteur
    double _a0;
    double _a1;
    double _a2;
    double _b0;
    double _A2 = _a1;
    double _A1 = _a0;
    double _A0 = 0;
  
    //variables parmètre pour le régulateur et la précommande
    double _T=0.16;
    double _w=3;
    double _omega=(_w+2*sqrt(_w-1))/_T;
    double _d0;
    double _d1;
    double _d2;
    double _d3;
    double _c1;
    double _c0;
    double _r1;
    double _r0;
    double _k=(_r0*_b0)/(_c0*_A0+_r0*_b0);
};
#endif