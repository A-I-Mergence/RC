#ifndef RC_H
#define RC_H
#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT  0
#define REVERSE  1
#include "mbed.h"

// comment
 
class RC 
{ 
public: 
    Timer RCtimer;
    RC(float* Input, float* Output, float* Setpoint); 
    void SetMode(int Mode);                             //   sets PID to either Manual (0) or Auto (non-0)
    
    void ParaMotor(float, float, float, float);     //   paramètres spécifique a chaque moteur
    void setParamRC(float);

    void CalculRC();                                    //   Calcul des différents coéfficients du regulateur RC

    bool Regule(float);                                 //   calcul de la régulation de mon moteur
    float computeCommande(float, float);                       //   calcul de la régulation de mon moteur
 
    void SetOutputLimits(float, float);               //   clamps the output to a specific range. 0-255 by 
                                                        //   default, but it's likely the user will want 
                                                        //   to change this depending on the application   

    void reset();


    float getPreCommande(){ return PreCommande[1]; }
    float getError(){ return e[1]; }
    float getRC(){ return Reg_RC[1]; }
    float getRC1s(){return Reg_1s[1];}

  private:
    void Initialize();
 
    //entrée sortie et setpoint pour la régulation
    float *myInput;
    float *myOutput;
    float *mySetpoint;

    //variables pour SetMode()  Initialize() et SetOutputLimits()     
    float outMin, outMax;
    bool inAuto;
    float lastTime;
    float outputSum, lastInput;
    
    //Fréquence d'échantillonnage
    float SampleTime;

    //variables pour les calculs des précommandes des intégrateur et du RC
    float e[2];                // e[0] - last, e[1] - actuelle
    float PreCommande[2];      // PreCommande[0] - last, PreCommande[1] - actuelle
    float Reg_RC[2];           // Reg_RC[0] - last, Reg_RC[1] - actuelle
    float Reg_1s[2];           // Reg_1s[0] - last, Reg_1s[1] - actuelle

    //variables paramètres du moteur
    float _a0;
    float _a1;
    float _a2;
    float _b0;
    float _A2 = _a1;
    float _A1 = _a0;
    float _A0 = 0;
  
    //variables parmètre pour le régulateur et la précommande
    float _T=0.3;
    float _w=3;
    float _omega;

    // RC regulator parameters
    float _c1;
    float _c0;
    float _r1;
    float _r0;
    float _k;
};
#endif