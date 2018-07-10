#include "PID.h"
#include <math.h>

PID::PID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::compute(float Setpoint, float Input, float* Output)
{
    static float errSum = 0.0f;
    static float lastErr = 0.0f;
    static double Ierror_abs = 0;
    static char needsTraining = 1;
    static double currentError = 0;
    static double previousError = 0;
    static double cumulativeError = 0;
    static int count = 0;
    const int epochLength = 200;     // CANTIDAD DE ITERACIONES POR EPOCH
    const double errorThreshold = 5; // THRESHOLD DE ERROR
    //////////////////////////////////////////////////////////////////
    static double learningRate = 0.01;

    /*Compute all the working error variables*/
    float error = Setpoint - Input;
    errSum += error;
    float dErr = (error - lastErr);
    Ierror_abs += fabs(error);
    cumulativeError += (error * error);

     count++;

    /*Compute PID Output*/
    *Output = kp * error + ki * errSum + kd * dErr;

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
                     kp -= kp * error * deltaError * learningRate;
                     ki -= ki * Ierror_abs * deltaError * learningRate;
                     kd -= kd * dErr * deltaError * learningRate;
                 }

                 // Resetear los contadores de errores, para el pr�ximo epoch.
                 Ierror_abs = 0;
                 cumulativeError = 0;
             }
     }

    // TODO: Threshold of output required??

    /*Remember some variables for next time*/
    lastErr = error;
}
