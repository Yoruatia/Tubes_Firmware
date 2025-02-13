#include "PID.h"
#include <cmath>


PIDController::PIDController(double Kp, double Ki, double Kd, double setpoint)
{
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;
    this -> setpoint = setpoint;
    this -> prevError = 0;
    this -> integral = 0;

    this -> tuneCount = 0;
    this -> lastError = 0;
}

double PIDController::get_Kp() {return Kp;}
double PIDController::get_Ki() {return Ki;}
double PIDController::get_Kd() {return Kd;}

void PIDController::set_tunings(double newKp, double newKi, double newKd)
{
    Kp = newKp;
    Ki = newKi;
    Kd = newKd;
}

void PIDController::set_setpoint(double new_setpoint)
{
    setpoint = new_setpoint;
}

double PIDController::compute(double input, double dt)
{
    double error = setpoint - input;
    integral += error * dt;
    double derivative = (error - prevError) / dt;
    prevError = error;

    double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    selfTune(error);

    double output;
}

void PIDController::set_PID(double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::selfTune(double error)
{
    // Jika error kecil tidak perlu tuning
    if (fabs(error) < 0.5)
    {
        return;
    }
    
    tuneCount++;
    
    // Jika error terlalu besar, tingkatkan Kp (agar mempercepat)
    if (fabs(error) > 5)
    {
        Kp += 0.5;
    }

    // Jika error tidak berubah (perubahan error tidak signifikan), naikan Ki 
    else if (fabs(error - lastError) < 0.05)
    {
        Ki += 0.01;
    }

    // Jika error kecil, turunkan Kd (agar stabil)
    else if (fabs(error) < 1)
    {
        Kd -= 0.01;
    }

    lastError = error;

    // batasan
    if (tuneCount > 50)
    {
        tuneCount = 0;
    }
}
