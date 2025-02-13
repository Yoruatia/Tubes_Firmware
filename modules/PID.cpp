#include "PID.h"
#include <cmath>


PIDController::PIDController(float Kp, float Ki, float Kd, float setpoint)
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

float PIDController::get_Kp() {return Kp;}
float PIDController::get_Ki() {return Ki;}
float PIDController::get_Kd() {return Kd;}

void PIDController::set_tunings(float newKp, float newKi, float newKd)
{
    Kp = newKp;
    Ki = newKi;
    Kd = newKd;
}

void PIDController::set_setpoint(float new_setpoint)
{
    setpoint = new_setpoint;
}

float PIDController::compute(float input, float dt)
{
    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    selfTune(error);

    return output;
}

void PIDController::set_PID(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::selfTune(float error)
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