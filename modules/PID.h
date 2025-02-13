#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController 
{
    private:
        float Kp, Ki, Kd;
        float prevError;    // untuk derifatif
        float integral;     // untuk integral
        float setpoint;     // jarak yang diharapkan

        // untuk self-tune
        int tuneCount;
        float lastError;

    public:
        PIDController(float Kp, float Ki, float Kd, float setpoint);
       
        // untuk mendapatkan nilai Kp, Ki, dan Kd (jika diinginkan)
        float get_Kp();
        float get_Ki();
        float get_Kd();

        // untuk set nilai Kp, Ki, dan Kd dalam PID (jika nilainya ingin diubah di serial monitor secara manual)
        void set_PID(float kp, float ki, float kd);

        void set_tunings(float newKp, float newKi, float newKd);
        void set_setpoint(float new_setpoint);
        float compute(float input, float dt);

        void selfTune(float error); // dipanggil di method compute
};

#endif
