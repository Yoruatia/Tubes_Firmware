#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController 
{
    private:
        double Kp, Ki, Kd;
        double prevError;    // untuk derifatif
        double integral;     // untuk integral
        double setpoint;     // jarak yang diharapkan

        // untuk self-tune
        int tuneCount;
        double lastError;

    public:
        PIDController(double Kp, double Ki, double Kd, double setpoint);
       
        // untuk mendapatkan nilai Kp, Ki, dan Kd (jika diinginkan)
        double get_Kp();
        double get_Ki();
        double get_Kd();

        // untuk set nilai Kp, Ki, dan Kd dalam PID (jika nilainya ingin diubah di serial monitor secara manual)
        void set_PID(double kp, double ki, double kd);

        void set_tunings(double newKp, double newKi, double newKd);
        void set_setpoint(double new_setpoint);
        double compute(double input, double dt);

        void selfTune(double error); // dipanggil di method compute
};

#endif
