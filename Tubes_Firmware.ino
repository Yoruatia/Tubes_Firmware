#include "PID.h"
#include <Servo.h>
#include <hcsr04.h>

// Definisikan pin
#define TRIGGER_PIN PA0
#define ECHO_PIN PA1
#define SERVO_PIN PB0

// Inisialisasi objek
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
Servo servo;

// PID variabel
double Kp = 1.0, Ki = 0.0, Kd = 0.0;
double desired_distance = 50.0; // Misal 50 cm
PID pidController(Kp, Ki, Kd);

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN); // Inisialisasi servo
    pidController.setSetpoint(desired_distance);
}

void loop() {
    double distance = ultrasonic.read(); // Baca jarak dari sensor ultrasonik
    double control = pidController.compute(distance); // Hitung output PID

    // Konversi output PID ke sudut servo (misal antara 0 hingga 180 derajat)
    int servo_angle = map(control, -100, 100, 0, 180);
    servo.write(servo_angle);

    // Tuning PID melalui Serial
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        sscanf(input.c_str(), "%lf,%lf,%lf", &Kp, &Ki, &Kd);
        pidController.setConstants(Kp, Ki, Kd);
        Serial.print("Kp: "); Serial.print(Kp);
        Serial.print(" Ki: "); Serial.print(Ki);
        Serial.print(" Kd: "); Serial.println(Kd);
    }

    delay(100); // Delay sesuai kebutuhan
}
