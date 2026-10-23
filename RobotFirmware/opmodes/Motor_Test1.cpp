#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};

class Motor_Test1 : public NRLOpMode {
    int _speed = 0;

public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
    }

    void loop() override {
        if (_speed < 220) _speed += 5; // creep up 5 units each pass
        leftMotor.setSpeed(_speed);
        rightMotor.setSpeed(_speed);
        telemetry.addData("speed", (float)_speed);
    }

    void stop() override {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }
};

REGISTER_OPMODE(Motor_Test1, "MotorTest", TELEOP);
