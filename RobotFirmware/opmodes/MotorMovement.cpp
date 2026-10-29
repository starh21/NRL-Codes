



#include "NRL.h"

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM , .flipped = true}};

class MotorMovement : public NRLOpMode {
    int _movespeed=170;   // 65% throttle
    int _reverseSpeed=130 //to keep control movement

public:
    void init() override {
        leftMotor.begin();  
        rightMotor.begin();
    }

    void loop() override {
    //make sure to test the lines one by one by bypassing other lines 
        moveForward(_moveSpeed);
        moveBackward(_reverseSpeed);
        turnLeft(_moveSpeed);
        turnRight(_moveSpeed);

        telemetry.addData("Speed Setting", (float)_moveSpeed);
        telemetry.addData("Reverse Speed", (float)_reverseSpeed);
    }

    void stop() override {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);.
    }

    //Forward movement
    void moveForward(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);

    //Backward movement
    void moveBackward(int speed) {
        leftMotor.setSpeed(-speed);
        rightMotor.setSpeed(-speed);
    }

    //Turn left movement
    void turnLeft(int speed) {
        leftMotor.setSpeed(-speed);
        rightMotor.setSpeed(speed);
    }

    //Turn right movement
    void turnRight(int speed) {
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(-speed);
  }
};




REGISTER_OPMODE(MotorMovement, "adb", TELEOP);
