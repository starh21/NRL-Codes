

/*
Whenever you edit any piece of code, make sure to write a comment explaining it.
Inscribe your initials in the comment so we know who made the change and who's suggesting changes to avoid confusion.
Example: //This is a comment (VD)
*/

#include "NRL.h"

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM , .flipped = true}};

class MotorMovement : public NRLOpMode {
    int _movespeed=0;   //Keep the speed at 0 to avoid any movement at the start of the program, else it'll probably do some weird bullshit (VD)
    int _reverseSpeed=0; //Same for this, do any speed changes in loop and not in the class-scope (VD)

public:
    void init() override {
        leftMotor.begin();  
        rightMotor.begin();
    }

    void loop() override {
    //make sure to test the lines one by one by bypassing other lines (AN)
    //This seems to be fine for now, however will need to add a ramp up feature to avoid sudden jerks, or maybe not. Will need to discuss (VD)
    //Will also require an if statement or some shit from the controller code to change this speed. (VD)

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
