#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM , .flipped = true}};
static TankDrive tank(leftMotor, rightMotor);

static HexaServo gripper {{ .signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0 }};
static HexaServo arm {{ .signalPin = SERVO_3, .startAngle = 120.0f, .settleMs = 0 }};

class Robot_Run : public NRLOpMode {
    int Scale = 1.0;
public:
    void init() override {
        // Runs once when INIT is pressed — begin() your hardware here.
        leftMotor.begin();  
        rightMotor.begin();
        gripper.begin();
        arm.begin();
    }

    void loop() override {
        //MOTOR CONTROL
        tank.setRampRate(0.5f); //Sets the ramp rate to half, makes the control smoother in exchange for slower response, may need to adjust(VD)

        tank.setScale(gamepad1.pressed(BTN_LB) ? 0.5f : 1.0f); //Sets the scale to 0.5 if the left bumper is pressed, otherwise sets it to 1.0 (VD)
        telemetry.addData("Scale", tank.getScale());

        tank.drive(gamepad1.leftY(), gamepad1.leftX());

        //Speed telemtry
        telemetry.addData("Left Motor Speed", leftMotor.getSpeed());
        telemetry.addData("Right Motor Speed", rightMotor.getSpeed());

        //Joystick telemetry
        telemetry.addData("Left Joystick Y", gamepad1.leftY());
        telemetry.addData("Left Joystick X", gamepad1.leftX());

        //SERVO CONTROL
        if(gamepad1.rightY() < -0.5f) {
            arm.setPosition(160.0f); // Move up by 10 degrees
            telemetry.addData("Arm Position", arm.getPosition());
        }
        else if(gamepad1.rightY() > 0.5f) {
            arm.setPosition(91.5f);
            telemetry.addData("Arm Position", arm.getPosition());
        }

        if(gamepad1.rightX() < -0.5f) {
            gripper.setPosition(55.0f); // Move up by 10 degrees
            telemetry.addData("Gripper Position", gripper.getPosition());
            
        }
        else if(gamepad1.rightX() > 0.5f) {
            gripper.setPosition(90.0f);
            telemetry.addData("Gripper Position", gripper.getPosition());
            
        }
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        leftMotor.setSpeed(0.0f);
        rightMotor.setSpeed(0.0f);
        gripper.detach();
        arm.detach();
    }
};

REGISTER_OPMODE(Robot_Run, "Final_Robot_Run", TELEOP);
