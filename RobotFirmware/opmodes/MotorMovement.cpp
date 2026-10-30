

/*
Whenever you edit any piece of code, make sure to write a comment explaining it.
Inscribe your initials in the comment so we know who made the change and who's suggesting changes to avoid confusion.
Example: //This is a comment (VD)
*/

#include "NRL.h"

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM , .flipped = true}};
static TankDrive tank(leftMotor, rightMotor);

class MotorMovement : public NRLOpMode
{
    int Scale = 1.0;
    
public:
    void init() override 
    {
        leftMotor.begin();  
        rightMotor.begin();
    }

    void loop() override 
    {
    //make sure to test the lines one by one by bypassing other lines (AN)
   
        tank.setRampRate(0.5f); //Sets the ramp rate to half, makes the control smoother in exchange for slower response, may need to adjust(VD)

        
        tank.setScale(gamepad1.pressed(button_LB) ? 0.5f : 1.0f); //Sets the scale to 0.5 if the left bumper is pressed, otherwise sets it to 1.0 (VD)
        telemetry.addData("Scale", tank.getScale());

        tank.drive(gamepad1.leftY(), gamepad1.rightX());

        //Speed telemtry
        telemetry.addData("Left Motor Speed", leftMotor.getSpeed());
        telemetry.addData("Right Motor Speed", rightMotor.getSpeed());

        //Joystick telemetry
        telemetry.addData("Left Joystick Y", gamepad1.leftY());
        telemetry.addData("Right Joystick X", gamepad1.rightX());
    }

    void stop() override   
    {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
    }

};



REGISTER_OPMODE(MotorMovement, "tank", TELEOP);
