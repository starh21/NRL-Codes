#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaServo servo1 {{ .signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0 }};

class Servo_Test1 : public NRLOpMode {
public:
    void init() override {
        // Runs once when INIT is pressed — begin() your hardware here.
        servo1.begin();
    }

    void loop() override {
        // Runs at 100 Hz until STOP — read gamepad1, drive motors here.
        //
        // Control each part in ONE place: loop() or an action, not
        // both. Actions run before loop() each tick, so loop() would
        // overwrite them. To share one part, step aside while a
        // sequence plays:  if (!isActionRunning()) { ... }
        servo1.setPosition(90);
        telemetry.addData("Servo1", servo1.getPosition());
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        servo1.detach();
    }
};

REGISTER_OPMODE(Servo_Test1, "PS5", TELEOP);
