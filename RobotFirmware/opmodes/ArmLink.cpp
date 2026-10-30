#include "NRL.h"

// Hardware configuration with safety limits to protect physical gears
static HexaServo arm {{ .signalPin = 1, .minAngle = 0.0f, .maxAngle = 180.0f, .startAngle = 0.0f }};

class ArmLink : public NRLOpMode {
public:
    // Simple state-tracking variables (Checks the current position of the arm and the last button state to detect toggles)
    bool armExtended = false;
    bool lastButtonState = false;
    
    // Your physical arm degree positions
    const float RETRACTED_ANGLE = 10.0f; 
    const float EXTENDED_ANGLE = 170.0f;

    void init() override {
        arm.begin();
        arm.setPosition(RETRACTED_ANGLE); // Starts parked safely
    }

    void loop() override {
        // 1. Check the status of the Right Toggle switch
        bool currentButtonState = gamepad1.right_toggle; 

        // 2. Flip the positions only on a clean toggle flip
        if (currentButtonState && !lastButtonState) {
            armExtended = !armExtended; 
        }
        lastButtonState = currentButtonState; 

        // 3. Command the physical servo to move
        if (armExtended) {
            arm.setPosition(EXTENDED_ANGLE);
        } else {
            arm.setPosition(RETRACTED_ANGLE);
        }
    }

    void stop() override {
        arm.detach(); // Power down the servo safely when finished
    }
};

REGISTER_OPMODE(ArmLink, "ArmLink", TELEOP);