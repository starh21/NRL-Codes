#include "NRL.h"

// Drive and line setup (VD)
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM,
.flipped = true, .minDrive = 29 }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,
.minDrive = 32 }};
static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{ hexaLineConfig() };
static LineActions lineActions(drive, line);

// Servo limits (VD)
constexpr float ARM_DOWN = 115.0f;
constexpr float ARM_UP = 45.0f;
constexpr float GRIP_OPEN = 125.0f;
constexpr float GRIP_CLOSE = 155.0f;
constexpr float ARM_DEG_S = 100.0f;
constexpr float GRIP_DEG_S = 200.0f;

// Safe start pose (VD)
static HexaServo arm {{ .signalPin = SERVO_1, .startAngle = ARM_UP }};
static HexaServo gripper{{ .signalPin = SERVO_2, .startAngle = GRIP_CLOSE }};
static int markJ1 = -1;
static const char* stage = "ready";

// Stage label (VD)
static ActionPtr at(const char* name) {
    return instant([name]{
        stage = name;
    });
}

// Smooth servo move (VD)
static ActionPtr moveServo(HexaServo& s, float to, unsigned long dwellMs) {
    auto arrivedMs = std::make_shared<uint32_t>(0);

    return sequential({
        instant([&s, arrivedMs]{
            s.setPosition(s.getPosition());
            *arrivedMs = 0;
        }),
        wait_until([&s, to, dwellMs, arrivedMs]{
            if (*arrivedMs == 0) {
                s.setPosition(to);
                if (fabsf(s.getPosition() - to) < 0.5f) {
                    *arrivedMs = millis() | 1;
                }
                return false;
            }
            return millis() - *arrivedMs >= dwellMs;
        }),
    });
}

// Pickup sequence (VD)
static ActionPtr pick() {
    return sequential({
        at("PICK"),
        moveServo(gripper, GRIP_OPEN, 200),
        moveServo(arm, ARM_DOWN, 300),
        moveServo(gripper, GRIP_CLOSE, 300),
        moveServo(arm, ARM_UP, 200),
    });
}

// Drop sequence (VD)
static ActionPtr place() {
    return sequential({
        at("PLACE"),
        moveServo(arm, ARM_DOWN, 300),
        moveServo(gripper, GRIP_OPEN, 300),
        moveServo(arm, ARM_UP, 200),
    });
}

// Route plan (VD)
static ActionPtr cargoMission() {
    return lineActions.auto_()
        .then(at("leave box")).leaveStart()
        .then(at("to J1")).followToJunction()
        .markHere(&markJ1)
        .then(at("to T1")).followToEnd()
        .then(place())
        .uTurn()
        .then(at("back to J1")).retraceTo(&markJ1)
        .left()
        .then(at("to J2")).followToJunction()
        .right()
        .then(at("to T2")).followToEnd()
        .then(pick())
        .uTurn()
        .then(at("back to J1")).retraceTo(&markJ1)
        .right()
        .then(at("to T1")).followToEnd()
        .then(place())
        .then(at("DONE"))
        .build();
}

// Op mode lifecycle (VD)
class CargoAuto : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();

        drive.setRampRate(0.12f); // smoothing (VD)

        arm.setRampRate(ARM_DEG_S);
        gripper.setRampRate(GRIP_DEG_S);

        arm.begin();
        gripper.begin();
    }

    void start() override {
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();

        stage = "ready";
        runAction(cargoMission());
    }

    void loop() override {
        const LineJunction& j = lineActions.lastJunction();
        char lrs[4] = { j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-', 0 };

        telemetry.addData("stage", stage); // current step (VD)
        telemetry.addData("fault", lineActions.faultName()); // fault state (VD)
        telemetry.addData("jx", (float)lineActions.junctionCount()); // junction count (VD)
        telemetry.addData("cnr", (float)lineActions.cornerCount()); // corner count (VD)
        telemetry.addData("turn", lineActions.lastTurnDegrees()); // last turn (VD)
        telemetry.addData("hdg", lineActions.gridHeading()); // heading (VD)
        telemetry.addData("last", lrs); // path state (VD)
        telemetry.addData("depth", (float)lineActions.routeDepth()); // route depth (VD)
    }

    void stop() override {
        drive.stop();
        leftMotor.setSpeed(0.0f);
        rightMotor.setSpeed(0.0f);
        gripper.detach();
        arm.detach();
    }
};

REGISTER_OPMODE(CargoAuto, "Cargo Auto", AUTO);