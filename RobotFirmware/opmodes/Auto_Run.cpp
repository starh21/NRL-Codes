#include "NRL.h"
#include <string.h>

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 32 }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,.flipped = true, .minDrive = 32 }};
static TankDrive tank(rightMotor, leftMotor);

static HexaServo gripper {{ .signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0 }};
static HexaServo arm {{ .signalPin = SERVO_3, .startAngle = 120.0f, .settleMs = 0 }};

constexpr float ARM_UP   = 160.0f;
constexpr float ARM_DOWN = 91.5f;
constexpr float GRIPPER_OPEN  = 55.0f;
constexpr float GRIPPER_CLOSED = 90.0f;
constexpr float ARM_DEG_S  = 100.0f;   // movement speed limits (deg/s)
constexpr float GRIP_DEG_S = 200.0f;

constexpr unsigned long WAIT_MS = 500;  // wait at the midpoint
constexpr unsigned long MID_MS  = 650;  // TIME to drive along the branch before stopping
                                        // (see tuning notes - adjust on your mat)

constexpr int  PARK_MS         = 400;    // roll further into the end box
constexpr unsigned long RUN_MS = 5000;   // Depends on the time it takes to bypass the junction in the route

static HexaLineArray line{hexaLineConfig()}; //assumption that the ch0 is on the left side of the robot, ch7 is on the right side of the robot
static LineTuning myTuning() {
    LineTuning t;
    // t.followSpeed = 0.35f;   // faster = riskier
    // t.pid.kP      = 0.50f;   // raise if it loses bends, lower if it zig-zags
    // t.pid.kD      = 0.04f;   // raise if it keeps wobbling
    return t;
}
static LineActions lineActions(tank, line, myTuning());

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

// static int markJ2 = -1;
// static int markJ5 = -1;
// static int markJ8 = -1;

static const char* stage = "ready";
static ActionPtr at(const char* name) {
    return instant([name]{ stage = name; });
}

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
                if (fabsf(s.getPosition() - to) < 0.5f) *arrivedMs = millis() | 1;
                return false;
            }
            return millis() - *arrivedMs >= dwellMs;
        }),
    });
}

static ActionPtr pickCore() {          // open, arm down, close, arm up
    return sequential({
        at("PICK"),
        moveServo(gripper, GRIPPER_OPEN,  200),
        moveServo(arm,     ARM_DOWN,   300),
        moveServo(gripper, GRIPPER_CLOSED, 300),
        moveServo(arm,     ARM_UP,     200),
    });
}

static ActionPtr releaseCore() {       // arm down, open, arm up
    return sequential({
        at("RELEASE"),
        moveServo(arm,     ARM_DOWN,  300),
        moveServo(gripper, GRIPPER_OPEN, 300),
        moveServo(arm,     ARM_UP,    200),
    });
}

static ActionPtr Automation(){
        return(lineActions.auto_()
            .then(at("leave box")).leaveBox()

        // ================= SIDE 1 =================
        // ---------- J1: pick, branch on the RIGHT ----------
        .then(at("to J1")).followToJunction(LineSide::RIGHT, 1)
        .right()    
        .then(at("to mid 1")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 1")).followToJunction()
        .right()                                  // original direction is on the right

        // ---------- J2: drop, branch on the LEFT ----------
        .then(at("to J2")).followToJunction(LineSide::LEFT, 1)
        .left()
        .then(at("to mid 2")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 2")).followToJunction()
        .left()                                   // original direction is on the left

        // ---------- J3: pick, branch on the RIGHT ----------
        .then(at("to J3")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 3")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 3")).followToJunction()
        .left()                                   // now heading BACK toward J2

        // ---------- back to J2: drop (branch is now on the RIGHT) ----------
        .then(at("to J2 again")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 2")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 2")).followToJunction()
        .left()                                   // forward again

        // ================= SIDE 2 =================
        // ---------- J4: pick (J3 is passed, then the corner) ----------
        .then(at("to J4")).followToJunction(LineSide::RIGHT, 2)
        .right()
        .then(at("to mid 4")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 4")).followToJunction()
        .right()

        // ---------- J5: drop ----------
        .then(at("to J5")).followToJunction(LineSide::LEFT, 1)
        .left()
        .then(at("to mid 5")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 5")).followToJunction()
        .left()

        // ---------- J6: pick ----------
        .then(at("to J6")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 6")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 6")).followToJunction()
        .left()                                   // heading BACK toward J5

        // ---------- back to J5: drop ----------
        .then(at("to J5 again")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 5")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 5")).followToJunction()
        .left()

        // ================= SIDE 3 =================
        // ---------- J7: pick (J6 is passed, then the corner) ----------
        .then(at("to J7")).followToJunction(LineSide::RIGHT, 2)
        .right()
        .then(at("to mid 7")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 7")).followToJunction()
        .right()

        // ---------- J8: drop ----------
        .then(at("to J8")).followToJunction(LineSide::LEFT, 1)
        .left()
        .then(at("to mid 8")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 8")).followToJunction()
        .left()

        // ---------- J9: pick ----------
        .then(at("to J9")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 9")).followFor(MID_MS)
        .stop()
        .then(pickCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 9")).followToJunction()
        .left()                                   // heading BACK toward J8

        // ---------- back to J8: drop ----------
        .then(at("to J8 again")).followToJunction(LineSide::RIGHT, 1)
        .right()
        .then(at("to mid 8")).followFor(MID_MS)
        .stop()
        .then(releaseCore())
        .then(at("WAIT")).sleepMs(WAIT_MS)
        .uTurn()
        .then(at("back 8")).followToJunction()
        .left()

        // ---------- finish in the end box (followToBox passes J9 and the corner) ----------
        .then(at("to end box")).followToBox()
        .park(PARK_MS)
        .then(at("DONE"))
        .build());
}

class Auto_Run : public NRLOpMode {
public:
    void init() override {
        // Runs once when INIT is pressed — begin() your hardware here.
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
        gripper.begin();
        arm.begin();
    }

    void start() override {
        // Runs once when the match starts — queue your routine
        // with runAction(...) here.
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();

        runAction(Automation());
    }

    void loop() override {
        // Runs at 100 Hz for up to 2:30 — telemetry and sensor reads go here.
        telemetry.addData("stage",  stage);
        telemetry.addData("fault",  lineActions.faultName());
        telemetry.addData("jx",     (float)lineActions.junctionCount());
        telemetry.addData("err",    (float)line.error());
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        tank.stop();
    }
};

REGISTER_OPMODE(Auto_Run, "Final_Auto_Run", AUTO);
