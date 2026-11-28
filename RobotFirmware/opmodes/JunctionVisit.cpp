#include "NRL.h"
#include <string.h>

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};

constexpr bool SWAP_SIDES      = true;  // true: MOTOR_L pins drive the right wheels
constexpr bool SENSOR_REVERSED = false;  // true: ch0 is on the robot's right

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 32 }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,.flipped = true, .minDrive = 32 }};
static TankDrive tank(SWAP_SIDES ? rightMotor : leftMotor, SWAP_SIDES ? leftMotor : rightMotor);
static HexaServo gripper {{ .signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0 }};
static HexaServo arm {{ .signalPin = SERVO_3, .startAngle = 120.0f, .settleMs = 0 }};

constexpr float ARM_UP   = 160.0f;
constexpr float ARM_DOWN = 91.5f;
constexpr float GRIPPER_OPEN  = 55.0f;
constexpr float GRIPPER_CLOSED = 90.0f;
constexpr float ARM_DEG_S  = 100.0f;   // movement speed limits (deg/s)
constexpr float GRIP_DEG_S = 200.0f;

// static HexaLineArrayConfig lineCfg() {           //Function which basically checks the configuration of the line sensor 
//     HexaLineArrayConfig c = hexaLineConfig();    //and returns the configuration to the HexaLineArray class.
//     c.reversed = SENSOR_REVERSED;                //if SENSOR_REVERSED is true, the ch0 is on the right side of the robot, ch7 is on the left side of the robot.
//     return c;                                    //if SENSOR_REVERSED is false, the ch0 is on the left side of the robot, ch7 is on the right side of the robot.
// }                                                //it will work like the hexalineConfig() function

//static HexaLineArray line{ lineCfg() };

static HexaLineArray line{hexaLineConfig()}; //assumption that the ch0 is on the left side of the robot, ch7 is on the right side of the robot

static LineTuning myTuning() {
    LineTuning t;
    // t.followSpeed = 0.40f;   // faster = riskier
    // t.pid.kP      = 0.50f;   // raise if it loses bends, lower if it zig-zags
    // t.pid.kD      = 0.04f;   // raise if it keeps wobbling
    //t.turnSlowSpeed = 0.25f; // slower = more accurate
    return t;
}
static LineActions lineActions(tank, line, myTuning());

//constexpr int  NUM_VISITS      = 2;      // how many junction branches to visit
constexpr unsigned long WAIT_MS = 500;  // wait at the midpoint
constexpr unsigned long MID_MS  = 600;   // TIME to drive along the branch before stopping
                                         // (see tuning notes - adjust on your mat)

// constexpr bool START_IN_BOX    = true;   // robot starts inside a black box -> leaveBox()
// constexpr bool FINISH_IN_BOX   = true;   // end in the next black box -> followToBox()+park()
constexpr int  PARK_MS         = 400;    // roll further into the end box

constexpr unsigned long RUN_MS = 150000;   // 150 s challenge length

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------
static const char* stage = "ready";
static ActionPtr at(const char* name) {
    return instant([name]{ stage = name; });
}
// static bool hasFault() { return strcmp(lineActions.faultName(), "OK") != 0; }

// // Branch on the robot's RIGHT: turn right in, turn right again coming back.
// static ActionPtr visitRight() {
//     return lineActions.auto_()
//         .right()                                   // onto the branch
//         .then(at("to midpoint")).followFor(MID_MS) // drive along it
//         .stop()
//         .then(at("WAITING")).sleepMs(WAIT_MS)
//         .uTurn()
//         .then(at("back to line")).followToJunction()
//         .right()                                   // original direction is on the right
//         .build();
// }

// // Branch on the robot's LEFT: mirror image.
// static ActionPtr visitLeft() {
//     return lineActions.auto_()
//         .left()
//         .then(at("to midpoint")).followFor(MID_MS)
//         .stop()
//         .then(at("WAITING")).sleepMs(WAIT_MS)
//         .uTurn()
//         .then(at("back to line")).followToJunction()
//         .left()
//         .build();
// }

// // ---------------------------------------------------------------------
// // Run control (a small state machine driven from loop())
// // ---------------------------------------------------------------------
// enum Phase { PH_BOOT, PH_SEEK, PH_AT_JUNCTION, PH_FINISHING, PH_DONE };
// static Phase    phase      = PH_BOOT;
// static int      visits     = 0;
// static uint32_t runStartMs = 0;

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

            // ---------- junction 1: branch on the RIGHT ----------
            .then(at("to J1")).followToJunction(LineSide::RIGHT, 1)
            .right()                                    // onto the branch
            .then(at("to mid 1")).followFor(MID_MS)
            .stop()
            .then(pickCore())
            .then(at("WAIT 0.5")).sleepMs(WAIT_MS)
            .uTurn()
            .then(at("back 1")).followToJunction()      // back on the main line
            .right()                                    // original direction is on the right

            // ---------- junction 2: branch on the LEFT ----------
            .then(at("to J2")).followToJunction(LineSide::LEFT, 1)
            .left()
            .then(at("to mid 2")).followFor(MID_MS)
            .stop()
            .then(releaseCore())
            .then(at("WAIT 0.5")).sleepMs(WAIT_MS)
            .uTurn()
            .then(at("back 2")).followToJunction()
            .left()                                     // original direction is on the left

            // ---------- finish in the end box ----------
            .then(at("to end box")).followToBox()
            .park(PARK_MS)
            .then(at("DONE"))
            .build());
}

class JunctionVisit : public NRLOpMode {
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
        // if (!isActionRunning() && phase != PH_DONE) {

        // const LineJunction& jj = lineActions.lastJunction();
        // char lrs[4] = { jj.left ? 'L' : '-', jj.right ? 'R' : '-', jj.straight ? 'S' : '-', 0 };
        telemetry.addData("stage",  stage);
        telemetry.addData("fault",  lineActions.faultName());
    //    telemetry.addData("visits", (float)visits);
        telemetry.addData("jx",     (float)lineActions.junctionCount());
    //    telemetry.addData("last",   lrs);
        telemetry.addData("err",    (float)line.error());
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        tank.stop();
    }
};

REGISTER_OPMODE(JunctionVisit, "JunctionVisit", AUTO);
