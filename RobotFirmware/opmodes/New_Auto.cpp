#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
// static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};

constexpr bool SWAP_SIDES      = true;  // true: MOTOR_L pins drive the right wheels
constexpr bool SENSOR_REVERSED = false;  // true: ch0 is on the robot's right

static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 30 }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,.flipped = true, .minDrive = 30 }};
static TankDrive tank(SWAP_SIDES ? rightMotor : leftMotor, SWAP_SIDES ? leftMotor : rightMotor);

// static HexaLineArrayConfig lineCfg() {           //Function which basically checks the configuration of the line sensor 
//     HexaLineArrayConfig c = hexaLineConfig();    //and returns the configuration to the HexaLineArray class.
//     c.reversed = SENSOR_REVERSED;                //if SENSOR_REVERSED is true, the ch0 is on the right side of the robot, ch7 is on the left side of the robot.
//     return c;                                    //if SENSOR_REVERSED is false, the ch0 is on the left side of the robot, ch7 is on the right side of the robot.
// }                                                //it will work like the hexalineConfig() function

//static HexaLineArray line{ lineCfg() };

static HexaLineArray line{hexaLineConfig()}; //assumption that the ch0 is on the left side of the robot, ch7 is on the right side of the robot

static LineTuning myTuning() {
    LineTuning t;
    // t.followSpeed = 0.35f;   // faster = riskier
    // t.pid.kP      = 0.50f;   // raise if it loses bends, lower if it zig-zags
    // t.pid.kD      = 0.04f;   // raise if it keeps wobbling
    return t;
}
static LineActions lineActions(tank, line, myTuning());

constexpr unsigned long RUN_MS = 150000;   // 150 s challenge length

class New_Auto : public NRLOpMode {
public:
    void init() override {
        // Runs once when INIT is pressed — begin() your hardware here.
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
    }

    void start() override {
        // Runs once when the match starts — queue your routine
        // with runAction(...) here.
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        // Follow the line for the whole run. Corners are turned automatically,
        // junctions are crossed straight. Stops by itself if the line is lost.
        runAction(lineActions.auto_()
                      .followFor(RUN_MS)
                      .stop()
                      .build());
    }

    void loop() override {
        // Runs at 100 Hz for up to 2:30 — telemetry and sensor reads go here.
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("err",   (float)line.error());
        telemetry.addData("jx",    (float)lineActions.junctionCount());
        telemetry.addData("cnr",   (float)lineActions.cornerCount());
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        tank.stop();
    }
};

REGISTER_OPMODE(New_Auto, "Simple_Auto", AUTO);
