class LineBench : public NRLOpMode {
public:
    void init() override { leftMotor.begin(); rightMotor.begin(); line.begin(); }
    void start() override {
        lineActions.clearFault(); lineActions.resetCounts(); lineActions.clearRoute();
    }
    void loop() override {
        if (gamepad1.justPressed(BTN_Y)) {            // STOP everything
            cancelActions();
            drive.stop();
            lineActions.clearFault();
        }
        if (!isActionRunning()) {                     // one step at a time
            if (gamepad1.justPressed(BTN_X))          runAction(lineActions.followToJunction());
            if (gamepad1.justPressed(BTN_DPAD_UP))    runAction(lineActions.followToEnd());
            if (gamepad1.justPressed(BTN_DPAD_LEFT))  runAction(lineActions.left());
            if (gamepad1.justPressed(BTN_DPAD_RIGHT)) runAction(lineActions.right());
            if (gamepad1.justPressed(BTN_DPAD_DOWN))  runAction(lineActions.uTurn());
        }
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("busy",  isActionRunning() ? "yes" : "no");
        telemetry.addData("jx",    (float)lineActions.junctionCount());
        telemetry.addData("cnr",   (float)lineActions.cornerCount());
        telemetry.addData("turn",  lineActions.lastTurnDegrees());
        telemetry.addData("err",   (float)line.error());
    }
    void stop() override { drive.stop(); }
};
REGISTER_OPMODE(LineBench, "Automation Test", TELEOP);