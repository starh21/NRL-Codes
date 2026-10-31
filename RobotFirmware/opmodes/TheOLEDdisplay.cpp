#include <NRL.h>
#include <HexaOLED.h>

class TheOLEDdisplay : public NRLOpMode {
public:
  HexaOLED _oled {HexaOLEDConfig()};
  bool _oledReady = false;
  unsigned long _lastUpdate = 0;

  void init() override {
    _oledReady = _oled.begin();
    telemetry.addData("OLED", _oledReady ? "found" : "not found");
  }

  void loop() override {
    if (!_oledReady) return;

    if (millis() - _lastUpdate < 200) return;   // ~5 refreshes per second
    _lastUpdate = millis();

    float volts = power.getBatteryVoltage();
    bool  low   = power.isBatteryLow();

    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(HexaOLED::WHITE);   

    // Line 1: battery voltage + low flag
    _oled.setCursor(0, 0);
    _oled.print("Batt: ");
    _oled.print(volts, 1);
    _oled.print("V ");
    _oled.print(low ? "LOW!" : "OK");

    // Line 2: motor rail current (prints the current on the motor power rail)
    _oled.setCursor(0, 16);
    _oled.print("MotI: ");
    _oled.print(power.getMotorCurrent(), 2);

    _oled.display();
  }

  void stop() override {
    if (_oledReady) {
      _oled.clearDisplay();
      _oled.display();
    }
  }
};

REGISTER_OPMODE(TheOLEDdisplay, "TheOLEDdisplay", TELEOP);