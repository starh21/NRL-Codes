#include <NRL.h>
#include <HexaOLED.h>

class TheOLEDdisplay : public NRLOpMode {
public:
  HexaOLED _oled {HexaOLEDConfig()};
  bool _oledReady = false;
   int _lastUpdatedPct = -1;   // percentage at the last OLED write (-1 = not drawn yet)(AN)
   bool _lastLow = false;     // LOW flag at the last OLED write (used to redraw when it changes) 

    // Estimate of battery percentage from voltage(AN)
  int getBatteryPercentage(float volts) {
    float minVolt = 9.0;    // 0%
    float maxVolt = 12.6;   // 100%
    int pct = (int)(((volts - minVolt) / (maxVolt - minVolt)) * 100);
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    return pct;
  }


  void init() override {
    _oledReady = _oled.begin();
    telemetry.addData("OLED", _oledReady ? "found" : "not found");
  }

  void loop() override {
    if (!_oledReady) return;
    float volts = power.getBatteryVoltage();
    int currentPct = getBatteryPercentage(volts);
    bool low = power.isBatteryLow();
    bool shouldUpdate = false;

    if (_lastUpdatedPct == -1) {
      shouldUpdate = true;                       // first draw(AN)
    } else {
      int pctDiff = currentPct - _lastUpdatedPct;

      if (pctDiff > 0) {
        if (pctDiff >= 5) shouldUpdate = true;   // charging: every 5%(AN)
      } else if (pctDiff < 0) {
        if (currentPct < 10) {
          shouldUpdate = true;                   // below 10%: every 1% drop(AN)
        } else if (-pctDiff >= 20) {
          shouldUpdate = true;                   // otherwise: every 20% drop(AN)
        }
      }
    }
    
    if (low != _lastLow) shouldUpdate = true;   // redraw when the LOW flag changes

    if (!shouldUpdate) return;
    _lastUpdatedPct = currentPct;
    _lastLow = low;

    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(HexaOLED::WHITE);   

    // Line 1: battery voltage + percentage + low flag
    _oled.setCursor(0, 0);
    _oled.print("Batt: ");
    _oled.print(volts, 1);
    _oled.print("V (");        // prints:  V (
    _oled.print(currentPct);   // prints:  percentage
    _oled.print("%) ");        // prints:  %)
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