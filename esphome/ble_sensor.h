#include "esphome.h"

class BLESensor : public PollingComponent, public Sensor {
 public:
  // constructor
  BLESensor() : PollingComponent(15000) {}

  void setup() override {
    // This will be called by App.setup()
  }
  void update() override {
    // This will be called every "update_interval" milliseconds.
    ESP_LOGI("BLESensor", "Scanning...");
  }
};