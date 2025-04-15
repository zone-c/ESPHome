// water_meter.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace water_meter {

class WaterMeterComponent : public Component {
 public:
  WaterMeterComponent() = default;

  void setup() override;
  void loop() override;

  sensor::Sensor *flow_sensor = new sensor::Sensor();
  sensor::Sensor *volume_sensor = new sensor::Sensor();
  sensor::Sensor *duration_sensor = new sensor::Sensor();

 protected:
  void onPulse();

  static constexpr uint32_t PULSE_FACTOR = 120000;
  static constexpr uint32_t SEND_FREQUENCY = 30000;
  static constexpr uint32_t READ_FREQUENCY = 10;
  static constexpr uint32_t MAX_FLOW = 40;
  const double ppl = static_cast<double>(PULSE_FACTOR) / 1000;

  volatile uint32_t pulseCount = 0;
  uint32_t oldPulseCount = 0;
  volatile uint32_t lastBlink = 0;
  volatile double flow = 0;
  uint32_t newBlink = 0;
  double oldflow = 0;
  double volume = 0;
  double oldvolume = 0;
  uint32_t lastSend = 0;
  uint32_t lastPulse = 0;
  uint32_t lastRead = 0;
  double curVolume = 0;
  double oldCurVolume = 0;
  uint32_t startPulse = 0;
  uint32_t duration = 0;
  uint32_t flowStart = 0;
  uint32_t readDelay = 5000;

  const int ledPin = LED_BUILTIN;
  int sensorPin = A0;

  float distance = 0.0f, oldDistance = 0.0f, sens = 0.0f;
  float alpha = 0.02f;
  bool trend = true;
  float slope = 0;
  float threshold = 0.5f;
  float ascendLimit = threshold;
  float descendLimit = -threshold;
};

}  // namespace water_meter
}  // namespace esphome
