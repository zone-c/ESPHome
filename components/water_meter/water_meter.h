#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_meter {

// Define constants for configuration keys used in component.py
const char *const CONF_SENSOR_PIN = "sensor_pin";
const char *const CONF_PULSE_FACTOR = "pulse_factor";
const char *const CONF_MAX_FLOW = "max_flow";
const char *const CONF_READ_FREQUENCY = "read_frequency";
const char *const CONF_SMOOTHING_ALPHA = "smoothing_alpha";
const char *const CONF_THRESHOLD = "threshold";
const char *const CONF_FLOW_SENSOR = "flow";
const char *const CONF_VOLUME_SENSOR = "volume";
const char *const CONF_DURATION_SENSOR = "duration";

class WaterMeterComponent : public PollingComponent, public sensor::Sensor {
 public:
  // Wait for everything to be ready before starting
  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void update() override; // Replaces the sending part of loop()
  void loop() override;   // Handles the reading part of loop()
  void dump_config() override;

  // Setters for configuration values
  void set_sensor_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_pulse_factor(uint32_t pulse_factor) { this->pulse_factor_ = pulse_factor; }
  void set_max_flow(float max_flow) { this->max_flow_ = max_flow; }
  void set_read_frequency(uint32_t read_frequency) { this->read_frequency_ = read_frequency; }
  void set_smoothing_alpha(float alpha) { this->alpha_ = alpha; }
  void set_threshold(float threshold) { this->threshold_ = threshold; }

  // Setters for associated sensors
  void set_flow_sensor(sensor::Sensor *flow_sensor) { this->flow_sensor_ = flow_sensor; }
  void set_volume_sensor(sensor::Sensor *volume_sensor) { this->volume_sensor_ = volume_sensor; }
  void set_duration_sensor(sensor::Sensor *duration_sensor) { this->duration_sensor_ = duration_sensor; }

 protected:
  void on_pulse_(); // Renamed to avoid conflict and indicate internal use

  // --- Configuration Members ---
  InternalGPIOPin *pin_{nullptr};
  uint32_t pulse_factor_ = 120000; // Pulses per m^3 (originally per 1000L)
  float max_flow_ = 40.0f;         // Max flow (l/min)
  uint32_t read_frequency_ = 10;   // ms between sensor reads
  float alpha_ = 0.02f;            // smoothing factor
  float threshold_ = 0.5f;         // slope threshold

  // --- Sensor Pointers ---
  sensor::Sensor *flow_sensor_{nullptr};
  sensor::Sensor *volume_sensor_{nullptr};
  sensor::Sensor *duration_sensor_{nullptr};

  // --- State Members ---
  double pulses_per_liter_{0.0};

  volatile uint32_t pulse_count_ = 0;
  uint32_t reported_pulse_count_ = 0; // Tracks last reported value to avoid redundant logs/sends
  volatile uint32_t last_blink_micros_ = 0;
  volatile double current_flow_l_min_ = 0.0; // Current calculated flow rate
  double reported_flow_l_min_ = -1.0;        // Tracks last reported value
  double total_volume_liters_ = 0.0;
  double reported_volume_liters_ = -1.0;     // Tracks last reported value

  uint32_t last_pulse_ms_ = 0;
  uint32_t last_read_ms_ = 0;
  uint32_t flow_start_ms_ = 0;
  uint32_t start_pulse_count_ = 0;           // Pulse count when flow started
  uint32_t current_duration_s_ = 0;
  uint32_t reported_duration_s_ = -1;        // Tracks last reported value

  // --- Reading & Calculation Members ---
  float current_distance_ = 0.0f;
  float previous_distance_ = 0.0f;
  float sens_reading_ = 0.0f;
  bool trend_ = true;               // decreasing = false, increasing = true
  float slope_ = 0.0f;              // slope value
  float ascend_limit_ = 0.0f;       // calculated ascend limit
  float descend_limit_ = 0.0f;      // calculated descend limit
  bool first_read_ = true;          // To initialize previous_distance_

#ifdef USE_ESP32 // ESP32 has LEDC for PWM
  int ledc_channel_ = -1;
#endif
  InternalGPIOPin *led_pin_{nullptr}; // Onboard LED pin
};

}  // namespace water_meter
}  // namespace esphome