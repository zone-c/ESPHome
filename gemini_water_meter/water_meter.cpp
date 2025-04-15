#include "water_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include <driver/ledc.h>
#endif

namespace esphome {
namespace water_meter {

static const char *const TAG = "water_meter";

void WaterMeterComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Water Meter...");
  if (this->pin_ == nullptr) {
    ESP_LOGE(TAG, "Sensor pin is not configured!");
    this->mark_failed();
    return;
  }
  this->pin_->setup();
  this->pin_->pin_mode(gpio::FLAG_INPUT); // Set pin as input

  this->pulses_per_liter_ = static_cast<double>(this->pulse_factor_) / 1000.0;
  this->ascend_limit_ = this->threshold_;
  this->descend_limit_ = -this->threshold_;

  // Setup onboard LED if available (better handled by a separate 'output' or 'light' component usually)
  this->led_pin_ = new InternalGPIOPin(LED_BUILTIN); // Use the board's built-in LED definition
  if (this->led_pin_ != nullptr) {
      this->led_pin_->setup();
      this->led_pin_->pin_mode(gpio::FLAG_OUTPUT);
#ifdef USE_ESP32
      // Setup LEDC for smoother blinking/dimming if needed, or just use digitalWrite
      static int s_ledc_channel_num = 0;
      if(s_ledc_channel_num < LEDC_CHANNEL_MAX) {
          this->ledc_channel_ = s_ledc_channel_num++;
          ledc_timer_config_t ledc_timer = {
              .speed_mode = LEDC_LOW_SPEED_MODE,
              .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit resolution (0-1023)
              .timer_num = LEDC_TIMER_0,
              .freq_hz = 5000, // 5 kHz frequency
              .clk_cfg = LEDC_AUTO_CLK
          };
          ledc_timer_config(&ledc_timer);

          ledc_channel_config_t ledc_channel = {
              .gpio_num = this->led_pin_->get_pin(),
              .speed_mode = LEDC_LOW_SPEED_MODE,
              .channel = (ledc_channel_t)this->ledc_channel_,
              .intr_type = LEDC_INTR_DISABLE,
              .timer_sel = LEDC_TIMER_0,
              .duty = 0, // Start with LED off
              .hpoint = 0
          };
          ledc_channel_config(&ledc_channel);
          ESP_LOGD(TAG, "Configured LEDC Channel %d for LED pin %d", this->ledc_channel_, this->led_pin_->get_pin());
      } else {
          ESP_LOGW(TAG, "No more LEDC channels available for LED.");
          this->ledc_channel_ = -1; // Fallback to digitalWrite
          this->led_pin_->digital_write(false);
      }
#else // ESP8266 or others
      this->led_pin_->digital_write(false); // Start with LED off
#endif
  } else {
       ESP_LOGW(TAG, "Could not allocate internal pin for LED_BUILTIN.");
  }


  this->pulse_count_ = 0;
  this->last_read_ms_ = millis();
  this->last_pulse_ms_ = millis(); // Initialize to prevent immediate "Flow stopped"
}

void WaterMeterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Water Meter:");
  LOG_PIN("  Sensor Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG, "  Pulse Factor (per m³): %u", this->pulse_factor_);
  ESP_LOGCONFIG(TAG, "  Pulses Per Liter: %.2f", this->pulses_per_liter_);
  ESP_LOGCONFIG(TAG, "  Max Flow (l/min): %.1f", this->max_flow_);
  ESP_LOGCONFIG(TAG, "  Read Frequency (ms): %u", this->read_frequency_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->get_update_interval());
  ESP_LOGCONFIG(TAG, "  Smoothing Alpha: %.3f", this->alpha_);
  ESP_LOGCONFIG(TAG, "  Threshold: %.2f", this->threshold_);
  LOG_SENSOR("  ", "Flow Sensor", this->flow_sensor_);
  LOG_SENSOR("  ", "Volume Sensor", this->volume_sensor_);
  LOG_SENSOR("  ", "Duration Sensor", this->duration_sensor_);
}

void WaterMeterComponent::loop() {
  uint32_t current_time = millis();

  // High frequency reading loop
  if ((current_time - this->last_read_ms_) >= this->read_frequency_) {
      this->last_read_ms_ = current_time;

      // Use pin_->analog_read() for ADC capable pins
      // Assuming the original A0 implies an analog sensor
      this->sens_reading_ = this->pin_->analog_read(); // Returns value between 0.0 and 1.0

      // Scale if needed, assuming original code used 0-1023 range directly
      float scaled_reading = this->sens_reading_ * 1023.0f;

      if (scaled_reading < 1024 && scaled_reading > 0) // Check validity
      {
          // Initialize previous distance on first valid read
          if (this->first_read_) {
              this->current_distance_ = scaled_reading;
              this->previous_distance_ = this->current_distance_;
              this->first_read_ = false;
          } else {
              this->current_distance_ = (1.0f - this->alpha_) * this->previous_distance_ + this->alpha_ * scaled_reading;
          }

          this->slope_ = this->current_distance_ - this->previous_distance_;
          this->previous_distance_ = this->current_distance_;

          // Detect rising edge (start of slope)
          if (this->slope_ > this->ascend_limit_ && !this->trend_) {
              this->trend_ = true;
              if (this->led_pin_ != nullptr) {
                  #ifdef USE_ESP32
                  if (this->ledc_channel_ != -1) {
                     ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_, 0); // Off
                     ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_);
                     delay(1); // Short delay
                     ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_, 1023); // Max bright
                     ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_);
                  } else {
                     this->led_pin_->digital_write(false); delay(1); this->led_pin_->digital_write(true);
                  }
                  #else
                     this->led_pin_->digital_write(false); delay(1); this->led_pin_->digital_write(true);
                  #endif
              }
              this->on_pulse_();
          }

          // Detect falling edge (peak reached)
          if (this->slope_ < this->descend_limit_ && this->trend_) {
              this->trend_ = false;
              if (this->led_pin_ != nullptr) {
                  #ifdef USE_ESP32
                  if (this->ledc_channel_ != -1) {
                     ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_, 0); // Off
                     ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_);
                     delay(1); // Short delay
                     ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_, 1023); // Max bright
                     ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_);
                  } else {
                     this->led_pin_->digital_write(false); delay(1); this->led_pin_->digital_write(true);
                  }
                  #else
                     this->led_pin_->digital_write(false); delay(1); this->led_pin_->digital_write(true);
                  #endif
              }
              this->on_pulse_();
          }
      } else {
         ESP_LOGW(TAG, "Sensor reading out of range: %.2f", scaled_reading);
      }
  }

  // Turn off LED after a short duration if using simple digital write
  #ifndef USE_ESP32
  if (this->led_pin_ != nullptr && this->led_pin_->digital_read() && (current_time - this->last_pulse_ms_) > 50) {
       this->led_pin_->digital_write(false);
  }
  #else
   if (this->led_pin_ != nullptr && this->ledc_channel_ != -1 && ledc_get_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_) > 0 && (current_time - this->last_pulse_ms_) > 50) {
       ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_, 0); // Off
       ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)this->ledc_channel_);
   }
  #endif
}

void WaterMeterComponent::update() {
  // This function is called periodically based on update_interval
  uint32_t current_time = millis();
  noInterrupts(); // Read volatile vars safely
  uint32_t current_pulse_count = this->pulse_count_;
  double flow_l_min = this->current_flow_l_min_;
  interrupts();

  // Check for flow stop (no pulse for 30 seconds)
  // Use a slightly longer interval than the update interval to avoid race conditions
  if (flow_l_min > 0 && (current_time - this->last_pulse_ms_) > 30000) {
    ESP_LOGD(TAG, "Flow stopped (no pulse for 30s)");
    flow_l_min = 0.0;
    this->current_flow_l_min_ = 0.0; // Update internal state too
    // Reset flow start time/count when flow actually stops
    this->flow_start_ms_ = 0;
    this->start_pulse_count_ = current_pulse_count; // Keep track of total pulses
  }

  // Publish Flow Rate
  if (this->flow_sensor_ != nullptr && flow_l_min != this->reported_flow_l_min_) {
    // Filter outliers
    float final_flow = (flow_l_min > this->max_flow_) ? 0.0 : flow_l_min; // Report 0 if exceeds max_flow
    if(final_flow != this->reported_flow_l_min_) { // Check again after filtering
        ESP_LOGD(TAG, "Publishing Flow: %.2f l/min", final_flow);
        this->flow_sensor_->publish_state(final_flow);
        this->reported_flow_l_min_ = final_flow; // Store the filtered value
    }
  }

  // Calculate and Publish Total Volume
  this->total_volume_liters_ = static_cast<double>(current_pulse_count) / this->pulses_per_liter_;
  if (this->volume_sensor_ != nullptr && this->total_volume_liters_ != this->reported_volume_liters_) {
     ESP_LOGD(TAG, "Pulse Count: %u, Publishing Volume: %.3f L", current_pulse_count, this->total_volume_liters_);
     this->volume_sensor_->publish_state(this->total_volume_liters_);
     this->reported_volume_liters_ = this->total_volume_liters_;
     this->reported_pulse_count_ = current_pulse_count; // Update reported pulse count
  } else if (current_pulse_count != this->reported_pulse_count_) {
    // Log pulse count changes even if volume hasn't changed enough to report (due to precision)
    ESP_LOGV(TAG, "Pulse Count changed: %u", current_pulse_count);
    this->reported_pulse_count_ = current_pulse_count;
  }


  // Calculate and Publish Current Flow Duration
  if (flow_l_min > 0 && this->flow_start_ms_ != 0) {
      this->current_duration_s_ = (this->last_pulse_ms_ - this->flow_start_ms_) / 1000;
  } else {
      // If flow is 0, duration should also be 0 or reset.
      this->current_duration_s_ = 0;
  }

  if (this->duration_sensor_ != nullptr && this->current_duration_s_ != this->reported_duration_s_) {
      ESP_LOGD(TAG, "Publishing Duration: %u s", this->current_duration_s_);
      this->duration_sensor_->publish_state(this->current_duration_s_);
      this->reported_duration_s_ = this->current_duration_s_;
      // Log current volume for this flow duration
      double current_flow_volume = (static_cast<double>(current_pulse_count - this->start_pulse_count_)) / this->pulses_per_liter_;
      ESP_LOGD(TAG, "  Volume during this flow: %.3f L", current_flow_volume);
  }

}

void WaterMeterComponent::on_pulse_() {
  uint32_t now_micros = micros();
  uint32_t now_millis = millis();
  uint32_t interval_micros = now_micros - this->last_blink_micros_;

  // Basic debounce: ignore pulses too close together (e.g., < 1ms)
  if (interval_micros < 1000) {
    return;
  }

  // If flow was previously stopped, mark the start time and pulse count
  if (this->current_flow_l_min_ == 0.0) {
      ESP_LOGD(TAG, "Flow started");
      this->start_pulse_count_ = this->pulse_count_; // Record count *before* incrementing
      this->flow_start_ms_ = now_millis;
  }

  // Calculate flow rate
  // Flow (L/min) = (1 pulse / interval_us) * (1,000,000 us / 1 s) * (60 s / 1 min) / (pulses / L)
  if (interval_micros > 0 && this->pulses_per_liter_ > 0) {
      this->current_flow_l_min_ = (60000000.0 / static_cast<double>(interval_micros)) / this->pulses_per_liter_;
       // Apply a simple filter to prevent reporting excessively high transient values immediately
       if (this->current_flow_l_min_ > this->max_flow_ * 1.5) { // Allow slightly higher transient
           this->current_flow_l_min_ = this->max_flow_ * 1.5;
       }
  } else {
    // Avoid division by zero or invalid ppl; set flow to 0 if interval is 0
     this->current_flow_l_min_ = 0.0;
  }


  this->last_blink_micros_ = now_micros;
  this->last_pulse_ms_ = now_millis;
  this->pulse_count_++; // Increment pulse count *after* calculations using the previous count state
}

}  // namespace water_meter
}  // namespace esphome