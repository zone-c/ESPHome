// water_meter.cpp
#include "water_meter.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_meter {

static const char *const TAG = "WaterMeter";

void WaterMeterComponent::setup() {
  pulseCount = 0;
  lastSend = millis();
  pinMode(ledPin, OUTPUT);
}

void WaterMeterComponent::loop() {
  uint32_t currentTime = millis();

  if ((currentTime - lastRead) > READ_FREQUENCY && currentTime > readDelay) {
    sens = analogRead(sensorPin);
    if (sens < 1024 && sens > 0) {
      lastRead = currentTime;
      readDelay = 0;
      distance = (1 - alpha) * distance + alpha * sens;

      slope = distance - oldDistance;
      oldDistance = distance;

      if (slope > ascendLimit && trend == false) {
        trend = true;
        analogWrite(ledPin, 0);
        delay(10);
        analogWrite(ledPin, 1023);
        onPulse();
      }

      if (slope < descendLimit && trend == true) {
        trend = false;
        analogWrite(ledPin, 0);
        delay(10);
        analogWrite(ledPin, 1023);
        onPulse();
      }
    }
  }

  if (currentTime - lastSend > SEND_FREQUENCY) {
    lastSend = currentTime;

    if (currentTime - lastPulse > 30000) {
      ESP_LOGI(TAG, "Flow stopped");
      flow = 0;
      startPulse = pulseCount;
      flowStart = lastPulse;
    }

    if (flow != oldflow) {
      oldflow = flow;
      ESP_LOGD(TAG, "l/min: %f", flow);
      if (flow < MAX_FLOW) {
        flow_sensor->publish_state(flow);
      }
    }

    if (pulseCount != oldPulseCount) {
      oldPulseCount = pulseCount;
      volume = static_cast<double>(pulseCount) / static_cast<double>(PULSE_FACTOR);
      if (volume != oldvolume) {
        oldvolume = volume;
        ESP_LOGD(TAG, "volume: %.4f", volume);
        volume_sensor->publish_state(volume * 1000);
      }
    }

    curVolume = static_cast<double>(pulseCount - startPulse) / static_cast<double>(PULSE_FACTOR);
    duration = (lastPulse - flowStart) / 1000;
    if (curVolume != oldCurVolume) {
      oldCurVolume = curVolume;
      ESP_LOGD(TAG, "Current flow volume: %.4f in %u seconds", curVolume, duration);
      duration_sensor->publish_state(duration);
    }
  }
}

void WaterMeterComponent::onPulse() {
  uint32_t newBlink = micros();
  uint32_t interval = newBlink - lastBlink;

  if (interval != 0) {
    if (flow == 0) {
      ESP_LOGI(TAG, "Flow started");
      startPulse = pulseCount;
      flowStart = millis();
    }
    lastPulse = millis();
    flow = (60000000.0 / interval) / ppl;
  }
  lastBlink = newBlink;
  pulseCount++;
}

}  // namespace water_meter
}  // namespace esphome
