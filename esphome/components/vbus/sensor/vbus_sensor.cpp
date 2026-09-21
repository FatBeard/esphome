#include "vbus_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::vbus {

static const char *const TAG = "vbus.sensor";

static inline uint16_t get_u16(const std::vector<uint8_t> &message, size_t start) {
  return (message[start + 1] << 8) + message[start];
}

static inline int16_t get_i16(const std::vector<uint8_t> &message, size_t start) {
  return (int16_t) ((message[start + 1] << 8) + message[start]);
}

// Heat quantity is reported either as three words (Wh, kWh, MWh) or as one 32 bit Wh counter.
static inline float get_heat_quantity(const std::vector<uint8_t> &message, size_t start) {
  return get_u16(message, start) + get_u16(message, start + 2) * 1000.0f + get_u16(message, start + 4) * 1000000.0f;
}

static inline float get_heat_quantity32(const std::vector<uint8_t> &message, size_t start) {
  return (static_cast<uint32_t>(get_u16(message, start + 2)) << 16) | get_u16(message, start);
}

// Payload offsets below are fixed per model, so a short message means the controller is not the
// configured model or the telegram was truncated; publishing then would read past the buffer.
static bool message_length_ok(const std::vector<uint8_t> &message, size_t length) {
  if (message.size() >= length)
    return true;
  ESP_LOGW(TAG, "message too short: %zu < %zu", message.size(), length);
  return false;
}

static void publish_if_set(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr)
    sensor->publish_state(value);
}

void DeltaSolSensorBase::dump_sensors_(const char *model) {
  ESP_LOGCONFIG(TAG, "%s:", model);
  LOG_SENSOR("  ", "Temperature 1", this->temperature1_sensor_);
  LOG_SENSOR("  ", "Temperature 2", this->temperature2_sensor_);
  LOG_SENSOR("  ", "Temperature 3", this->temperature3_sensor_);
  LOG_SENSOR("  ", "Temperature 4", this->temperature4_sensor_);
  LOG_SENSOR("  ", "Temperature 5", this->temperature5_sensor_);
  LOG_SENSOR("  ", "Pump Speed 1", this->pump_speed1_sensor_);
  LOG_SENSOR("  ", "Pump Speed 2", this->pump_speed2_sensor_);
  LOG_SENSOR("  ", "Operating Hours 1", this->operating_hours1_sensor_);
  LOG_SENSOR("  ", "Operating Hours 2", this->operating_hours2_sensor_);
  LOG_SENSOR("  ", "Heat Quantity", this->heat_quantity_sensor_);
  LOG_SENSOR("  ", "System Time", this->time_sensor_);
  LOG_SENSOR("  ", "FW Version", this->version_sensor_);
  LOG_SENSOR("  ", "Flow Rate", this->flow_rate_sensor_);
}

void DeltaSolBSPlusSensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 28))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[8]);
  publish_if_set(this->pump_speed2_sensor_, message[9]);
  publish_if_set(this->time_sensor_, get_u16(message, 12));
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 16));
  publish_if_set(this->operating_hours2_sensor_, get_u16(message, 18));
  publish_if_set(this->heat_quantity_sensor_, get_heat_quantity(message, 20));
  publish_if_set(this->version_sensor_, get_u16(message, 26) * 0.01f);
}

void DeltaSolBS2009Sensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 34))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[8]);
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 10));
  publish_if_set(this->pump_speed2_sensor_, message[12]);
  publish_if_set(this->operating_hours2_sensor_, get_u16(message, 18));
  publish_if_set(this->time_sensor_, get_u16(message, 22));
  publish_if_set(this->heat_quantity_sensor_, get_u16(message, 28) + get_u16(message, 30) * 1000.0f);
  publish_if_set(this->version_sensor_, get_u16(message, 32) * 0.01f);
}

void DeltaSolCSensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 24))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[8]);
  publish_if_set(this->pump_speed2_sensor_, message[9]);
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 12));
  publish_if_set(this->operating_hours2_sensor_, get_u16(message, 14));
  publish_if_set(this->heat_quantity_sensor_, get_heat_quantity(message, 16));
  publish_if_set(this->time_sensor_, get_u16(message, 22));
}

void DeltaSolCS2Sensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 30))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[12]);
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 14));
  publish_if_set(this->heat_quantity_sensor_, get_heat_quantity32(message, 24));
  publish_if_set(this->version_sensor_, get_u16(message, 28) * 0.01f);
}

void DeltaSolCS4Sensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 40))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[8]);
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 10));
  publish_if_set(this->pump_speed2_sensor_, message[12]);
  publish_if_set(this->operating_hours2_sensor_, get_u16(message, 14));
  publish_if_set(this->time_sensor_, get_u16(message, 22));
  publish_if_set(this->heat_quantity_sensor_, get_heat_quantity32(message, 28));
  publish_if_set(this->version_sensor_, get_u16(message, 32) * 0.01f);
  publish_if_set(this->temperature5_sensor_, get_i16(message, 36) * 0.1f);
  publish_if_set(this->flow_rate_sensor_, get_u16(message, 38));
}

void DeltaSolBS2Sensor::handle_message(std::vector<uint8_t> &message) {
  if (!message_length_ok(message, 26))
    return;
  publish_if_set(this->temperature1_sensor_, get_i16(message, 0) * 0.1f);
  publish_if_set(this->temperature2_sensor_, get_i16(message, 2) * 0.1f);
  publish_if_set(this->temperature3_sensor_, get_i16(message, 4) * 0.1f);
  publish_if_set(this->temperature4_sensor_, get_i16(message, 6) * 0.1f);
  publish_if_set(this->pump_speed1_sensor_, message[8]);
  publish_if_set(this->pump_speed2_sensor_, message[9]);
  publish_if_set(this->operating_hours1_sensor_, get_u16(message, 12));
  publish_if_set(this->operating_hours2_sensor_, get_u16(message, 14));
  publish_if_set(this->heat_quantity_sensor_, get_heat_quantity(message, 16));
  publish_if_set(this->version_sensor_, get_u16(message, 24) * 0.01f);
}

void VBusCustomSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "VBus Custom Sensor:");
  if (this->source_ == 0xffff) {
    ESP_LOGCONFIG(TAG, "  Source address: ANY");
  } else {
    ESP_LOGCONFIG(TAG, "  Source address: 0x%04x", this->source_);
  }
  if (this->dest_ == 0xffff) {
    ESP_LOGCONFIG(TAG, "  Dest address: ANY");
  } else {
    ESP_LOGCONFIG(TAG, "  Dest address: 0x%04x", this->dest_);
  }
  if (this->command_ == 0xffff) {
    ESP_LOGCONFIG(TAG, "  Command: ANY");
  } else {
    ESP_LOGCONFIG(TAG, "  Command: 0x%04x", this->command_);
  }
  ESP_LOGCONFIG(TAG, "  Sensors:");
  for (VBusCustomSubSensor *sensor : this->sensors_) {
    LOG_SENSOR("  ", "-", sensor);
  }
}

void VBusCustomSensor::handle_message(std::vector<uint8_t> &message) {
  for (VBusCustomSubSensor *sensor : this->sensors_)
    sensor->parse_message(message);
}

void VBusCustomSubSensor::parse_message(std::vector<uint8_t> &message) {
  this->publish_state(this->message_parser_(message));
}

}  // namespace esphome::vbus
