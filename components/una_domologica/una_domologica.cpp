#include "esphome/core/log.h"
#include "una_domologica.h"

namespace esphome {
namespace una_domologica {

static const char *TAG = "una_domologica.switch";

typedef struct {
  uint8_t destination[3];
  uint8_t counter;
} DeviceCounter;

#define MAX_DEVICES 10
static DeviceCounter device_counters[MAX_DEVICES];
static int num_devices = 0;

// Helper function to calculate CRC (adjust as per your protocol)
uint16_t calculate_crc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  uint16_t poly = 0xA001;  // Polynomial for CRC-16
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ poly;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc ^ 0x0000;  // Final XOR value
}

// Function to get the message counter for a device
uint8_t get_counter(const uint8_t *destination) {
  for (int i = 0; i < num_devices; i++) {
    if (memcmp(device_counters[i].destination, destination, 3) == 0) {
      device_counters[i].counter = (device_counters[i].counter + 1) % 256;
      return device_counters[i].counter;
    }
  }
  if (num_devices < MAX_DEVICES) {
    memcpy(device_counters[num_devices].destination, destination, 3);
    device_counters[num_devices].counter = 0;
    num_devices++;
    return 0;
  }
  return 0;  // If max devices reached, return 0
}

// Function to construct a message (adjust as per your protocol)
void construct_message(const uint8_t *destination, const uint8_t *payload, size_t payload_length, uint8_t *message, size_t *message_length) {
  uint8_t counter = get_counter(destination);
  message[0] = 16;  // Start byte
  message[1] = 0;
  message[2] = payload_length + 13;  // Message length
  memcpy(&message[3], destination, 3);  // Destination address
  message[6] = 255;  // Source address (assuming 255)
  message[7] = 0;
  message[8] = 1;  // Packet type
  message[9] = counter;  // Counter
  memcpy(&message[10], payload, payload_length);  // Payload
  uint16_t crc = calculate_crc(message, payload_length + 10);
  message[10 + payload_length] = (crc >> 8) & 0xFF;  // CRC high byte
  message[11 + payload_length] = crc & 0xFF;         // CRC low byte
  message[12 + payload_length] = 2;  // End byte
  *message_length = 13 + payload_length;
}

// Implementation of UnaDomologica class methods

void UnaDomologica::dump_config() {
  ESP_LOGCONFIG(TAG, "Una Domologica:");
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", this->poll_interval_);
  ESP_LOGCONFIG(TAG, "  Poll timeout: %u ms", this->poll_timeout_);
}

void UnaDomologica::register_switch(RelaySwitch *relay_switch, uint8_t relay_number) {
  relay_switches_.push_back({relay_switch, relay_number});
}

void UnaDomologica::setup() {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not set!");
    this->mark_failed();
    return;
  }
}

void UnaDomologica::loop() {
  const uint32_t now = millis();
  if (!poll_in_progress_ && (now - last_poll_ >= poll_interval_)) {
    send_poll_request_();
    poll_in_progress_ = true;
    poll_start_time_ = now;
    last_poll_ = now;
  }
  if (poll_in_progress_ && (now - poll_start_time_ >= poll_timeout_)) {
    poll_in_progress_ = false;
    ESP_LOGW(TAG, "Poll timeout");
  }
  while (uart_parent_->available()) {
    handle_incoming_data_();
  }
}

void UnaDomologica::handle_incoming_data_() {
  while (uart_parent_->available() && receive_index_ < BUFFER_SIZE) {
    uint8_t byte;
    if (!uart_parent_->read_byte(&byte)) {
      break;  // Failed to read byte
    }
    receive_buffer_[receive_index_++] = byte;
    if (parse_message_()) {
      poll_in_progress_ = false;
      receive_index_ = 0;
      return;
    }
  }
  if (receive_index_ >= BUFFER_SIZE) {
    ESP_LOGW(TAG, "Buffer overflow");
    receive_index_ = 0;
    poll_in_progress_ = false;
  }
}

void UnaDomologica::send_poll_request_() {
  uint8_t message[BUFFER_SIZE];
  size_t message_length;
  uint8_t device_address[3] = {0xAF, 0x24, 0x36};  // Replace with your device's address
  uint8_t query_relays_status[] = {0x30};          // Command to query relay status
  construct_message(device_address, query_relays_status, sizeof(query_relays_status), message, &message_length);
  uart_parent_->flush();  // Clear UART buffer
  uart_parent_->write_array(message, message_length);
  uart_parent_->flush();  // Ensure message is sent
}

bool UnaDomologica::parse_message_() {
  if (receive_index_ < 13) return false;  // Message not complete yet

  // Check for end byte (assuming 2 is the end byte)
  if (receive_buffer_[receive_index_ - 1] != 2) return false;

  // Validate start of message
  if (receive_buffer_[0] != 16) {
    ESP_LOGW(TAG, "Invalid start byte");
    receive_index_ = 0;
    return false;
  }

  // Retrieve message length
  size_t message_length = receive_buffer_[2];
  if (receive_index_ < message_length) return false;  // Wait until full message is received

  // Validate CRC
  uint16_t received_crc = (receive_buffer_[message_length - 3] << 8) | receive_buffer_[message_length - 2];
  uint16_t calculated_crc = calculate_crc(receive_buffer_, message_length - 3);
  if (received_crc != calculated_crc) {
    ESP_LOGW(TAG, "Invalid checksum");
    receive_index_ = 0;
    return false;
  }

  // Extract relay states from the message
  // Adjust the index based on your protocol
  // Assuming the state byte is immediately after the header
  uint8_t payload_start = 10;  // After header and before CRC
  size_t payload_length = message_length - payload_start - 3;  // Exclude CRC and end byte
  
  if (payload_length == 0) {
    ESP_LOGW(TAG, "Empty payload");
    receive_index_ = 0;
    return false;
  }
  
  uint8_t *payload = &receive_buffer_[payload_start];

  // Assuming the first byte of the payload is the state byte
  uint8_t current_state = payload[0];

  if (current_state != last_received_state_) {
    last_received_state_ = current_state;
    update_switch_states_(current_state);
  }

  receive_index_ = 0;  // Reset buffer after successful parse
  return true;
}

void UnaDomologica::update_switch_states_(uint8_t state_byte) {
  for (const auto &relay_info : relay_switches_) {
    bool state = (state_byte >> (relay_info.relay_number - 1)) & 0x01;
    relay_info.relay_switch->update_state(state);
  }
}

void UnaDomologica::send_command_(uint8_t relay_number, bool state) {
  uint8_t message[BUFFER_SIZE];
  size_t message_length;
  uint8_t device_address[3] = {0xAF, 0x24, 0x36};  // Replace with your device's address

  // Command to set relay state: 0x31, relay number, state (0x01 for ON, 0x00 for OFF)
  uint8_t set_state[] = {
    0x31,
    relay_number,
    state ? 0x01 : 0x00
  };

  construct_message(device_address, set_state, sizeof(set_state), message, &message_length);

  if (this->uart_parent_ != nullptr) {
    uart_parent_->flush();  // Clear UART buffer
    this->uart_parent_->write_array(message, message_length);
    uart_parent_->flush();  // Ensure message is sent
  }
}

// Implementation of RelaySwitch class methods

void RelaySwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->send_command_(relay_number_, state);
  }
}

void RelaySwitch::update_state(bool state) {
  if (state != this->state) {
    this->publish_state(state);
  }
}

}  // namespace una_domologica
}  // namespace esphome