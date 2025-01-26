#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include <vector>

namespace esphome {
namespace una_domologica {

class RelaySwitch; // Forward declaration

class UnaDomologica : public Component {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // UART configuration
  void set_uart_parent(uart::UARTComponent *parent) { uart_parent_ = parent; }
  void set_poll_interval(uint32_t interval) { poll_interval_ = interval; }
  void set_device_address(uint8_t address) { device_address_ = address; }

  // Register a RelaySwitch
  void register_switch(RelaySwitch *relay_switch, uint8_t relay_number);

  // Send command to set relay state
  void send_command_(uint8_t relay_number, bool state);

protected:
  // UART communication
  uart::UARTComponent *uart_parent_{nullptr};

  // Protocol specific members
  uint8_t device_address_{0x01};
  static const uint8_t BUFFER_SIZE = 64;
  uint8_t receive_buffer_[BUFFER_SIZE];
  size_t receive_index_{0};

  // Polling control
  uint32_t last_poll_{0};
  uint32_t poll_interval_{1000}; // Default poll interval in milliseconds
  bool poll_in_progress_{false};
  uint32_t poll_timeout_{100}; // Timeout for poll response in milliseconds
  uint32_t poll_start_time_{0}; // When the last poll was initiated

  // Protocol handling methods
  void handle_incoming_data_();
  bool parse_message_();
  void send_poll_request_();

  // State tracking
  uint8_t last_received_state_{0xFF}; // Initialize to an invalid state

  // Store registered RelaySwitches
  struct RelaySwitchInfo {
    RelaySwitch *relay_switch;
    uint8_t relay_number;
  };
  std::vector<RelaySwitchInfo> relay_switches_;

  // Update the state of the registered switches
  void update_switch_states_(uint8_t state_byte);
};

class RelaySwitch : public switch_::Switch {
public:
  void set_parent(UnaDomologica *parent) { parent_ = parent; }
  void set_relay_number(uint8_t relay_number) { relay_number_ = relay_number; }
  void write_state(bool state) override;

  // Called by parent to update state
  void update_state(bool state);

protected:
  UnaDomologica *parent_{nullptr};
  uint8_t relay_number_{0};
};

} // namespace una_domologica
} // namespace esphome