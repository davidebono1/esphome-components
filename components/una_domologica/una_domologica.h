#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include <vector>

namespace esphome {
namespace una_domologica {

class RelaySwitch;  // forward declaration

// Master side of the "una domologica" RS485 bus.
//
// Frame (master -> device):
//   10 00 <len> <dst[3]> FF 00 01 <ctr> <payload...> <crc_hi> <crc_lo> 02
// Device reply:
//   10 00 <len> FF 00 01 <src[3]> <ctr> <payload...> <crc_hi> <crc_lo> 02
//
// len = total frame length (payload_len + 13). CRC-16/MODBUS (init 0xFFFF,
// poly 0xA001), high byte first, over all bytes before the CRC.
class UnaDomologica : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_poll_interval(uint32_t interval) { this->poll_interval_ = interval; }
  void set_device_address(uint32_t address);

  void register_switch(RelaySwitch *relay_switch, uint8_t relay_number);
  void send_command_(uint8_t relay_number, bool state);

 protected:
  static const size_t BUFFER_SIZE = 64;

  uart::UARTComponent *uart_parent_{nullptr};

  uint8_t device_address_[3] = {0xAF, 0x24, 0x36};

  // RX assembly / parsing
  uint8_t receive_buffer_[BUFFER_SIZE];
  size_t receive_index_{0};

  // polling control
  uint32_t last_poll_{0};
  uint32_t poll_interval_{1000};
  bool poll_in_progress_{false};
  uint32_t poll_timeout_{100};
  uint32_t poll_start_time_{0};

  void handle_incoming_data_();
  bool parse_message_();      // true when a complete frame was consumed
  void send_poll_request_();
  bool send_frame_(const uint8_t *payload, size_t payload_length);

  struct RelaySwitchInfo {
    RelaySwitch *relay_switch;
    uint8_t relay_number;
  };
  std::vector<RelaySwitchInfo> relay_switches_;
};

class RelaySwitch : public switch_::Switch {
 public:
  void set_parent(UnaDomologica *parent) { this->parent_ = parent; }
  void set_relay_number(uint8_t relay_number) { this->relay_number_ = relay_number; }
  void write_state(bool state) override;

 protected:
  UnaDomologica *parent_{nullptr};
  uint8_t relay_number_{0};
};

}  // namespace una_domologica
}  // namespace esphome
