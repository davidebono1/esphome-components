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
//
// Bring-up (mirrors what the proprietary master does at boot, see the
// domologica repo sniff_discovery.csv):
//   bare "30" status probe to the configured address at ctr 0
//   -> if silent: "B2" broadcast, then "B4 <start3> <end3>" range scans;
//      every device inside a range answers with its own address + 2 type
//      bytes + 1 flag byte. The first device found is adopted.
//   -> steady "30" polling; after many consecutive misses, bring-up reruns.
// We deliberately never send "B3" (claim): unclaimed devices keep answering
// later scans, which keeps re-discovery possible.
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
  static const size_t MAX_PAYLOAD = 48;

  // bring-up timings (ms), taken from the bench firmware that worked
  static const uint32_t COLD_WAIT = 400;
  static const uint32_t PREDISC_WAIT = 200;
  static const uint32_t RANGE_WAIT = 600;
  static const uint32_t MISSES_BEFORE_REDISCOVER = 60;
  static const uint32_t BOOT_DELAY = 5000;  // let WiFi/API come up so bring-up logs are visible

  uart::UARTComponent *uart_parent_{nullptr};

  uint8_t device_address_[3] = {0xAF, 0x24, 0x36};

  // bring-up state machine
  enum class Phase : uint8_t { ColdProbe, WaitCold, PreDisc, WaitPreDisc, RangeScan, WaitRange, Polling };
  Phase phase_{Phase::ColdProbe};
  uint32_t phase_start_{0};
  size_t range_index_{0};
  bool discovered_{false};
  uint32_t misses_{0};

  // last valid device reply
  bool last_reply_valid_{false};
  uint8_t last_src_[3] = {0, 0, 0};
  uint8_t last_payload_[MAX_PAYLOAD];
  size_t last_payload_len_{0};

  // polling control
  bool poll_in_progress_{false};
  uint32_t poll_start_time_{0};
  uint32_t last_poll_{0};
  uint32_t poll_interval_{1000};
  uint32_t poll_timeout_{100};

  // RX assembly / parsing
  uint8_t receive_buffer_[BUFFER_SIZE];
  size_t receive_index_{0};

  void handle_incoming_data_();
  bool parse_message_();  // true once a complete (not necessarily valid) frame is consumed
  bool send_frame_(const uint8_t *destination, const uint8_t *payload, size_t payload_length, uint8_t ctr_override);
  void start_bringup_();
  void start_polling_();

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
