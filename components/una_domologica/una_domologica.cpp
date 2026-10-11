#include "una_domologica.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace una_domologica {

static const char *TAG = "una_domologica";

static const uint8_t FRAME_START0 = 0x10;
static const uint8_t FRAME_START1 = 0x00;
static const uint8_t FRAME_END = 0x02;
static const uint8_t MASTER_ADDR[3] = {0xFF, 0x00, 0x01};
static const uint8_t BROADCAST[3] = {0x00, 0x00, 0x00};

static const uint8_t CMD_STATUS = 0x30;       // status probe, answered by all device types
static const uint8_t CMD_PREDISCOVER = 0xB2;  // pre-discovery broadcast

// "B4 <start3> <end3>" range scans. The first two are the exact ranges the
// real master scanned around the relay board; the last is the whole-space
// frame seen in the captures (Foglio1, CRC-verified).
static const uint8_t DISCOVERY_RANGES[][6] = {
    {0xAF, 0x24, 0x00, 0xAF, 0x24, 0xFF},
    {0xAF, 0x00, 0x00, 0xAF, 0xFF, 0xFF},
    {0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFE},
};
static const size_t NUM_RANGES = sizeof(DISCOVERY_RANGES) / sizeof(DISCOVERY_RANGES[0]);

// per-destination sequence counter table (the device echoes it back)
#define MAX_DEVICES 10
static uint8_t device_counters_addr[MAX_DEVICES][3];
static uint8_t device_counters_val[MAX_DEVICES];
static int num_device_counters = 0;

static void reset_counters() { num_device_counters = 0; }

static uint8_t get_counter(const uint8_t *destination) {
  for (int i = 0; i < num_device_counters; i++) {
    if (memcmp(device_counters_addr[i], destination, 3) == 0) {
      device_counters_val[i] = (device_counters_val[i] + 1) % 256;
      return device_counters_val[i];
    }
  }
  if (num_device_counters < MAX_DEVICES) {
    memcpy(device_counters_addr[num_device_counters], destination, 3);
    device_counters_val[num_device_counters] = 0;
    num_device_counters++;
  }
  return 0;
}

static uint16_t calculate_crc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// Build "10 00 <len> <dst[3]> FF 00 01 <ctr> <payload> <crc_hi> <crc_lo> 02".
// Returns false if it does not fit in message_size.
static bool construct_message(const uint8_t *destination, const uint8_t *payload, size_t payload_length,
                              uint8_t ctr_override, uint8_t *message, size_t message_size,
                              size_t *message_length) {
  if (payload_length + 13 > message_size)
    return false;
  uint8_t counter = (ctr_override == 0xFF) ? get_counter(destination) : ctr_override;
  message[0] = FRAME_START0;
  message[1] = FRAME_START1;
  message[2] = payload_length + 13;
  memcpy(&message[3], destination, 3);
  memcpy(&message[6], MASTER_ADDR, 3);
  message[9] = counter;
  memcpy(&message[10], payload, payload_length);
  uint16_t crc = calculate_crc(message, payload_length + 10);
  message[10 + payload_length] = (crc >> 8) & 0xFF;  // CRC high byte first
  message[11 + payload_length] = crc & 0xFF;
  message[12 + payload_length] = FRAME_END;
  *message_length = 13 + payload_length;
  return true;
}

void UnaDomologica::set_device_address(uint32_t address) {
  this->device_address_[0] = (address >> 16) & 0xFF;
  this->device_address_[1] = (address >> 8) & 0xFF;
  this->device_address_[2] = address & 0xFF;
}

void UnaDomologica::dump_config() {
  ESP_LOGCONFIG(TAG, "Una Domologica:");
  ESP_LOGCONFIG(TAG, "  Device address: %02X:%02X:%02X", this->device_address_[0], this->device_address_[1],
                this->device_address_[2]);
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", this->poll_interval_);
  ESP_LOGCONFIG(TAG, "  Poll timeout: %u ms", this->poll_timeout_);
}

void UnaDomologica::register_switch(RelaySwitch *relay_switch, uint8_t relay_number) {
  this->relay_switches_.push_back({relay_switch, relay_number});
}

void UnaDomologica::setup() {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not set!");
    this->mark_failed();
    return;
  }
  this->start_bringup_();
}

void UnaDomologica::start_bringup_() {
  reset_counters();
  this->phase_ = Phase::ColdProbe;
  this->range_index_ = 0;
  this->discovered_ = false;
  this->misses_ = 0;
  this->poll_in_progress_ = false;
  this->last_reply_valid_ = false;
}

void UnaDomologica::start_polling_() {
  this->phase_ = Phase::Polling;
  this->poll_in_progress_ = false;
  this->last_poll_ = millis() - this->poll_interval_;  // poll right away
  this->misses_ = 0;
}

void UnaDomologica::loop() {
  if (this->uart_parent_ == nullptr)
    return;
  this->handle_incoming_data_();
  const uint32_t now = millis();

  switch (this->phase_) {
    case Phase::ColdProbe:
      // first frame after boot: bare "30" at ctr 0 - exactly the frame Mini
      // Eva was seen answering, with no discovery in front of it
      this->send_frame_(this->device_address_, &CMD_STATUS, 1, 0x00);
      this->last_reply_valid_ = false;
      this->phase_ = Phase::WaitCold;
      this->phase_start_ = now;
      break;

    case Phase::WaitCold:
      if (this->last_reply_valid_ && memcmp(this->last_src_, this->device_address_, 3) == 0) {
        ESP_LOGI(TAG, "Device answered cold probe at %02X:%02X:%02X", this->device_address_[0],
                 this->device_address_[1], this->device_address_[2]);
        this->start_polling_();
      } else if (now - this->phase_start_ >= COLD_WAIT) {
        ESP_LOGW(TAG, "No answer from %02X:%02X:%02X, starting discovery", this->device_address_[0],
                 this->device_address_[1], this->device_address_[2]);
        this->phase_ = Phase::PreDisc;
      }
      break;

    case Phase::PreDisc:
      // B2 broadcast, ctr 0, sent once (observed in sniff_discovery.csv row 50)
      this->send_frame_(BROADCAST, &CMD_PREDISCOVER, 1, 0x00);
      this->last_reply_valid_ = false;
      this->phase_ = Phase::WaitPreDisc;
      this->phase_start_ = now;
      break;

    case Phase::WaitPreDisc:
      if (now - this->phase_start_ >= PREDISC_WAIT)
        this->phase_ = Phase::RangeScan;
      break;

    case Phase::RangeScan: {
      const uint8_t *r = DISCOVERY_RANGES[this->range_index_];
      uint8_t payload[7] = {0xB4, r[0], r[1], r[2], r[3], r[4], r[5]};
      ESP_LOGI(TAG, "Discovery scan %02X:%02X:%02X..%02X:%02X:%02X", r[0], r[1], r[2], r[3], r[4], r[5]);
      this->send_frame_(BROADCAST, payload, sizeof(payload), 0x00);
      this->last_reply_valid_ = false;
      this->phase_ = Phase::WaitRange;
      this->phase_start_ = now;
      break;
    }

    case Phase::WaitRange:
      if (this->last_reply_valid_) {
        this->last_reply_valid_ = false;
        if (this->last_payload_len_ >= 3 && memcmp(this->last_payload_, this->last_src_, 3) == 0) {
          // discovery reply: own address + 2 type bytes + 1 flag byte
          ESP_LOGI(TAG, "Discovered device %02X:%02X:%02X type %02X%02X flag %02X", this->last_src_[0],
                   this->last_src_[1], this->last_src_[2], this->last_payload_len_ > 4 ? this->last_payload_[3] : 0,
                   this->last_payload_len_ > 5 ? this->last_payload_[4] : 0,
                   this->last_payload_len_ > 6 ? this->last_payload_[5] : 0);
          if (!this->discovered_) {
            memcpy(this->device_address_, this->last_src_, 3);
            this->discovered_ = true;
            ESP_LOGI(TAG, "Adopting address %02X:%02X:%02X", this->device_address_[0], this->device_address_[1],
                     this->device_address_[2]);
          }
        }
      }
      if (now - this->phase_start_ >= RANGE_WAIT) {
        this->range_index_++;
        if (this->range_index_ < NUM_RANGES) {
          this->phase_ = Phase::RangeScan;
        } else {
          if (!this->discovered_)
            ESP_LOGW(TAG, "Discovery found nothing; polling %02X:%02X:%02X anyway", this->device_address_[0],
                     this->device_address_[1], this->device_address_[2]);
          this->start_polling_();
        }
      }
      break;

    case Phase::Polling:
      if (!this->poll_in_progress_ && (now - this->last_poll_ >= this->poll_interval_)) {
        this->send_frame_(this->device_address_, &CMD_STATUS, 1, 0xFF);  // auto counter
        this->poll_in_progress_ = true;
        this->poll_start_time_ = now;
        this->last_poll_ = now;
      }
      if (this->last_reply_valid_) {
        this->last_reply_valid_ = false;
        this->misses_ = 0;
        this->poll_in_progress_ = false;
      }
      if (this->poll_in_progress_ && (now - this->poll_start_time_ >= this->poll_timeout_)) {
        this->poll_in_progress_ = false;
        this->misses_++;
        if (this->misses_ % 20 == 0)
          ESP_LOGW(TAG, "%u consecutive polls without reply", (unsigned) this->misses_);
        if (this->misses_ >= MISSES_BEFORE_REDISCOVER) {
          ESP_LOGW(TAG, "Device silent for too long, re-running bring-up");
          this->start_bringup_();
        }
      }
      break;
  }
}

// Feed UART bytes into the assembly buffer and try to close frames off it.
void UnaDomologica::handle_incoming_data_() {
  uint8_t byte;
  while (this->receive_index_ < BUFFER_SIZE && this->uart_parent_->read_byte(&byte)) {
    this->receive_buffer_[this->receive_index_++] = byte;
    this->parse_message_();
  }
  if (this->receive_index_ >= BUFFER_SIZE) {
    ESP_LOGW(TAG, "Buffer overflow, resyncing");
    this->receive_index_ = 0;
    this->poll_in_progress_ = false;
  }
}

// Attempt to parse a frame out of the front of the receive buffer.
// Returns true once a complete (not necessarily valid) frame is consumed.
bool UnaDomologica::parse_message_() {
  if (this->receive_index_ < 13)
    return false;  // shortest possible frame

  // not a frame start yet: drop one byte and resync
  if (this->receive_buffer_[0] != FRAME_START0 || this->receive_buffer_[1] != FRAME_START1) {
    this->receive_index_--;
    memmove(this->receive_buffer_, this->receive_buffer_ + 1, this->receive_index_);
    return false;
  }

  size_t frame_length = this->receive_buffer_[2];
  if (frame_length < 14 || frame_length > BUFFER_SIZE) {
    // bogus length byte: drop the first byte and resync
    this->receive_index_--;
    memmove(this->receive_buffer_, this->receive_buffer_ + 1, this->receive_index_);
    return false;
  }
  if (this->receive_index_ < frame_length)
    return false;  // wait for the full frame

  // a complete length-delimited frame is in front of the buffer: consume it
  // either way, valid or not
  const uint8_t *f = this->receive_buffer_;
  uint16_t crc_calc = calculate_crc(f, frame_length - 3);
  bool crc_ok = f[frame_length - 1] == FRAME_END && f[frame_length - 3] == (crc_calc >> 8) &&
                f[frame_length - 2] == (crc_calc & 0xFF);
  // device -> master replies carry FF 00 01 (us) in bytes 3..5 and the device
  // address in bytes 6..8. Our own TX echoes carry the device address there
  // and are rejected here, even though their CRC is valid.
  bool dir_ok = f[3] == MASTER_ADDR[0] && f[4] == MASTER_ADDR[1] && f[5] == MASTER_ADDR[2];

  size_t payload_length = frame_length - 13;
  if (crc_ok && dir_ok && payload_length > 0) {
    const uint8_t *payload = &f[10];
    ESP_LOGD(TAG, "Reply from %02X:%02X:%02X ctr=%02X: %s", f[6], f[7], f[8], f[9],
             format_hex_pretty(payload, payload_length).c_str());
    if (payload_length <= MAX_PAYLOAD) {
      memcpy(this->last_src_, &f[6], 3);
      memcpy(this->last_payload_, payload, payload_length);
      this->last_payload_len_ = payload_length;
      this->last_reply_valid_ = true;
    }
    this->poll_in_progress_ = false;
  } else if (!crc_ok) {
    ESP_LOGW(TAG, "Invalid checksum");
  } else if (!dir_ok) {
    ESP_LOGD(TAG, "Ignoring non-reply frame (own echo or foreign traffic)");
  }

  size_t consumed = frame_length;
  this->receive_index_ -= consumed;
  memmove(this->receive_buffer_, this->receive_buffer_ + consumed, this->receive_index_);
  return true;
}

bool UnaDomologica::send_frame_(const uint8_t *destination, const uint8_t *payload, size_t payload_length,
                                uint8_t ctr_override) {
  uint8_t message[BUFFER_SIZE];
  size_t message_length;
  if (!construct_message(destination, payload, payload_length, ctr_override, message, sizeof(message),
                         &message_length))
    return false;
  this->uart_parent_->write_array(message, message_length);
  this->uart_parent_->flush();  // wait until the frame is fully on the wire
  return true;
}

void UnaDomologica::send_command_(uint8_t relay_number, bool state) {
  // set relay state
  uint8_t set_state[3] = {0x31, relay_number, static_cast<uint8_t>(state ? 0x01 : 0x00)};
  this->send_frame_(this->device_address_, set_state, sizeof(set_state), 0xFF);
}

void RelaySwitch::write_state(bool state) {
  // optimistic: publish what we asked for; replies are not decoded yet
  this->publish_state(state);
  if (this->parent_ != nullptr)
    this->parent_->send_command_(this->relay_number_, state);
}

}  // namespace una_domologica
}  // namespace esphome
