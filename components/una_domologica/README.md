# una_domologica

ESPHome master for the "una domologica" RS485 home bus (see the `domologica`
repo for the reverse-engineered protocol: 57600 8N1, CRC-16/MODBUS high byte
first, frames `10 00 <len> <dst[3]> FF 00 01 <ctr> <payload> <crc> 02`).

```yaml
external_components:
  - source: github://davidebono1/esphome-components
    components: [una_domologica]

uart:
  id: uart_rs485
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 57600
  stop_bits: 1

una_domologica:
  id: una_domologica_1
  uart_id: uart_rs485
  # 24-bit device address, default 0xAF2436 (the "Mini Eva" relay board)
  device_address: 0xAF2436
  switches:
    - id: relay1
      name: "Relay 1"
      relay_number: 1
    - id: relay2
      name: "Relay 2"
      relay_number: 2
    - id: relay3
      name: "Relay 3"
      relay_number: 3
    - id: relay4
      name: "Relay 4"
      relay_number: 4
```

Switches are optimistic: they publish what you send, since the status reply
payload (`FF FF 80 00` + `80`x16) is not decoded yet. Every second the
component sends a `30` status probe; valid device replies are logged at DEBUG
with `logger: level: DEBUG`.
