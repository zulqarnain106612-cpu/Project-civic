# Civic CAN ESP32 dashboard

- Connect Android to Wi-Fi `Civic-CAN-Monitor` (password `CANmonitor26`) and open `http://192.168.4.1/`.
- Dashboard ESP32 UART2 default pins: GPIO16 RX from STM32 USART2 TX (PA2/ST-LINK VCP UART output), GPIO17 TX to STM32 USART2 RX (PA3) through compatible logic levels.
- Build: `pio run -d esp32-dashboard`.
- The dashboard shows bitrate scan/lock status, received-frame count, last CAN ID, last-frame age, raw frames on demand (up to 250 in the browser), and decoded OBD-II Mode 01 PIDs 04, 05, 0C, 0D, 0F, 11, 2F when valid responses arrive.
- Read mode is passive: it does not request OBD-II PIDs or transmit CAN frames. Therefore vehicle values appear only if matching response frames are already present on the bus.
- Write mode is included as a clearly locked preview. The STM32 firmware must keep `CAN_TX_CONFIRMED=0` until the physical transceiver wiring and receive path have been verified; no write is transmitted by this dashboard build.
- The UART CSV parser follows `timestamp,format,id,dlc,rtr,payload_hex` from `can_capture_frame_to_csv()` and normalizes `0x7E8` IDs for OBD decoding.
- Software build success does not confirm UART/CAN wiring or live vehicle data; those are reserved for the later practical test.
