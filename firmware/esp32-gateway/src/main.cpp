#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Optional second ESP32 UART/ESP-NOW relay. It forwards newline-delimited STM32 records.
// ESP-NOW peer MAC must be configured to the dashboard ESP32 before deploying both nodes.
static constexpr int STM32_RX_PIN=16, STM32_TX_PIN=17;
static constexpr uint32_t BAUD=115200;
static constexpr uint8_t WIFI_CHANNEL=6;
static uint8_t dashboardPeer[6]={0x24,0x6F,0x28,0x00,0x00,0x00}; // replace with dashboard ESP32 STA MAC
HardwareSerial Stm32(2);
struct Packet { uint8_t kind; char data[220]; } __attribute__((packed));
static String line;
static bool espNowReady=false;
static void onSent(const uint8_t*, esp_now_send_status_t) {}
static void onReceive(const uint8_t*,const uint8_t *data,int len){if(len<=0||len>=(int)sizeof(Packet))return;Packet p{};memcpy(&p,data,len);if(p.kind==2){Stm32.print(p.data);Stm32.print('\n');}}
void setup(){Serial.begin(115200);Stm32.begin(BAUD,SERIAL_8N1,STM32_RX_PIN,STM32_TX_PIN);WiFi.mode(WIFI_STA);esp_wifi_set_channel(WIFI_CHANNEL,WIFI_SECOND_CHAN_NONE);if(esp_now_init()==ESP_OK){esp_now_register_send_cb(onSent);esp_now_register_recv_cb(onReceive);esp_now_peer_info_t peer{};memcpy(peer.peer_addr,dashboardPeer,6);peer.channel=WIFI_CHANNEL;peer.encrypt=false;espNowReady=esp_now_add_peer(&peer)==ESP_OK;}}
void loop(){while(Stm32.available()){char c=Stm32.read();if(c=='\n'){if(espNowReady&&line.length()){Packet p{};p.kind=1;line.toCharArray(p.data,sizeof(p.data));esp_now_send(dashboardPeer,(uint8_t*)&p,sizeof(p));}line="";}else if(c!='\r'&&line.length()<219)line+=c;else if(line.length()>=219)line="";}}
