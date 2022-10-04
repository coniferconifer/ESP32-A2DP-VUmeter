/*
  ESP32-A2DP-VUmeter.ino
  Streaming Music from Bluetooth with VU meter for tera terminal
  based on ESP32-A2DP by Phil Schatzmann
  at https://github.com/pschatzmann/ESP32-A2DP

coniferconifer added simple VU meter for tera term https://ttssh2.osdn.jp/
I have no idea why the serial monitor by Arduino IDE does not provide realtime VUmeter, just 
by printing VU bar and CR at the end.
 
See #define VUMETER and #ifdef VUMETER section.
Max volume for BT audio output is the best for ESP32-A2DP.
Partition Scheme: Huge APP(3MB NO OTA ,1MB SPIFF)  is recomended to build the sketch.

Output example 1kHz 0dB signal
Ex1. 0dB 1kHz input 

VU meter by ESP32-A2DP
Device: MyMusic

a2dp audio_cfg_cb , codec type 0
configure audio player 21-15-2-35
audio player configured, samplerate=44100

Left ******************************* Right *******************************

-5dB 1kHz input
Left ******************............. Right ******************.............

-10dB 1kHz input
Left **********..................... Right **********.....................


PCM5102A board is connected by
GPIO23 -> DIN
GPIO25 -> LCK
GPIO26 -> BCK
5V -> VIN (inout to the on board 3.3V regulator)
GND -> GND
On board switch settings: H1L（FLT),H2L（DEMP),H4L（FMT) are shorted to GND
H3L（XMST un mute ) is connected to High side(3.3V)

When ESP32 devkitC is connected to PC USB port, audio output is slightly noisy.
USB power from battery is ideal to remove noise, but I don't care noise when 
digital volume at PC or smartphone (Bluetooth source) is max.

copyright 2022 by coniferconifer
LICENSED under apache
*/
#include "BluetoothA2DPSink.h"

i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = 44100,
  .bits_per_sample = (i2s_bits_per_sample_t)16,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = 0,  // default interrupt priority
  .dma_buf_count = 8,
  .dma_buf_len = 64,
  .use_apll = false
};
i2s_pin_config_t pin_config = {
  .bck_io_num = 26,
  .ws_io_num = 25,
  .data_out_num = 23,  //original is 22 , but 22 should be used for I2C
  .data_in_num = I2S_PIN_NO_CHANGE
};
BluetoothA2DPSink a2dp_sink;
#define VUMETER
#define INTERVAL 100 //VUmeter update interval
#ifdef VUMETER
bool isFirst = true;
long elapsed = 0;
void data_stream_reader_callback(const uint8_t *data, uint32_t len) {
  //Serial.printf("Data packet received %d\r\n", len);
  int16_t minRight = 0;
  int16_t maxRight = 0;
  int16_t minLeft = 0;
  int16_t maxLeft = 0;
  if (millis() - elapsed < INTERVAL) {
    return;
  }
  for (int i = 0; i < len;) {
    int16_t rightData = data[i] | data[i + 1] << 8;
    int16_t leftData = data[i + 2] | data[i + 3] << 8;

  //  if (minRight < rightData) minRight = rightData;
    if (maxRight > rightData) maxRight = rightData;
  //  if (minLeft < leftData) minLeft = leftData;
    if (maxLeft > leftData) maxLeft = leftData;
    i = i + 4;
  }
  if (maxRight < 0) maxRight = -1 * maxRight;
  if (maxLeft < 0) maxLeft = -1 * maxLeft;
#define SHIFTSIZE 10
  if (isFirst == true) {
    Serial.println();
    isFirst = false;
  }
  Serial.print("Left ");
  printVUmeter(maxRight >> SHIFTSIZE);  //devide by 1024, reducing max 32768 to 32
  Serial.print(" Right ");
  printVUmeter(maxLeft >> SHIFTSIZE);  //devide by 1024, reducing max 32768 to 32
  Serial.printf("\r");

  elapsed = millis();
}
void printVUmeter(uint8_t val) {
#define BARLENGTH (0x7fff >> SHIFTSIZE)  // should be less than 32
  int i = 0;
  char bar[BARLENGTH + 1];
  for (i = 0; i < BARLENGTH; i++) {
    if (i < val) {
      bar[i] = '*';
    } else {
      bar[i] = '.';
    }
  }
  bar[i] = 0x00;

  Serial.printf("%s", bar);  //for teraterm serial monitor
}
#endif
void setup() {
  Serial.begin(115200);
  a2dp_sink.set_pin_config(pin_config);  //change default I2S pin assingment
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("MyMusic");
#ifdef VUMETER
  Serial.printf("\r\nVU meter by ESP32-A2DP\r\n");
  Serial.printf("Device: MyMusic\r\n");
  a2dp_sink.set_stream_reader(data_stream_reader_callback);
  Serial.println();
#endif
}
void loop() {
}
