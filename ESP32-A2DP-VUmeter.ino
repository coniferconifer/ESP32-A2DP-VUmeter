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
bool is_active = true;
#define VUMETER
#ifdef VUMETER
#define INTERVAL 50  //VUmeter update interval

#define SHIFTSIZE 10
#define L_PIN 2
#define L_PWMCH 1

#define R_PIN 4
#define R_PWMCH 0
#define LED_OFFSET 30
esp_a2d_audio_state_t last_state;
// for esp_a2d_connection_state_t see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_a2dp.html#_CPPv426esp_a2d_connection_state_t
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  // Serial.printf("connection_state:%s\r\n", a2dp_sink.to_str(state));
}

// for esp_a2d_audio_state_t see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_a2dp.html#_CPPv421esp_a2d_audio_state_t
void audio_state_changed(esp_a2d_audio_state_t state, void *ptr) {
  //  Serial.printf("audio_state:%s\r\n", a2dp_sink.to_str(state));
}

// https://www.firstpr.com.au/dsp/pink-noise/#Filtering
// under R&D
double pinkNoise(double max) {
  static double b0, b1, b2, b3, b4, b5, b6;
  double white = (double)esp_random() / 4294967296.0;
  float rate = 0.2;
  b0 = (1.0-rate) * b0 + white * rate;  //low pass filter
  //  b0 = 0.99765 * b0 + white * 0.0990460;
  //  b1 = 0.96300 * b1 + white * 0.2965164;
  //  b2 = 0.57000 * b2 + white * 1.0526913;
  double pink = b0 * max;
  // Serial.println(pink);
  /*   b0 = 0.99886 * b0 + white * 0.0555179;
    b1 = 0.99332 * b1 + white * 0.0750759;
    b2 = 0.96900 * b2 + white * 0.1538520;
    b3 = 0.86650 * b3 + white * 0.3104856;
    b4 = 0.55000 * b4 + white * 0.5329522;
    b5 = -0.7616 * b5 - white * 0.0168980;
    double pink = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362) * max / 6.0;
    b6 = white * 0.115926;
    */
  return pink;
}

uint32_t counter = 0;
// uint8_t max limitter with input*multiplier+offset
uint8_t limit(uint8_t input, uint8_t multiplier, uint8_t offset) {
  uint16_t result = input * multiplier + offset;
  if (result > UINT8_MAX) {
    return UINT8_MAX;
  }
  return static_cast<uint8_t>(result);
}
/// callback which is notified on update
void rssi(esp_bt_gap_cb_param_t::read_rssi_delta_param  &rssiParam){
  Serial.print("rssi value: ");
  Serial.println(rssiParam.rssi_delta);
}


bool isFirst = true;
long elapsed = 0;
long elapsed_offset=0;
uint8_t val_L=0;
uint8_t val_R=0;
void data_stream_reader_callback(const uint8_t *data, uint32_t len) {
  //Serial.printf("Data packet received %d\r\n", len);
  //  int16_t minRight = 0;
  // counter++;
  int16_t maxRight = 0;
  //  int16_t minLeft = 0;
  int16_t maxLeft = 0;
  if (millis() - elapsed < INTERVAL) {
    return;
  }
  for (int i = 0; i < len;) {
    int16_t leftData = data[i] | data[i + 1] << 8;
    int16_t rightData = data[i + 2] | data[i + 3] << 8;

    //  if (minRight < rightData) minRight = rightData;
    if (maxRight < rightData) maxRight = rightData;
    //  if (minLeft < leftData) minLeft = leftData;
    if (maxLeft < leftData) maxLeft = leftData;
    i = i + 4;
  }
  if (maxRight < 0) maxRight = -1 * maxRight;
  if (maxLeft < 0) maxLeft = -1 * maxLeft;

  if (isFirst == true) {
    Serial.println();
    isFirst = false;
  }

  uint8_t val;
  uint8_t led_offset;
  //  led_offset=(uint8_t)(random(0,LED_OFFSET));
  //  Serial.printf("\r\n%d ",led_offset);

  led_offset = (uint32_t)pinkNoise(LED_OFFSET);
  // Serial.printf("\r\n%d ", led_offset);
  Serial.printf("\r%08d ", elapsed-elapsed_offset);

  Serial.print("L ");
  val = maxLeft >> SHIFTSIZE;

  float rate = 0.3; // to simulate 300msec response of VU meter
  val_L = (uint8_t)((1.0-rate) * (float)val_L + (float)val * rate);
  printVUmeter(val_L);  //devide by 1024, reducing max 32768 to 32
  uint8_t led_L;
  led_L = limit(val_L, 8, led_offset);  //Red LED
  ledcWrite(L_PWMCH, led_L);          //VU LED at GPIO PIN

  Serial.print(" R ");
  val = maxRight >> SHIFTSIZE;
  val_R = (uint8_t)((1.0-rate) * (float)val_R + (float)val * rate);
  printVUmeter(val_R);  //devide by 1024, reducing max 32768 to 32
  uint8_t led_R;
  led_R = limit(val_R, 14, led_offset);  //yellow LED
  ledcWrite(R_PWMCH, led_R);           //VU LED at GPIO PIN
  Serial.printf("\r");

  elapsed = millis();
}

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  uint32_t value;
  //Serial.printf("\r\nAVRC metadata rsp: id 0x%x, %s\r\n", data1, data2);
  switch (data1) {
    case 0x01:
      Serial.printf("\r\n %s\r\n", data2);
      elapsed_offset=millis();
      break;
    case 0x40:
      char *endptr;
      value = strtoul((char *)data2, &endptr, 10);
      Serial.printf("duration(sec):%d\r\n", value / 1000);
      break;

    default:

      break;
  }
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
  a2dp_sink.set_rssi_active(true);
  a2dp_sink.set_rssi_callback(rssi);
#ifdef VUMETER
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.set_on_audio_state_changed(audio_state_changed);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_PLAYING_TIME);
  a2dp_sink.set_stream_reader(data_stream_reader_callback);

  a2dp_sink.start("MyMusic");

  Serial.printf("\r\nVU meter by ESP32-A2DP\r\n");
  Serial.printf("Device: MyMusic\r\n");
  Serial.printf("VU Bar length = %d\r\n", BARLENGTH);
  // setup LED VU meter at GPIO PIN
  pinMode(R_PIN, OUTPUT);
  ledcSetup(R_PWMCH, 10000, 8);  //PWM at 10kHz
  ledcAttachPin(R_PIN, R_PWMCH);

  pinMode(L_PIN, OUTPUT);
  ledcSetup(L_PWMCH, 10000, 8);  //PWM at 10kHz
  ledcAttachPin(L_PIN, L_PWMCH);

  Serial.printf("VU LED at GPIO=%d,%d\r\n", L_PIN, R_PIN);
  Serial.print(" L ");
  printVUmeter(BARLENGTH);  //devide by 1024, reducing max 32768 to 32
  Serial.print(" R ");
  printVUmeter(BARLENGTH);  //devide by 1024, reducing max 32768 to 32
  Serial.printf("\r\n");
  Serial.println();
#endif
}
void loop() {
  uint8_t val;
  bool is_started;

  // check state
  // esp_a2d_connection_state_t state = a2dp_sink.get_connection_state();
  esp_a2d_audio_state_t state = a2dp_sink.get_audio_state();
  is_started = state == ESP_A2D_AUDIO_STATE_STARTED;
  if (last_state != state) {
    //  Serial.println(is_started ? "started" : "suspended");
    last_state = state;
  }
  if (state == ESP_A2D_AUDIO_STATE_STARTED) {
    //  Serial.println("playing");
    delay(1000);  //get audio state slowly
  } else {
#define STANDBY_LED
#ifdef STANDBY_LED  // When audio play is suspended, the LED flickers due to random noise
    val = (uint8_t)pinkNoise(LED_OFFSET);
    ledcWrite(R_PWMCH, val);
    delay(INTERVAL);
#endif
  }
}
