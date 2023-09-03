#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <string.h>
// #include "esp_camera.h"
// // #include <WiFi.h>
// #include "esp_timer.h"
// #include "img_converters.h"
// #include "Arduino.h"
// #include "fb_gfx.h"
#include "soc/soc.h"           //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
// #include "esp_http_server.h"



#define CAMERA_MODEL_AI_THINKER   // Has PSRAM
#define servo1_pin 13             // servo motor
#define servo2_pin 14
#define loopDelay 5     
#define posChangeMul 1            // the amount of change in each movement
#define pn Serial.print
#define pln Serial.println

#include "esp_camera.h"
#include "camera_pins.h"


//--- Set parameters -------------------------------------------

// --- Servo objects
Servo servo1;
Servo servo2;

// --- Servo value initial value
int servo1Pos = 90;
int servo2Pos = 90;
bool posChanged = false;

// --- WiFi Settings
const char *ssid = "wifi_name";
const char *password = "wifi_password";

// --- MQTT Broker
const char *mqtt_broker = "mqtt.server.secured.domain"; // change to your own domain
const char *send_topic = "esp32/value";
const char *mqtt_username = "esp32client";
const char *mqtt_password = "esp32client";
const int mqtt_port = 8883;

// secure wifi client
WiFiClientSecure espClient;
PubSubClient client(espClient);


void startCameraServer();
void setupLedFlash(int pin);



// set cetificate for TSL connection
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";


//--- util functions -------------------------------------------


void initCamera() {
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      
      config.frame_size = FRAMESIZE_CIF;
      config.jpeg_quality = 11;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_CIF;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

}


void initServo() {

  servo1.attach(servo1_pin);  // attaches the servo on pin to the servo object
  servo2.attach(servo2_pin);  // attaches the servo on pin to the servo object

  servo1.write(servo1Pos);
  servo2.write(servo2Pos);
  delay(loopDelay);
}

// change each servo position regarding current position
void ChangeServoPosRel(int verticalChange, int horizontalChange) {
  int newPos1 = servo1Pos;
  int newPos2 = servo2Pos;

  if (horizontalChange != 0) {
    newPos2 += (posChangeMul * horizontalChange);

    if (newPos2 > 10 & newPos2 < 170) {
      servo2Pos = newPos2;
      pn("\nSetting servo2Pos to ");
      pln(newPos2);

      posChanged = true;

    } else {
      pn("New servo 2 pose ");
      pn(newPos2);
      pln("is out of range (10,170)");
    }
  }


  if (verticalChange != 0) {
    newPos1 += (posChangeMul * verticalChange);

    if (newPos1 > 10 & newPos1 < 170) {
      servo1Pos = newPos1;
      pn("\nSetting servo1Pos to ");
      pln(newPos1);

      posChanged = true;

    } else {
      pn("New servo 1 pose ");
      pn(newPos1);
      pln("is out of range (5,170)");
    }
  }
}

void servoLoop() {
  servo1.write(servo1Pos);
  delay(5);
  servo2.write(servo2Pos);
  delay(3);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    pln("Connecting to WiFi ..");
    delay(2000);
  }
  pn("Connected to the Wi-Fi network. your local IP is");
  pln(WiFi.localIP());
}

void connectMQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    pn("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp32client", mqtt_username, mqtt_password)) {
      pln("connected");

      // Subscribe
      client.subscribe("esp32/servo/change");
      // client.subscribe("esp32/servo/set");


    } else {
      pn("failed, rc=");
      pn(client.state());
      pln(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *message, unsigned int length) {
  pn(F("Message arrived in topic: "));
  pln(topic);
  pn(F("Message -> "));
  String messageTemp;
  int p1 = 0, p2 = 0, delimiter = 0;

  for (int i = 0; i < length; i++) {
    pn((char)message[i]);

    if ((char)message[i] == ',') {
      delimiter = i;  // split index
    }
    messageTemp += (char)message[i];
  }
  pln();
  const char *msgtmp = messageTemp.c_str();
  int first = (messageTemp.substring(0, delimiter)).toInt();
  int second = (messageTemp.substring(delimiter + 1, length)).toInt();
  if (String(topic) == "esp32/servo/change") {  // format "p1,p2" change the current pose with respect to p1 * posChangeMul
    ChangeServoPosRel(first, second);
  }
  // else if (String(topic) == "esp32/servo/set") {  // reset the servo pos to an specific value
  // }
}

//--- main program -----------------------------------------------

void setup() {

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector
  // Set software serial baud to 115200;
  Serial.begin(115200);
  initCamera();
  initWiFi();

  initServo();

  // Set mqtt broker certificate for secured connection
  espClient.setCACert(root_ca);

  // Connecting to the mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  connectMQTT();

  // Start streaming web server
  startCameraServer();

  // Publish and subscribe
  // client.publish(topic, "Hi, I'm ESP32 ^^");
  // client.subscribe(topic);
}

void loop() {

  if (!client.connected()) {
    connectMQTT();
  }

  client.loop();

  if (posChanged) {
    servoLoop();
    posChanged = false;
  }

  delay(loopDelay);

  // client.publish(topic, "Hi, I'm ESP32 ^^");
}
