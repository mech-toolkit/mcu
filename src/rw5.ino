
/**********************************************************************
  Filename    : Mech Warrior - v1.1
  Description : Built to run on an ESP32-CAM module to control servos. Connects to a wifi
                newtwork, streams video and waits for commands.
  Authors     : Ken, Ric
  Created     : 20221231
  Version     : 2.0.1 - 20230216
  ||||
  Notes:
  On first setup of the MECH scan your Wifi network to discover the ESP WiFi Manager.
  The SSID for the network is "MECH_AP" This service will allow you to select your
  local Wifi network and add the MECH to it plus configure some paramters
**********************************************************************/
//
//
//
#include <Arduino.h>
#include "version.h"
#define ESP_DRD_USE_SPIFFS true
#include <FS.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
//
#include "esp_camera.h"  // https://github.com/espressif/esp32-camera/blob/master/driver/include/esp_camera.h
#include <Servo.h>       // https://github.com/RoboticsBrno/ServoESP32
#include <WiFiManager.h> // WiFiManager by tzapu - https://github.com/tzapu/WiFiManager
#include <NewPing.h>     // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ESP_DoubleResetDetector.h> // https://github.com/khoih-prog/ESP_DoubleResetDetector
//
#define DRD_TIMEOUT 3                   // Number of seconds after reset that will be considered a double reset.
#define DRD_ADDRESS 0                   // RTC Memory Address for the DoubleResetDetector to use
#define JSON_CONFIG_FILE "/config.json" // JSON configuration file
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
// #define CAMERA_MODEL_WROVER_KIT // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//
#define LED_BUILTIN 33 // ESP32-CAM AI Thinker module it is GPIO_33 for the RED LED and PIN GPIO_4 for the White Flash LED
#include "camera_pins.h"
#define SONAR_NUM 1     // Number of sensors.
#define MAX_DISTANCE 20 // Maximum distance (in cm) to ping.
//
NewPing sonar[SONAR_NUM] = {
    // Sensor object array.
    NewPing(14, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
};
//
DoubleResetDetector *drd;
bool delayStop = false; // delayStop STOPS robot after each motion command.
int delayStopVal = 200; // this is the time to run motion command for before stopping
//
//
String rfcWelcome = "RFC";     // The RFC server message broadcasted to see who wants to fight
String rfcAcknowledge = "ack"; // This is the string the RFC server is wanting to see once a command has been processed
WiFiUDP udp;
IPAddress broadcastIP(255, 255, 255, 255);
IPAddress senderIP;
char packetBuffer[255];
bool udpIsSetup = false;
//
// ===========================
// WiFi credentials
// ===========================
// You do not need to fill in the wifi details as this code has WifiManager library running and handles the connection
const char *ssid = "####";      // input your wifi name
const char *password = "#####"; // input your wifi password

TaskHandle_t punch_handle = NULL;

// Servo setup
Servo punchServo;
Servo leftServo;
Servo rightServo;
int leftServoVel, rightServoVel, actionType;
char dataReceived[20];
static const int servoPunchPin = 13;
static const int servoLeftPin = 12;
static const int servoRightPin = 15;
int punchHome = 10;
int punchHit = 90;
int leftStop = 90;
int rightStop = 90;

// Variables to hold data from custom textboxes
char roboName[100] = "MECH_default"; // MECH name
char portNumber[6] = "6969";         // Port the MECH server is broadcasting on
char leftServoOffset[4] = "0";       // Offset for left servo centre position
char rightServoOffset[4] = "0";      // Offset for Right servo centre position
char action[3] = "0";                // 0 = Punch (90deg servo), 1 = Snap Punch (360deg servo)
bool shouldSaveConfig = false;       // Controlled by saveConfigCallback

void punch(void *parameters)
{
  for (;;)
  {
    vTaskSuspend(NULL);
    punchServo.write(punchHit);
    vTaskDelay(620 / portTICK_PERIOD_MS);
    punchServo.write(punchHome);
  }
}

void saveConfigFile()
// Save Config in JSON format
{
  Serial.println(F("Saving configuration..."));
  // Create a JSON document
  StaticJsonDocument<512> json;
  json["roboName"] = roboName;
  json["portNumber"] = portNumber;
  json["leftServoOffset"] = leftServoOffset;
  json["rightServoOffset"] = rightServoOffset;
  json["action"] = action;
  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }
  // Serialize JSON data to write to file
  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0)
  {
    // Error writing file
    Serial.println(F("Failed to write to file"));
  }
  configFile.close();
}

bool loadConfigFile()
// Load existing configuration file
{
  // Uncomment if we need to format filesystem
  // SPIFFS.format();
  //
  // Read configuration from FS json
  Serial.println("Mounting File System...");
  // May need to make it begin(true) first time you are using SPIFFS
  if (SPIFFS.begin(false) || SPIFFS.begin(true))
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE))
    {
      // The file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        Serial.println("Opened configuration file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error)
        {
          Serial.println("Parsing JSON");
          strcpy(roboName, json["roboName"]);
          strcpy(portNumber, json["portNumber"]);
          strcpy(leftServoOffset, json["leftServoOffset"]);
          strcpy(rightServoOffset, json["rightServoOffset"]);
          strcpy(action, json["action"]);
          return true;
        }
        else
        {
          // Error loading JSON data
          Serial.println("Failed to load json config");
        }
      }
    }
  }
  else
  {
    // Error mounting file system
    Serial.println("Failed to mount FS");
  }
  return false;
}

void saveConfigCallback()
// Callback notifying us of the need to save configuration
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void configModeCallback(WiFiManager *myWiFiManager)
// Called when config mode launched
{
  Serial.println("Entered Configuration Mode");
  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void punchSystemSetup()
{
  Serial.print("Action: ");
  Serial.println(atoi(action));
  if (atoi(action) == 1)
  {
    punchHome = 90; // 360 Continuous Servo, 90deg = Stop
    punchHit = 0;   // 360 continuous Servo, 90deg = clockwise
  }
}

bool sonicDistance()
{
  int distance = sonar[0].ping_cm();
  Serial.print("Sonar Distance: ");
  Serial.println(distance);
  if (distance == 0 || distance > 10)
  {
    return false;
  }
  return true;
}

void stopMotors()
{
  // punchServo.write(punchHome);
  leftServo.write(leftStop);
  rightServo.write(rightStop);
  Serial.println("Motors Stopped");
}

void motionControl()
{
  if (String(dataReceived) == "RFC")
  {
    if (sonicDistance() || actionType == 2)
    {
      if (punch_handle != NULL)
      {
        vTaskResume(punch_handle);
      }
    }
    leftServo.write(leftServoVel + atoi(leftServoOffset));
    rightServo.write(rightServoVel + atoi(rightServoOffset));
    if (delayStop)
    {
      delay(delayStopVal);
      stopMotors();
    }
  }
  else if (String(dataReceived) == "STOP")
  {
    stopMotors();
    udpIsSetup = false;
    Serial.println("RFC Server told us to shutdown and wait");
  }
  else
  {
    stopMotors();
  }
}

void startCameraServer();

bool initCamera()
{
  Serial.println("Initialising camera");
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;       // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  config.pixel_format = PIXFORMAT_JPEG; //
  config.frame_size = FRAMESIZE_HVGA;
  config.grab_mode = CAMERA_GRAB_LATEST; // CAMERA_GRAB_WHEN_EMPTY
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10; // 0-63 lower number means higher quality
  config.fb_count = 1;

  // Check the esp32cam board has a psram chip installed (extra memory used for storing captured images)
  // Note: if not using "AI thinker esp32 cam" in the Arduino IDE, PSRAM must be enabled
  if (psramFound())
  {
    config.jpeg_quality = 10; // 0-63 lower number means higher quality
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST; // CAMERA_GRAB_LATEST,
  }
  else
  {
    // Limit the frame size when PSRAM is not available
    config.jpeg_quality = 30; // 0-63 lower number means higher quality
    config.frame_size = FRAMESIZE_HVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  //
  // Initialise the camera
  esp_err_t camera = esp_camera_init(&config);
  if (camera != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", camera);
  }
  cameraImageSettings();     // apply custom camera settings
  return (camera == ESP_OK); // return boolean result of camera initialisation
}

bool cameraImageSettings()
{
  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  return 1;
  //    // More camera settings available:
  //    // If you enable gain_ctrl or exposure_ctrl it will prevent a lot of the other settings having any effect
  //    // more info on settings here: https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/
  //    s->set_gain_ctrl(s, 0);                       // auto gain off (1 or 0)
  //    s->set_exposure_ctrl(s, 0);                   // auto exposure off (1 or 0)
  //    s->set_agc_gain(s, cameraImageGain);          // set gain manually (0 - 30)
  //    s->set_aec_value(s, cameraImageExposure);     // set exposure manually  (0-1200)
  //    s->set_vflip(s, cameraImageInvert);           // Invert image (0 or 1)
  //    s->set_quality(s, 10);                        // (0 - 63)
  //    s->set_gainceiling(s, GAINCEILING_32X);       // Image gain (GAINCEILING_x2, x4, x8, x16, x32, x64 or x128)
  //    s->set_brightness(s, cameraImageBrightness);  // (-2 to 2) - set brightness
  //    s->set_lenc(s, 1);                            // lens correction? (1 or 0)
  //    s->set_saturation(s, 0);                      // (-2 to 2)
  //    s->set_contrast(s, cameraImageContrast);      // (-2 to 2)
  //    s->set_sharpness(s, 0);                       // (-2 to 2)
  //    s->set_hmirror(s, 0);                         // (0 or 1) flip horizontally
  //    s->set_colorbar(s, 0);                        // (0 or 1) - show a testcard
  //    s->set_special_effect(s, 0);                  // (0 to 6?) apply special effect
  //    s->set_whitebal(s, 0);                        // white balance enable (0 or 1)
  //    s->set_awb_gain(s, 1);                        // Auto White Balance enable (0 or 1)
  //    s->set_wb_mode(s, 0);                         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //    s->set_dcw(s, 0);                             // downsize enable? (1 or 0)?
  //    s->set_raw_gma(s, 1);                         // (1 or 0)
  //    s->set_aec2(s, 0);                            // automatic exposure sensor?  (0 or 1)
  //    s->set_ae_level(s, 0);                        // auto exposure levels (-2 to 2)
  //    s->set_bpc(s, 0);                             // black pixel correction
  //    s->set_wpc(s, 0);                             // white pixel correction
  //
}

void initServos()
{
  Serial.println("Initialising servos");
  leftStop = leftStop + atoi(leftServoOffset);
  rightStop = rightStop + atoi(rightServoOffset);
  punchServo.attach(servoPunchPin);
  leftServo.attach(servoLeftPin);
  rightServo.attach(servoRightPin);
  punchServo.write(punchHome);
  leftServo.write(leftStop);
  rightServo.write(rightStop);
}

bool initWifi(bool forceConfig)
{
  WiFi.setSleep(false);
  WiFiManager wm;
  WiFi.mode(WIFI_STA); // Explicitly set WiFi mode
  // wm.resetSettings(); // comment this out when not testing wifi manager anymore
  wm.setMinimumSignalQuality(30); // set minimu quality of signal so it ignores AP's under that quality
  //
  // Custom elements
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter tb_robo_name("name", "Enter MECH name here", roboName, 100);
  WiFiManagerParameter tb_port_number("port", "Set the Port Number here", portNumber, 6);
  WiFiManagerParameter tb_left_servo_offset("lso", "Left Servo Offset", leftServoOffset, 4);
  WiFiManagerParameter tb_right_servo_offset("rso", "Right Servo Offset", rightServoOffset, 4);
  WiFiManagerParameter tb_action("action", "Action Type", action, 4);
  //
  wm.setSaveConfigCallback(saveConfigCallback); // Set callback - save config
  wm.setAPCallback(configModeCallback);         // Set callback - connecting to previous WiFi fails, enters Access Point mode
  //
  // Add all defined custom parameters
  wm.addParameter(&tb_robo_name);
  wm.addParameter(&tb_port_number);
  wm.addParameter(&tb_left_servo_offset);
  wm.addParameter(&tb_right_servo_offset);
  wm.addParameter(&tb_action);
  //
  if (forceConfig) // Run if we need a configuration
  {
    if (!wm.startConfigPortal("MECH_AP", ""))
    {
      Serial.println("failed to connect and hit timeout");
      delay(1000);
      // reset and try again
      ESP.restart();
      delay(3000);
    }
  }
  else
  {
    if (!wm.autoConnect("MECH_AP", ""))
    {
      Serial.println("failed to connect and hit timeout");
      delay(1000);
      // reset and try again
      ESP.restart();
      delay(3000);
    }
  }
  //
  //
  //
  Serial.println("");
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // WiFi.setAutoConnect(true);
  // WiFi.setAutoReconnect(true);
  //
  //  Custom config values
  strcpy(roboName, tb_robo_name.getValue());
  strcpy(portNumber, tb_port_number.getValue());
  strcpy(leftServoOffset, tb_left_servo_offset.getValue());
  strcpy(rightServoOffset, tb_right_servo_offset.getValue());
  strcpy(action, tb_action.getValue());
  //
  Serial.println("The values in the file are: ");
  Serial.println("roboName: " + String(roboName));
  Serial.println("portNumber: " + String(portNumber));
  Serial.println("leftServoOffset: " + String(leftServoOffset));
  Serial.println("rightServoOffset: " + String(rightServoOffset));
  Serial.println("action: " + String(action));
  Serial.println("...");
  //
  // Save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfigFile();
  }
  return 1;
}

void flashRedLed(int pulse)
{
  // Flash LED
  digitalWrite(LED_BUILTIN, HIGH);
  delay(pulse);
  digitalWrite(LED_BUILTIN, LOW);
  delay(pulse);
}

bool udpReceive()
{
  // Check for reply
  udp.flush();
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    if (udp.remoteIP() == senderIP)
    {
      // Serial.print("RFC Server sent packet of size: ");
      // Serial.println(packetSize);
      int len = udp.read(packetBuffer, 255);
      if (len > 0)
      {
        packetBuffer[len] = 0;
      }
      Serial.print("RFC Server Command: ");
      Serial.println(packetBuffer);
      int commaCount = sscanf(packetBuffer, "%d,%d,%d,%s", &leftServoVel, &rightServoVel, &actionType, dataReceived);
      if (commaCount == 4)
      {
        // Print the substrings
        // printf("servoLeft: %d\n", leftServoVel);
        // printf("servoRight: %d\n", rightServoVel);
        // printf("actionType: %d\n", actionType);
        // printf("data: %s\n", dataReceived);
      }
      else
      {
        return false;
      }
      return true;
    }
    else
    {
      Serial.println("other traffic");
    }
  }
  return false;
}

void udpSend(IPAddress destIP, String message)
{
  // The below String structure actually works so keep this
  udp.beginPacket(destIP, atoi(portNumber));
  udp.write((uint8_t *)message.c_str(), message.length());
  udp.endPacket();
}

bool udpSetup()
{
  udp.flush();
  // Check for reply
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Somebody sent a UDP packet of size: ");
    Serial.println(packetSize);
    int len = udp.read(packetBuffer, 255);
    if (len > 0)
    {
      packetBuffer[len] = 0;
    }
    Serial.print("And it contained the message: ");
    Serial.println(packetBuffer);
    Serial.print("From IP: ");
    Serial.println(udp.remoteIP());
    if (String(packetBuffer) == rfcWelcome)
    {
      senderIP = udp.remoteIP();
      Serial.println("This is the RFC server we are looking for");
      Serial.println("Congrats, Signed up with RFC server");
      Serial.print("RFC Server IP set to: ");
      Serial.println(senderIP);
      Serial.println(".............");
      Serial.println("Now let's fight!");
      Serial.println(".............");
      return true;
    }
    else
    {
      Serial.println("Not the server we are looking for");
    }
  }
  return false;
}

void isUdpRunning()
{
  while (!udpIsSetup)
  {
    Serial.println("Attempting to connect to RFC Server");
    udpSend(senderIP, String(roboName) + "|" + VERSION);
    udpIsSetup = udpSetup();
    udp.flush();
    flashRedLed(500);
  }
}

bool initDrd()
{
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset())
  {
    Serial.println(F("Forcing config mode as there was a Double reset detected"));
    return true;
  }
  return false;
}

bool initSpiffs(bool forceConfig)
{
  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup)
  {
    Serial.println(F("Forcing config mode as there is no saved config on MECH"));
    return true;
  }
  return forceConfig;
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.printf("MECH Version: %s \n", VERSION);
  //
  bool forceConfig = false;     //
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  digitalWrite(LED_BUILTIN, LOW);
  //
  forceConfig = initDrd();
  forceConfig = initSpiffs(forceConfig);
  initWifi(forceConfig);
  initCamera();
  startCameraServer();
  punchSystemSetup();
  initServos();
  //
  xTaskCreate(
      punch,        // Function Name
      "Punch",      // Task Name
      700,          // Stack Size
      NULL,         // Task Parameters
      1,            // Task Priority
      &punch_handle // Task Handle
  );
  //
  udp.begin(atoi(portNumber));
  senderIP = broadcastIP;
  udpIsSetup = false;
}

void loop()
{
  isUdpRunning();
  flashRedLed(20);
  udpSend(senderIP, rfcAcknowledge);
  // Serial.println("waiting for command");
  if (udpReceive())
  {
    Serial.println("processing command");
    motionControl();
    Serial.println("processed command");
  }
  else
  {
    delay(20);
  }
}
