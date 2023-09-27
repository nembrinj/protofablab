/*
 * @Descripttion: original code from Elegoo, comments translated to English
 * @version: 
 * @Author: Elegoo
 * @Date: 2020-06-04 11:42:27
 */
// WARNING!!! Make sure that you have selected Board ---> ESP32 Dev Module
//            Partition Scheme ---> Huge APP (3MB No OTA/1MB SPIFFS)
//            PSRAM ---> enabled
//Configurations importantes:
// #define HTTPD_DEFAULT_CONFIG()             
//   {                                        
//     .task_priority = tskIDLE_PRIORITY + 5, 
//     .stack_size = 4096,                    
//     .server_port = 80,                     
//     .ctrl_port = 32768,                    
//     .max_open_sockets = 7,                 
//     .max_uri_handlers = 8,                 
//     .max_resp_headers = 8,                 
//     .backlog_conn = 5,                     
//     .lru_purge_enable = false,             
//     .recv_wait_timeout = 5,                
//     .send_wait_timeout = 5,                
//     .global_user_ctx = NULL,               
//     .global_user_ctx_free_fn = NULL,       
//     .global_transport_ctx = NULL,          
//     .global_transport_ctx_free_fn = NULL,  
//     .open_fn = NULL,                       
//     .close_fn = NULL,                      
//   }

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM

//#define CAMERA_MODEL_M5STACK_WIDE

//#define CAMERA_MODEL_AI_THINKER

#define CAMERA_MODEL_TEDDY

#include "CameraWebServer_AP.h"
#include "camera_pins.h"
#include "esp_system.h"

// #include "BLEAdvertisedDevice.h"
// BLEAdvertisedDevice _BLEAdvertisedDevice;

#define OWN_WIFI 0   // 1 if the ESP32 WROVER emits its own wifi, 0 otherwise.

void startCameraServer();       // defined in app_httpd.cpp
void CameraWebServer_AP::CameraWebServer_AP_Init(void)
{
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
  config.xclk_freq_hz = 10000000;//20000000
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    Serial.println("[CameraWebServer_AP] psramFound");
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    Serial.println("[CameraWebServer_AP] nto psramFound");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
  //s->set_framesize(s, FRAMESIZE_SXGA); //Longueur d'octet Valeur d'échantillonnage:60000                 #9 (Haute qualité d'image)  1280x1024
  s->set_framesize(s, FRAMESIZE_SVGA); //Longueur d'octet Valeur d'échantillonnage:40000                   #7 (Qualité de l'image en)  800x600
  // s->set_framesize(s, FRAMESIZE_QVGA); //Longueur d'octet Valeur d'échantillonnage:10000                #4 (Faible qualité d'image)  320x240

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
#endif
  s->set_vflip(s, 0);   //Paramètres d'orientation de l'image (haut et bas)
  s->set_hmirror(s, 0); //Paramètres d'orientation de l'image (gauche et droite)

  // s->set_vflip(s, 1);   //Paramètres d'orientation de l'image (haut et bas)
  // s->set_hmirror(s, 1); //Paramètres d'orientation de l'image (gauche et droite)

  Serial.println("\r\n");

  uint64_t chipid = ESP.getEfuseMac();
  char string[10];
  sprintf(string, "%04X", (uint16_t)(chipid >> 32));
  String mac0_default = String(string);
  sprintf(string, "%08X", (uint32_t)chipid);
  String mac1_default = String(string);
  String url = ssid + mac0_default + mac1_default;
  const char *mac_default = url.c_str();

  if(OWN_WIFI){
    Serial.println(":----------------------------:");
    Serial.print("wifi_name:");
    Serial.println(mac_default);
    Serial.println(":----------------------------:");
    wifi_name = mac0_default + mac1_default;

    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(mac_default, password, 9);
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("' to connect");
}
