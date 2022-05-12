/*
 * Author: Silux Castañeda
 * Board: ESP32-CAM MODEL AI THINKER
 * Description: Simple video streaming over WiFi
 * Based on https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
 */

// ********** Headers ********** //
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"  // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include "esp_http_server.h"

// ********** User Defines ********** //
#define DEBUGON 1

#if DEBUGON == 1
#define debugInit(x) Serial.begin(x)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debugIniy(x)
#define debug(x)
#define debugln(x)
#endif

// ********** Network Credentials ********** //
const char* ssid = "Telcel-4F15";
const char* pass = "T18FR96H1M1";

// ********** Camera Defines ********** //
#define PART_BOUNDARY "123456789000000000000987654321"

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ********** User Variables ********** //
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


httpd_handle_t stream_httpd = NULL;

// ********** User Functions ********** //

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t* _jpg_buf = NULL;
  char* part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if(!fb){
      debugln("Camera capture failed");
      res = ESP_FAIL;
    }
    else{
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb,80,&_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            debugln("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else{
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    // NOTE: The next line order MATTERS
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char*)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len); 
    }
    
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    // debugln(MJPG: %uB\n, (uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t configh = HTTPD_DEFAULT_CONFIG();
  //configh.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  //debugln("Starting web server on port: '%d'\n", config.server_port);
  if(httpd_start(&stream_httpd, &configh) == ESP_OK){
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

// *********** Setup Function ********** //
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout

  debugInit(115200);

  // Camera configuration
  camera_config_t configc;
  configc.ledc_channel = LEDC_CHANNEL_0;
  configc.ledc_timer = LEDC_TIMER_0;
  configc.pin_d0 = Y2_GPIO_NUM;
  configc.pin_d1 = Y3_GPIO_NUM;
  configc.pin_d2 = Y4_GPIO_NUM;
  configc.pin_d3 = Y5_GPIO_NUM;
  configc.pin_d4 = Y6_GPIO_NUM;
  configc.pin_d5 = Y7_GPIO_NUM;
  configc.pin_d6 = Y8_GPIO_NUM;
  configc.pin_d7 = Y9_GPIO_NUM;
  configc.pin_xclk = XCLK_GPIO_NUM;
  configc.pin_pclk = PCLK_GPIO_NUM;
  configc.pin_vsync = VSYNC_GPIO_NUM;
  configc.pin_href = HREF_GPIO_NUM;
  configc.pin_sscb_sda = SIOD_GPIO_NUM;
  configc.pin_sscb_scl = SIOC_GPIO_NUM;
  configc.pin_pwdn = PWDN_GPIO_NUM;
  configc.pin_reset = RESET_GPIO_NUM;
  configc.xclk_freq_hz = 20000000;
  configc.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    configc.frame_size = FRAMESIZE_UXGA;
    configc.jpeg_quality = 10;
    configc.fb_count = 2;
  }
  else{
    configc.frame_size = FRAMESIZE_SVGA;
    configc.jpeg_quality = 12;
    configc.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&configc);
  if(err != ESP_OK){
    debug("Camera init failed with error 0x");
    debug(err);
    return;
  }

  // WiFi connection
  WiFi.begin(ssid,pass);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    debug(".");
  }
  debugln("\nWiFi connected");
  debug("Camera Stream Ready! Go to: http://");
  debug(WiFi.localIP());

  // Start streaming web server
  startCameraServer();
}

// ********** Loop Fucntion ********** //
void loop() {
  delay(1);
}
