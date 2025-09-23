#include <stdio.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"

static const char *TAG = "CAMERA_STREAM";

// ==== Wi-Fi Config ====
#define WIFI_SSID      "LIN"
#define WIFI_PASS      "22131081"

// ==== Freenove ESP32-S3 WROOM Camera Pins ====
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_PCLK  13
#define CAM_PIN_XCLK  15
#define CAM_PIN_D7    16
#define CAM_PIN_D6    17
#define CAM_PIN_D5    18
#define CAM_PIN_D4    12
#define CAM_PIN_D3    10
#define CAM_PIN_D2    8
#define CAM_PIN_D1    9
#define CAM_PIN_D0    11

// ==== Camera Config (base) ====
static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,

    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href  = CAM_PIN_HREF,
    .pin_pclk  = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count     = 1
};

// ==== Try camera init ====
esp_err_t try_camera_init(framesize_t frame_size, int fb_count) {
    camera_config.frame_size = frame_size;
    camera_config.fb_count   = fb_count;

    ESP_LOGI(TAG, "嘗試初始化攝影機，frame_size=%d, fb_count=%d", frame_size, fb_count);
    esp_err_t err = esp_camera_init(&camera_config);
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "Camera sensor detected, PID=0x%04x", s->id.PID);

            // === Fix upside-down video ===
            s->set_vflip(s, 1);      // flip vertically
            s->set_hmirror(s, 0);    // mirror if needed (set 1 if left/right reversed)
            ESP_LOGI(TAG, "Camera orientation corrected (vflip=1, hmirror=0)");
        }
        return ESP_OK;
    }
    ESP_LOGE(TAG, "攝影機初始化失敗: 0x%x", err);
    return err;
}

// ==== Robust init with fallback ====
esp_err_t init_camera_safe() {
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    int fb_count = (psram_size > 0) ? 2 : 1;

    if (try_camera_init(FRAMESIZE_QVGA, fb_count) == ESP_OK) return ESP_OK;
    if (try_camera_init(FRAMESIZE_QVGA, 1) == ESP_OK) return ESP_OK;
    if (try_camera_init(FRAMESIZE_QQVGA, 1) == ESP_OK) return ESP_OK;

    ESP_LOGE(TAG, "所有攝影機初始化嘗試皆失敗！");
    return ESP_FAIL;
}

// ==== HTTP Stream Handler ====
esp_err_t stream_handler(httpd_req_t *req) {
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    char part_buf[64];
    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }
        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
        httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        httpd_resp_send_chunk(req, part_buf, hlen);
        httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        vTaskDelay(50 / portTICK_PERIOD_MS); // ~20 FPS max
    }
    return ESP_OK;
}

esp_err_t index_handler(httpd_req_t *req) {
    const char* resp_str =
        "<html><body>"
        "<h2>ESP32-S3 Camera Stream</h2>"
        "<img src=\"/stream\">"
        "</body></html>";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri="/", .method=HTTP_GET, .handler=index_handler };
        httpd_uri_t stream_uri = { .uri="/stream", .method=HTTP_GET, .handler=stream_handler };
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        ESP_LOGI(TAG, "Web server started");
    }
}

// ==== Wi-Fi event handler ====
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi 斷線，嘗試重新連線...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Wi-Fi 連線成功，IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// ==== Init Wi-Fi ====
void init_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi 初始化完成，正在連線...");
}

// ==== Main App ====
void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_wifi();

    if (init_camera_safe() == ESP_OK) {
        vTaskDelay(2000 / portTICK_PERIOD_MS); // let sensor warm up
        start_webserver();
    } else {
        ESP_LOGE(TAG, "攝影機無法啟動");
    }

    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0) {
        ESP_LOGI("MAIN", "PSRAM detected: %d KB", psram_size / 1024);
    } else {
        ESP_LOGW("MAIN", "No PSRAM detected, using internal RAM only");
    }
}
