#include <stdio.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// Wi-Fi Provisioning
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"

static const char *TAG = "CAMERA_STREAM";
#define MAX_RETRY 5

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_PROVISIONED_BIT BIT1
#define WIFI_FAIL_BIT      BIT2

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

// ==== Camera Config ====
static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
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

// ==== Camera init helpers ====
esp_err_t try_camera_init(framesize_t frame_size, int fb_count) {
    camera_config.frame_size = frame_size;
    camera_config.fb_count   = fb_count;
    ESP_LOGI(TAG, "Init camera, frame_size=%d, fb_count=%d", frame_size, fb_count);
    esp_err_t err = esp_camera_init(&camera_config);
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "Camera detected, PID=0x%04x", s->id.PID);
            s->set_vflip(s, 1);
            s->set_hmirror(s, 0);
        }
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    return err;
}

esp_err_t init_camera_safe() {
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    int fb_count = (psram_size > 0) ? 2 : 1;
    if (try_camera_init(FRAMESIZE_QVGA, fb_count) == ESP_OK) return ESP_OK;
    if (try_camera_init(FRAMESIZE_QVGA, 1) == ESP_OK) return ESP_OK;
    if (try_camera_init(FRAMESIZE_QQVGA, 1) == ESP_OK) return ESP_OK;
    ESP_LOGE(TAG, "All camera init attempts failed!");
    return ESP_FAIL;
}

// ==== HTTP Stream Handlers ====
static httpd_handle_t server = NULL;

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
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri="/", .method=HTTP_GET, .handler=index_handler };
        httpd_uri_t stream_uri = { .uri="/stream", .method=HTTP_GET, .handler=stream_handler };
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        ESP_LOGI(TAG, "Web server started");
    }
}

// ==== Wi-Fi Events ====
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < MAX_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAX_RETRY);
                } else {
                    ESP_LOGW(TAG, "WiFi connect failed after %d retries", MAX_RETRY);
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_END) {
            ESP_LOGI(TAG, "Provisioning ended, de-initializing manager.");
            wifi_prov_mgr_deinit();
            xEventGroupSetBits(s_wifi_event_group, WIFI_PROVISIONED_BIT);
        }
    }
}

// ==== Wi-Fi Connection and Provisioning Logic ====
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif  = NULL;

void start_web_prov(void) {
    s_wifi_event_group = xEventGroupCreate();

    // 註冊所有必要的事件處理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // 初始化 Provisioning Manager
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap, // 使用 SoftAP 方案
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (provisioned) {
        ESP_LOGI(TAG, "Device already provisioned, connecting to the network...");
        sta_netif = esp_netif_create_default_wifi_sta();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "Failed to connect after %d retries, starting web provisioning", s_retry_num);

            // Destroy STA netif before switching
            if (sta_netif) {
                esp_netif_destroy_default_wifi(sta_netif);
                sta_netif = NULL;
            }

            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

            // 啟動基於 HTTPD 的配網方案
            wifi_prov_mgr_start_provisioning(
                WIFI_PROV_SECURITY_0,
                NULL,
                "PROV_ESP32",
                NULL);

            xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        }
    } else {
        ESP_LOGI(TAG, "Device not provisioned, starting web provisioning...");
        ap_netif = esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

        // 啟動基於 HTTPD 的配網方案
        wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_0,
            NULL,
            "PROV_ESP32",
            NULL);

        xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    }
}
/*
// ==== Wi-Fi Connection and Provisioning Logic ====
void start_web_prov(void) {
    s_wifi_event_group = xEventGroupCreate();

    // 註冊事件處理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // 初始化 Provisioning Manager
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (provisioned) {
        ESP_LOGI(TAG, "Device already provisioned, trying to connect...");
        esp_netif_create_default_wifi_sta();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        // 等待連線結果
        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE,
            portMAX_DELAY);

        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "Failed to connect after %d retries → fallback to provisioning", MAX_RETRY);
            // 啟動 SoftAP 配網
            esp_netif_destroy_default_wifi_sta();
            esp_netif_create_default_wifi_ap();
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            wifi_prov_mgr_start_provisioning(
                WIFI_PROV_SECURITY_0,
                NULL,                 // no proof-of-possession
                "PROV_ESP32",         // SSID
                "12345678"            // password
            );
            xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        }
    } else {
        ESP_LOGI(TAG, "Device not provisioned, starting provisioning...");
        esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_0,
            NULL,
            "PROV_ESP32",
            "12345678"
        );
        xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    }
}
*/
/*
// ==== Wi-Fi Events ====
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < MAX_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", s_retry_num, MAX_RETRY);
                } else {
                    ESP_LOGI(TAG, "WiFi connect failed after %d retries", MAX_RETRY);
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_END) {
            ESP_LOGI(TAG, "Provisioning ended, de-initializing manager.");
            wifi_prov_mgr_deinit();
            xEventGroupSetBits(s_wifi_event_group, WIFI_PROVISIONED_BIT);
        }
    }
}

// ==== Wi-Fi Connection and Provisioning Logic ====
void start_web_prov(void) {
    s_wifi_event_group = xEventGroupCreate();

    // 註冊所有必要的事件處理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // 初始化 Provisioning Manager
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_softap, // 使用 SoftAP 方案
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (provisioned) {
        ESP_LOGI(TAG, "Device already provisioned, connecting to the network...");
        esp_netif_create_default_wifi_sta();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "Failed to connect after %d retries, starting web provisioning", MAX_RETRY);
            esp_netif_destroy_default_wifi_sta();
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            // 啟動基於 HTTPD 的配網方案
            wifi_prov_mgr_start_provisioning(
                WIFI_PROV_SECURITY_0,
                NULL,
                "PROV_ESP32",
                NULL);
            xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        }
    } else {
        ESP_LOGI(TAG, "Device not provisioned, starting web provisioning...");
        esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        // 啟動基於 HTTPD 的配網方案
        wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_0,
            NULL,
            "PROV_ESP32",
            NULL);
        xEventGroupWaitBits(s_wifi_event_group, WIFI_PROVISIONED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    }
}
*/
// ==== Main App ====
void app_main() {
    // --- NVS Flash 初始化 ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // --- Wi-Fi 堆棧和事件循環初始化 ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // --- Wi-Fi 驅動初始化 ---
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 啟動 Wi-Fi 連線或配網流程
    start_web_prov();

    // --- 攝影機初始化 ---
    if (init_camera_safe() != ESP_OK) {
        ESP_LOGE(TAG, "Camera failed to start, check PSRAM configuration.");
    }

    // 輸出 PSRAM 資訊
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM: %d KB", psram_size / 1024);

    // 啟動網頁伺服器
    start_webserver();
}
