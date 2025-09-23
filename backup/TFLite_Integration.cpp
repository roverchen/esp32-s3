// TFLite_Integration.cpp
#include <Arduino.h>
#include <cstdint>
#include <esp_err.h>
#include <esp_http_server.h>
#include <esp_camera.h>
#include "tinyyolov2_road_int8.h"
#include "img_converters.h"

// TensorFlow Lite Micro 核心函式庫
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// 定義模型輸入尺寸
#define INPUT_W 224
#define INPUT_H 224

// --- 模型推論所需的記憶體 ---
const int kTensorArenaSize = 250 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// --- JPEG decode → 灰階 224×224 ---
bool jpeg2gray224(const uint8_t *jpg_buf, size_t jpg_len, uint8_t *out_buf, int fb_width, int fb_height) {
    uint8_t *rgb_buf = (uint8_t*)malloc(fb_width * fb_height * 3);
    if (!rgb_buf) {
        Serial.println("RGB buffer malloc failed!");
        return false;
    }

    bool ok = fmt2rgb888(jpg_buf, jpg_len, PIXFORMAT_JPEG, rgb_buf);
    if (!ok) {
        Serial.println("JPEG decode failed!");
        free(rgb_buf);
        return false;
    }

    // 最近鄰縮放 & 灰階化
    for (int y = 0; y < INPUT_H; y++) {
        for (int x = 0; x < INPUT_W; x++) {
            int src_x = (int)((float)x * fb_width / INPUT_W);
            int src_y = (int)((float)y * fb_height / INPUT_H);
            int src_idx = (src_y * fb_width + src_x) * 3;

            uint8_t r = rgb_buf[src_idx + 0];
            uint8_t g = rgb_buf[src_idx + 1];
            uint8_t b = rgb_buf[src_idx + 2];

            uint8_t gray = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
            out_buf[y * INPUT_W + x] = gray;
        }
    }

    free(rgb_buf);
    return true;
}

// --- 灰階 [0..255] → int8_t [-128..127] ---
void gray2int8tensor(const uint8_t *gray_buf, int8_t *out_buf, int width, int height) {
    float scale = input->params.scale;
    int zero_point = input->params.zero_point;

    for (int i = 0; i < width * height; i++) {
        float norm = (float)gray_buf[i] / 255.0f;
        int val = (int)(norm / scale + zero_point);
        out_buf[i] = (int8_t)std::clamp(val, -128, 127);
    }
/*
    for (int i = 0; i < width * height; i++) {
        float norm = ((float)gray_buf[i] / 255.0f) * 2.0f - 1.0f;
        int val = (int)(norm * 127.0f);
        if (val > 127) val = 127;
        if (val < -128) val = -128;
        out_buf[i] = (int8_t)val;
    }
*/
}

static tflite::MicroInterpreter *interpreter = nullptr;

void init_tflite() {
    const tflite::Model* model = tflite::GetModel(g_model);
    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, nullptr);
    interpreter = &static_interpreter;
    interpreter->AllocateTensors();
}

// --- Detect Handler (整合推論) ---
static esp_err_t detect_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    static uint8_t input_gray[INPUT_W * INPUT_H];
    if (!jpeg2gray224(fb->buf, fb->len, input_gray, fb->width, fb->height)) {
        esp_camera_fb_return(fb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    esp_camera_fb_return(fb);

    static int8_t input_tensor[INPUT_W * INPUT_H];
    gray2int8tensor(input_gray, input_tensor, INPUT_W, INPUT_H);

    // --- 執行 TensorFlow Lite 推論 ---
    const tflite::Model* model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("Model schema version is incompatible.");
        return ESP_FAIL;
    }

    static tflite::AllOpsResolver resolver;
    
    // 修正: 加上 nullptr 作為第五個參數
    //tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, kTensorArenaSize, nullptr);
    TfLiteTensor* input = interpreter->input(0);

    TfLiteStatus allocate_status = interpreter.AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        Serial.println("Tensor allocation failed.");
        return ESP_FAIL;
    }

    TfLiteTensor* input = interpreter.input(0);
    memcpy(input->data.int8, input_tensor, INPUT_W * INPUT_H);

    TfLiteStatus invoke_status = interpreter.Invoke();
    if (invoke_status != kTfLiteOk) {
        Serial.println("Model invoke failed.");
        return ESP_FAIL;
    }

    TfLiteTensor* output = interpreter.output(0);
    
    // TODO: 解析輸出 tensor 來取得實際的偵測結果和置信度
    // 這一部分取決於您的模型輸出格式。
    // For Tiny-YOLOv2, the output tensor likely has a shape like 1x1x(13x13x(5+N))
    // where N is the number of classes.
    // 您需要遍歷輸出張量，找到置信度最高的偵測框。
    
    // 這裡只是示範如何存取輸出的第一個值
    // 請注意，這不代表最終的置信度
    float conf = 0.0;
    // 範例: 如果輸出是 float32 格式
    // if (output->type == kTfLiteFloat32) {
    //     conf = output->data.f[4]; // 假設置信度在第五個位置
    // }
    
    // 由於您的模型是 INT8，需要將輸出值反量化
    if (output->type == kTfLiteInt8) {
        // 反量化公式: float_value = (int8_value - zero_point) * scale
        // 您需要從模型中取得 zero_point 和 scale
        // conf = (output->data.int8[4] - output->params.zero_point) * output->params.scale;
        
        // 為了簡單起見，這裡先用模擬值
        //conf = 0.85; 
        float conf = (output->data.int8[i] - output->params.zero_point) * output->params.scale;
    }
    
    bool road = true; // 模擬偵測結果
    
    char json[128];
    snprintf(json, sizeof(json),
         "{\"road_detected\": %s, \"confidence\": %.2f}",
         road ? "true" : "false", conf);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, strlen(json));
}
