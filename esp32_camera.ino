#include <tomno2-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define IMG_W 160
#define IMG_H 120

uint16_t *snapshot_buf;

QueueHandle_t frameQueue;

volatile int frame_count = 0;

static camera_config_t config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, 
    .frame_size   = FRAMESIZE_QQVGA,

    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
};

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset;

    for (size_t i = 0; i < length; i++) {
        uint16_t pixel = snapshot_buf[pixel_ix++];

        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5)  & 0x3F;
        uint8_t b = pixel & 0x1F;

        r <<= 3; g <<= 2; b <<= 3;

        out_ptr[i] = (r << 16) | (g << 8) | b;
    }
    return 0;
}

void taskCamera(void *pvParameters)
{
    while (1)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;

        frame_count++;

        if (frame_count % 1 == 0)
        {
            tft.pushImage(0, 4, IMG_W, IMG_H, (uint16_t*)fb->buf);
        }

        if (frame_count % 3 == 0)
        {
            xQueueOverwrite(frameQueue, &fb);
        }
        else
        {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(1);
    }
}

void taskAI(void *pvParameters)
{
    camera_fb_t *fb;
    while (1)
    {
        if (xQueueReceive(frameQueue, &fb, portMAX_DELAY))
        {
            memcpy(snapshot_buf, fb->buf, IMG_W * IMG_H * 2);

            ei::signal_t signal;
            signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
            signal.get_data = &ei_camera_get_data;

            ei_impulse_result_t result = {0};

            EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

            if (err == EI_IMPULSE_OK)
            {
                float max_val = 0;
                const char *label = "";

                for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
                {
                    if (result.classification[i].value > max_val)
                    {
                        max_val = result.classification[i].value;
                        label = ei_classifier_inferencing_categories[i];
                    }
                }

                if (frame_count % 3 == 0)
                {                    
                    tft.setTextColor(tft.color565(102, 57, 49), tft.color565(203, 219, 252));
                    tft.setCursor(5, 107);
                    tft.printf("%s %.2f   ", label, max_val);
                }
            }
            esp_camera_fb_return(fb);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // TFT SPI
    SPI.begin(14, -1, 15, 2);
    SPI.setFrequency(40000000);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);

    // PSRAM
    snapshot_buf = (uint16_t*)ps_malloc(IMG_W * IMG_H * 2);
    // if (!snapshot_buf) {
    //     Serial.println("PSRAM FAIL");
    //     while(1);
    // }

    // Camera
    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("CAM FAIL");
        while(1);
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    frameQueue = xQueueCreate(1, sizeof(camera_fb_t*));

    xTaskCreatePinnedToCore(taskCamera, "CAM", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskAI, "AI", 4096, NULL, 1, NULL, 1);
}

void loop() {}
