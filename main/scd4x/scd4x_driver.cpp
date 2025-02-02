/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "scd4x_driver.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>

#include <scd4x.h>

static const char * TAG = "scd4x";

typedef struct {
    scd4x_sensor_config_t *config;
    i2c_dev_t dev;
    esp_timer_handle_t timer;
    bool is_initialized = false;
} scd4x_sensor_ctx_t;

static scd4x_sensor_ctx_t s_ctx;

static void timer_cb_internal(void *arg)
{
    auto *ctx = (scd4x_sensor_ctx_t *) arg;
    if (!(ctx && ctx->config)) {
        return;
    }

    uint16_t co2;
    float temp, humidity;

    esp_err_t err = scd4x_read_measurement(&s_ctx.dev, &co2, &temp, &humidity);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scd4x_read_measurement failed, err:%d", err);
        return;
    }
    if (ctx->config->co2.cb){
        ctx->config->co2.cb(ctx->config->co2.endpoint_id, static_cast<float>(co2), ctx->config->user_data);
    }
    if (ctx->config->temperature.cb) {
        ctx->config->temperature.cb(ctx->config->temperature.endpoint_id, temp, ctx->config->user_data);
    }
    if (ctx->config->humidity.cb) {
        ctx->config->humidity.cb(ctx->config->humidity.endpoint_id, humidity, ctx->config->user_data);
    }
}

esp_err_t scd4x_sensor_init(scd4x_sensor_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // we need at least one callback so that we can start notifying application layer
    if (config->temperature.cb == NULL || config->humidity.cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // keep the pointer to config
    s_ctx.config = config;    

    ESP_ERROR_CHECK(i2cdev_init());

    esp_err_t err = scd4x_init_desc(&s_ctx.dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scd4x_init_desc failed, err:%d", err);
        return err;
    }

    ESP_LOGI(TAG, "Initializing sensor...");
    ESP_ERROR_CHECK(scd4x_wake_up(&s_ctx.dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&s_ctx.dev));
    ESP_ERROR_CHECK(scd4x_reinit(&s_ctx.dev));
    ESP_LOGI(TAG, "Sensor initialized");

    err =  scd4x_start_periodic_measurement(&s_ctx.dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scd4x_start_periodic_measurement failed, err:%d", err);
        return err;
    }

    esp_timer_create_args_t args = {
        .callback = timer_cb_internal,
        .arg = &s_ctx,
    };

    err = esp_timer_create(&args, &s_ctx.timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
        return err;
    }

    err = esp_timer_start_periodic(s_ctx.timer, config->interval_ms * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
        return err;
    }

    s_ctx.is_initialized = true;
    ESP_LOGI(TAG, "shtc3 initialized successfully");

    return ESP_OK;
}