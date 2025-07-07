#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "ES8311_TEST"

// I2C Configuration
#define I2C_MASTER_SCL_IO           2      // GPIO2 for I2C SCL
#define I2C_MASTER_SDA_IO           1      // GPIO1 for I2C SDA
#define I2C_MASTER_NUM              0      // I2C master number
#define I2C_MASTER_FREQ_HZ          400000 // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS       1000

// ES8311 I2C Address
#define ES8311_ADDR                 0x18

// I2S Configuration
#define I2S_NUM                     I2S_NUM_0
#define I2S_BCLK_IO                 40     // Bit clock
#define I2S_WS_IO                   41     // Word select (LRCK)
#define I2S_DO_IO                   38     // Data out (ES8311 -> ESP32)
#define I2S_DI_IO                   39     // Data in (ESP32 -> ES8311)
#define I2S_SAMPLE_RATE             48000
#define I2S_CHANNELS                2
#define I2S_BITS_PER_SAMPLE        I2S_DATA_BIT_WIDTH_16BIT

// ES8311 Register Addresses
#define ES8311_CLK_MANAGER_REG01    0x01
#define ES8311_CLK_MANAGER_REG02    0x02
#define ES8311_CLK_MANAGER_REG03    0x03
#define ES8311_ADC_REG15           0x15
#define ES8311_DAC_REG31           0x31
#define ES8311_SYSTEM_REG0B        0x0B
#define ES8311_RESET_REG00         0x00

static esp_err_t es8311_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, ES8311_ADDR, write_buf, sizeof(write_buf), 
                                    I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t es8311_read_reg(uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ES8311_ADDR, &reg_addr, 1, data, 1, 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_MASTER_NUM, &conf), TAG, "I2C config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0), TAG, 
                       "I2C driver install failed");
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

static esp_err_t i2s_init(void)
{
    // I2S configuration
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;
    
    // Create TX channel
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle), TAG, 
                       "I2S channel creation failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_CHANNELS),
        .gpio_cfg = {
            .mclk = -1,        // MCLK disabled
            .bclk = I2S_BCLK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DI_IO,
            .din = I2S_DO_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    // Disable MCLK output
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_handle, &std_cfg), TAG, 
                       "I2S TX channel std mode initialization failed");
    
    if (rx_handle) {
        ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_handle, &std_cfg), TAG, 
                           "I2S RX channel std mode initialization failed");
    }
    
    ESP_LOGI(TAG, "I2S initialized successfully");
    return ESP_OK;
}

static esp_err_t es8311_init(void)
{
    // Reset ES8311
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_RESET_REG00, 0x1F), TAG, "Reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for reset
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_RESET_REG00, 0x00), TAG, "Clear reset failed");
    
    // Configure clock settings
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_CLK_MANAGER_REG01, 0x30), TAG, "Clock config failed");
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_CLK_MANAGER_REG02, 0x00), TAG, "Clock config failed");
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_CLK_MANAGER_REG03, 0x10), TAG, "Clock config failed");
    
    // Configure ADC/DAC
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_ADC_REG15, 0x40), TAG, "ADC config failed");
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_DAC_REG31, 0x40), TAG, "DAC config failed");
    
    // Power up system
    ESP_RETURN_ON_ERROR(es8311_write_reg(ES8311_SYSTEM_REG0B, 0x00), TAG, "Power up failed");
    
    ESP_LOGI(TAG, "ES8311 initialized successfully");
    return ESP_OK;
}

static void es8311_test_task(void *arg)
{
    uint8_t reg_val;
    
    while (1) {
        // Read and verify some registers
        if (es8311_read_reg(ES8311_CLK_MANAGER_REG01, &reg_val) == ESP_OK) {
            ESP_LOGI(TAG, "CLK_REG01: 0x%02X", reg_val);
        }
        
        if (es8311_read_reg(ES8311_SYSTEM_REG0B, &reg_val) == ESP_OK) {
            ESP_LOGI(TAG, "SYS_REG0B: 0x%02X", reg_val);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ES8311 test...");
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_init());
    
    // Initialize I2S
    ESP_ERROR_CHECK(i2s_init());
    
    // Initialize ES8311
    ESP_ERROR_CHECK(es8311_init());
    
    // Create test task
    xTaskCreate(es8311_test_task, "es8311_test", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Test setup completed");
}