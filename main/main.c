#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"

// 函数声明
static void mclk_task(void *arg);

// es8311 I2C 引脚定义
#define AUDIO_CODEC_ES8311_ADDR 0x18
#define AUDIO_CODEC_I2C_SDA_PIN     GPIO_NUM_1
#define AUDIO_CODEC_I2C_SDC_PIN     GPIO_NUM_2
#define AUDIO_CODEC_I2C_MCLK_PIN    GPIO_NUM_42

// I2C配置
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define I2C_TIMEOUT_MS              1000

// 全局I2C总线句柄
static i2c_master_bus_handle_t bus_handle = NULL;

// I2C初始化函数
static esp_err_t i2c_master_init(void)
{
    // 新版I2C Master配置
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = AUDIO_CODEC_I2C_SDC_PIN,
        .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    // 创建I2C Master总线
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C总线创建失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置MCLK引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AUDIO_CODEC_I2C_MCLK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // 启动MCLK输出任务
    xTaskCreate(mclk_task, "mclk_task", 2048, NULL, 5, NULL);
    
    ESP_LOGI("I2C", "I2C主机初始化成功");
    return ESP_OK;
}

// MCLK输出任务
static void mclk_task(void *arg)
{
    ESP_LOGI("MCLK", "开始输出MCLK信号");
    
    // 使用硬件时钟输出代替软件模拟
    // 配置MCLK为时钟输出
    // 注意：ESP32-S3可以使用LEDC外设产生时钟信号
    
    // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT, // 设置为1位分辨率，产生50%占空比
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 12000000, // 12MHz MCLK for ES8311
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    
    // 配置LEDC通道
    ledc_channel_config_t ledc_channel = {
        .gpio_num = AUDIO_CODEC_I2C_MCLK_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1, // 50%占空比 (对于1位分辨率，值为1表示50%)
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    
    ESP_LOGI("MCLK", "MCLK硬件时钟配置完成");
    
    // 任务完成后删除自己
    vTaskDelete(NULL);
}

// 扫描I2C总线上的所有设备
static void i2c_scan_devices(void)
{
    ESP_LOGI("I2C", "开始扫描I2C设备...");
    uint8_t devices_found = 0;
    
    // 创建设备句柄配置
    i2c_device_config_t dev_cfg = {
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    };
    
    for (uint8_t i = 1; i < 128; i++) {
        // 设置当前扫描的设备地址
        dev_cfg.dev_addr = i;
        
        // 创建临时设备句柄
        i2c_master_dev_handle_t device_handle;
        esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device_handle);
        
        if (ret == ESP_OK) {
            // 尝试与设备通信
            uint8_t dummy_data = 0;
            ret = i2c_master_transmit(device_handle, &dummy_data, 0, I2C_TIMEOUT_MS);
            
            // 删除临时设备句柄
            i2c_master_bus_rm_device(device_handle);
            
            if (ret == ESP_OK) {
                ESP_LOGI("I2C", "检测到设备: 0x%02x", i);
                devices_found++;
                
                // 如果是ES8311地址，特别标记
                if (i == AUDIO_CODEC_ES8311_ADDR) {
                    ESP_LOGI("I2C", "找到ES8311编解码器! (地址: 0x%02x)", i);
                }
            }
        }
        
        // 添加短暂延时，避免I2C总线过载
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (devices_found == 0) {
        ESP_LOGW("I2C", "未检测到任何I2C设备！请检查连接");
    } else {
        ESP_LOGI("I2C", "扫描完成, 共发现 %d 个设备", devices_found);
    }
}

#define LCD_HOST    SPI2_HOST

//个人定义变量
int my_clock = 0;

// LCD GPIO定义
#define PIN_NUM_MOSI 10
#define PIN_NUM_CLK  11
#define PIN_NUM_CS   8
#define PIN_NUM_DC   9
#define PIN_NUM_RST  3
#define PIN_NUM_BLK  18

#define LCD_WIDTH   240 //lcd屏幕，宽度像素数量
#define LCD_HEIGHT  320 //lcd屏幕，高度像素数量

static const char *TAG = "st7789";  //驱动标识
static spi_device_handle_t spi; //spi操作柄

// 发送命令到LCD
void lcd_cmd(const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;
    gpio_set_level(PIN_NUM_DC, 0);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
    
}

// 发送数据到LCD
void lcd_data(const uint8_t *data, int len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void*)1;
    gpio_set_level(PIN_NUM_DC, 1);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

// 初始化ST7789
void lcd_init(void) {
    // 复位LCD
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 初始化命令序列
    lcd_cmd(0x11);    // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x36);    // Memory Data Access Control
    uint8_t data = 0x00;
    lcd_data(&data, 1);

    lcd_cmd(0x3A);    // Interface Pixel Format
    data = 0x05;      // 16-bit pixel
    lcd_data(&data, 1);

    lcd_cmd(0xB2);    // Porch Setting
    uint8_t porch_data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    lcd_data(porch_data, 5);

    lcd_cmd(0xB7);    // Gate Control
    data = 0x35;
    lcd_data(&data, 1);

    lcd_cmd(0xBB);    // VCOM Setting
    data = 0x19;
    lcd_data(&data, 1);

    lcd_cmd(0xC0);    // LCM Control
    data = 0x2C;
    lcd_data(&data, 1);

    lcd_cmd(0xC2);    // VDV and VRH Command Enable
    data = 0x01;
    lcd_data(&data, 1);

    lcd_cmd(0xC3);    // VRH Set
    data = 0x12;
    lcd_data(&data, 1);

    lcd_cmd(0xC4);    // VDV Set
    data = 0x20;
    lcd_data(&data, 1);

    lcd_cmd(0xC6);    // Frame Rate Control in Normal Mode
    data = 0x0F;
    lcd_data(&data, 1);

    lcd_cmd(0xD0);    // Power Control 1
    uint8_t power_data[] = {0xA4, 0xA1};
    lcd_data(power_data, 2);

    lcd_cmd(0xE0);    // Positive Voltage Gamma Control
    uint8_t gamma_pos[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    lcd_data(gamma_pos, 14);

    lcd_cmd(0xE1);    // Negative Voltage Gamma Control
    uint8_t gamma_neg[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    lcd_data(gamma_neg, 14);

    lcd_cmd(0x21);    // Display Inversion On

    lcd_cmd(0x29);    // Display On

    // 打开背光
    gpio_set_level(PIN_NUM_BLK, 1);
}

// 填充整个屏幕为指定颜色
void fill_screen(uint16_t color) {
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    
    lcd_cmd(0x2A);    // Column Address Set
    uint8_t col_data[] = {0x00, 0x00, (LCD_WIDTH-1) >> 8, (LCD_WIDTH-1) & 0xFF};
    lcd_data(col_data, 4);

    lcd_cmd(0x2B);    // Row Address Set
    uint8_t row_data[] = {0x00, 0x00, (LCD_HEIGHT-1) >> 8, (LCD_HEIGHT-1) & 0xFF};
    lcd_data(row_data, 4);

    lcd_cmd(0x2C);    // Memory Write
    
    for(int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        uint8_t color_data[] = {color_high, color_low};
        lcd_data(color_data, 2);
    }
}


//打印内存信息
void print_heap_info() {
    ESP_LOGI("MEM", "MALLOC_CAP_8BIT: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("MEM", "MALLOC_CAP_32BIT: %d", heap_caps_get_free_size(MALLOC_CAP_32BIT));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("MEM", "MALLOC_CAP_INTERNAL: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("MEM", "MALLOC_CAP_EXEC: %d", heap_caps_get_free_size(MALLOC_CAP_EXEC));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("MEM", "MALLOC_CAP_SPIRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    vTaskDelay(pdMS_TO_TICKS(10));
}

// 释放I2C资源
static void i2c_cleanup(void)
{
    if (bus_handle != NULL) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        ESP_LOGI("I2C", "I2C资源已释放");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing ST7789 LCD");

    // 初始化I2C并检测ES8311设备
    ESP_LOGI(TAG, "初始化I2C总线");
    esp_err_t i2c_ret = i2c_master_init();
    if (i2c_ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C初始化失败: %s", esp_err_to_name(i2c_ret));
    } else {
        ESP_LOGI(TAG, "I2C初始化成功，开始扫描设备");
        // 扫描所有I2C设备
        i2c_scan_devices();
    }
    
    // 配置GPIO
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BLK, GPIO_MODE_OUTPUT);

    // 配置SPI总线
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };

    // 配置SPI设备
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40*1000*1000,    // 40MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY
    };

    // 初始化SPI总线
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // 添加SPI设备
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));

    // 初始化LCD
    lcd_init();

    ESP_LOGI(TAG, "ST7789 LCD initialized successfully");

    // 测试显示 - 循环显示不同颜色
    while(1) {

        print_heap_info();

        // // 打印所有内存区域
        // heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
        
        // // 打印DRAM信息
        // heap_caps_print_heap_info(MALLOC_CAP_8BIT);
        
        // // 打印SPIRAM信息
        // heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
        
        // // 获取可用内存大小
        // size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        // size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        
        // ESP_LOGI("MEMORY", "Free DRAM: %zu bytes", free_dram);
        // vTaskDelay(pdMS_TO_TICKS(10));

        // ESP_LOGI("MEMORY", "Free SPIRAM: %zu bytes", free_spiram);
        // vTaskDelay(pdMS_TO_TICKS(10));
        
        int free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        int total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "Free spiram: %u Total spiram: %u", free_psram, total_psram);


        // 红色
        fill_screen(0xF800);
        vTaskDelay(pdMS_TO_TICKS(1000));

        //printf("此时系统时间：%d\n",my_clock);
        
        // 绿色
        fill_screen(0x07E0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        //printf("此时系统时间：%d\n",my_clock);
        
        // 蓝色
        fill_screen(0x001F);
        vTaskDelay(pdMS_TO_TICKS(1000));

        //printf("此时系统时间：%d\n",my_clock);
    }
} 