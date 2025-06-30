#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"




#include <esp_heap_caps.h>
#include <esp_log.h>


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

void app_main(void) {


    ESP_LOGI(TAG, "Initializing ST7789 LCD");

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