#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "Arduino.h"

#include "esp_lcd_jd9365.h"
#include "jd9365_lcd.h"

#define LCD_H_RES 800
#define LCD_V_RES 1280

#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)
#define LCD_BIT_PER_PIXEL (16)

// “VDD_MIPI_DPHY”应供电 2.5V，可从内部 LDO 稳压器或外部 LDO 芯片获取电源
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN 3 // LDO_VO3 连接至 VDD_MIPI_DPHY
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT GPIO_NUM_23

static const char *TAG = "example";
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;

jd9365_lcd::jd9365_lcd(int8_t lcd_rst)
{
    _lcd_rst = lcd_rst;
}

void jd9365_lcd::example_bsp_enable_dsi_phy_power()
{
    // 打开 MIPI DSI PHY 的电源，使其从“无电”状态进入“关机”状态
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
#ifdef EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif
}

void jd9365_lcd::example_bsp_init_lcd_backlight()
{
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT
        };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif
}

void jd9365_lcd::example_bsp_set_lcd_backlight(uint32_t level)
{
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, level);
#endif
}

void jd9365_lcd::begin()
{   
    example_bsp_enable_dsi_phy_power();
    example_bsp_init_lcd_backlight();
    example_bsp_set_lcd_backlight(EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

    // 首先创建 MIPI DSI 总线，它还将初始化 DSI PHY
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // 我们使用DBI接口发送LCD命令和参数
    esp_lcd_dbi_io_config_t dbi_config = JD9365_PANEL_IO_DBI_CONFIG();

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io_handle));

    // 创建JD9365控制面板
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);

    jd9365_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = _lcd_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9365(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle,true,true));

    // 打开背光
    example_bsp_set_lcd_backlight(EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
}

void jd9365_lcd::lcd_draw_bitmap(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *color_data)
{
    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

void jd9365_lcd::draw16bitbergbbitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *color_data)
{
    uint16_t x_start = x;
    uint16_t y_start = y;
    uint16_t x_end = w + x;
    uint16_t y_end = h + y;

    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

void jd9365_lcd::fillScreen(uint16_t color)
{
    // **CRITICAL: Fixed for 800x1280 display with DMA-capable memory**
    const int CHUNK_HEIGHT = 100; // Process 100 lines at a time
    const int TOTAL_PIXELS_PER_CHUNK = LCD_H_RES * CHUNK_HEIGHT;
    
    // **CRITICAL: Allocate DMA-capable buffer for ESP32-P4**
    uint16_t *color_data = (uint16_t *)heap_caps_malloc(
        TOTAL_PIXELS_PER_CHUNK * sizeof(uint16_t), 
        MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM
    );
    
    if (!color_data) {
        // Fallback to smaller buffer if allocation fails
        const int SMALL_CHUNK = 50;
        color_data = (uint16_t *)heap_caps_malloc(
            LCD_H_RES * SMALL_CHUNK * sizeof(uint16_t), 
            MALLOC_CAP_DMA
        );
        
        if (!color_data) {
            ESP_LOGE("LCD", "Failed to allocate memory for fillScreen");
            return;
        }
        
        // Fill buffer with color
        for (int i = 0; i < LCD_H_RES * SMALL_CHUNK; i++) {
            color_data[i] = color;
        }
        
        // Draw in small chunks
        for (int y = 0; y < LCD_V_RES; y += SMALL_CHUNK) {
            int height = (y + SMALL_CHUNK > LCD_V_RES) ? (LCD_V_RES - y) : SMALL_CHUNK;
            draw16bitbergbbitmap(0, y, LCD_H_RES, height, color_data);
        }
        
        heap_caps_free(color_data);
        return;
    }
    
    // Fill buffer with color
    for (int i = 0; i < TOTAL_PIXELS_PER_CHUNK; i++) {
        color_data[i] = color;
    }
    
    // **CRITICAL: Draw in chunks for 800x1280 display**
    for (int y = 0; y < LCD_V_RES; y += CHUNK_HEIGHT) {
        int height = (y + CHUNK_HEIGHT > LCD_V_RES) ? (LCD_V_RES - y) : CHUNK_HEIGHT;
        
        // Update buffer size if this is the last chunk
        if (height != CHUNK_HEIGHT) {
            for (int i = LCD_H_RES * height; i < TOTAL_PIXELS_PER_CHUNK; i++) {
                color_data[i] = color;
            }
        }
        
        draw16bitbergbbitmap(0, y, LCD_H_RES, height, color_data);
    }
    
    heap_caps_free(color_data);
}


void jd9365_lcd::te_on()
{
    esp_lcd_panel_io_tx_param(io_handle, 0x35,new (uint8_t[]){0x00}, 1);
}

void jd9365_lcd::te_off()
{
    esp_lcd_panel_io_tx_param(io_handle, 0x34,new (uint8_t[]){0x00}, 0);
}

uint16_t jd9365_lcd::width()
{
    return LCD_H_RES;
}

uint16_t jd9365_lcd::height()
{
    return LCD_V_RES;
}
