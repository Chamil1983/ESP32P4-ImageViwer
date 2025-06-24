#pragma GCC push_options
#pragma GCC optimize("O2")

// Include required libraries
#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include <FS.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include "JPEGDEC.h"
#include "lv_conf.h"
#include "pins_config.h"
#include "debug_logger.h"
#include "src/lcd/jd9365_lcd.h"
#include "src/touch/gsl3680_touch.h"

// JPEG decoder instance
JPEGDEC jpeg;

// System status tracking
bool lcd_ready = false;
bool touch_ready = false;
bool sd_ready = false;
bool wifi_ready = false;
bool lvgl_ready = false;
bool ui_ready = false;
bool server_ready = false;

// Watchdog settings
#define WDT_TIMEOUT_SECONDS 120

// WiFi AP configuration
#define WIFI_AP_SSID "ESP32P4-ImageViewer"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP IPAddress(192, 168, 4, 1)
#define WIFI_AP_GATEWAY IPAddress(192, 168, 4, 1)
#define WIFI_AP_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_CHANNEL 6
#define MAX_WIFI_CLIENTS 2

// Web Server on port 80
WebServer server(80);
DNSServer dnsServer;

// Hardware objects
jd9365_lcd lcd = jd9365_lcd(LCD_RST);
gsl3680_touch touch = gsl3680_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

// LVGL display buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

// Image gallery variables
#define MAX_IMAGES 100
char *image_list[MAX_IMAGES];
volatile int image_count = 0;
volatile int current_image = 0;
uint32_t last_ui_update = 0;
uint32_t last_system_update = 0;

// JPEG buffer for drawing
#define DRAW_BUFFER_SIZE (32 * 1024)
uint8_t *drawBuffer = nullptr;

// Global flag for image rotation (used by JPEG decoder and draw callback)
bool image_needs_rotation = false;

// Global flag to track image loading state
bool image_loading = false;

// DNS server status tracking
bool dnsServerActive = false;

// Slideshow variables
bool slideshow_active = false;
uint32_t slideshow_last_change = 0;
uint32_t slideshow_interval = 5000;
hw_timer_t *slideshow_timer = NULL;
portMUX_TYPE slideshow_timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t slideshow_task_handle = NULL;

// UI elements
lv_obj_t *main_screen = nullptr;
lv_obj_t *status_bar = nullptr;
lv_obj_t *image_view = nullptr;
lv_obj_t *image_display = nullptr;
lv_obj_t *control_bar = nullptr;
lv_obj_t *status_label = nullptr;
lv_obj_t *wifi_label = nullptr;
lv_obj_t *image_counter = nullptr;
lv_obj_t *prev_btn = nullptr;
lv_obj_t *next_btn = nullptr;
lv_obj_t *sys_info = nullptr;
lv_style_t style_btn;
lv_style_t style_btn_pressed;

// Upload variables
#define UPLOAD_BUFFER_SIZE 4096
#define SDMMC_FREQ_PROBING 125000
#define RETRY_DELAY 1000
#define SD_POWER_STABILIZE_DELAY 2000
bool upload_active = false;
uint32_t upload_start_time = 0;
uint32_t upload_size = 0;
char upload_filename[64];

// Mutex for thread safety
SemaphoreHandle_t sd_mutex = NULL;
SemaphoreHandle_t jpeg_mutex = NULL;
SemaphoreHandle_t ui_mutex = NULL;

// Control flags
volatile bool wdt_enabled = false;
volatile bool reset_requested = false;

// Task handles
TaskHandle_t lvgl_task_handle = NULL;
TaskHandle_t main_task_handle = NULL;

// Function declarations (keeping your existing ones)
bool initializeFileSystem();
bool checkSPIFFSPartition();
void diagnosePartitions();
void safeLoggerInit(bool spiffsAvailable);
bool initializeWithRetry(const char *componentName, bool (*initFunction)(), int maxRetries);
TaskHandle_t createTaskSafely(TaskFunction_t taskFunction, const char *name, uint32_t stackSize, void *param, UBaseType_t priority, BaseType_t coreID);
bool setup_watchdog();
void feed_watchdog();
bool setup_lcd();
bool setup_touch();
bool setup_sd_card_reliable();
bool setup_wifi_ap();
bool setup_lvgl();
bool setup_ui();
bool setup_webserver();
void lvgl_task(void *pvParameters);
void slideshowTask(void *parameter);
void scan_images();
void display_image(int index);
bool isValidJPEG(const char *filename);
int jpeg_draw_callback(JPEGDRAW *pDraw);
void clearLCDScreen();
bool render_jpeg_file(const char *filename);
void displayErrorMessage(const char *message);
void updateImageCounter(int index);
void updateStatusAfterImageLoad(int index, bool success);
void startSlideshow(uint32_t interval_ms);
void stopSlideshow();
void toggleSlideshow(uint32_t interval_ms = 5000);
void handle_upload();
void handle_image();
void handle_info();
void handle_test();
void add_test_page_handler();
bool deleteImageFile(const char *filename);
void handle_delete();
void handle_api_delete();
void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
void lvgl_touch_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
void prev_btn_event_cb(lv_event_t *e);
void next_btn_event_cb(lv_event_t *e);
void play_pause_btn_event_cb(lv_event_t *e);
uint32_t max_u32(uint32_t a, uint32_t b);
uint32_t min_u32(uint32_t a, uint32_t b);

// Helper functions
uint32_t max_u32(uint32_t a, uint32_t b) { return (a > b) ? a : b; }
uint32_t min_u32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

// Your existing file system and initialization functions (keeping them as they are working)
bool checkSPIFFSPartition() {
    Serial.println("Checking SPIFFS partition availability...");
    
    const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    
    if (partition == NULL) {
        Serial.println("ERROR: SPIFFS 'storage' partition not found in partition table");
        return false;
    }
    
    Serial.printf("Found SPIFFS partition: %s\n", partition->label);
    Serial.printf("  Address: 0x%08X\n", partition->address);
    Serial.printf("  Size: %u bytes (%.1f KB)\n", partition->size, partition->size / 1024.0);
    Serial.printf("  Type: %d, Subtype: %d\n", partition->type, partition->subtype);
    
    return true;
}

bool initializeFileSystem() {
    Serial.println("Initializing file system with enhanced partition checking...");
    
    if (!checkSPIFFSPartition()) {
        Serial.println("SPIFFS partition check failed - trying LittleFS alternatives");
        
        Serial.println("Trying LittleFS on 'storage' partition...");
        if (LittleFS.begin(false, "storage")) {
            Serial.println("LittleFS mounted successfully on 'storage' partition");
            return true;
        }
        
        Serial.println("Trying LittleFS with default partition...");
        if (LittleFS.begin(false)) {
            Serial.println("LittleFS mounted successfully with default partition");
            return true;
        }
        
        Serial.println("All file system initialization attempts failed");
        return false;
    }
    
    Serial.println("SPIFFS partition found - attempting to mount...");
    
    if (SPIFFS.begin(false)) {
        Serial.println("SPIFFS mounted successfully on existing partition");
        
        File testFile = SPIFFS.open("/test.txt", FILE_WRITE);
        if (testFile) {
            testFile.println("SPIFFS test");
            testFile.close();
            SPIFFS.remove("/test.txt");
            Serial.println("SPIFFS write test successful");
            return true;
        } else {
            Serial.println("SPIFFS write test failed - filesystem may be corrupted");
        }
    } else {
        Serial.println("SPIFFS mount failed - partition may need formatting");
    }
    
    Serial.println("Attempting to format SPIFFS partition...");
    
    SPIFFS.end();
    delay(100);
    
    if (SPIFFS.format()) {
        Serial.println("SPIFFS format successful - attempting mount...");
        
        if (SPIFFS.begin(false)) {
            Serial.println("SPIFFS formatted and mounted successfully");
            
            File testFile = SPIFFS.open("/test.txt", FILE_WRITE);
            if (testFile) {
                testFile.println("SPIFFS format test");
                testFile.close();
                SPIFFS.remove("/test.txt");
                Serial.println("SPIFFS format verification successful");
                return true;
            } else {
                Serial.println("SPIFFS format verification failed");
            }
        } else {
            Serial.println("SPIFFS mount failed after format");
        }
    } else {
        Serial.println("SPIFFS format failed");
    }
    
    Serial.println("SPIFFS failed completely - trying LittleFS fallback...");
    
    Serial.println("Trying LittleFS on 'storage' partition...");
    if (LittleFS.begin(false, "storage")) {
        Serial.println("LittleFS mounted successfully on 'storage' partition");
        return true;
    }
    
    Serial.println("Trying LittleFS with default partition...");
    if (LittleFS.begin(false)) {
        Serial.println("LittleFS mounted successfully with default partition");
        return true;
    }
    
    Serial.println("Trying to format and mount LittleFS...");
    if (LittleFS.format()) {
        if (LittleFS.begin(false)) {
            Serial.println("LittleFS formatted and mounted successfully");
            return true;
        }
    }
    
    Serial.println("All file system initialization attempts failed - continuing without file system");
    return false;
}

void safeLoggerInit(bool fsAvailable) {
    try {
        if (fsAvailable) {
            bool spiffsWorking = false;
            bool littlefsWorking = false;
            
            if (SPIFFS.begin(false)) {
                File testFile = SPIFFS.open("/logger_test.txt", FILE_WRITE);
                if (testFile) {
                    testFile.println("Logger test");
                    testFile.close();
                    SPIFFS.remove("/logger_test.txt");
                    spiffsWorking = true;
                    Serial.println("SPIFFS is working for logger");
                }
            }
            
            if (!spiffsWorking) {
                if (LittleFS.begin(false, "storage") || LittleFS.begin(false)) {
                    File testFile = LittleFS.open("/logger_test.txt", FILE_WRITE);
                    if (testFile) {
                        testFile.println("Logger test");
                        testFile.close();
                        LittleFS.remove("/logger_test.txt");
                        littlefsWorking = true;
                        Serial.println("LittleFS is working for logger");
                    }
                }
            }
            
            if (spiffsWorking) {
                Logger.init(true, true, LOG_LEVEL_DEBUG);
                Logger.enableFlashOutput(true, &SPIFFS, "/syslog.txt");
                Serial.println("Logger using SPIFFS for flash output");
            } else if (littlefsWorking) {
                Logger.init(true, true, LOG_LEVEL_DEBUG);
                Logger.enableFlashOutput(true, &LittleFS, "/syslog.txt");
                Serial.println("Logger using LittleFS for flash output");
            } else {
                Logger.init(true, false, LOG_LEVEL_DEBUG);
                Serial.println("Logger using serial-only output - no working filesystem");
            }
        } else {
            Logger.init(true, false, LOG_LEVEL_DEBUG);
            Serial.println("Logger using serial-only output - filesystem unavailable");
        }
        
        Logger.setLogLevel(LOG_LEVEL_DEBUG);
        Logger.enableSerialOutput(true);
    } catch (...) {
        Serial.println("Logger initialization failed, using serial-only logging");
        
        try {
            Logger.init(true, false, LOG_LEVEL_DEBUG);
            Logger.setLogLevel(LOG_LEVEL_DEBUG);
            Logger.enableSerialOutput(true);
        } catch (...) {
            Serial.println("CRITICAL: Logger completely failed to initialize");
        }
    }
}

void diagnosePartitions() {
    Serial.println("\n=== Partition Diagnosis ===");
    
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it != NULL) {
        const esp_partition_t* part = esp_partition_get(it);
        Serial.printf("Partition: %s\n", part->label);
        Serial.printf("  Type: %d, Subtype: %d\n", part->type, part->subtype);
        Serial.printf("  Address: 0x%08X, Size: %u bytes\n", part->address, part->size);
        
        if (strcmp(part->label, "storage") == 0) {
            Serial.println("  >>> This is our target storage partition");
            
            uint8_t buffer[16];
            esp_err_t err = esp_partition_read(part, 0, buffer, sizeof(buffer));
            if (err == ESP_OK) {
                Serial.print("  First 16 bytes: ");
                for (int i = 0; i < 16; i++) {
                    Serial.printf("%02X ", buffer[i]);
                }
                Serial.println();
                
                if (buffer[0] == 0x10 && buffer[1] == 0xFA) {
                    Serial.println("  >>> Appears to contain SPIFFS data");
                } else if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
                    Serial.println("  >>> Appears to be erased/unformatted");
                } else {
                    Serial.println("  >>> Contains unknown data format");
                }
            } else {
                Serial.printf("  Failed to read partition: %d\n", err);
            }
        }
        
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    Serial.println("=== End Partition Diagnosis ===\n");
}

// Keep all your existing component setup functions as they are working perfectly
bool setup_watchdog() {
    Logger.info("Setting up watchdog with critical error protection...");
    
    esp_task_wdt_deinit();
    delay(100);
    
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = false
    };
    
    esp_err_t err = esp_task_wdt_init(&wdt_config);
    if (err != ESP_OK) {
        Logger.error("Watchdog init failed: %d - continuing without WDT", err);
        wdt_enabled = false;
        return false;
    }
    
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (currentTask != NULL) {
        esp_err_t add_err = esp_task_wdt_add(currentTask);
        if (add_err != ESP_OK) {
            Logger.warn("Could not add main task to WDT: %d - continuing anyway", add_err);
        } else {
            Logger.info("Main task subscribed to watchdog");
        }
    }
    
    main_task_handle = currentTask;
    wdt_enabled = (err == ESP_OK);
    
    Logger.info("Watchdog initialized with %d second timeout, panic mode: OFF", 
               wdt_config.timeout_ms / 1000);
    
    return (err == ESP_OK);
}

void feed_watchdog() {
    if (wdt_enabled && main_task_handle) {
        esp_task_wdt_reset();
    }
}

bool initializeWithRetry(const char *componentName, bool (*initFunction)(), int maxRetries = 3) {
    Logger.info("Initializing %s with up to %d attempts...", componentName, maxRetries);
    
    bool success = false;
    for (int retry = 0; retry < maxRetries; retry++) {
        success = initFunction();
        if (success) {
            Logger.info("%s initialized successfully on attempt %d", componentName, retry + 1);
            break;
        } else {
            Logger.warn("%s initialization failed, attempt %d/%d", 
                      componentName, retry + 1, maxRetries);
            delay(500 * (retry + 1));
        }
    }
    
    if (!success) {
        Logger.error("%s initialization failed after %d attempts", componentName, maxRetries);
    }
    
    return success;
}

TaskHandle_t createTaskSafely(TaskFunction_t taskFunction, const char *name,
                              uint32_t stackSize, void *param, UBaseType_t priority,
                              BaseType_t coreID) {
    TaskHandle_t handle = NULL;
    
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,
        name,
        stackSize,
        param,
        priority,
        &handle,
        coreID);
    
    if (result != pdPASS || handle == NULL) {
        Logger.error("Failed to create task: %s", name);
        return NULL;
    }
    
    Logger.info("Created task: %s on core %d", name, coreID);
    return handle;
}

// Keep all your existing component setup functions (LCD, Touch, SD, WiFi, LVGL, UI)
bool setup_lcd() {
    Logger.info("Setting up LCD...");
    
    lcd_ready = false;
    
    try {
        lcd.begin();
        lcd.example_bsp_set_lcd_backlight(255);
        
        Logger.info("LCD initialized successfully with backlight ON");
        lcd_ready = true;
    } catch (...) {
        Logger.error("Exception during LCD initialization");
        lcd_ready = false;
    }
    
    return lcd_ready;
}

bool setup_touch() {
    Logger.info("Setting up Touch...");
    
    touch_ready = false;
    
    try {
        touch.begin();
        Logger.info("Touch initialized successfully");
        touch_ready = true;
    } catch (...) {
        Logger.error("Exception during touch initialization");
        touch_ready = false;
    }
    
    return touch_ready;
}

bool setup_sd_card_reliable() {
    Logger.info("Setting up SD Card with robust initialization...");
    
    Logger.info("Waiting for power stabilization...");
    delay(SD_POWER_STABILIZE_DELAY);
    
    Logger.info("Resetting SD pins...");
    gpio_reset_pin((gpio_num_t)SDMMC_CLK_PIN);
    gpio_reset_pin((gpio_num_t)SDMMC_CMD_PIN);
    gpio_reset_pin((gpio_num_t)SDMMC_D0_PIN);
    gpio_reset_pin((gpio_num_t)SDMMC_D1_PIN);
    gpio_reset_pin((gpio_num_t)SDMMC_D2_PIN);
    gpio_reset_pin((gpio_num_t)SDMMC_D3_PIN);
    delay(100);
    
    Logger.info("Ensuring SD_MMC is deinitialized...");
    SD_MMC.end();
    delay(500);
    
    Logger.info("Configuring SD pins...");
    SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
    
    Logger.info("SD init attempt 1: 1-bit mode at 125KHz (ultra conservative)");
    if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING, 5)) {
        Logger.error("SD Card initialization failed");
        return false;
    }
    
    Logger.info("Validating SD card...");
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Logger.error("No SD card attached");
        return false;
    }
    
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Logger.info("SD Card Type: %s", (cardType == CARD_MMC) ? "MMC" : 
                (cardType == CARD_SD) ? "SDSC" : 
                (cardType == CARD_SDHC) ? "SDHC" : "UNKNOWN");
    Logger.info("SD Card Size: %lluMB", cardSize);
    
    if (!SD_MMC.exists("/images")) {
        if (SD_MMC.mkdir("/images")) {
            Logger.info("Created /images directory");
        } else {
            Logger.warn("Failed to create /images directory");
        }
    }
    
    Logger.info("SD card initialized successfully in ultra-conservative mode");
    return true;
}

bool setup_wifi_ap() {
    Logger.info("Setting up WiFi in AP mode with maximum reliability...");
    
    WiFi.disconnect(true);
    delay(100);
    
    WiFi.mode(WIFI_AP);
    delay(100);
    
    Logger.info("Starting AP with SSID: %s", WIFI_AP_SSID);
    Logger.info("AP start attempt 1...");
    
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_CHANNEL, 0, MAX_WIFI_CLIENTS)) {
        Logger.error("Failed to start WiFi AP");
        return false;
    }
    
    delay(200);
    
    if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
        Logger.error("Failed to configure WiFi AP");
        return false;
    }
    
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(84);
    
    Logger.info("WiFi AP Mode active with enhanced stability");
    Logger.info("AP SSID: %s", WIFI_AP_SSID);
    Logger.info("AP IP Address: %s", WiFi.softAPIP().toString().c_str());
    Logger.info("AP Channel: %d", WIFI_CHANNEL);
    Logger.info("AP Max Clients: %d", MAX_WIFI_CLIENTS);
    Logger.info("AP Power: Maximum");
    
    return true;
}

void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    if (lcd_ready) {
        lcd.lcd_draw_bitmap(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p);
    }
    lv_disp_flush_ready(disp_drv);
}

void lvgl_touch_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    if (touch_ready) {
        uint16_t x, y;
        bool touched = touch.getTouch(&x, &y);
        
        if (touched) {
            data->point.x = x;
            data->point.y = y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

bool setup_lvgl() {
    Logger.info("Setting up LVGL...");
    
    lv_init();
    
    size_t buffer_size = LCD_WIDTH * 80;
    Logger.debug("Allocating LVGL buffers: %d bytes each", buffer_size * sizeof(lv_color_t));
    
    buf = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf1 = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    
    if (!buf || !buf1) {
        Logger.error("Failed to allocate LVGL buffers");
        return false;
    }
    
    lv_disp_draw_buf_init(&draw_buf, buf, buf1, buffer_size);
    
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
    
    Logger.info("LVGL initialized successfully");
    return true;
}

void prev_btn_event_cb(lv_event_t *e) {
    if (image_count > 0) {
        current_image = (current_image - 1 + image_count) % image_count;
        display_image(current_image);
    }
}

void next_btn_event_cb(lv_event_t *e) {
    if (image_count > 0) {
        current_image = (current_image + 1) % image_count;
        display_image(current_image);
    }
}

void play_pause_btn_event_cb(lv_event_t *e) {
    toggleSlideshow(5000);
}

bool setup_ui() {
    Logger.info("Setting up UI...");
    
    main_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(main_screen, lv_color_black(), 0);
    lv_scr_load(main_screen);
    
    image_view = lv_obj_create(main_screen);
    lv_obj_set_size(image_view, LCD_WIDTH, LCD_HEIGHT - 120);
    lv_obj_set_pos(image_view, 0, 60);
    lv_obj_set_style_bg_color(image_view, lv_color_black(), 0);
    lv_obj_set_style_border_width(image_view, 0, 0);
    lv_obj_set_style_pad_all(image_view, 0, 0);
    
    image_display = lv_img_create(image_view);
    lv_obj_center(image_display);
    
    status_bar = lv_obj_create(main_screen);
    lv_obj_set_size(status_bar, LCD_WIDTH, 60);
    lv_obj_set_pos(status_bar, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    
    status_label = lv_label_create(status_bar);
    lv_label_set_text(status_label, "ESP32P4 Image Gallery");
    lv_obj_set_pos(status_label, 10, 5);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    
    wifi_label = lv_label_create(status_bar);
    lv_label_set_text(wifi_label, "WiFi: AP Mode");
    lv_obj_set_pos(wifi_label, 10, 30);
    lv_obj_set_style_text_color(wifi_label, lv_color_hex(0x00FF00), 0);
    
    image_counter = lv_label_create(status_bar);
    lv_label_set_text(image_counter, "0/0");
    lv_obj_align(image_counter, LV_ALIGN_TOP_RIGHT, -10, 15);
    lv_obj_set_style_text_color(image_counter, lv_color_white(), 0);
    
    control_bar = lv_obj_create(main_screen);
    lv_obj_set_size(control_bar, LCD_WIDTH, 60);
    lv_obj_set_pos(control_bar, 0, LCD_HEIGHT - 60);
    lv_obj_set_style_bg_color(control_bar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(control_bar, 0, 0);
    
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, 8);
    lv_style_set_bg_color(&style_btn, lv_color_hex(0x555555));
    lv_style_set_text_color(&style_btn, lv_color_white());
    lv_style_set_pad_all(&style_btn, 8);
    
    lv_style_init(&style_btn_pressed);
    lv_style_set_radius(&style_btn_pressed, 8);
    lv_style_set_bg_color(&style_btn_pressed, lv_color_hex(0x777777));
    lv_style_set_text_color(&style_btn_pressed, lv_color_white());
    lv_style_set_pad_all(&style_btn_pressed, 8);
    
    prev_btn = lv_btn_create(control_bar);
    lv_obj_set_size(prev_btn, 80, 40);
    lv_obj_set_pos(prev_btn, 20, 10);
    lv_obj_add_style(prev_btn, &style_btn, 0);
    lv_obj_add_style(prev_btn, &style_btn_pressed, LV_STATE_PRESSED);
    lv_obj_add_event_cb(prev_btn, prev_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *prev_label = lv_label_create(prev_btn);
    lv_label_set_text(prev_label, "Prev");
    lv_obj_center(prev_label);
    
    next_btn = lv_btn_create(control_bar);
    lv_obj_set_size(next_btn, 80, 40);
    lv_obj_set_pos(next_btn, 120, 10);
    lv_obj_add_style(next_btn, &style_btn, 0);
    lv_obj_add_style(next_btn, &style_btn_pressed, LV_STATE_PRESSED);
    lv_obj_add_event_cb(next_btn, next_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *next_label = lv_label_create(next_btn);
    lv_label_set_text(next_label, "Next");
    lv_obj_center(next_label);
    
    Logger.info("UI initialized successfully");
    return true;
}

// Keep all your existing JPEG and image handling functions as they work perfectly
int jpeg_draw_callback(JPEGDRAW *pDraw) {
    if (!lcd_ready || !pDraw || !pDraw->pPixels) {
        return 0;
    }
    
    static uint32_t lastDrawTime = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastDrawTime < 5) {
        delay(5);
    }
    
    if (pDraw->iWidth <= 0 || pDraw->iHeight <= 0 ||
        pDraw->iWidth > 1280 || pDraw->iHeight > 1280) {
        Logger.error("Invalid JPEG draw dimensions: %dx%d", pDraw->iWidth, pDraw->iHeight);
        return 0;
    }
    
    int imgWidth = jpeg.getWidth();
    int imgHeight = jpeg.getHeight();
    
    if (imgWidth <= 0 || imgHeight <= 0 || imgWidth > 5000 || imgHeight > 5000) {
        Logger.error("Invalid JPEG dimensions: %dx%d", imgWidth, imgHeight);
        return 0;
    }
    
    bool needsRotation = image_needs_rotation;
    bool drawFailed = false;
    
    if (!needsRotation) {
        int centerX = (lcd.width() - imgWidth) / 2;
        int centerY = (lcd.height() - imgHeight) / 2;
        
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        int x = pDraw->x + centerX;
        int y = pDraw->y + centerY;
        
        if (x >= 0 && y >= 0 && 
            x + pDraw->iWidth <= lcd.width() && 
            y + pDraw->iHeight <= lcd.height()) {
            
            try {
                lcd.lcd_draw_bitmap(x, y, 
                                   x + pDraw->iWidth, 
                                   y + pDraw->iHeight, 
                                   (uint16_t*)pDraw->pPixels);
            } catch (...) {
                Logger.error("Exception in direct LCD drawing");
                drawFailed = true;
            }
        } else {
            Logger.warn("Draw coordinates out of bounds: x=%d, y=%d, w=%d, h=%d", 
                      x, y, pDraw->iWidth, pDraw->iHeight);
            drawFailed = true;
        }
    } else {
        // Handle rotation for landscape images on portrait display
        int rotatedWidth = imgHeight;
        int rotatedHeight = imgWidth;
        
        int centerX = (lcd.width() - rotatedWidth) / 2;
        int centerY = (lcd.height() - rotatedHeight) / 2;
        
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        int rotX = centerX + pDraw->y;
        int rotY = centerY + (imgWidth - pDraw->x - pDraw->iWidth);
        
        int rotWidth = pDraw->iHeight;
        int rotHeight = pDraw->iWidth;
        
        if (rotWidth <= 0 || rotHeight <= 0 || rotWidth * rotHeight > 100000) {
            Logger.error("Invalid rotation dimensions: %dx%d", rotWidth, rotHeight);
            return 0;
        }
        
        uint16_t* rotatedBlock = nullptr;
        bool useStack = false;
        uint16_t stackBuffer[64];
        
        if (rotWidth * rotHeight <= 64) {
            rotatedBlock = stackBuffer;
            useStack = true;
        } else {
            rotatedBlock = (uint16_t*)ps_calloc(rotWidth * rotHeight, sizeof(uint16_t));
            
            if (!rotatedBlock) {
                rotatedBlock = (uint16_t*)calloc(rotWidth * rotHeight, sizeof(uint16_t));
                
                if (!rotatedBlock) {
                    Logger.error("Failed to allocate rotation buffer (%d bytes)",
                              rotWidth * rotHeight * sizeof(uint16_t));
                    return 0;
                }
            }
        }
        
        if (!rotatedBlock) {
            Logger.error("Rotation buffer is null despite checks");
            return 0;
        }
        
        uint16_t* pixels = (uint16_t*)pDraw->pPixels;
        for (int y = 0; y < pDraw->iHeight; y++) {
            for (int x = 0; x < pDraw->iWidth; x++) {
                int newX = y;
                int newY = pDraw->iWidth - 1 - x;
                
                if (newX >= 0 && newX < rotWidth && 
                    newY >= 0 && newY < rotHeight &&
                    y < pDraw->iHeight && x < pDraw->iWidth &&
                    y * pDraw->iWidth + x < pDraw->iWidth * pDraw->iHeight) {
                    
                    rotatedBlock[newY * rotWidth + newX] = pixels[y * pDraw->iWidth + x];
                }
            }
        }
        
        if (rotX >= 0 && rotY >= 0 && 
            rotX + rotWidth <= lcd.width() && 
            rotY + rotHeight <= lcd.height()) {
                
            try {
                lcd.lcd_draw_bitmap(rotX, rotY, 
                                  rotX + rotWidth, 
                                  rotY + rotHeight, 
                                  rotatedBlock);
            } catch (...) {
                Logger.error("Exception in rotated LCD drawing");
                drawFailed = true;
            }
        } else {
            Logger.warn("Rotated draw coordinates out of bounds: x=%d, y=%d, w=%d, h=%d", 
                      rotX, rotY, rotWidth, rotHeight);
            drawFailed = true;
        }
        
        if (!useStack && rotatedBlock) {
            free(rotatedBlock);
            rotatedBlock = nullptr;
        }
    }
    
    lastDrawTime = currentTime;
    
    return drawFailed ? 0 : 1;
}

void clearLCDScreen() {
    uint32_t start_time = millis();
    
    if (lcd_ready) {
        lcd.fillScreen(0x0000);
    }
    
    uint32_t clear_time = millis() - start_time;
    Logger.debug("Screen cleared in %u ms", clear_time);
}

bool render_jpeg_file(const char *filename) {
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or invalid filename");
        return false;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    if (!isValidJPEG(filename)) {
        Logger.error("JPEG validation failed: %s", filename);
        return false;
    }
    
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Logger.error("Failed to take SD mutex for JPEG loading");
        return false;
    }
    
    File jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
        Logger.error("Failed to open JPEG file: %s", filename);
        xSemaphoreGive(sd_mutex);
        return false;
    }
    
    size_t fileSize = jpegFile.size();
    if (fileSize < 1024) {
        Logger.error("File too small: %s (%d bytes)", filename, fileSize);
        jpegFile.close();
        xSemaphoreGive(sd_mutex);
        return false;
    }
    
    Logger.info("Starting JPEG decoding for %s (%u bytes)", filename, fileSize);
    
    image_needs_rotation = false;
    
    Logger.info("Decoding image at full resolution");
    
    const char *errorMsg = nullptr;
    bool decodeFailed = false;
    
    if (jpeg.open(jpegFile, jpeg_draw_callback)) {
        int imgWidth = jpeg.getWidth();
        int imgHeight = jpeg.getHeight();
        
        if (imgWidth <= 0 || imgHeight <= 0) {
            Logger.error("Invalid image dimensions: %dx%d", imgWidth, imgHeight);
            jpeg.close();
            jpegFile.seek(0);
            decodeFailed = true;
        } else {
            image_needs_rotation = (imgWidth > imgHeight);
            
            if (jpeg.decode(0, 0, 0)) {
                if (image_needs_rotation) {
                    Logger.info("JPEG decoded successfully (rotated): %dx%d -> %dx%d",
                                imgWidth, imgHeight, imgHeight, imgWidth);
                } else {
                    Logger.info("JPEG decoded successfully: %dx%d", imgWidth, imgHeight);
                }
                success = true;
            } else {
                Logger.warn("Full resolution decode failed, trying half resolution");
                decodeFailed = true;
                errorMsg = "Full resolution decode failed";
            }
        }
    } else {
        Logger.error("Failed to open JPEG for decoding");
        decodeFailed = true;
        errorMsg = "Failed to open JPEG decoder";
    }
    
    if (decodeFailed) {
        jpeg.close();
        jpegFile.seek(0);
        decodeFailed = false;
        
        if (jpeg.open(jpegFile, jpeg_draw_callback)) {
            if (jpeg.decode(0, 0, JPEG_SCALE_HALF)) {
                Logger.info("JPEG decoded successfully at half resolution");
                success = true;
            } else {
                Logger.warn("Half resolution decode failed, trying quarter resolution");
                decodeFailed = true;
                errorMsg = "Half resolution decode failed";
            }
        }
    }
    
    if (decodeFailed) {
        jpeg.close();
        jpegFile.seek(0);
        
        if (jpeg.open(jpegFile, jpeg_draw_callback)) {
            if (jpeg.decode(0, 0, JPEG_SCALE_QUARTER)) {
                Logger.info("JPEG decoded successfully at quarter resolution");
                success = true;
            } else {
                Logger.error("JPEG decode failed at all resolutions");
                errorMsg = "All resolution attempts failed";
            }
        }
    }
    
    jpeg.close();
    jpegFile.close();
    xSemaphoreGive(sd_mutex);
    
    uint32_t decodeTime = millis() - startTime;
    Logger.info("JPEG decoding took %lu ms, result: %s", 
               decodeTime, success ? "success" : "failed");
    
    if (!success && errorMsg) {
        Logger.error("JPEG decode error: %s for file: %s", errorMsg, filename);
    }
    
    return success;
}

bool isValidJPEG(const char *filename) {
    if (!sd_ready || !filename) return false;
    
    File file = SD_MMC.open(filename, FILE_READ);
    if (!file) return false;
    
    size_t fileSize = file.size();
    if (fileSize < 100) {
        file.close();
        return false;
    }
    
    uint8_t header[4];
    if (file.read(header, 4) != 4) {
        file.close();
        return false;
    }
    
    bool isJPEG = (header[0] == 0xFF && header[1] == 0xD8);
    
    if (isJPEG) {
        file.seek(fileSize - 2);
        uint8_t footer[2];
        if (file.read(footer, 2) == 2) {
            if (footer[0] != 0xFF || footer[1] != 0xD9) {
                Logger.warn("JPEG missing EOI marker - may be corrupted: %s", filename);
            }
        }
    }
    
    file.close();
    return isJPEG;
}

void scan_images() {
    Logger.info("Scanning for images with enhanced validation...");
    
    for (int i = 0; i < image_count; i++) {
        if (image_list[i]) {
            free(image_list[i]);
            image_list[i] = nullptr;
        }
    }
    image_count = 0;
    
    if (!sd_ready) {
        Logger.warn("SD card not ready for image scanning");
        return;
    }
    
    File root = SD_MMC.open("/images");
    if (!root || !root.isDirectory()) {
        Logger.warn("Images directory not found or not accessible");
        return;
    }
    
    Logger.info("Phase 1: Identifying valid and corrupt images...");
    
    File file = root.openNextFile();
    while (file && image_count < MAX_IMAGES) {
        if (!file.isDirectory()) {
            String filename = file.name();
            String fullPath = "/images/" + filename;
            
            if (filename.endsWith(".jpg") || filename.endsWith(".jpeg") || 
                filename.endsWith(".JPG") || filename.endsWith(".JPEG")) {
                
                if (isValidJPEG(fullPath.c_str())) {
                    image_list[image_count] = (char*)malloc(fullPath.length() + 1);
                    if (image_list[image_count]) {
                        strcpy(image_list[image_count], fullPath.c_str());
                        Logger.debug("Found image %d: %s (%u bytes)", 
                                    image_count, fullPath.c_str(), file.size());
                        image_count++;
                    }
                }
            }
        }
        file = root.openNextFile();
    }
    
    root.close();
    
    Logger.info("Found %d valid images, removed 0 corrupted files", image_count);
    Logger.info("Found %d images", image_count);
}

void display_image(int index) {
    if (!sd_ready || index < 0 || index >= image_count) {
        Logger.error("Cannot display image - invalid index %d or SD not ready", index);
        return;
    }
    
    static SemaphoreHandle_t displaySemaphore = NULL;
    if (displaySemaphore == NULL) {
        displaySemaphore = xSemaphoreCreateMutex();
    }
    
    if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
        Logger.warn("Image loading already in progress - ignoring request");
        return;
    }
    
    feed_watchdog();
    
    image_loading = true;
    
    uint8_t originalBrightness = 255;
    
    current_image = index;
    
    Logger.info("Displaying image %d: %s", index, image_list[index]);
    updateImageCounter(index);
    
    lcd.example_bsp_set_lcd_backlight(50);
    
    clearLCDScreen();
    
    char localFilename[128] = {0};
    strncpy(localFilename, image_list[index], sizeof(localFilename) - 1);
    
    bool isValid = isValidJPEG(localFilename);
    if (!isValid) {
        Logger.error("Image file failed validation check: %s", localFilename);
        displayErrorMessage("Image failed validation");
        
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        updateStatusAfterImageLoad(index, false);
        
        image_loading = false;
        xSemaphoreGive(displaySemaphore);
        return;
    }
    
    delay(50);
    
    if (xSemaphoreTake(jpeg_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Logger.error("Failed to take JPEG mutex for display");
        displayErrorMessage("Resource contention error");
        
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        updateStatusAfterImageLoad(index, false);
        
        image_loading = false;
        xSemaphoreGive(displaySemaphore);
        return;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    try {
        success = render_jpeg_file(localFilename);
        
        if (!success) {
            Logger.error("Failed to render image %d: %s", index, localFilename);
        }
    } catch (...) {
        Logger.error("Exception during JPEG rendering");
        success = false;
    }
    
    xSemaphoreGive(jpeg_mutex);
    
    uint32_t renderTime = millis() - startTime;
    if (renderTime > 500) {
        Logger.warn("JPEG rendering took %lu ms", renderTime);
    } else {
        Logger.debug("JPEG rendering took %lu ms", renderTime);
    }
    
    if (success) {
        delay(20);
        
        const int fadeStep = 10;
        const int fadeDelay = 10;
        
        for (int brightness = 50; brightness <= originalBrightness; brightness += fadeStep) {
            lcd.example_bsp_set_lcd_backlight(brightness);
            delay(fadeDelay);
        }
        
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        
        Logger.info("Successfully displayed image %d", index);
    } else {
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        displayErrorMessage("Image could not be displayed");
        Logger.error("Failed to display image %d", index);
    }
    
    updateStatusAfterImageLoad(index, success);
    
    image_loading = false;
    xSemaphoreGive(displaySemaphore);
    feed_watchdog();
}

void displayErrorMessage(const char *message) {
    if (lcd_ready) {
        clearLCDScreen();
        Logger.error("Display error: %s", message);
    }
}

void updateImageCounter(int index) {
    if (image_counter && image_count > 0) {
        char counter_text[32];
        snprintf(counter_text, sizeof(counter_text), "%d/%d", index + 1, image_count);
        lv_label_set_text(image_counter, counter_text);
    }
}

void updateStatusAfterImageLoad(int index, bool success) {
    if (status_label) {
        if (success) {
            lv_label_set_text(status_label, "Image loaded successfully");
        } else {
            lv_label_set_text(status_label, "Failed to load image");
        }
    }
}

// Slideshow functions
void slideshowTask(void *parameter) {
    Logger.info("Slideshow task started");
    
    while (slideshow_active) {
        if (image_count > 1) {
            uint32_t now = millis();
            if (now - slideshow_last_change >= slideshow_interval) {
                current_image = (current_image + 1) % image_count;
                display_image(current_image);
                slideshow_last_change = now;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        feed_watchdog();
    }
    
    Logger.info("Slideshow task ended");
    slideshow_task_handle = NULL;
    vTaskDelete(NULL);
}

void startSlideshow(uint32_t interval_ms) {
    if (image_count <= 1) {
        Logger.warn("Cannot start slideshow: need at least 2 images");
        return;
    }
    
    slideshow_interval = interval_ms;
    slideshow_active = true;
    slideshow_last_change = millis();
    
    if (slideshow_task_handle == NULL) {
        xTaskCreatePinnedToCore(
            slideshowTask,
            "Slideshow Task",
            4096,
            NULL,
            1,
            &slideshow_task_handle,
            0
        );
        Logger.info("Created task: Slideshow Task on core 0");
    }
    
    Logger.info("Slideshow started with %u ms interval", interval_ms);
}

void stopSlideshow() {
    slideshow_active = false;
    Logger.info("Slideshow stopped");
}

void toggleSlideshow(uint32_t interval_ms) {
    if (slideshow_active) {
        stopSlideshow();
    } else {
        startSlideshow(interval_ms);
    }
}


/**
 * FIXED: Complete web server setup with ALL missing API endpoints
 * Updated: 2025-06-22 09:45:00
 * User: Chamil1983
 */
bool setup_webserver() {
    Logger.info("Setting up WebServer with enhanced connectivity...");
    
    try {
        server.enableCORS(true);
        server.enableCrossOrigin(true);
        
        // Root route handler with creative interface (keeping your existing HTML)
        server.on("/", HTTP_GET, []() {
            Logger.info("Serving main page");
            
            String response = "<!DOCTYPE html><html><head>";
            response += "<meta charset='UTF-8'>";
            response += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
            response += "<title>ESP32P4 Gallery</title>";
            response += "<style>";
            response += "body {font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: #ffffff; padding: 0; margin: 0; min-height: 100vh;}";
            response += ".container {max-width: 1200px; margin: 0 auto; padding: 20px;}";
            response += ".header {text-align: center; padding: 30px 0; background: rgba(255,255,255,0.1); border-radius: 15px; margin-bottom: 30px; backdrop-filter: blur(10px);}";
            response += ".header h1 {font-size: 2.5em; margin: 0; text-shadow: 2px 2px 4px rgba(0,0,0,0.3);}";
            response += ".header p {font-size: 1.2em; margin: 10px 0 0 0; opacity: 0.9;}";
            response += ".card {background: rgba(255,255,255,0.15); padding: 25px; border-radius: 15px; margin-bottom: 25px; backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2);}";
            response += ".card h2 {color: #ffffff; margin-top: 0; font-size: 1.8em; text-shadow: 1px 1px 2px rgba(0,0,0,0.3);}";
            response += ".btn {background: linear-gradient(45deg, #FF6B6B, #4ECDC4); color: white; border: none; padding: 12px 25px; border-radius: 25px; cursor: pointer; font-size: 16px; font-weight: bold; text-decoration: none; display: inline-block; margin: 8px; transition: all 0.3s ease; box-shadow: 0 4px 15px rgba(0,0,0,0.2);}";
            response += ".btn:hover {transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0,0,0,0.3);}";
            response += ".btn-secondary {background: linear-gradient(45deg, #667eea, #764ba2);}";
            response += ".btn-danger {background: linear-gradient(45deg, #FF416C, #FF4B2B);}";
            response += ".gallery {display: grid; grid-template-columns: repeat(auto-fill, minmax(250px, 1fr)); gap: 20px; margin: 25px 0;}";
            response += ".thumbnail {width: 100%; height: 200px; object-fit: cover; border-radius: 10px; cursor: pointer; transition: all 0.3s ease; box-shadow: 0 4px 15px rgba(0,0,0,0.2);}";
            response += ".thumbnail:hover {transform: scale(1.05); box-shadow: 0 8px 25px rgba(0,0,0,0.3);}";
            response += ".image-item {background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; text-align: center; transition: all 0.3s ease;}";
            response += ".image-item:hover {background: rgba(255,255,255,0.2);}";
            response += ".controls {display: flex; justify-content: center; gap: 15px; margin: 25px 0; flex-wrap: wrap;}";
            response += ".status {margin-top: 15px; padding: 15px; background: rgba(0,0,0,0.2); border-radius: 10px; border-left: 4px solid #4ECDC4;}";
            response += ".upload-area {border: 2px dashed rgba(255,255,255,0.5); border-radius: 15px; padding: 30px; text-align: center; margin: 20px 0; transition: all 0.3s ease; cursor: pointer;}";
            response += ".upload-area:hover {border-color: #4ECDC4; background: rgba(255,255,255,0.05);}";
            response += ".stats-grid {display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0;}";
            response += ".stat-card {background: rgba(255,255,255,0.1); padding: 20px; border-radius: 10px; text-align: center;}";
            response += ".stat-number {font-size: 2em; font-weight: bold; color: #4ECDC4;}";
            response += ".progress-bar {width: 100%; height: 8px; background: rgba(255,255,255,0.2); border-radius: 4px; overflow: hidden; margin: 10px 0;}";
            response += ".progress-fill {height: 100%; background: linear-gradient(90deg, #4ECDC4, #44A08D); transition: width 0.3s ease; width: 0%;}";
            response += ".upload-progress {display: none; margin-top: 20px;}";
            response += "@media (max-width: 768px) {.gallery {grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 15px;} .controls {flex-direction: column; align-items: center;}}";
            response += "</style></head><body>";
            
            response += "<div class='container'>";
            
            // Enhanced header with system status
            response += "<div class='header'>";
            response += "<h1>&#x1F5BC;&#xFE0F; ESP32P4 Image Gallery</h1>";
            response += "<p>Professional Image Viewer & Management System</p>";
            response += "<div class='status'>Server: " + WiFi.softAPIP().toString() + " | ";
            response += "Clients: " + String(WiFi.softAPgetStationNum()) + " | ";
            response += "Uptime: " + String(millis() / 1000) + "s | ";
            response += "Status: <span style='color: #4ECDC4;'>&#x25CF;</span> Online</div>";
            response += "</div>";
            
            // System statistics
            response += "<div class='card'>";
            response += "<h2>&#x1F4CA; System Statistics</h2>";
            response += "<div class='stats-grid'>";
            response += "<div class='stat-card'><div class='stat-number'>" + String(image_count) + "</div><div>Images</div></div>";
            response += "<div class='stat-card'><div class='stat-number'>" + String(ESP.getFreeHeap() / 1024) + "</div><div>Free RAM (KB)</div></div>";
            response += "<div class='stat-card'><div class='stat-number'>" + String(ESP.getFreePsram() / 1024) + "</div><div>Free PSRAM (KB)</div></div>";
            response += "<div class='stat-card'><div class='stat-number'>" + String(WiFi.softAPgetStationNum()) + "</div><div>Connected</div></div>";
            response += "</div>";
            response += "</div>";
            
            // Main controls with enhanced styling
            response += "<div class='card'>";
            response += "<h2>&#x1F3AE; Gallery Controls</h2>";
            response += "<div class='controls'>";
            response += "<button class='btn' onclick='prevImage()'>&#x23EE;&#xFE0F; Previous</button>";
            response += "<button class='btn btn-secondary' onclick='toggleSlideshow()' id='slideshowBtn'>";
            response += slideshow_active ? "&#x23F8;&#xFE0F; Stop Slideshow" : "&#x25B6;&#xFE0F; Start Slideshow";
            response += "</button>";
            response += "<button class='btn' onclick='nextImage()'>&#x23ED;&#xFE0F; Next</button>";
            response += "<button class='btn btn-secondary' onclick='refreshGallery()'>&#x1F504; Refresh</button>";
            response += "</div>";
            response += "</div>";
            
            // Enhanced upload section
            response += "<div class='card'>";
            response += "<h2>&#x1F4E4; Upload New Images</h2>";
            response += "<form method='POST' action='/upload' enctype='multipart/form-data' id='uploadForm'>";
            response += "<div class='upload-area' onclick='document.getElementById(\"fileInput\").click()' ondrop='handleDrop(event)' ondragover='handleDragOver(event)'>";
            response += "<input type='file' id='fileInput' name='image' accept='.jpg,.jpeg' style='display: none;' onchange='handleFileSelect()'>";
            response += "<h3>&#x1F4C1; Click to Select Images</h3>";
            response += "<p>Drag & drop files here or click to browse</p>";
            response += "<p style='font-size: 0.9em; opacity: 0.8;'>Supports: JPEG only | Max: 2MB per file</p>";
            response += "</div>";
            response += "<div id='fileList'></div>";
            response += "<button type='submit' class='btn' id='uploadBtn' style='display: none;'>&#x1F680; Upload Selected File</button>";
            response += "</form>";
            response += "<div class='upload-progress' id='uploadProgress'>";
            response += "<div class='progress-bar'><div class='progress-fill' id='progressFill'></div></div>";
            response += "<p id='uploadStatus'>Uploading...</p>";
            response += "</div>";
            response += "</div>";
            
            // Enhanced image gallery with delete functionality
            response += "<div class='card'>";
            response += "<h2>&#x1F5BC;&#xFE0F; Image Gallery (" + String(image_count) + " images)</h2>";
            
            if (image_count > 0 && sd_ready) {
                response += "<div class='gallery'>";
                int img_count = (int)image_count;  
                int max_display = (img_count < 12) ? img_count : 12;
                
                for (int i = 0; i < max_display; i++) {
                    String filename = String(image_list[i]);
                    String displayName = filename.substring(filename.lastIndexOf('/') + 1);
                    if (displayName.length() > 20) {
                        displayName = displayName.substring(0, 17) + "...";
                    }
                    response += "<div class='image-item'>";
                    response += "<img src='/thumb?index=" + String(i) + "' class='thumbnail' alt='Image " + String(i+1) + "' loading='lazy' onclick='showImage(" + String(i) + ")'>";
                    response += "<p style='margin: 10px 0 5px 0; font-weight: bold;'>" + displayName + "</p>";
                    response += "<p style='margin: 0; font-size: 0.8em; opacity: 0.8;'>Image " + String(i+1) + "</p>";
                    
                    // Add control buttons for each image
                    response += "<div style='margin-top: 10px;'>";
                    response += "<button class='btn' style='font-size: 12px; padding: 5px 10px; margin: 2px;' onclick='showImage(" + String(i) + ")'>&#x1F441;&#xFE0F; View</button>";
                    response += "<button class='btn btn-danger' style='font-size: 12px; padding: 5px 10px; margin: 2px;' onclick='deleteImage(\"" + String(image_list[i]) + "\", \"" + displayName + "\")'>&#x1F5D1;&#xFE0F; Delete</button>";
                    response += "</div>";
                    response += "</div>";
                }
                
                if (img_count > 12) {
                    response += "<div class='image-item' style='display: flex; align-items: center; justify-content: center; min-height: 200px;'>";
                    response += "<div style='text-align: center;'>";
                    response += "<div style='font-size: 3em; margin-bottom: 10px;'>&#x1F4C1;</div>";
                    response += "<h3>+" + String(img_count - 12) + " More</h3>";
                    response += "<p>Total: " + String(img_count) + " images</p>";
                    response += "</div></div>";
                }
                
                response += "</div>";
            } else {
                response += "<div style='text-align: center; padding: 60px 20px;'>";
                response += "<div style='font-size: 4em; margin-bottom: 20px; opacity: 0.5;'>&#x1F4F7;</div>";
                response += "<h3>No Images Available</h3>";
                response += "<p>Upload some images using the form above to get started!</p>";
                response += "</div>";
            }
            response += "</div>";
            
            // Advanced features section
            response += "<div class='card'>";
            response += "<h2>&#x2699;&#xFE0F; Advanced Features</h2>";
            response += "<div class='controls'>";
            response += "<a href='/info' class='btn btn-secondary'>&#x1F4CB; System Info</a>";
            response += "<a href='/test' class='btn btn-secondary'>&#x1F527; Connection Test</a>";
            response += "<button class='btn btn-danger' onclick='confirmReset()'>&#x1F504; Restart System</button>";
            response += "</div>";
            response += "</div>";
            
            response += "</div>"; // Close container
            
            // FIXED: Complete JavaScript functionality with ALL API calls
            response += "<script>";
            response += "let currentImage = " + String(current_image) + ";";
            response += "let totalImages = " + String(image_count) + ";";
            response += "let slideshowActive = " + String(slideshow_active ? "true" : "false") + ";";
            
            // Image navigation functions
            response += "function prevImage() {";
            response += "  console.log('Previous image requested');";
            response += "  fetch('/api/prev', {method: 'POST'})";
            response += "    .then(r => r.text())";
            response += "    .then(data => {";
            response += "      console.log('Previous image response:', data);";
            response += "      setTimeout(() => location.reload(), 1000);";
            response += "    })";
            response += "    .catch(e => console.error('Previous error:', e));";
            response += "}";
            
            response += "function nextImage() {";
            response += "  console.log('Next image requested');";
            response += "  fetch('/api/next', {method: 'POST'})";
            response += "    .then(r => r.text())";
            response += "    .then(data => {";
            response += "      console.log('Next image response:', data);";
            response += "      setTimeout(() => location.reload(), 1000);";
            response += "    })";
            response += "    .catch(e => console.error('Next error:', e));";
            response += "}";
            
            response += "function showImage(index) {";
            response += "  console.log('Show image requested:', index);";
            response += "  fetch('/api/show?index=' + index, {method: 'POST'})";
            response += "    .then(r => r.text())";
            response += "    .then(data => {";
            response += "      console.log('Show image response:', data);";
            response += "      currentImage = index;";
            response += "      setTimeout(() => location.reload(), 1000);";
            response += "    })";
            response += "    .catch(e => console.error('Show image error:', e));";
            response += "}";
            
            response += "function toggleSlideshow() {";
            response += "  console.log('Slideshow toggle requested');";
            response += "  fetch('/api/slideshow', {method: 'POST'})";
            response += "    .then(r => r.text())";
            response += "    .then(data => {";
            response += "      console.log('Slideshow response:', data);";
            response += "      slideshowActive = (data === 'started');";
            response += "      const btn = document.getElementById('slideshowBtn');";
            response += "      if (btn) {";
            response += "        btn.innerHTML = slideshowActive ? '&#x23F8;&#xFE0F; Stop Slideshow' : '&#x25B6;&#xFE0F; Start Slideshow';";
            response += "      }";
            response += "    })";
            response += "    .catch(e => console.error('Slideshow error:', e));";
            response += "}";
            
            response += "function refreshGallery() {";
            response += "  console.log('Refreshing gallery');";
            response += "  location.reload();";
            response += "}";
            
            // Enhanced file upload handling
            response += "function handleFileSelect() {";
            response += "  const fileInput = document.getElementById('fileInput');";
            response += "  const fileList = document.getElementById('fileList');";
            response += "  const uploadBtn = document.getElementById('uploadBtn');";
            response += "  ";
            response += "  if (fileInput.files.length > 0) {";
            response += "    const file = fileInput.files[0];";
            response += "    if (file.size > 2 * 1024 * 1024) {";
            response += "      alert('File too large! Maximum size is 2MB.');";
            response += "      fileInput.value = '';";
            response += "      return;";
            response += "    }";
            response += "    if (!file.type.startsWith('image/jpeg') && !file.name.toLowerCase().match(/\\.(jpg|jpeg)$/)) {";
            response += "      alert('Please select a JPEG image file only.');";
            response += "      fileInput.value = '';";
            response += "      return;";
            response += "    }";
            response += "    let html = '<h4>Selected File:</h4>';";
            response += "    html += '<p>&#x1F4C4; ' + file.name + ' (' + Math.round(file.size/1024) + ' KB)</p>';";
            response += "    fileList.innerHTML = html;";
            response += "    uploadBtn.style.display = 'inline-block';";
            response += "  }";
            response += "}";
            
            response += "function handleDragOver(event) {";
            response += "  event.preventDefault();";
            response += "  event.dataTransfer.dropEffect = 'copy';";
            response += "  event.target.style.borderColor = '#4ECDC4';";
            response += "  event.target.style.background = 'rgba(255,255,255,0.1)';";
            response += "}";
            
            response += "function handleDrop(event) {";
            response += "  event.preventDefault();";
            response += "  event.target.style.borderColor = 'rgba(255,255,255,0.5)';";
            response += "  event.target.style.background = 'rgba(255,255,255,0.05)';";
            response += "  const files = event.dataTransfer.files;";
            response += "  const fileInput = document.getElementById('fileInput');";
            response += "  if (files.length > 0) {";
            response += "    const dt = new DataTransfer();";
            response += "    dt.items.add(files[0]);";
            response += "    fileInput.files = dt.files;";
            response += "    handleFileSelect();";
            response += "  }";
            response += "}";
            
            // Image deletion function
            response += "function deleteImage(filepath, displayName) {";
            response += "  if (confirm('Are you sure you want to permanently delete \"' + displayName + '\"?\\n\\nThis action cannot be undone.')) {";
            response += "    console.log('Deleting image:', filepath);";
            response += "    fetch('/api/delete', {";
            response += "      method: 'POST',";
            response += "      headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
            response += "      body: 'file=' + encodeURIComponent(filepath)";
            response += "    })";
            response += "    .then(r => r.json())";
            response += "    .then(data => {";
            response += "      if (data.success) {";
            response += "        console.log('Image deleted successfully');";
            response += "        alert('Image \"' + displayName + '\" deleted successfully!');";
            response += "        setTimeout(() => location.reload(), 1000);";
            response += "      } else {";
            response += "        console.error('Delete failed:', data.error);";
            response += "        alert('Failed to delete image: ' + data.error);";
            response += "      }";
            response += "    })";
            response += "    .catch(e => {";
            response += "      console.error('Delete error:', e);";
            response += "      alert('Error deleting image: ' + e.message);";
            response += "    });";
            response += "  }";
            response += "}";
            
            response += "function confirmReset() {";
            response += "  if (confirm('Are you sure you want to restart the system?')) {";
            response += "    fetch('/api/reset', {method: 'POST'}).then(() => {";
            response += "      alert('System restart initiated. Please wait...');";
            response += "    }).catch(e => console.error('Reset error:', e));";
            response += "  }";
            response += "}";
            
            // Enhanced upload form handling
            response += "document.addEventListener('DOMContentLoaded', function() {";
            response += "  const form = document.getElementById('uploadForm');";
            response += "  const fileInput = document.getElementById('fileInput');";
            response += "  const uploadProgress = document.getElementById('uploadProgress');";
            response += "  const progressFill = document.getElementById('progressFill');";
            response += "  const uploadStatus = document.getElementById('uploadStatus');";
            response += "  ";
            response += "  if (form) {";
            response += "    form.addEventListener('submit', function(e) {";
            response += "      e.preventDefault();";
            response += "      if (!fileInput.files || fileInput.files.length === 0) {";
            response += "        alert('Please select a file to upload');";
            response += "        return;";
            response += "      }";
            response += "      const file = fileInput.files[0];";
            response += "      console.log('Starting upload:', file.name, file.size);";
            response += "      uploadProgress.style.display = 'block';";
            response += "      progressFill.style.width = '0%';";
            response += "      uploadStatus.textContent = 'Preparing upload...';";
            response += "      const formData = new FormData();";
            response += "      formData.append('image', file);";
            response += "      const xhr = new XMLHttpRequest();";
            response += "      xhr.upload.addEventListener('progress', function(e) {";
            response += "        if (e.lengthComputable) {";
            response += "          const percent = Math.round((e.loaded / e.total) * 100);";
            response += "          progressFill.style.width = percent + '%';";
            response += "          uploadStatus.textContent = 'Uploading: ' + percent + '% (' + Math.round(e.loaded/1024) + ' KB)';";
            response += "          console.log('Upload progress:', percent + '%');";
            response += "        }";
            response += "      });";
            response += "      xhr.addEventListener('load', function() {";
            response += "        if (xhr.status === 200) {";
            response += "          progressFill.style.width = '100%';";
            response += "          uploadStatus.textContent = 'Upload complete! Redirecting...';";
            response += "          console.log('Upload successful');";
            response += "          setTimeout(() => location.reload(), 2000);";
            response += "        } else {";
            response += "          uploadStatus.textContent = 'Upload failed: ' + xhr.responseText;";
            response += "          console.error('Upload failed:', xhr.status, xhr.responseText);";
            response += "          setTimeout(() => uploadProgress.style.display = 'none', 3000);";
            response += "        }";
            response += "      });";
            response += "      xhr.addEventListener('error', function() {";
            response += "        uploadStatus.textContent = 'Upload failed due to network error';";
            response += "        console.error('Upload network error');";
            response += "        setTimeout(() => uploadProgress.style.display = 'none', 3000);";
            response += "      });";
            response += "      xhr.timeout = 60000;";
            response += "      xhr.addEventListener('timeout', function() {";
            response += "        uploadStatus.textContent = 'Upload timed out - file may be too large';";
            response += "        console.error('Upload timeout');";
            response += "        setTimeout(() => uploadProgress.style.display = 'none', 3000);";
            response += "      });";
            response += "      xhr.open('POST', '/upload', true);";
            response += "      xhr.send(formData);";
            response += "    });";
            response += "  }";
            response += "});";
            
            // Auto-refresh status every 30 seconds
            response += "setInterval(() => {";
            response += "  fetch('/api/status')";
            response += "    .then(r => r.json())";
            response += "    .then(data => {";
            response += "      console.log('Status update:', data);";
            response += "    })";
            response += "    .catch(e => console.log('Status update failed'));";
            response += "}, 30000);";
            
            response += "</script>";
            response += "</body></html>";
            
            server.send(200, "text/html", response);
            Logger.info("Enhanced creative gallery page served successfully");
        });
        
        // **CRITICAL: These are the MISSING API endpoints that make the buttons work**
        server.on("/api/prev", HTTP_POST, []() {
            Logger.info("API: Previous image requested");
            if (image_count > 0) {
                current_image = (current_image - 1 + image_count) % image_count;
                display_image(current_image);
                server.send(200, "text/plain", "OK");
                Logger.info("API: Previous image loaded successfully");
            } else {
                server.send(400, "text/plain", "No images available");
                Logger.warn("API: No images available for previous");
            }
        });
        
        server.on("/api/next", HTTP_POST, []() {
            Logger.info("API: Next image requested");
            if (image_count > 0) {
                current_image = (current_image + 1) % image_count;
                display_image(current_image);
                server.send(200, "text/plain", "OK");
                Logger.info("API: Next image loaded successfully");
            } else {
                server.send(400, "text/plain", "No images available");
                Logger.warn("API: No images available for next");
            }
        });
        
        server.on("/api/show", HTTP_POST, []() {
            int index = server.arg("index").toInt();
            Logger.info("API: Show image %d requested", index);
            if (index >= 0 && index < image_count) {
                display_image(index);
                server.send(200, "text/plain", "OK");
                Logger.info("API: Image %d loaded successfully", index);
            } else {
                server.send(400, "text/plain", "Invalid index");
                Logger.warn("API: Invalid image index %d", index);
            }
        });
        
        server.on("/api/slideshow", HTTP_POST, []() {
            Logger.info("API: Slideshow toggle requested");
            toggleSlideshow(5000);
            String status = slideshow_active ? "started" : "stopped";
            server.send(200, "text/plain", status);
            Logger.info("API: Slideshow %s", status.c_str());
        });
        
        server.on("/api/status", HTTP_GET, []() {
            String json = "{";
            json += "\"images\":" + String(image_count) + ",";
            json += "\"current\":" + String(current_image) + ",";
            json += "\"slideshow\":" + String(slideshow_active ? "true" : "false") + ",";
            json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
            json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
            json += "\"uptime\":" + String(millis());
            json += "}";
            server.send(200, "application/json", json);
        });
        
        server.on("/api/reset", HTTP_POST, []() {
            Logger.info("API: System reset requested");
            server.send(200, "text/plain", "Restarting...");
            delay(1000);
            ESP.restart();
        });
        
        // API endpoint for deleting images
        server.on("/api/delete", HTTP_POST, []() {
            String filepath = server.arg("file");
            if (filepath.length() == 0) {
                server.send(400, "application/json", "{\"error\":\"No file specified\"}");
                return;
            }
            
            // Security check
            if (!filepath.startsWith("/images/")) {
                server.send(403, "application/json", "{\"error\":\"Access denied\"}");
                return;
            }
            
            bool success = deleteImageFile(filepath.c_str());
            
            if (success) {
                String json = "{";
                json += "\"success\":true,";
                json += "\"message\":\"File deleted successfully\",";
                json += "\"file\":\"" + filepath + "\",";
                json += "\"remaining_images\":" + String(image_count);
                json += "}";
                server.send(200, "application/json", json);
            } else {
                server.send(500, "application/json", "{\"error\":\"Failed to delete file\"}");
            }
        });
        
        // Thumbnail endpoint
        server.on("/thumb", HTTP_GET, []() {
            int index = server.arg("index").toInt();
            if (index >= 0 && index < image_count && image_list[index]) {
                server.sendHeader("Location", "/image?file=" + String(image_list[index]));
                server.send(302, "text/plain", "");
            } else {
                server.send(404, "text/plain", "Thumbnail not found");
            }
        });
        
        // Enhanced upload handler
        server.on("/upload", HTTP_POST, []() {
            server.send(200, "text/plain", "");
        }, handle_upload);
        
        // Image serving endpoint
        server.on("/image", HTTP_GET, handle_image);
        
        // Info and test pages
        server.on("/info", HTTP_GET, handle_info);
        server.on("/test", HTTP_GET, handle_test);
        
        // Simple ping endpoint for testing connection speed
        server.on("/ping", HTTP_GET, []() {
            server.send(200, "text/plain", "pong");
        });
        
        // Handle 404 with style
        server.onNotFound([]() {
            String html = "<!DOCTYPE html><html><head><title>404 - Not Found</title>";
            html += "<meta charset='UTF-8'>";
            html += "<style>body{font-family:Arial;text-align:center;padding:50px;background:linear-gradient(135deg,#667eea,#764ba2);color:white;}";
            html += ".error{font-size:4em;margin:20px;}</style></head><body>";
            html += "<div class='error'>&#x1F50D;</div><h1>404 - Page Not Found</h1>";
            html += "<p>The requested resource could not be found.</p>";
            html += "<a href='/' style='color:#4ECDC4;'>&#x2190; Back to Gallery</a></body></html>";
            server.send(404, "text/html", html);
        });
        
        server.begin();
        Logger.info("Enhanced WebServer started on port 80 with creative interface");
        
        return true;
        
    } catch (...) {
        Logger.error("Exception in webserver setup");
        return false;
    }
}

// Enhanced image deletion functionality
bool deleteImageFile(const char *filename) {
    if (!sd_ready || !filename) {
        Logger.error("Cannot delete image - SD not ready or invalid filename");
        return false;
    }
    
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Logger.error("Failed to take SD mutex for image deletion");
        return false;
    }
    
    Logger.info("Attempting to delete image: %s", filename);
    
    if (!SD_MMC.exists(filename)) {
        Logger.error("File does not exist: %s", filename);
        xSemaphoreGive(sd_mutex);
        return false;
    }
    
    bool success = SD_MMC.remove(filename);
    xSemaphoreGive(sd_mutex);
    
    if (success) {
        Logger.info("Successfully deleted image: %s", filename);
        scan_images();
        
        if (image_count > 0) {
            if (current_image >= image_count) {
                current_image = 0;
            }
            display_image(current_image);
        } else {
            clearLCDScreen();
            current_image = 0;
        }
        
        return true;
    } else {
        Logger.error("Failed to delete image: %s", filename);
        return false;
    }
}

// Enhanced upload handler with crash prevention
void handle_upload() {
    Logger.info("Upload request received");
    
    HTTPUpload& upload = server.upload();
    static File uploadFile;
    static bool upload_error = false;
    
    if (upload.status == UPLOAD_FILE_START) {
        upload_active = true;
        upload_start_time = millis();
        upload_size = 0;
        upload_error = false;
        
        feed_watchdog();
        
        String filename = upload.filename;
        if (!filename.endsWith(".jpg") && !filename.endsWith(".jpeg") && 
            !filename.endsWith(".JPG") && !filename.endsWith(".JPEG")) {
            filename += ".jpg";
        }
        
        filename.replace("..", "");
        filename.replace("/", "");
        filename.replace("\\", "");
        
        String filepath = "/images/" + filename;
        strncpy(upload_filename, filepath.c_str(), sizeof(upload_filename) - 1);
        upload_filename[sizeof(upload_filename) - 1] = '\0';
        
        Logger.info("Starting upload: %s", upload_filename);
        
        if (!sd_ready) {
            Logger.error("SD card not ready for upload");
            upload_error = true;
            upload_active = false;
            return;
        }
        
        if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            Logger.error("Failed to take SD mutex for upload");
            upload_error = true;
            upload_active = false;
            return;
        }
        
        uploadFile = SD_MMC.open(upload_filename, FILE_WRITE);
        if (!uploadFile) {
            Logger.error("Failed to create upload file: %s", upload_filename);
            xSemaphoreGive(sd_mutex);
            upload_error = true;
            upload_active = false;
            return;
        }
        
        Logger.info("Upload file created successfully: %s", upload_filename);
        
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (upload_error) {
            return;
        }
        
        if (uploadFile && upload.currentSize > 0) {
            static uint32_t last_wdt_feed = 0;
            if (millis() - last_wdt_feed > 1000) {
                feed_watchdog();
                last_wdt_feed = millis();
            }
            
            size_t chunkSize = min(upload.currentSize, (size_t)4096);
            size_t written = 0;
            size_t totalWritten = 0;
            
            for (size_t offset = 0; offset < upload.currentSize; offset += chunkSize) {
                size_t currentChunk = min(chunkSize, upload.currentSize - offset);
                
                written = uploadFile.write(upload.buf + offset, currentChunk);
                totalWritten += written;
                
                if (written != currentChunk) {
                    Logger.error("Write error during upload: wrote %d of %d bytes", written, currentChunk);
                    uploadFile.close();
                    xSemaphoreGive(sd_mutex);
                    SD_MMC.remove(upload_filename);
                    upload_error = true;
                    upload_active = false;
                    return;
                }
                
                if (offset % 8192 == 0) {
                    delay(1);
                }
            }
            
            upload_size += totalWritten;
            
            if (upload_size % 102400 == 0) {
                Logger.debug("Upload progress: %u bytes", upload_size);
            }
            
            if (upload_size > (2 * 1024 * 1024)) {
                Logger.error("File too large: %u bytes (max 2MB)", upload_size);
                uploadFile.close();
                xSemaphoreGive(sd_mutex);
                SD_MMC.remove(upload_filename);
                upload_error = true;
                upload_active = false;
                return;
            }
        }
        
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
            uploadFile.flush();
            uploadFile.close();
        }
        
        xSemaphoreGive(sd_mutex);
        
        if (upload_error) {
            Logger.error("Upload failed due to previous errors");
            upload_active = false;
            return;
        }
        
        uint32_t upload_time = millis() - upload_start_time;
        Logger.info("Upload completed: %s (%u bytes in %u ms)", 
                    upload_filename, upload_size, upload_time);
        
        if (upload_size > 0 && isValidJPEG(upload_filename)) {
            Logger.info("Uploaded file is valid JPEG");
            scan_images();
            
            String response = "<!DOCTYPE html><html><head>";
            response += "<meta charset='UTF-8'>";
            response += "<meta http-equiv='refresh' content='3;url=/'>";
            response += "<title>Upload Success</title>";
            response += "<style>body{font-family:Arial;text-align:center;padding:50px;background:linear-gradient(135deg,#667eea,#764ba2);color:white;}</style>";
            response += "</head><body>";
            response += "<h2>&#x2705; Upload Successful!</h2>";
            response += "<p>File: " + String(upload_filename) + "</p>";
            response += "<p>Size: " + String(upload_size) + " bytes</p>";
            response += "<p>Upload time: " + String(upload_time) + " ms</p>";
            response += "<p>Redirecting to gallery in 3 seconds...</p>";
            response += "<a href='/' style='color:#4ECDC4;'>Return to Gallery</a>";
            response += "</body></html>";
            
            server.send(200, "text/html", response);
            Logger.info("Upload success response sent");
        } else {
            Logger.error("Uploaded file is not a valid JPEG or is empty");
            SD_MMC.remove(upload_filename);
            
            String response = "<!DOCTYPE html><html><head>";
            response += "<meta charset='UTF-8'>";
            response += "<title>Upload Failed</title>";
            response += "<style>body{font-family:Arial;text-align:center;padding:50px;background:linear-gradient(135deg,#667eea,#764ba2);color:white;}</style>";
            response += "</head><body>";
            response += "<h2>&#x274C; Upload Failed!</h2>";
            response += "<p>The uploaded file is not a valid JPEG image.</p>";
            response += "<a href='/' style='color:#4ECDC4;'>Return to Gallery</a>";
            response += "</body></html>";
            
            server.send(400, "text/html", response);
        }
        
        upload_active = false;
        
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Logger.error("Upload aborted");
        if (uploadFile) {
            uploadFile.close();
            xSemaphoreGive(sd_mutex);
        }
        SD_MMC.remove(upload_filename);
        upload_active = false;
        upload_error = true;
    }
    
    feed_watchdog();
}

void handle_image() {
    String filepath = server.arg("file");
    if (filepath.length() == 0) {
        server.send(400, "text/plain", "No file specified");
        return;
    }
    
    if (!SD_MMC.exists(filepath)) {
        server.send(404, "text/plain", "File not found");
        return;
    }
    
    File file = SD_MMC.open(filepath, FILE_READ);
    if (!file) {
        server.send(500, "text/plain", "Failed to open file");
        return;
    }
    
    server.streamFile(file, "image/jpeg");
    file.close();
}

void handle_info() {
    Logger.info("System info page requested");
    
    String response = "<!DOCTYPE html><html><head>";
    response += "<meta charset='UTF-8'>";
    response += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    response += "<title>System Information</title>";
    response += "<style>";
    response += "body { font-family: Arial, sans-serif; margin: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }";
    response += ".container { max-width: 1000px; margin: 0 auto; background: rgba(255,255,255,0.1); padding: 20px; border-radius: 10px; }";
    response += "table { width: 100%; border-collapse: collapse; margin: 15px 0; }";
    response += "th, td { padding: 12px; text-align: left; border-bottom: 1px solid rgba(255,255,255,0.2); }";
    response += "th { background-color: rgba(255,255,255,0.1); font-weight: bold; }";
    response += ".btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; text-decoration: none; display: inline-block; margin: 10px 5px; }";
    response += "</style></head><body>";
    
    response += "<div class='container'>";
    response += "<h1>&#x1F4CA; ESP32P4 System Information</h1>";
    
    // Memory Information
    response += "<h2>&#x1F4BE; Memory Information</h2>";
    response += "<table>";
    response += "<tr><th>Type</th><th>Total (KB)</th><th>Free (KB)</th><th>Used (KB)</th></tr>";
    response += "<tr><td>Heap</td><td>" + String(ESP.getHeapSize() / 1024) + "</td>";
    response += "<td>" + String(ESP.getFreeHeap() / 1024) + "</td>";
    response += "<td>" + String((ESP.getHeapSize() - ESP.getFreeHeap()) / 1024) + "</td></tr>";
    response += "<tr><td>PSRAM</td><td>" + String(ESP.getPsramSize() / 1024) + "</td>";
    response += "<td>" + String(ESP.getFreePsram() / 1024) + "</td>";
    response += "<td>" + String((ESP.getPsramSize() - ESP.getFreePsram()) / 1024) + "</td></tr>";
    response += "</table>";
    
    // System Information
    response += "<h2>&#x2699;&#xFE0F; System Information</h2>";
    response += "<table>";
    response += "<tr><th>Parameter</th><th>Value</th></tr>";
    response += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
    response += "<tr><td>Flash Size</td><td>" + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB</td></tr>";
    response += "<tr><td>Flash Speed</td><td>" + String(ESP.getFlashChipSpeed() / 1000000) + " MHz</td></tr>";
    response += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
    response += "<tr><td>SDK Version</td><td>" + String(ESP.getSdkVersion()) + "</td></tr>";
    response += "</table>";
    
    response += "<a href='/' class='btn'>Back to Gallery</a>";
    response += "</div>";
    response += "</body></html>";
    
    server.send(200, "text/html", response);
}

void handle_test() {
    Logger.info("Connection test page requested");
    
    String response = "<!DOCTYPE html><html><head>";
    response += "<meta charset='UTF-8'>";
    response += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    response += "<title>Connection Test</title>";
    response += "<style>";
    response += "body { font-family: Arial, sans-serif; margin: 20px; text-align: center; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }";
    response += ".container { max-width: 600px; margin: 0 auto; background: rgba(255,255,255,0.1); padding: 40px; border-radius: 10px; }";
    response += ".success { color: #4ECDC4; font-size: 24px; margin: 20px 0; }";
    response += ".btn { background: #007bff; color: white; padding: 15px 30px; border: none; border-radius: 5px; text-decoration: none; display: inline-block; margin: 10px; font-size: 16px; }";
    response += "</style></head><body>";
    
    response += "<div class='container'>";
    response += "<h1>&#x1F517; Connection Test</h1>";
    response += "<div class='success'>&#x2705; Connection successful!</div>";
    response += "<p>Your device is successfully connected to the ESP32P4 Image Gallery.</p>";
    response += "<a href='/' class='btn'>Go to Gallery</a>";
    response += "</div>";
    response += "</body></html>";
    
    server.send(200, "text/html", response);
}

// LVGL task function
void lvgl_task(void *pvParameters) {
    Logger.info("LVGL task started on core %d", xPortGetCoreID());
    
    while (true) {
        if (lvgl_ready) {
            if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                lv_timer_handler();
                xSemaphoreGive(ui_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        
        static uint32_t last_wdt_feed = 0;
        if (millis() - last_wdt_feed > 1000) {
            feed_watchdog();
            last_wdt_feed = millis();
        }
    }
}


void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("=== ESP32P4 Gallery Debug Output ===");
    Serial.println("Starting initialization sequence with enhanced error handling...");
    
    esp_task_wdt_deinit();
    
    diagnosePartitions();
    
    bool fsAvailable = initializeFileSystem();
    safeLoggerInit(fsAvailable);
    
    Logger.info("====================================");
    Logger.info("ESP32P4 Image Gallery Starting");
    Logger.info("Date: 2025-06-22 09:45:00 UTC");
    Logger.info("User: Chamil1983");
    Logger.info("====================================");
    
    sd_mutex = xSemaphoreCreateMutex();
    jpeg_mutex = xSemaphoreCreateMutex();
    ui_mutex = xSemaphoreCreateMutex();
    Logger.info("System mutexes initialized");
    
    Logger.info("Initializing enhanced watchdog system");
    if (setup_watchdog()) {
        Logger.info("Watchdog initialized successfully");
    } else {
        Logger.warn("Watchdog initialization failed - continuing without WDT");
    }
    
    esp_log_level_set("*", ESP_LOG_NONE);
    
    lcd_ready = initializeWithRetry("LCD", setup_lcd, 2);
    touch_ready = initializeWithRetry("Touch", setup_touch, 2);
    sd_ready = initializeWithRetry("SD Card", setup_sd_card_reliable, 3);
    wifi_ready = initializeWithRetry("WiFi", setup_wifi_ap, 2);
    lvgl_ready = initializeWithRetry("LVGL", setup_lvgl, 1);
    ui_ready = initializeWithRetry("UI", setup_ui, 1);
    server_ready = initializeWithRetry("WebServer", setup_webserver, 2);
    
    if (lvgl_ready) {
        lvgl_task_handle = createTaskSafely(
            lvgl_task,
            "LVGL Task",
            8192,
            NULL,
            2,
            1
        );
    }
    
    if (sd_ready) {
        Logger.info("Scanning for images...");
        scan_images();
        Logger.info("Found %d images", image_count);
        
        if (image_count > 0 && lvgl_ready && ui_ready) {
            Logger.info("Loading first image");
            display_image(0);
        }
    } else {
        Logger.warn("SD card not ready, skipping image scan");
    }
    
    if (lvgl_ready && ui_ready) {
        slideshow_task_handle = createTaskSafely(
            slideshowTask,
            "Slideshow Task",
            4096,
            NULL,
            1,
            0
        );
    }
    
    Logger.info("Initialization complete, system status:");
    Logger.info("LCD: %s, Touch: %s, SD: %s", 
               lcd_ready ? "OK" : "FAIL",
               touch_ready ? "OK" : "FAIL", 
               sd_ready ? "OK" : "FAIL");
    Logger.info("WiFi: %s, LVGL: %s, UI: %s, Server: %s",
               wifi_ready ? "OK" : "FAIL",
               lvgl_ready ? "OK" : "FAIL",
               ui_ready ? "OK" : "FAIL",
               server_ready ? "OK" : "FAIL");
}

void loop() {
    static uint32_t last_wdt_feed = 0;
    static uint32_t last_status_update = 0;
    static uint32_t last_dns_process = 0;
    static uint32_t last_wifi_check = 0;
    static uint32_t last_connection_report = 0;
    uint32_t now = millis();
    
    if (now - last_wdt_feed >= 2000) {
        feed_watchdog();
        last_wdt_feed = now;
    }
    
    if (dnsServerActive && now - last_dns_process >= 30) {
        try {
            dnsServer.processNextRequest();
        } catch (...) {
        }
        last_dns_process = now;
    }
    
    if (now - last_wifi_check >= 10000) {
        if (wifi_ready && WiFi.softAPgetStationNum() > 0) {
            if (now - last_connection_report >= 60000) {
                Logger.info("WiFi status: %d clients connected", WiFi.softAPgetStationNum());
                last_connection_report = now;
            }
        } else if (wifi_ready && WiFi.softAPgetStationNum() == 0) {
            Logger.debug("WiFi AP running but no clients connected");
        }
        last_wifi_check = now;
    }
    
    if (server_ready) {
        try {
            server.handleClient();
        } catch (...) {
            Logger.warn("Exception in server.handleClient() - continuing");
        }
    }
    
    if (reset_requested) {
        Logger.info("System reset requested");
        delay(200);
        ESP.restart();
    }
    
    delay(5);
}

#pragma GCC pop_options


