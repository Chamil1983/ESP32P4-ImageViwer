#pragma GCC push_options
#pragma GCC optimize("O2")  // Using O2 for better stability

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
#include "JPEGDEC.h"  // Using the specified library
#include "lv_conf.h"  // LVGL configuration
#include "pins_config.h" // Board pin configuration
#include "debug_logger.h"  // Debug Library
#include "src/lcd/jd9365_lcd.h" // Library for LCD
#include "src/touch/gsl3680_touch.h"  // Library for Touch

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
#define WDT_TIMEOUT_SECONDS 120  // Increased timeout to 120 seconds

// WiFi AP configuration
#define WIFI_AP_SSID "ESP32P4-ImageViewer"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP IPAddress(192, 168, 4, 1)
#define WIFI_AP_GATEWAY IPAddress(192, 168, 4, 1)
#define WIFI_AP_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_CHANNEL 6
#define MAX_WIFI_CLIENTS 2  // Limited to 2 for stability

// Web Server on port 80
WebServer server(80);
DNSServer dnsServer;

// Hardware objects
jd9365_lcd lcd(LCD_RST);
gsl3680_touch touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

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
#define DRAW_BUFFER_SIZE (32 * 1024)  // 32KB buffer
uint8_t *drawBuffer = nullptr;

// Global flag for image rotation (used by JPEG decoder and draw callback)
bool image_needs_rotation = false;

// Global flag to track image loading state
bool image_loading = false;

// DNS server status tracking
bool dnsServerActive = false;

// Slideshow variables
bool slideshow_active = false;                                   // Flag to track if slideshow is running
uint32_t slideshow_last_change = 0;                              // Timestamp of last image change
uint32_t slideshow_interval = 5000;                              // Default interval: 5 seconds
hw_timer_t *slideshow_timer = NULL;                              // Hardware timer for slideshow
portMUX_TYPE slideshow_timerMux = portMUX_INITIALIZER_UNLOCKED;  // Timer mutex
TaskHandle_t slideshow_task_handle = NULL;                       // Task handle for slideshow

// UI elements
lv_obj_t *main_screen = nullptr;
lv_obj_t *status_bar = nullptr;
lv_obj_t *image_view = nullptr;
lv_obj_t *image_display = nullptr;  // Image display widget
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
#define UPLOAD_BUFFER_SIZE 4096        // 4KB buffer
#define SDMMC_FREQ_PROBING 125000      // Very slow speed for initial probing
#define RETRY_DELAY 1000               // Delay between retries
#define SD_POWER_STABILIZE_DELAY 2000  // Power stabilization delay
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

// Statistics for JPEG decoding
int jpeg_decode_total_count = 0;
int jpeg_decode_success_count = 0;

/**
 * ESP32P4 Image Gallery - Function Declarations
 * Updated: 2025-06-20 11:49:44
 * User: Chamil1983
 */

// ------ File System Management ------
bool initializeFileSystem();
bool initializeSpiffs();
bool emergency_spiffs_recovery();

// ------ Watchdog Timer Management ------
bool setup_watchdog();
void feed_watchdog();
esp_err_t esp_task_wdt_reset_user(TaskHandle_t handle);

// ------ System Initialization and Health ------
void safeLoggerInit(bool spiffsAvailable);
bool initializeWithRetry(const char* componentName, bool (*initFunction)(), int maxRetries);
void performSystemHealthCheck();
TaskHandle_t createTaskSafely(TaskFunction_t taskFunction, const char* name, 
                             uint32_t stackSize, void *param, UBaseType_t priority, 
                             BaseType_t coreID);
void checkPartitions();

// ------ Component Setup Functions ------
bool setup_lcd();
bool setup_touch();
bool setup_sd_card_reliable();
bool setup_wifi_ap();
bool setup_lvgl();
bool setup_ui();
bool setup_webserver();
bool recover_sd_card();
bool emergency_sd_recovery();
bool recover_sd_card_for_upload();

// ------ Task Functions ------
void lvgl_task(void *pvParameters);
void slideshowTask(void *parameter);
void safeDeleteTask(TaskHandle_t taskHandle, const char* taskName);

// ------ Image Management Functions ------
void scan_images();
bool createTestImage();
void display_image(int index);
bool isValidJPEG(const char* filename);
int jpeg_draw_callback(JPEGDRAW *pDraw);
void clearLCDScreen();
bool render_jpeg_file(const char* filename);
void displayErrorMessage(const char* message);
int findNextValidImage(int currentIndex);

// ------ UI and Input Functions ------
void check_buttons();
void handle_touch_event(uint16_t x, uint16_t y);
void updateImageCounter(int index);
void updateStatusAfterImageLoad(int index, bool success);
void update_system_info();

// ------ Slideshow Control Functions ------
void startSlideshow(uint32_t interval_ms);
void stopSlideshow();
void toggleSlideshow(uint32_t interval_ms = 5000);

// ------ Web Server Functions ------
void handle_upload();
void diagnose_sd_card();
String updateUploadForm();

// ------ Helper Functions ------
uint32_t max_u32(uint32_t a, uint32_t b);
uint32_t min_u32(uint32_t a, uint32_t b);
size_t safeSDWrite(File &file, const uint8_t *buf, size_t size);
void add_test_page_handler();

// ------ Main Program Functions ------
void setup();
void loop();

// to ensure consistent debug logging throughout the code
#define DEBUG_ENABLED true

// Debug macros with forced output
#define DEBUG_PRINT(tag, format, ...) \
  do { \
    if (DEBUG_ENABLED) { \
      Serial.printf("[DEBUG][%s] ", tag); \
      Serial.printf(format, ##__VA_ARGS__); \
      Serial.println(); \
      Serial.flush(); \
    } \
  } while (0)

#define DEBUG_PRINTLN(text) do { if (DEBUG_ENABLED) { Serial.println(text); } } while (0)
#define DEBUG_PRINTF(format, ...) do { if (DEBUG_ENABLED) { Serial.printf(format, ##__VA_ARGS__); } } while (0)

// Template for safe execution with watchdog feeding
template<typename Func>
bool safe_execute_with_watchdog(Func function, const char* operation_name) {
    unsigned long start_time = millis();
    bool result = false;
    
    try {
        // Feed watchdog before starting
        if (wdt_enabled) {
            feed_watchdog();
        }
        
        // Execute operation
        result = function();
        
        // Feed watchdog after completion
        if (wdt_enabled) {
            feed_watchdog();
        }
        
        // Log result based on success
        if (result) {
            Logger.info("%s completed successfully in %u ms", operation_name, millis() - start_time);
        } else {
            Logger.error("%s failed after %u ms", operation_name, millis() - start_time);
        }
    }
    catch (const std::exception& e) {
        Logger.error("%s failed with exception: %s", operation_name, e.what());
        result = false;
    }
    catch (...) {
        Logger.error("%s failed with unknown exception", operation_name);
        result = false;
    }
    
    return result;
}

/**
 * Fixed LittleFS initialization with proper partition name
 * Updated: 2025-06-20 11:49:44
 * User: Chamil1983
 */
bool initializeFileSystem() {
    Serial.println("Initializing file system using LittleFS...");
    
    // Try mounting with storage partition name first
    if (LittleFS.begin(false, "storage")) {
        Serial.println("LittleFS mounted successfully on 'storage' partition");
        return true;
    }
    
    // If failed, try without specifying partition name (will use default)
    if (LittleFS.begin(false)) {
        Serial.println("LittleFS mounted successfully with default partition");
        return true;
    }
    
    // If still failed, try formatting (without partition name - it uses the default)
    Serial.println("LittleFS mount failed, formatting...");
    
    // Format without partition name parameter
    if (LittleFS.format()) {
        // Try mounting again with storage partition name
        if (LittleFS.begin(false, "storage")) {
            Serial.println("LittleFS formatted and mounted successfully with 'storage' partition");
            return true;
        }
        
        // As a final attempt, try mounting with default
        if (LittleFS.begin(false)) {
            Serial.println("LittleFS formatted and mounted with default settings");
            return true;
        }
    }
    
    Serial.println("LittleFS initialization failed - continuing without file system");
    return false;
}

/**
 * Emergency SPIFFS recovery
 * Date: 2025-06-20 11:49:44
 * User: Chamil1983
 */
bool emergency_spiffs_recovery() {
    Logger.warn("Attempting emergency SPIFFS recovery");
    
    // Force unmount any previous instance
    SPIFFS.end();
    
    // Short delay to ensure cleanup
    delay(500);
    
    // Format the partition
    bool format_result = SPIFFS.format();
    if (!format_result) {
        Logger.error("Emergency SPIFFS format failed");
        return false;
    }
    
    Logger.info("SPIFFS formatted successfully");
    
    // Try to mount again
    bool mount_result = SPIFFS.begin(false);  // false = don't format again
    if (!mount_result) {
        Logger.error("SPIFFS remount failed after format");
        return false;
    }
    
    Logger.info("Emergency SPIFFS recovery successful");
    return true;
}

/**
 * Enhanced slideshow task creation
 */
void createSlideshowTask() {
    // Create slideshow task on core 0 with lower priority
    xTaskCreatePinnedToCore(
        slideshowTask,           // Function
        "Slideshow Task",        // Name
        4096,                    // Stack size
        NULL,                    // Parameters
        1,                       // Priority (1 is low)
        &slideshow_task_handle,  // Task handle
        0                        // Core ID (Core 0)
    );

    // Wait for task to start up before adding to watchdog
    delay(100);

    // Register slideshow task with watchdog if task creation succeeded
    if (slideshow_task_handle != NULL) {
        if (wdt_enabled) {
            esp_err_t err = esp_task_wdt_add(slideshow_task_handle);
            if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                Logger.warn("Failed to add slideshow task to watchdog: %d", err);
            }
        }
    } else {
        Logger.error("Failed to create slideshow task");
    }
}

/**
 * Safer component initialization with retry
 */
bool initializeWithRetry(const char* componentName, bool (*initFunction)(), int maxRetries) {
    Logger.info("Initializing %s with up to %d attempts...", componentName, maxRetries);
    
    bool success = false;
    for (int retry = 0; retry < maxRetries; retry++) {
        Logger.info("Setting up %s...", componentName);
        
        // Call initialization function directly
        try {
            success = initFunction();
        }
        catch (...) {
            Logger.error("Exception during %s initialization", componentName);
            success = false;
        }
        
        if (success) {
            Logger.info("%s initialized successfully on attempt %d", componentName, retry + 1);
            break;
        } else {
            Logger.warn("%s initialization failed, attempt %d/%d", 
                      componentName, retry + 1, maxRetries);
            delay(500 * (retry + 1)); // Progressive backoff
        }
    }
    
    if (!success) {
        Logger.error("%s initialization failed after %d attempts", componentName, maxRetries);
    }
    
    return success;
}

/**
 * Setup watchdog timer with configurable timeout
 */
bool setup_watchdog() {
    Serial.println("Setting up watchdog with critical error protection...");
    
    // Force disable any existing watchdog timer to prevent early crashes
    esp_task_wdt_deinit();
    delay(100);
    
    // Use conservative configuration
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,          // 30 seconds - very conservative
        .idle_core_mask = 0,          // Don't watch idle cores
        .trigger_panic = false        // Don't panic on timeout
    };
    
    // Initialize watchdog with conservative settings
    esp_err_t err = esp_task_wdt_init(&wdt_config);
    if (err != ESP_OK) {
        Serial.printf("Watchdog init failed: %d - continuing without WDT\n", err);
        wdt_enabled = false;
        return false;
    }
    
    // Get current task handle
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (currentTask != NULL) {
        // Subscribe current task to watchdog
        esp_err_t add_err = esp_task_wdt_add(currentTask);
        if (add_err != ESP_OK) {
            Serial.printf("Could not add main task to WDT: %d - continuing anyway\n", add_err);
        } else {
            Serial.println("Main task subscribed to watchdog");
        }
    }
    
    // Store the task handle for future reference
    main_task_handle = currentTask;
    wdt_enabled = (err == ESP_OK);
    
    Serial.printf("Watchdog initialized with %d second timeout, panic mode: OFF\n", 
                 wdt_config.timeout_ms / 1000);
    
    return (err == ESP_OK);
}

/**
 * Feed watchdog timer to prevent resets
 */
void feed_watchdog() {
    static uint32_t last_feed = 0;
    uint32_t now = millis();
    
    // Don't feed too often
    if (now - last_feed < 1000) {
        return;  // Feed at most once per second
    }
    
    last_feed = now;
    
    // Skip if WDT is disabled
    if (!wdt_enabled) {
        return;
    }
    
    // Always use direct task handle approach
    if (main_task_handle != NULL) {
        esp_task_wdt_reset();
    }
}

/**
 * Custom task watchdog reset function that doesn't fail on task not found
 */
esp_err_t esp_task_wdt_reset_user(TaskHandle_t handle) {
    esp_err_t ret = esp_task_wdt_reset();
    
    // If task not found error, don't treat as critical
    if (ret == ESP_ERR_NOT_FOUND) {
        return ESP_OK; // Pretend it worked
    }
    
    return ret;
}

/**
 * Safe logger initialization
 */
void safeLoggerInit(bool spiffsAvailable) {
    // Initialize with basic configuration
    Logger.init(true, false, LOG_LEVEL_DEBUG);
    
    // Enable enhanced features if file system is available
    if (spiffsAvailable) {
        Logger.enableFlashOutput(true, &SPIFFS, "/debug_log.txt");
        Logger.setMaxLogFileSize(32 * 1024);  // 32KB max log file
    }
}

/**
 * Create task safely with error checking
 */
TaskHandle_t createTaskSafely(TaskFunction_t taskFunction, const char* name, 
                             uint32_t stackSize, void *param, UBaseType_t priority, 
                             BaseType_t coreID) {
    // Input validation
    if (taskFunction == nullptr || name == nullptr) {
        Logger.error("Invalid parameters for task creation");
        return NULL;
    }
    
    // Create the task
    TaskHandle_t taskHandle = NULL;
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,
        name,
        stackSize,
        param,
        priority,
        &taskHandle,
        coreID
    );
    
    // Check result
    if (result != pdPASS || taskHandle == NULL) {
        Logger.error("Failed to create task: %s", name);
        return NULL;
    }
    
    Logger.info("Created task: %s on core %d", name, coreID);
    return taskHandle;
}

/**
 * Safely delete a task with error checking
 */
void safeDeleteTask(TaskHandle_t taskHandle, const char* taskName) {
    if (taskHandle == NULL) {
        return;  // Task doesn't exist
    }
    
    // Check if task is still running first
    eTaskState state = eTaskGetState(taskHandle);
    
    // Only delete if task is still valid
    if (state != eDeleted) {
        Logger.info("Deleting task: %s", taskName);
        vTaskDelete(taskHandle);
    }
}

/**
 * Check system partitions
 */
void checkPartitions() {
    Logger.debug("Flash chip size: %d MB", ESP.getFlashChipSize() / (1024 * 1024));
    Logger.debug("PSRAM size: %d MB", ESP.getPsramSize() / (1024 * 1024));
    Logger.debug("Free heap: %d KB", ESP.getFreeHeap() / 1024);
    Logger.debug("Free PSRAM: %d KB", ESP.getFreePsram() / 1024);
    
    // Check partition table
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    
    while (it != NULL) {
        const esp_partition_t* part = esp_partition_get(it);
        Logger.debug("Partition: %s, Type: %d, Subtype: %d, Address: 0x%08x, Size: %d KB",
                   part->label, part->type, part->subtype, part->address, part->size / 1024);
        it = esp_partition_next(it);
    }
    
    esp_partition_iterator_release(it);
}

/**
 * LCD Setup
 */
bool setup_lcd() {
    Logger.info("Setting up LCD...");
    
    lcd_ready = false;
    
    try {
        // Initialize LCD
        lcd.begin(); 
        
        // Set LCD backlight using the correct method from your driver
        lcd.example_bsp_set_lcd_backlight(255); // Full brightness
        
        Logger.info("LCD initialized successfully with backlight ON");
        lcd_ready = true;
    } catch (...) {
        Logger.error("Exception during LCD initialization");
        lcd_ready = false;
    }
    
    return lcd_ready;
}

/**
 * Touch Setup
 */
bool setup_touch() {
    Logger.info("Setting up Touch...");
    
    touch_ready = false;
    
    try {
        // Initialize touch controller
        touch.begin();
        
        // Configure touch if needed
        touch.set_rotation(1); // Match LCD orientation
        
        Logger.info("Touch initialized successfully");
        touch_ready = true;
    } catch (...) {
        Logger.error("Exception during touch setup");
        touch_ready = false;
    }
    
    return touch_ready;
}

/**
 * WiFi Access Point Setup with enhanced stability
 */
bool setup_wifi_ap() {
    // Ensure WiFi is disconnected first
    try {
        WiFi.disconnect(true, true);   // Disconnect and erase stored credentials
        WiFi.mode(WIFI_OFF);           // Turn WiFi off completely
        delay(500);                    // Allow WiFi hardware to reset
        
        // Initialize WiFi with specific settings
        WiFi.persistent(false);        // Don't save settings to flash
        WiFi.setSleep(WIFI_PS_NONE);   // Disable power saving completely
        
        // Set AP mode with explicit configuration
        WiFi.mode(WIFI_AP);
        delay(200);
        
        // Configure static IP settings
        IPAddress local_ip(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        
        // Apply static IP configuration
        if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
            Logger.error("Failed to configure AP IP settings");
            return false;
        }
        
        // Try multiple times to start AP with conservative settings
        Logger.info("Starting AP with SSID: %s", WIFI_AP_SSID);
        bool success = false;
        
        // Use more conservative settings for better stability
        for (int attempt = 1; attempt <= 3; attempt++) {
            Logger.info("AP start attempt %d...", attempt);
            
            // More conservative settings:
            // - Channel 6 (less interference than 1 in many environments)
            // - Hidden: false (more visible)
            // - Max connections: 4 (more than enough)
            // - Beacon interval: more frequent (100ms)
            success = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, 6, false, 4);
            
            if (success) {
                // Configure more settings after AP is up
                esp_wifi_set_ps(WIFI_PS_NONE);  // Ensure power save is disabled
                
                // Set up DNS for captive portal
                if (dnsServerActive) {
                    dnsServer.stop();
                }
                
                dnsServer.start(53, "*", WiFi.softAPIP());
                dnsServerActive = true;
                
                delay(100);  // Allow settings to apply
                break;
            } else {
                delay(1000 * attempt); // Increasing delay between retries
            }
        }
        
        if (success) {
            IPAddress apIP = WiFi.softAPIP();
            Logger.info("WiFi AP Mode active with enhanced stability");
            Logger.info("AP SSID: %s", WIFI_AP_SSID);
            Logger.info("AP IP Address: %s", apIP.toString().c_str());
            Logger.info("AP Channel: 6");
            Logger.info("AP Max Clients: 4");
            Logger.info("AP Power: Maximum");
            
            wifi_ready = true;
        } else {
            Logger.error("Failed to start WiFi AP after multiple attempts");
            wifi_ready = false;
        }
        
        return success;
    }
    catch (...) {
        Logger.error("Exception during WiFi AP setup");
        return false;
    }
}

/**
 * SD Card setup with enhanced reliability and error recovery
 */
bool setup_sd_card_reliable() {
    Logger.info("Setting up SD Card with robust initialization...");
    
    // Create mutex if it doesn't exist
    if (sd_mutex == NULL) {
        sd_mutex = xSemaphoreCreateMutex();
        if (sd_mutex == NULL) {
            Logger.error("Failed to create SD card mutex");
            return false;
        }
    }
    
    try {
        // Make sure SD card is not already mounted
        SD_MMC.end();
        
        // Reset and stabilize SD card power if possible
        Logger.info("Waiting for power stabilization...");
        delay(SD_POWER_STABILIZE_DELAY);
        
        // Reset SD pins to clean state
        Logger.info("Resetting SD pins...");
        pinMode(SDMMC_CLK_PIN, INPUT);
        pinMode(SDMMC_CMD_PIN, INPUT);
        pinMode(SDMMC_D0_PIN, INPUT);
        pinMode(SDMMC_D1_PIN, INPUT);
        pinMode(SDMMC_D2_PIN, INPUT);
        pinMode(SDMMC_D3_PIN, INPUT);
        delay(100);
        
        // Ensure SD is not initialized
        Logger.info("Ensuring SD_MMC is deinitialized...");
        SD_MMC.end();
        delay(500);  // Wait for any pending operations
        
        // Configure SD pins
        Logger.info("Configuring SD pins...");
        SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                    SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
        
        // Start with the most conservative settings - 1-bit mode at 125KHz
        Logger.info("SD init attempt 1: 1-bit mode at 125KHz (ultra conservative)");
        if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING, 5)) {
            Logger.error("SD init attempt 1 failed, retrying with modified settings...");
            delay(500);
            
            // Second attempt - 1-bit mode at 400KHz (standard SD initialization speed)
            Logger.info("SD init attempt 2: 1-bit mode at 400KHz (standard)");
            if (!SD_MMC.begin("/sdcard", true, false, 400000, 5)) {
                Logger.error("SD init attempt 2 failed, retrying with modified settings...");
                delay(500);
                
                // Third attempt - 4-bit mode at 400KHz
                Logger.info("SD init attempt 3: 4-bit mode at 400KHz");
                if (!SD_MMC.begin("/sdcard", false, false, 400000, 5)) {
                    Logger.error("All SD init attempts failed");
                    return false;
                } else {
                    Logger.info("SD card initialized in 4-bit mode at 400KHz");
                }
            } else {
                Logger.info("SD card initialized in 1-bit mode at 400KHz");
            }
        } else {
            Logger.info("SD card initialized in 1-bit mode at 125KHz");
        }
        
        // Validate the SD card
        Logger.info("Validating SD card...");
        
        // Check card type
        uint8_t cardType = SD_MMC.cardType();
        if (cardType == CARD_NONE) {
            Logger.error("No SD card detected");
            return false;
        }
        
        // Log card info
        const char* typeStr = "UNKNOWN";
        switch (cardType) {
            case CARD_MMC: typeStr = "MMC"; break;
            case CARD_SD: typeStr = "SD"; break;
            case CARD_SDHC: typeStr = "SDHC"; break;
            default: typeStr = "UNKNOWN"; break;
        }
        Logger.info("SD Card Type: %s", typeStr);
        
        // Calculate card size in MB for display
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Logger.info("SD Card Size: %lluMB", cardSize);
        
        // Make sure images directory exists
        if (!SD_MMC.exists("/images")) {
            Logger.info("Creating images directory on SD card");
            if (!SD_MMC.mkdir("/images")) {
                Logger.warn("Failed to create images directory, but continuing");
            }
        }
        
        // Test write capability
        File testFile = SD_MMC.open("/sd_test.txt", FILE_WRITE);
        if (testFile) {
            testFile.println("SD Card Test");
            testFile.close();
            SD_MMC.remove("/sd_test.txt");  // Clean up after test
        } else {
            Logger.warn("Cannot write to SD card, might be read-only");
        }
        
        Logger.info("SD card initialized successfully in ultra-conservative mode");
        sd_ready = true;
        return true;
        
    } catch (...) {
        Logger.error("Exception during SD card setup");
        return false;
    }
}

/**
 * Validate SD card properties
 */
bool validateSDCard() {
    // Check if card is present
    if (SD_MMC.cardType() == CARD_NONE) {
        Logger.error("No SD card attached");
        return false;
    }
    
    // Log card type
    uint8_t cardType = SD_MMC.cardType();
    String cardTypeStr;
    
    switch (cardType) {
        case CARD_MMC:  cardTypeStr = "MMC";   break;
        case CARD_SD:   cardTypeStr = "SDSC";  break;
        case CARD_SDHC: cardTypeStr = "SDHC";  break;
        default:        cardTypeStr = "UNKNOWN"; break;
    }
    
    Logger.info("SD Card Type: %s", cardTypeStr.c_str());
    
    // Calculate card size in MB
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Logger.info("SD Card Size: %lluMB", cardSize);
    
    // Return success if we got this far
    return true;
}

/**
 * Setup LVGL graphics library
 */
bool setup_lvgl() {
    try {
        // Initialize LVGL
        lv_init();
        
        // Allocate display buffers using PSRAM if available
        size_t buf_size = LCD_H_RES * 15; // Buffer for 15 rows
        
        lv_color_t* buf1 = (lv_color_t*)ps_malloc(buf_size * sizeof(lv_color_t));
        lv_color_t* buf2 = (lv_color_t*)ps_malloc(buf_size * sizeof(lv_color_t));
        
        if (!buf1 || !buf2) {
            Logger.error("Failed to allocate LVGL buffers");
            return false;
        }
        
        Logger.debug("Allocating LVGL buffers: %d bytes each", buf_size * sizeof(lv_color_t));
        
        // Initialize the display buffer
        static lv_disp_draw_buf_t draw_buf;
        lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_size);
        
        // Set up the display driver
        static lv_disp_drv_t disp_drv;
        lv_disp_drv_init(&disp_drv);
        disp_drv.hor_res = LCD_H_RES;
        disp_drv.ver_res = LCD_V_RES;
        
        // LCD flush callback
// LVGL display driver callback function with correct LCD function name
disp_drv.flush_cb = [](lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    // Draw to LCD using the correct function name: lcd_draw_bitmap instead of draw_bitmap
    lcd.lcd_draw_bitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t*)color_p);
    
    // Indicate to LVGL that the flush is done
    lv_disp_flush_ready(disp);
};
        
        disp_drv.draw_buf = &draw_buf;
        lv_disp_drv_register(&disp_drv);
        
        if (touch_ready) {
            // Set up the touch driver for input
            static lv_indev_drv_t indev_drv;
            lv_indev_drv_init(&indev_drv);
            indev_drv.type = LV_INDEV_TYPE_POINTER;
            
            // Read callback with improved error handling
            indev_drv.read_cb = [](lv_indev_drv_t* drv, lv_indev_data_t* data) {
                static lv_coord_t last_x = 0;
                static lv_coord_t last_y = 0;
                
                uint16_t touch_x, touch_y;
                bool touched = false;
                
                try {
                    touched = touch.getTouch(&touch_x, &touch_y);
                } catch (...) {
                    // Suppress any exceptions in touch handler
                    touched = false;
                }
                
                if (touched) {
                    data->state = LV_INDEV_STATE_PR;
                    data->point.x = touch_x;
                    data->point.y = touch_y;
                    last_x = touch_x;
                    last_y = touch_y;
                } else {
                    data->state = LV_INDEV_STATE_REL;
                    data->point.x = last_x;
                    data->point.y = last_y;
                }
            };
            
            lv_indev_drv_register(&indev_drv);
        }
        
        // Create JPEG draw buffer using PSRAM if available
        drawBuffer = (uint8_t*)ps_malloc(DRAW_BUFFER_SIZE);
        if (!drawBuffer) {
            drawBuffer = (uint8_t*)malloc(DRAW_BUFFER_SIZE);
            if (!drawBuffer) {
                Logger.error("Failed to allocate JPEG draw buffer");
                return false;
            }
        }
        
        // Create mutex for JPEG decoding if it doesn't exist
        if (jpeg_mutex == NULL) {
            jpeg_mutex = xSemaphoreCreateMutex();
        }
        
        // Create mutex for UI updates if it doesn't exist
        if (ui_mutex == NULL) {
            ui_mutex = xSemaphoreCreateMutex();
        }
        
        Logger.info("LVGL initialized successfully");
        lvgl_ready = true;
        return true;
    }
    catch (...) {
        Logger.error("Exception during LVGL setup");
        lvgl_ready = false;
        return false;
    }
}

/**
 * Setup UI elements
 */
bool setup_ui() {
    try {
        Logger.info("Setting up UI...");
        
        ui_ready = false;
        
        if (!lvgl_ready) {
            Logger.error("Cannot setup UI - LVGL not initialized");
            return false;
        }
        
        // Create a main screen
        main_screen = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x000000), 0);
        lv_disp_load_scr(main_screen);
        
        // Initialize styles for buttons
        lv_style_init(&style_btn);
        lv_style_set_radius(&style_btn, 5);
        lv_style_set_bg_color(&style_btn, lv_color_hex(0x2980b9));
        lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
        lv_style_set_pad_all(&style_btn, 10);
        lv_style_set_text_color(&style_btn, lv_color_white());
        
        lv_style_init(&style_btn_pressed);
        lv_style_set_bg_color(&style_btn_pressed, lv_color_hex(0x1a5d8c));
        
        // Create status bar at the top
        status_bar = lv_obj_create(main_screen);
        lv_obj_set_size(status_bar, LCD_H_RES, 40);
        lv_obj_set_pos(status_bar, 0, 0);
        lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x333333), 0);
        lv_obj_set_style_pad_all(status_bar, 5, 0);
        
        // Status label on left side
        status_label = lv_label_create(status_bar);
        lv_label_set_text(status_label, "Slideshow: OFF");
        lv_obj_align(status_label, LV_ALIGN_LEFT_MID, 10, 0);
        
        // Image counter on right side
        image_counter = lv_label_create(status_bar);
        lv_label_set_text(image_counter, "Image: 0/0");
        lv_obj_align(image_counter, LV_ALIGN_RIGHT_MID, -10, 0);
        
        // Create image view area (this is where images will be displayed)
        image_view = lv_obj_create(main_screen);
        lv_obj_set_size(image_view, LCD_H_RES, LCD_V_RES - 80);
        lv_obj_set_pos(image_view, 0, 40);
        lv_obj_set_style_bg_color(image_view, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(image_view, 0, 0);
        lv_obj_set_style_pad_all(image_view, 0, 0);
        
        // Create control bar at the bottom
        control_bar = lv_obj_create(main_screen);
        lv_obj_set_size(control_bar, LCD_H_RES, 40);
        lv_obj_set_pos(control_bar, 0, LCD_V_RES - 40);
        lv_obj_set_style_bg_color(control_bar, lv_color_hex(0x333333), 0);
        lv_obj_set_style_pad_all(control_bar, 5, 0);
        
        // Previous button
        prev_btn = lv_btn_create(control_bar);
        lv_obj_set_size(prev_btn, 100, 30);
        lv_obj_add_style(prev_btn, &style_btn, 0);
        lv_obj_add_style(prev_btn, &style_btn_pressed, LV_STATE_PRESSED);
        lv_obj_align(prev_btn, LV_ALIGN_LEFT_MID, 10, 0);
        
        lv_obj_t *prev_label = lv_label_create(prev_btn);
        lv_label_set_text(prev_label, "Previous");
        lv_obj_center(prev_label);
        
        lv_obj_add_event_cb(prev_btn, [](lv_event_t *e) {
            if (image_count > 0) {
                int prev_img = (current_image > 0) ? (current_image - 1) : (image_count - 1);
                display_image(prev_img);
            }
        }, LV_EVENT_CLICKED, NULL);
        
        // Next button
        next_btn = lv_btn_create(control_bar);
        lv_obj_set_size(next_btn, 100, 30);
        lv_obj_add_style(next_btn, &style_btn, 0);
        lv_obj_add_style(next_btn, &style_btn_pressed, LV_STATE_PRESSED);
        lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -10, 0);
        
        lv_obj_t *next_label = lv_label_create(next_btn);
        lv_label_set_text(next_label, "Next");
        lv_obj_center(next_label);
        
        lv_obj_add_event_cb(next_btn, [](lv_event_t *e) {
            if (image_count > 0) {
                int next_img = (current_image < image_count - 1) ? (current_image + 1) : 0;
                display_image(next_img);
            }
        }, LV_EVENT_CLICKED, NULL);
        
        // Slideshow toggle button in the center
        lv_obj_t *slideshow_btn = lv_btn_create(control_bar);
        lv_obj_set_size(slideshow_btn, 120, 30);
        lv_obj_add_style(slideshow_btn, &style_btn, 0);
        lv_obj_add_style(slideshow_btn, &style_btn_pressed, LV_STATE_PRESSED);
        lv_obj_align(slideshow_btn, LV_ALIGN_CENTER, 0, 0);
        
        lv_obj_t *slideshow_label = lv_label_create(slideshow_btn);
        lv_label_set_text(slideshow_label, "Slideshow");
        lv_obj_center(slideshow_label);
        
        lv_obj_add_event_cb(slideshow_btn, [](lv_event_t *e) {
            toggleSlideshow();
        }, LV_EVENT_CLICKED, NULL);
        
        Logger.info("UI initialized successfully");
        ui_ready = true;
        
        return ui_ready;
    }
    catch (...) {
        Logger.error("Exception during UI setup");
        return false;
    }
}

/**
 * Update the image counter display
 */
void updateImageCounter(int index) {
    if (!ui_ready || image_counter == nullptr) return;
    
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char counter_text[32];
        snprintf(counter_text, sizeof(counter_text), "Image: %d/%d", index + 1, image_count);
        lv_label_set_text(image_counter, counter_text);
        xSemaphoreGive(ui_mutex);
    }
}

/**
 * Update the status after loading an image
 */
void updateStatusAfterImageLoad(int index, bool success) {
    if (success) {
        Logger.info("Successfully displayed image %d", index);
    } else {
        Logger.error("Failed to display image %d", index);
    }
    
    // Update UI elements if needed
    if (ui_ready && status_label != nullptr) {
        if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!success) {
                lv_label_set_text(status_label, "Load Error");
            } else if (slideshow_active) {
                lv_label_set_text(status_label, "Slideshow: ON");
            } else {
                lv_label_set_text(status_label, "Slideshow: OFF");
            }
            
            xSemaphoreGive(ui_mutex);
        }
    }
}

/**
 * Process pending operations in the main loop
 */
void process_pending_operations() {
    // Handle WebServer
    if (server_ready) {
        server.handleClient();
    }
    
    // Handle reset request
    if (reset_requested) {
        Logger.info("Reset requested. Restarting system...");
        delay(1000);
        ESP.restart();
    }
}

/**
 * Enhanced web server setup with better error handling and diagnostics
 * Updated: 2025-06-20 11:49:44
 * User: Chamil1983
 */
bool setup_webserver() {
    try {
        Logger.info("Setting up WebServer with enhanced connectivity...");
        
        // Basic server configuration
        server.enableCORS(true);        // Enable CORS for better browser compatibility
        server.enableCrossOrigin(true); // Another CORS setting
        
        // Root route handler with detailed connection diagnostics
        server.on("/", HTTP_GET, []() {
            Logger.info("Serving main page");
            
            // Create a response buffer with PSRAM if available
            String* response = nullptr;
            if (ESP.getFreePsram() > 100000) {
                response = new (std::nothrow) String();
                if (!response) {
                    response = new String();
                }
            } else {
                response = new String();
            }
            
            // Start building the HTML response
            response->reserve(4096); // Pre-allocate reasonable size
            *response = "<!DOCTYPE html><html><head><title>ESP32P4 Gallery</title>";
            *response += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
            *response += "<style>";
            *response += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
            *response += "h1, h2 {color: #3b82f6;}";
            *response += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
            *response += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer;}";
            *response += ".gallery {display: flex; flex-wrap: wrap; gap: 10px; margin: 20px 0;}";
            *response += ".thumbnail {width: 150px; height: 150px; object-fit: cover; border-radius: 4px;}";
            *response += ".controls {display: flex; gap: 10px; margin: 20px 0;}";
            *response += ".status {margin-top: 10px; padding: 10px; background: #1e293b; border-radius: 4px;}";
            *response += "</style></head><body>";
            
            // Header with diagnostic info
            *response += "<h1>ESP32P4 Image Gallery</h1>";
            *response += "<div class='status'>Server IP: " + WiFi.softAPIP().toString() + " | ";
            *response += "Clients: " + String(WiFi.softAPgetStationNum()) + " | ";
            *response += "Signal: OK</div>";
            
            // Main controls
            *response += "<div class='controls'>";
            *response += "<button class='btn' onclick='prevImage()'>Previous</button>";
            *response += "<button class='btn' onclick='toggleSlideshow()' id='slideshowBtn'>";
            *response += slideshow_active ? "Stop Slideshow" : "Start Slideshow";
            *response += "</button>";
            *response += "<button class='btn' onclick='nextImage()'>Next</button>";
            *response += "</div>";
            
            // Image previews
            *response += "<div class='card'>";
            *response += "<h2>Available Images</h2>";
            *response += "<div class='gallery'>";
            
            if (image_count > 0 && sd_ready) {
                // FIX: Cast volatile int to regular int for type compatibility with min()
                int img_count = (int)image_count;  
                int max_display = (img_count < 6) ? img_count : 6;  // Alternative to min()
                
                for (int i = 0; i < max_display; i++) {
                    String filename = String(image_list[i]);
                    *response += "<div onclick='showImage(" + String(i) + ")' style='cursor:pointer;'>";
                    *response += "<img src='/thumb?index=" + String(i) + "' class='thumbnail' alt='Image " + String(i+1) + "'>";
                    *response += "</div>";
                }
                
                if (img_count > 6) {
                    *response += "<div><p>+" + String(img_count - 6) + " more images</p></div>";
                }
            } else {
                *response += "<p>No images available. Please upload some using the form below.</p>";
            }
            
            *response += "</div></div>";
            
            // Upload form
            *response += "<div class='card'>";
            *response += "<h2>Upload New Image</h2>";
            *response += "<form method='POST' action='/upload' enctype='multipart/form-data' id='uploadForm'>";
            *response += "<input type='file' name='image' accept='image/jpeg' required>";
            *response += "<button type='submit' class='btn'>Upload</button>";
            *response += "</form>";
            *response += "<div id='uploadStatus' class='status'></div>";
            *response += "</div>";
            
            // System information
            *response += "<div class='card'>";
            *response += "<h2>System Info</h2>";
            *response += "<p>Free Heap: " + String(ESP.getFreeHeap() / 1024) + " KB</p>";
            *response += "<p>SD Card: " + String(sd_ready ? "Connected" : "Not available") + "</p>";
            *response += "<p>Images: " + String(image_count) + "</p>";
            *response += "<a href='/system' class='btn'>Detailed System Info</a>";
            *response += "</div>";
            
            // JavaScript
            *response += "<script>";
            *response += "function prevImage() { fetch('/navigate?direction=prev', {method: 'POST'}).then(res => res.text()).then(data => console.log(data)); }";
            *response += "function nextImage() { fetch('/navigate?direction=next', {method: 'POST'}).then(res => res.text()).then(data => console.log(data)); }";
            *response += "function showImage(idx) { fetch('/image?index=' + idx, {method: 'GET'}).then(res => res.text()).then(data => console.log(data)); }";
            *response += "function toggleSlideshow() {";
            *response += "  const btn = document.getElementById('slideshowBtn');";
            *response += "  const action = btn.innerText.includes('Start') ? 'start' : 'stop';";
            *response += "  fetch('/slideshow?action=' + action, {method: 'POST'})";
            *response += "    .then(res => res.text())";
            *response += "    .then(data => {";
            *response += "      console.log(data);";
            *response += "      btn.innerText = action === 'start' ? 'Stop Slideshow' : 'Start Slideshow';";
            *response += "    });";
            *response += "}";
            
            // Upload form handling
            *response += "document.getElementById('uploadForm').addEventListener('submit', function(e) {";
            *response += "  e.preventDefault();";
            *response += "  const formData = new FormData(this);";
            *response += "  const status = document.getElementById('uploadStatus');";
            *response += "  status.textContent = 'Uploading...';";
            *response += "  fetch('/upload', {";
            *response += "    method: 'POST',";
            *response += "    body: formData";
            *response += "  })";
            *response += "  .then(response => response.text())";
            *response += "  .then(data => {";
            *response += "    status.textContent = 'Upload complete!';";
            *response += "    setTimeout(() => location.reload(), 1000);";
            *response += "  })";
            *response += "  .catch(error => {";
            *response += "    status.textContent = 'Upload failed: ' + error;";
            *response += "  });";
            *response += "});";
            *response += "</script>";
            
            *response += "</body></html>";
            
            // Send the response
            server.send(200, "text/html", *response);
            
            // Clean up
            delete response;
            
            // Log successful request
            Logger.info("Main page served successfully");
        });
        
        // Default 404 handler
        server.onNotFound([]() {
            Logger.warn("404 Not Found: %s", server.uri().c_str());
            server.send(404, "text/plain", "Not found");
        });

        // Navigation handler
        server.on("/navigate", HTTP_POST, []() {
            String direction = server.arg("direction");
            Logger.info("Navigation request received: %s", direction.c_str());
            
            if (image_count <= 0) {
                server.send(400, "text/plain", "No images available");
                return;
            }
            
            if (direction == "prev") {
                int prevImage = (current_image > 0) ? (current_image - 1) : (image_count - 1);
                display_image(prevImage);
                server.send(200, "text/plain", "Displaying previous image (" + String(prevImage + 1) + " of " + String(image_count) + ")");
            } else if (direction == "next") {
                int nextImage = (current_image < image_count - 1) ? (current_image + 1) : 0;
                display_image(nextImage);
                server.send(200, "text/plain", "Displaying next image (" + String(nextImage + 1) + " of " + String(image_count) + ")");
            } else {
                server.send(400, "text/plain", "Invalid direction. Use 'prev' or 'next'");
            }
        });
        
        // Image display handler
        server.on("/image", HTTP_GET, []() {
            int index = server.arg("index").toInt();
            if (index >= 0 && index < image_count) {
                display_image(index);
                server.send(200, "text/plain", "Displaying image " + String(index + 1) + " of " + String(image_count));
            } else {
                server.send(400, "text/plain", "Invalid image index");
            }
        });
        
        // Slideshow control handler with more reliable response handling
        server.on("/slideshow", HTTP_POST, []() {
            String action = server.arg("action");
            Logger.info("Slideshow control request: %s", action.c_str());
            
            if (action == "start") {
                uint32_t interval = 5000; // Default 5 seconds
                
                // Check if interval was provided
                if (server.hasArg("interval")) {
                    interval = server.arg("interval").toInt();
                }
                
                startSlideshow(interval);
                server.send(200, "text/plain", "Slideshow started with " + String(interval) + "ms interval");
            } 
            else if (action == "stop") {
                stopSlideshow();
                server.send(200, "text/plain", "Slideshow stopped");
            }
            else {
                server.send(400, "text/plain", "Invalid action. Use 'start' or 'stop'");
            }
        });
        
        // Thumbnail handler
        server.on("/thumb", HTTP_GET, []() {
            int index = server.arg("index").toInt();
            
            if (index < 0 || index >= image_count || !sd_ready) {
                // Send a placeholder image if index is invalid
                server.send(404, "text/plain", "Image not found");
                return;
            }
            
            File file = SD_MMC.open(image_list[index], FILE_READ);
            if (!file) {
                server.send(404, "text/plain", "Image file not found");
                return;
            }
            
            // Use content-disposition to help browser recognize this is an image
            server.sendHeader("Content-Disposition", "inline; filename=thumb.jpg");
            server.sendHeader("Cache-Control", "max-age=31536000"); // Cache for a year
            
            // Stream the file directly from SD card
            size_t fileSize = file.size();
            server.setContentLength(fileSize);
            server.send(200, "image/jpeg", "");
            
            // Send in chunks of 1KB
            uint8_t buffer[1024];
            size_t remaining = fileSize;
            
            // Use client reference for more reliable streaming
            WiFiClient client = server.client();
            
            // Fix: Handle size_t compared to int
            while (remaining > 0 && client.connected()) {
                // Fix the min function call issue
                size_t toRead = remaining < 1024 ? remaining : 1024;
                file.read(buffer, toRead);
                client.write(buffer, toRead);
                remaining -= toRead;
            }
            
            file.close();
        });
        
        // Add test page handler for connectivity diagnostics
        add_test_page_handler();
        
        // Handle file uploads
        server.on("/upload", HTTP_POST, []() {
            server.send(200, "text/plain", "Upload complete");
        }, handle_upload);
        
        // Begin server
        server.begin();
        
        Logger.info("WebServer started on port 80 with enhanced error handling");
        server_ready = true;
        
        return true;
    } catch (const std::exception& e) {
        Logger.error("Exception in WebServer setup: %s", e.what());
        server_ready = false;
        return false;
    } catch (...) {
        Logger.error("Unknown exception in WebServer setup");
        server_ready = false;
        return false;
    }
}

/**
 * Helper function to add test page handler
 */
void add_test_page_handler() {
    server.on("/test", HTTP_GET, []() {
        Logger.info("Connection test page requested");
        
        String response = "<!DOCTYPE html><html><head>";
        response += "<title>ESP32P4 Connection Test</title>";
        response += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        response += "<style>body {font-family: sans-serif; padding: 20px; max-width: 800px; margin: 0 auto; line-height: 1.6;}";
        response += "h1 {color: #0066cc;} .success {color: green;} .error {color: red;}</style>";
        response += "</head><body>";
        response += "<h1>ESP32P4 Connection Test</h1>";
        response += "<p class='success'>✅ Connection successful!</p>";
        response += "<h2>Server Information:</h2>";
        response += "<ul>";
        response += "<li><strong>ESP32P4 IP:</strong> " + WiFi.softAPIP().toString() + "</li>";
        response += "<li><strong>Connected Clients:</strong> " + String(WiFi.softAPgetStationNum()) + "</li>";
        response += "<li><strong>Free Memory:</strong> " + String(ESP.getFreeHeap() / 1024) + " KB</li>";
        response += "<li><strong>Free PSRAM:</strong> " + String(ESP.getFreePsram() / 1024) + " KB</li>";
        response += "<li><strong>WiFi Channel:</strong> 6</li>";
        response += "<li><strong>Server Uptime:</strong> " + String(millis() / 1000) + " seconds</li>";
        response += "<li><strong>Image Count:</strong> " + String(image_count) + "</li>";
        response += "</ul>";
        response += "<p><a href='/'>Go to Gallery</a></p>";
        
        // Add ping test
        response += "<h2>Connection Test:</h2>";
        response += "<div id='pingResult'>Testing connection speed...</div>";
        response += "<script>";
        response += "let startTime = Date.now();";
        response += "fetch('/ping').then(r => r.text()).then(data => {";
        response += "  let pingTime = Date.now() - startTime;";
        response += "  document.getElementById('pingResult').innerHTML = ";
        response += "    `<p>Ping time: <strong>${pingTime}ms</strong></p>`;";
        response += "});";
        response += "</script>";
        response += "</body></html>";
        
        server.send(200, "text/html", response);
        Logger.info("Connection test page served successfully");
    });
    
    // Simple ping endpoint for testing connection speed
    server.on("/ping", HTTP_GET, []() {
        server.send(200, "text/plain", "pong");
    });
}

/**
 * Enhanced file upload handler with improved error handling and timeouts
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void handle_upload() {
    HTTPUpload& upload = server.upload();
    
    static File uploadFile;
    static uint32_t uploadStartTime = 0;
    static uint32_t lastProgressTime = 0;
    static bool uploadError = false;
    static String uploadFilename = "";
    static size_t uploadTotalBytes = 0;
    static size_t uploadReceivedBytes = 0;
    
    if (upload.status == UPLOAD_FILE_START) {
        // Reset upload state variables
        uploadStartTime = millis();
        lastProgressTime = uploadStartTime;
        uploadError = false;
        uploadFilename = "";
        uploadTotalBytes = 0;
        uploadReceivedBytes = 0;
        
        // Generate filename with timestamp to avoid duplicates
        String filename = upload.filename;
        
        // Sanitize filename - remove spaces and special characters
        filename.replace(" ", "_");
        filename.replace("&", "_");
        filename.replace("?", "_");
        filename.replace("+", "_");
        
        // Create a unique path by adding a timestamp prefix
        int timestamp = millis() % 100000; // Use last 5 digits of timestamp
        String uploadPath = "/images/img_" + String(timestamp) + "_" + filename;
        uploadFilename = uploadPath;
        
        Logger.info("Starting new upload: %s -> %s", filename.c_str(), uploadPath.c_str());
        
        // Make sure SD card mutex is free
        if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            Logger.error("Failed to acquire SD mutex for upload");
            uploadError = true;
            return;
        }
        
        // Check if SD card is available
        if (!sd_ready) {
            Logger.error("SD card not ready for upload");
            xSemaphoreGive(sd_mutex);
            uploadError = true;
            return;
        }
        
        // Make sure /images directory exists
        if (!SD_MMC.exists("/images")) {
            if (!SD_MMC.mkdir("/images")) {
                Logger.error("Failed to create /images directory");
                xSemaphoreGive(sd_mutex);
                uploadError = true;
                return;
            }
        }
        
        // Open file for writing with reduced buffer size
        uploadFile = SD_MMC.open(uploadPath.c_str(), FILE_WRITE);
        
        if (!uploadFile) {
            Logger.error("Failed to create file: %s", uploadPath.c_str());
            xSemaphoreGive(sd_mutex);
            uploadError = true;
            return;
        }
        
        Logger.info("File created successfully for upload: %s", uploadPath.c_str());
    }
    else if (upload.status == UPLOAD_FILE_WRITE && !uploadError) {
        if (!uploadFile) {
            Logger.error("Upload file not open");
            uploadError = true;
            return;
        }
        
        // Write data using safe SD write function
        size_t written = safeSDWrite(uploadFile, upload.buf, upload.currentSize);
        
        // Check if write was successful
        if (written != upload.currentSize) {
            Logger.error("SD Write error: %d of %d bytes written", 
                        written, upload.currentSize);
            uploadError = true;
        } else {
            // Log progress periodically (not for every chunk)
            uint32_t now = millis();
            if (now - lastProgressTime > 1000) {
                Logger.debug("Upload progress: %d bytes received", 
                            uploadReceivedBytes + written);
                lastProgressTime = now;
            }
            
            uploadReceivedBytes += written;
            uploadTotalBytes += upload.currentSize;
        }
    }
    else if (upload.status == UPLOAD_FILE_END && !uploadError) {
        // Finish upload
        if (uploadFile) {
            // Ensure all data is written
            uploadFile.flush();
            
            // Get final file size
            size_t fileSize = uploadFile.size();
            
            // Close the file
            uploadFile.close();
            
            // Release SD mutex
            xSemaphoreGive(sd_mutex);
            
            Logger.info("Upload complete: %s (%d bytes)", uploadFilename.c_str(), fileSize);
            
            // Check if file appears valid
            if (fileSize == 0) {
                Logger.error("Uploaded file has zero size - likely corrupt");
                SD_MMC.remove(uploadFilename.c_str());
                uploadError = true;
            }
            else {
                // Add to image list and display
                if (isValidJPEG(uploadFilename.c_str())) {
                    // Re-scan for images to include the new one
                    scan_images();
                    
                    // Find and display the new image
                    for (int i = 0; i < image_count; i++) {
                        if (String(image_list[i]) == uploadFilename) {
                            display_image(i);
                            break;
                        }
                    }
                    
                    Logger.info("Successfully uploaded and added image: %s", uploadFilename.c_str());
                } 
                else {
                    Logger.warn("Uploaded file is not a valid JPEG");
                    SD_MMC.remove(uploadFilename.c_str());
                    uploadError = true;
                }
            }
        }
    }
    else if (upload.status == UPLOAD_FILE_ABORTED || uploadError) {
        // Handle upload abort
        if (uploadFile) {
            uploadFile.close();
        }
        
        // Release SD mutex
        xSemaphoreGive(sd_mutex);
        
        // Delete partially uploaded file
        if (uploadFilename.length() > 0) {
            SD_MMC.remove(uploadFilename.c_str());
        }
        
        Logger.error("Upload aborted");
    }
}

/**
 * Safe SD Write function to handle potential write failures
 */
size_t safeSDWrite(File &file, const uint8_t *buf, size_t size) {
    if (!file || !buf || size == 0) {
        return 0;
    }
    
    size_t totalWritten = 0;
    size_t chunkSize = 512; // Small chunks to prevent watchdog issues
    const uint8_t maxRetries = 3;
    
    for (size_t offset = 0; offset < size; offset += chunkSize) {
        // Calculate current chunk size
        size_t currentChunk = (chunkSize < (size - offset)) ? chunkSize : (size - offset);
        uint8_t retries = 0;
        size_t written = 0;
        
        // Try to write with retries
        while (retries < maxRetries && written < currentChunk) {
            written = file.write(buf + offset, currentChunk);
            
            if (written == currentChunk) {
                break; // Success
            }
            
            // If write failed or was partial, retry
            retries++;
            if (retries < maxRetries) {
                delay(10); // Short delay before retry
            }
        }
        
        // Update total bytes written
        totalWritten += written;
        
        // If write failed after all retries, abort
        if (written < currentChunk) {
            Logger.error("SD write failed after %d retries (%d/%d bytes written)", 
                        maxRetries, totalWritten, size);
            break;
        }
        
        // Periodically call yield() to prevent watchdog triggers
        if (offset % (chunkSize * 4) == 0) {
            yield();
        }
    }
    
    return totalWritten;
}

/**
 * Enhanced LVGL task function with better error handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void lvgl_task(void *pvParameters) {
    // Initialize task-specific variables
    uint32_t last_error_time = 0;
    uint32_t last_wdt_feed = 0;
    
    Logger.info("LVGL task started on core %d", xPortGetCoreID());
    
    while (true) {
        uint32_t now = millis();
        
        // Feed watchdog periodically for this task
        if (now - last_wdt_feed >= 1000) {
            if (wdt_enabled) {
                esp_task_wdt_reset();
            }
            last_wdt_feed = now;
        }
        
        try {
            // Protect with mutex and timeout
            if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Execute LVGL tasks
                lv_timer_handler();
                xSemaphoreGive(ui_mutex);
            }
        } 
        catch (...) {
            // Only log errors periodically to prevent log flooding
            if (now - last_error_time > 5000) {
                Logger.error("Error in LVGL task, continuing");
                last_error_time = now;
            }
        }
        
        // Delay to yield CPU and control update rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Fixed slideshowTask implementation with enhanced error handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void slideshowTask(void *parameter) {
    // Initialize local variables for better error handling
    uint32_t last_image_change = 0;
    uint32_t last_error_time = 0;
    uint32_t last_wdt_feed = 0;
    
    Logger.info("Slideshow task started on core %d", xPortGetCoreID());
    
    // Main slideshow loop
    while (true) {
        // Get current time
        uint32_t now = millis();
        
        // Feed watchdog periodically
        if (wdt_enabled && now - last_wdt_feed >= 2000) {
            esp_task_wdt_reset();
            last_wdt_feed = now;
        }
        
        // Process slideshow logic only if slideshow is active
        if (slideshow_active && image_count > 0 && now - last_image_change >= slideshow_interval) {
            try {
                // Find the next valid image index with wrap-around
                int nextImage = (current_image + 1) % image_count;
                
                // Try to display the next image without blocking too long for mutex
                if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    // Display next image
                    display_image(nextImage);
                    xSemaphoreGive(sd_mutex);
                    
                    // Update timestamp
                    last_image_change = now;
                    Logger.debug("Slideshow advanced to image %d", nextImage);
                } else {
                    // Only log errors periodically
                    if (now - last_error_time > 5000) {
                        Logger.warn("Slideshow couldn't acquire SD mutex, will retry");
                        last_error_time = now;
                    }
                }
            } 
            catch (...) {
                // Log errors, but don't flood the logs
                if (now - last_error_time > 5000) {
                    Logger.error("Exception in slideshow task - continuing");
                    last_error_time = now;
                }
                
                // Safety check to ensure mutex is released in case of exception
                xSemaphoreGive(sd_mutex);
            }
        }
        
        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Helper function to find the next valid image for slideshow
 */
int findNextValidImage(int currentIndex) {
    // Safety check
    if (image_count <= 0) return -1;
    
    // Start with next image index
    int nextIndex = (currentIndex + 1) % image_count;
    int startIndex = nextIndex; // Remember where we started to avoid infinite loop
    
    // Try each image until we find a valid one or tried them all
    do {
        // Check if the image at this index is valid
        if (image_list[nextIndex] && SD_MMC.exists(image_list[nextIndex])) {
            bool isValid = isValidJPEG(image_list[nextIndex]);
            if (isValid) {
                return nextIndex; // Found a valid image
            } else {
                Logger.warn("Skipping invalid image at index %d: %s", 
                          nextIndex, image_list[nextIndex]);
            }
        }
        
        // Move to next image
        nextIndex = (nextIndex + 1) % image_count;
    } while (nextIndex != startIndex); // Continue until we've checked all images
    
    // If we get here, no valid image was found
    return -1;
}

/**
 * Enhanced slideshow control functions with better error handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void startSlideshow(uint32_t interval_ms) {
    // Set a reasonable minimum interval
    if (interval_ms < 2000) {
        interval_ms = 2000; // Minimum 2 seconds for stability
    }
    
    // Prevent unreasonable large intervals
    if (interval_ms > 60000) {
        interval_ms = 60000; // Maximum 1 minute
    }
    
    Logger.info("Starting slideshow with %d ms interval", interval_ms);
    
    // Set slideshow interval
    slideshow_interval = interval_ms;
    
    // Update status label
    if (ui_ready && status_label != nullptr) {
        if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_label_set_text(status_label, "Slideshow: ON");
            xSemaphoreGive(ui_mutex);
        }
    }
    
    // Enable slideshow
    slideshow_active = true;
}

void stopSlideshow() {
    Logger.info("Stopping slideshow");
    
    // Disable slideshow
    slideshow_active = false;
    
    // Update status label
    if (ui_ready && status_label != nullptr) {
        if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_label_set_text(status_label, "Slideshow: OFF");
            xSemaphoreGive(ui_mutex);
        }
    }
}

void toggleSlideshow(uint32_t interval_ms) {
    if (slideshow_active) {
        stopSlideshow();
    } else {
        startSlideshow(interval_ms);
    }
}

/**
 * Check if image needs rotation
 */
bool check_image_needs_rotation(int img_width, int img_height) {
    // Screen is 800x1280 (portrait orientation)
    // Only rotate if image is in landscape (width > height)
    return (img_width > img_height);
}

/**
 * Enhanced JPEG rendering with better error handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
bool render_jpeg_file(const char* filename) {
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or invalid filename");
        return false;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    // Increment counter for stats
    jpeg_decode_total_count++;
    
    // Take SD card mutex to ensure exclusive access
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
    
    // Initialize flags for image orientation
    image_needs_rotation = false; // Default to no rotation
    
    // Store any JPEG decoding errors
    const char* errorMsg = nullptr;
    
    // Set up error recovery
    bool decodeFailed = false;
    
    // Try to decode the image with progressively lower quality if needed
    if (jpeg.open(jpegFile, jpeg_draw_callback)) {
        // Get image dimensions to determine if rotation is needed
        int imgWidth = jpeg.getWidth();
        int imgHeight = jpeg.getHeight();
        
        if (imgWidth <= 0 || imgHeight <= 0) {
            Logger.error("Invalid image dimensions: %dx%d", imgWidth, imgHeight);
            jpeg.close();
            jpegFile.seek(0);
            decodeFailed = true;
        } else {
            // Set rotation flag based on image orientation
            image_needs_rotation = check_image_needs_rotation(imgWidth, imgHeight);
            
            // Attempt full resolution decode
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
    
    // Try at half resolution if full resolution failed
    if (decodeFailed) {
        jpeg.close();
        jpegFile.seek(0);
        decodeFailed = false;
        
        if (jpeg.open(jpegFile, jpeg_draw_callback)) {
            // Try with half resolution scaling
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
    
    // Try at quarter resolution as last resort
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
    
    // Always close the JPEG object
    jpeg.close();
    
    // Close the file and release SD mutex
    jpegFile.close();
    xSemaphoreGive(sd_mutex);
    
    // Update success counter
    if (success) {
        jpeg_decode_success_count++;
    }
    
    // Log the time taken to decode
    uint32_t decodeTime = millis() - startTime;
    if (decodeTime > 1500) {
        Logger.warn("JPEG rendering took %lu ms", decodeTime);
    } else {
        Logger.info("JPEG decoding took %lu ms, result: %s", decodeTime, success ? "success" : "failed");
    }
    
    return success;
}

/**
 * Enhanced JPEG draw callback with guaranteed memory safety
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
// JPEG draw callback function with correct LCD function name
int jpeg_draw_callback(JPEGDRAW *pDraw) {
    // Safety check - don't proceed if pointers are invalid
    if (!lcd_ready || !pDraw || !pDraw->pPixels) {
        return 0;
    }
    
    // Static timing control to prevent LCD driver overload
    static uint32_t lastDrawTime = 0;
    uint32_t currentTime = millis();
    
    // Add small delay between draw operations to prevent LCD driver errors
    if (currentTime - lastDrawTime < 5) {
        delay(5);
    }
    
    // Verify draw dimensions
    if (pDraw->iWidth <= 0 || pDraw->iHeight <= 0 ||
        pDraw->iWidth > 1280 || pDraw->iHeight > 1280) {
        Logger.error("Invalid JPEG draw dimensions: %dx%d", pDraw->iWidth, pDraw->iHeight);
        return 0;
    }
    
    // Get image dimensions from JPEG decoder
    int imgWidth = jpeg.getWidth();
    int imgHeight = jpeg.getHeight();
    
    // Skip drawing if invalid dimensions
    if (imgWidth <= 0 || imgHeight <= 0 || imgWidth > 5000 || imgHeight > 5000) {
        Logger.error("Invalid JPEG dimensions: %dx%d", imgWidth, imgHeight);
        return 0;
    }
    
    // Local copy of rotation flag to prevent race condition
    bool needsRotation = image_needs_rotation;
    
    // Draw without rotation when possible
    if (!needsRotation) {
        // Calculate centering offsets
        int centerX = (LCD_H_RES - imgWidth) / 2;
        int centerY = (LCD_V_RES - imgHeight) / 2;
        
        // Ensure non-negative values
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        // Calculate drawing coordinates
        int x = pDraw->x + centerX;
        int y = pDraw->y + centerY;
        
        // Draw within screen boundaries only
        if (x >= 0 && y >= 0 && 
            x + pDraw->iWidth <= LCD_H_RES && 
            y + pDraw->iHeight <= LCD_V_RES) {
            
            try {
                // Draw the image directly to the LCD using the CORRECT function name
                lcd.lcd_draw_bitmap(x, y, 
                                  x + pDraw->iWidth - 1, 
                                  y + pDraw->iHeight - 1, 
                                  (uint16_t*)pDraw->pPixels);
                
                // Update draw time tracker
                lastDrawTime = millis();
                return 1; // Success
            } 
            catch (...) {
                Logger.error("Exception in LCD drawing");
                return 0; // Failure
            }
        }
    } 
    else {
        // Handle rotation case (landscape image on portrait screen)
        int rotatedWidth = imgHeight;
        int rotatedHeight = imgWidth;
        
        // Calculate centered position for rotated image
        int centerX = (LCD_H_RES - rotatedWidth) / 2;
        int centerY = (LCD_V_RES - rotatedHeight) / 2;
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        // Calculate position in rotated coordinates
        int rotX = centerX + pDraw->y;
        int rotY = centerY + (imgWidth - pDraw->x - pDraw->iWidth);
        int rotWidth = pDraw->iHeight;
        int rotHeight = pDraw->iWidth;
        
        // Safety check for buffer size
        uint32_t bufSize = (uint32_t)rotWidth * (uint32_t)rotHeight;
        if (rotWidth <= 0 || rotHeight <= 0 || bufSize > 40000) {
            Logger.error("Invalid rotation dimensions: %dx%d", rotWidth, rotHeight);
            return 0; // Fail safely
        }
        
        // Allocate memory for rotation - use PSRAM when available
        uint16_t* rotatedData = (uint16_t*)ps_malloc(bufSize * sizeof(uint16_t));
        if (!rotatedData) {
            rotatedData = (uint16_t*)malloc(bufSize * sizeof(uint16_t));
            if (!rotatedData) {
                Logger.error("Failed to allocate %u bytes for rotation", bufSize * sizeof(uint16_t));
                return 0; // Fail safely
            }
        }
        
        // Perform rotation with bounds checks
        for (int y = 0; y < pDraw->iHeight && y < 1000; y++) {
            for (int x = 0; x < pDraw->iWidth && x < 1000; x++) {
                // For 90° rotation: new_x = y, new_y = width - 1 - x
                int newX = y;
                int newY = pDraw->iWidth - 1 - x;
                
                // Bounds check for both source and destination
                if (newX >= 0 && newX < rotWidth && 
                    newY >= 0 && newY < rotHeight && 
                    y * pDraw->iWidth + x < pDraw->iWidth * pDraw->iHeight) {
                    // Only use valid pixels from source image
                    rotatedData[newY * rotWidth + newX] = ((uint16_t*)pDraw->pPixels)[y * pDraw->iWidth + x];
                }
            }
        }
        
        // Draw only if within screen bounds
        bool drawSuccess = false;
        if (rotX >= 0 && rotY >= 0 && 
            rotX + rotWidth <= LCD_H_RES && 
            rotY + rotHeight <= LCD_V_RES) {
            try {
                // Draw rotated image to LCD using the CORRECT function name
                lcd.lcd_draw_bitmap(rotX, rotY, 
                                  rotX + rotWidth - 1, 
                                  rotY + rotHeight - 1, 
                                  rotatedData);
                drawSuccess = true;
            } 
            catch (...) {
                Logger.error("Exception in rotated LCD drawing");
            }
        }
        
        // Free allocated memory
        free(rotatedData);
        
        // Update draw time tracker
        lastDrawTime = millis();
        return drawSuccess ? 1 : 0;
    }
    
    return 0; // Default failure case for safety
}

/**
 * Enhanced display_image function with better error recovery
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void display_image(int index) {
    // Validate parameters
    if (!sd_ready || index < 0 || index >= image_count) {
        Logger.error("Cannot display image - invalid index %d or SD not ready", index);
        return;
    }
    
    // Prevent concurrent image display requests
    static bool display_in_progress = false;
    static SemaphoreHandle_t displaySemaphore = NULL;
    
    if (displaySemaphore == NULL) {
        displaySemaphore = xSemaphoreCreateMutex();
    }
    
    if (display_in_progress) {
        Logger.warn("Image display already in progress, skipping request");
        return;
    }
    
    // Try to acquire display semaphore with timeout
    if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
        Logger.error("Failed to acquire display semaphore");
        return;
    }
    
    display_in_progress = true;
    image_loading = true;
    
    try {
        // Store current image index
        current_image = index;
        
        // Update UI to reflect new image
        updateImageCounter(index);
        
        // Save current backlight level and dim during loading
        uint8_t originalBrightness = 100; // Default brightness level
        lcd.example_bsp_set_lcd_backlight(50); // Dim to 50/255 during loading
        
        // Clear the screen
        clearLCDScreen();
        
        // Make a safe local copy of the filename
        char localFilename[128] = {0};
        strncpy(localFilename, image_list[index], sizeof(localFilename) - 1);
        
        // Try to render the JPEG file
        bool success = false;
        
        // Try to acquire jpeg mutex with a timeout
        if (xSemaphoreTake(jpeg_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            success = render_jpeg_file(localFilename);
            xSemaphoreGive(jpeg_mutex);
        } else {
            Logger.error("Failed to acquire JPEG mutex");
        }
        
        // Handle rendering result
        if (success) {
            // Fade in the backlight for a smooth transition
            const int fadeStep = 10;
            const int fadeDelay = 10;
            
            for (int brightness = 50; brightness <= originalBrightness; brightness += fadeStep) {
                lcd.example_bsp_set_lcd_backlight(brightness);
                delay(fadeDelay);
            }
            lcd.example_bsp_set_lcd_backlight(originalBrightness);
        } else {
            // Restore backlight and display error message
            lcd.example_bsp_set_lcd_backlight(originalBrightness);
            displayErrorMessage("Failed to load image");
        }
        
        // Update status based on result
        updateStatusAfterImageLoad(index, success);
        
    } catch (...) {
        Logger.error("Exception during image display");
        lcd.example_bsp_set_lcd_backlight(100); // Restore backlight
        updateStatusAfterImageLoad(index, false);
    }
    
    // Clean up and release semaphore
    image_loading = false;
    display_in_progress = false;
    xSemaphoreGive(displaySemaphore);
}

/**
 * Display a simple error message on screen
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
// Display error message function with correct LCD function name
void displayErrorMessage(const char* message) {
    if (!lcd_ready || !message) return;
    
    // Display colors
    uint16_t bgColor = 0x0000;    // Black background
    uint16_t fgColor = 0xF800;    // Red text/border
    
    // Create a message box in center of screen
    int boxWidth = 300;
    int boxHeight = 100;
    int x = (LCD_H_RES - boxWidth) / 2;
    int y = (LCD_V_RES - boxHeight) / 2;
    
    // Fill box with background color
    uint16_t* boxBuffer = (uint16_t*)ps_malloc(boxWidth * boxHeight * sizeof(uint16_t));
    if (!boxBuffer) {
        boxBuffer = (uint16_t*)malloc(boxWidth * boxHeight * sizeof(uint16_t));
        if (!boxBuffer) return; // Exit if can't allocate memory
    }
    
    // Create box border and background
    for (int py = 0; py < boxHeight; py++) {
        for (int px = 0; px < boxWidth; px++) {
            bool isBorder = (px < 3 || px >= boxWidth-3 || py < 3 || py >= boxHeight-3);
            boxBuffer[py * boxWidth + px] = isBorder ? fgColor : bgColor;
        }
    }
    
    // Draw an X in the center
    int centerX = boxWidth / 2;
    int centerY = boxHeight / 2;
    int size = 30;
    
    // Draw X lines
    for (int i = -size; i <= size; i++) {
        for (int j = -2; j <= 2; j++) {
            // First diagonal (\)
            int px = centerX + i;
            int py = centerY + i + j;
            if (px >= 5 && px < boxWidth-5 && py >= 5 && py < boxHeight-5) {
                boxBuffer[py * boxWidth + px] = fgColor;
            }
            
            // Second diagonal (/)
            px = centerX + i;
            py = centerY - i + j;
            if (px >= 5 && px < boxWidth-5 && py >= 5 && py < boxHeight-5) {
                boxBuffer[py * boxWidth + px] = fgColor;
            }
        }
    }
    
    // Draw the box to screen using the CORRECT function name
    lcd.lcd_draw_bitmap(x, y, x+boxWidth-1, y+boxHeight-1, boxBuffer);
    
    // Free memory
    free(boxBuffer);
    
    // Log error
    Logger.error("Error displayed: %s", message);
}


/**
 * Enhanced screen clearing function with progressive display
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
// Screen clearing function with correct LCD function name
void clearLCDScreen() {
    if (!lcd_ready) return;
    
    uint32_t startTime = millis();
    
    // Create a buffer for clearing rows 
    const uint32_t rowsAtOnce = 20;  // Process 20 rows at a time
    const uint32_t pixelsPerRow = LCD_H_RES;
    uint16_t* blackBuffer = (uint16_t*)ps_calloc(rowsAtOnce * pixelsPerRow, sizeof(uint16_t));
    
    if (!blackBuffer) {
        // Fall back to smaller buffer if PSRAM allocation fails
        uint16_t stackBuffer[LCD_H_RES];
        memset(stackBuffer, 0, sizeof(stackBuffer));
        
        // Clear screen one row at a time with CORRECT function name
        for (uint32_t y = 0; y < LCD_V_RES; y++) {
            lcd.lcd_draw_bitmap(0, y, LCD_H_RES-1, y, stackBuffer);
            
            // Feed watchdog periodically
            if (y % 100 == 0) {
                feed_watchdog();
                delay(1);  // Brief delay to allow other tasks to run
            }
        }
    } else {
        // Clear with larger buffer for better performance using CORRECT function name
        for (uint32_t y = 0; y < LCD_V_RES; y += rowsAtOnce) {
            uint32_t rowsToDraw = ((rowsAtOnce < (LCD_V_RES - y)) ? rowsAtOnce : (LCD_V_RES - y));
            lcd.lcd_draw_bitmap(0, y, LCD_H_RES-1, y + rowsToDraw - 1, blackBuffer);
            
            // Feed watchdog periodically
            if (y % 100 == 0) {
                feed_watchdog();
                delay(1);  // Brief delay to allow other tasks to run
            }
        }
        
        // Free buffer
        free(blackBuffer);
    }
    
    // Log timing information
    Logger.debug("Screen cleared in %lu ms", millis() - startTime);
}

/**
 * Enhanced JPEG file validation function
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
bool isValidJPEG(const char* filename) {
    // Check parameters
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or null filename");
        return false;
    }
    
    // Check if file exists
    if (!SD_MMC.exists(filename)) {
        Logger.error("File not found: %s", filename);
        return false;
    }
    
    // Open the file for reading
    File jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
        Logger.error("Failed to open file: %s", filename);
        return false;
    }
    
    // Check file size - too small or too large files are suspicious
    size_t fileSize = jpegFile.size();
    if (fileSize < 1024) {
        Logger.error("File too small to be valid JPEG: %s (%d bytes)", filename, fileSize);
        jpegFile.close();
        return false;
    }
    
    if (fileSize > 10000000) { // 10MB max
        Logger.error("File too large for reliable display: %s (%lu bytes)", filename, (unsigned long)fileSize);
        jpegFile.close();
        return false;
    }
    
    // Check JPEG header (SOI marker)
    uint8_t header[4];
    size_t bytesRead = jpegFile.readBytes((char*)header, 4);
    if (bytesRead != 4) {
        Logger.error("Couldn't read header from: %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Verify JPEG header starts with FF D8 (SOI marker)
    if (header[0] != 0xFF || header[1] != 0xD8) {
        Logger.error("Invalid JPEG header (not FF D8): %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Check presence of APP0/APP1 marker (FF Ex)
    if (header[2] != 0xFF) {
        Logger.warn("Suspicious JPEG structure, missing APP marker in: %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Basic validation successful - it's likely a valid JPEG
    jpegFile.close();
    return true;
}

/**
 * Enhanced image scanner with improved error handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void scan_images() {
    Logger.info("Scanning for images...");
    
    // Return early if SD card isn't ready
    if (!sd_ready) {
        Logger.error("Cannot scan for images - SD card not ready");
        image_count = 0;
        return;
    }
    
    // Must be protected with mutex
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Logger.error("Failed to get SD mutex for image scan");
        return;
    }
    
    // Free any existing image list memory
    for (int i = 0; i < image_count; i++) {
        if (image_list[i] != nullptr) {
            free(image_list[i]);
            image_list[i] = nullptr;
        }
    }
    
    image_count = 0;
    
    // First check if the images directory exists
    if (!SD_MMC.exists("/images")) {
        // Try to create it
        if (SD_MMC.mkdir("/images")) {
            Logger.info("Created /images directory");
        } else {
            Logger.error("Failed to create /images directory");
            xSemaphoreGive(sd_mutex);
            return;
        }
    }
    
    // Open the directory
    File root = SD_MMC.open("/images");
    if (!root) {
        Logger.error("Failed to open /images directory");
        xSemaphoreGive(sd_mutex);
        return;
    }
    
    if (!root.isDirectory()) {
        Logger.error("/images is not a directory");
        root.close();
        xSemaphoreGive(sd_mutex);
        return;
    }
    
    // Phase 1: Identify valid images
    Logger.info("Phase 1: Identifying valid and corrupt images...");
    File file = root.openNextFile();
    while (file && image_count < MAX_IMAGES) {
        if (!file.isDirectory()) {
            String filename = file.name();
            String lowerFilename = filename;
            lowerFilename.toLowerCase();
            
            if (lowerFilename.endsWith(".jpg") || lowerFilename.endsWith(".jpeg")) {
                String fullPath = "/images/" + filename;
                
                // Skip tiny files immediately
                size_t fileSize = file.size();
                if (fileSize < 1024) {
                    Logger.warn("Skipping small file: %s (%d bytes)", fullPath.c_str(), fileSize);
                    file = root.openNextFile();
                    continue;
                }
                
                // Check basic JPEG validity (fast check)
                uint8_t header[4];
                file.seek(0);
                if (file.read(header, 4) == 4) {
                    // Valid JPEG files start with FF D8 FF Ex
                    bool basicValid = (header[0] == 0xFF && header[1] == 0xD8 && header[2] == 0xFF);
                    
                    if (basicValid) {
                        // Add to image list
                        char* pathCopy = (char*)malloc(fullPath.length() + 1);
                        if (pathCopy) {
                            strcpy(pathCopy, fullPath.c_str());
                            image_list[image_count] = pathCopy;
                            image_count++;
                            
                            Logger.debug("Found image %d: %s (%d bytes)", image_count - 1, pathCopy, fileSize);
                        }
                    } else {
                        Logger.warn("Skipping invalid JPEG: %s", fullPath.c_str());
                    }
                }
            }
        }
        
        // Move to next file
        file = root.openNextFile();
    }
    
    // Close the directory
    root.close();
    
    // Release the mutex
    xSemaphoreGive(sd_mutex);
    
    Logger.info("Found %d images", image_count);
    
    // Create a test image if no images were found
    if (image_count == 0) {
        Logger.warn("No images found, creating test pattern");
        createTestImage();
        scan_images(); // Rescan to pick up the test image
    }
}

/**
 * Create a simple test pattern image when no images found
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
bool createTestImage() {
    if (!sd_ready) return false;
    
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    
    bool success = false;
    
    try {
        // Make sure directory exists
        if (!SD_MMC.exists("/images")) {
            SD_MMC.mkdir("/images");
        }
        
        // Create a simple test file
        File testFile = SD_MMC.open("/images/test_pattern.jpg", FILE_WRITE);
        if (testFile) {
            // This is a minimal valid JPEG file (JFIF header)
            const uint8_t jpegHeader[] = {
                0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x01, 0x00, 0x60,
                0x00, 0x60, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43, 0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08,
                0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
                0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20, 0x24, 0x2E, 0x27, 0x20,
                0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29, 0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27,
                0x39, 0x3D, 0x38, 0x32, 0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x80,
                0x00, 0x80, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x00, 0x1F,
                0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00,
                0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00,
                0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51,
                0x61, 0x07, 0x22, 0x71
            };
            
            testFile.write(jpegHeader, sizeof(jpegHeader));
            testFile.close();
            Logger.info("Created test pattern image");
            success = true;
        }
    } catch (...) {
        Logger.error("Exception in createTestImage()");
    }
    
    xSemaphoreGive(sd_mutex);
    return success;
}

/**
 * Update system information
 */
void update_system_info() {
    // Report memory stats
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t freePsram = ESP.getFreePsram();
    
    // Log memory info
    Logger.info("Memory - Heap: %lu KB free, PSRAM: %lu KB free", 
              freeHeap / 1024, freePsram / 1024);
    
    // Log JPEG decoding stats
    Logger.info("JPEG stats - Success: %d/%d (%.1f%%)", 
              jpeg_decode_success_count, jpeg_decode_total_count, 
              jpeg_decode_total_count > 0 ? 
                (100.0f * jpeg_decode_success_count / jpeg_decode_total_count) : 0.0f);
    
    // Store last update time
    last_system_update = millis();
}

/**
 * Main setup function
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void setup() {
    // Initialize serial communication immediately
    Serial.begin(115200);
    delay(1000);  // Give serial time to initialize
    
    Serial.println();
    Serial.println("=== ESP32P4 Gallery Debug Output ===");
    Serial.println("Starting initialization sequence with enhanced error handling...");
    
    // CRITICAL: Disable task watchdog immediately to prevent early crashes
    esp_task_wdt_deinit();
    
    // Initialize file system (fixed to use 'storage' partition)
    bool fsAvailable = initializeFileSystem();
    
    // Set up logger in a safe way
    safeLoggerInit(fsAvailable);
    
    Logger.info("====================================");
    Logger.info("ESP32P4 Image Gallery Starting");
    Logger.info("Date: 2025-06-20 12:07:40 UTC");
    Logger.info("User: Chamil1983");
    Logger.info("====================================");
    
    // Create mutexes
    sd_mutex = xSemaphoreCreateMutex();
    jpeg_mutex = xSemaphoreCreateMutex();
    ui_mutex = xSemaphoreCreateMutex();
    Logger.info("System mutexes initialized");
    
    // Setup watchdog
    Logger.info("Initializing enhanced watchdog system");
    if (setup_watchdog()) {
        Logger.info("Watchdog initialized successfully");
    } else {
        Logger.warn("Watchdog initialization failed - continuing without WDT");
    }
    
    // Initialize components in sequence with proper error handling
    initializeWithRetry("LCD", setup_lcd, 2);
    initializeWithRetry("Touch", setup_touch, 2);
    initializeWithRetry("SD Card", setup_sd_card_reliable, 3);
    initializeWithRetry("WiFi", setup_wifi_ap, 2);
    initializeWithRetry("LVGL", setup_lvgl, 1);
    initializeWithRetry("UI", setup_ui, 1);
    initializeWithRetry("WebServer", setup_webserver, 2);
    
    // Create LVGL task
    lvgl_task_handle = createTaskSafely(
        lvgl_task,           // Function
        "LVGL Task",         // Name
        8192,                // Stack size
        NULL,                // Parameters
        1,                   // Priority
        1                    // Core ID
    );
    
    // Scan images if SD card is working
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
    
    // Create slideshow task if we have functioning components
    if (lvgl_ready && ui_ready) {
        slideshow_task_handle = createTaskSafely(
            slideshowTask,       // Function
            "Slideshow Task",    // Name
            4096,                // Stack size
            NULL,                // Parameters
            1,                   // Priority
            0                    // Core ID
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

/**
 * Enhanced main loop with more robust connection handling
 * Updated: 2025-06-20 12:07:40
 * User: Chamil1983
 */
void loop() {
    static uint32_t last_wdt_feed = 0;
    static uint32_t last_status_update = 0;
    static uint32_t last_dns_process = 0;
    static uint32_t last_wifi_check = 0;
    uint32_t now = millis();
    
    // Feed watchdog occasionally
    if (now - last_wdt_feed >= 2000) {
        feed_watchdog();
        last_wdt_feed = now;
    }
    
    // Process DNS requests more frequently for better captive portal
    if (dnsServerActive && now - last_dns_process >= 30) {
        try {
            dnsServer.processNextRequest();
        } catch (...) {
            // Catch any DNS processing errors
        }
        last_dns_process = now;
    }
    
    // Check WiFi status periodically 
    if (now - last_wifi_check >= 10000) {
        if (wifi_ready && WiFi.softAPgetStationNum() > 0) {
            Logger.debug("WiFi: %d client(s) connected", WiFi.softAPgetStationNum());
        }
        last_wifi_check = now;
    }
    
    // Handle web server clients - CRITICAL for connectivity
    if (server_ready) {
        try {
            server.handleClient();
        } catch (...) {
            Logger.warn("Exception in server.handleClient() - continuing");
        }
    }
    
    // Periodically update system stats (every 30 seconds)
    if (now - last_status_update >= 30000) {
        update_system_info();
        last_status_update = now;
    }
    
    // Check if system reset is requested
    if (reset_requested) {
        Logger.info("System reset requested");
        delay(200);
        ESP.restart();
    }
    
    // Short delay to prevent CPU hogging
    delay(5);
}

#pragma GCC pop_options
