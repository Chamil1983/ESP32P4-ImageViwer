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
#include "lv_conf.h"  // File link (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/blob/main/Image_ViwerV4/lv_conf.h)
#include "pins_config.h" // Board pin configuration (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/blob/main/Image_ViwerV4/pins_config.h)
#include "debug_logger.h"  // Debug Library link (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/blob/main/Image_ViwerV4/debug_logger.h) and (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/blob/main/Image_ViwerV4/debug_logger.cpp)
#include "src/lcd/jd9365_lcd.h" // Library for LCD (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/tree/main/src/lcd)
#include "src/touch/gsl3680_touch.h"  // Library for Touch (https://github.com/Chamil1983/ESP32P4_LCDJC8012P4A1C/tree/main/src/touch) 

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
// Add these to your global variables section
int16_t jpeg_x_offset = 0;
int16_t jpeg_y_offset = 0;

// Mutex for thread safety
SemaphoreHandle_t sd_mutex = NULL;
SemaphoreHandle_t jpeg_mutex = NULL;
SemaphoreHandle_t ui_mutex = NULL;
SemaphoreHandle_t lvgl_mutex = NULL;

// Control flags
volatile bool wdt_enabled = false;
volatile bool reset_requested = false;

// Task handles
TaskHandle_t lvgl_task_handle = NULL;
TaskHandle_t main_task_handle = NULL;

/**
 * ESP32P4 Image Gallery - Function Declarations
 * Updated: 2025-06-17 11:05:01
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

#define DEBUG_INIT(tag) DEBUG_PRINT(tag, "Initializing...")
#define DEBUG_SUCCESS(tag) DEBUG_PRINT(tag, "Initialization successful")
#define DEBUG_FAIL(tag, reason) DEBUG_PRINT(tag, "Initialization failed: %s", reason)

// Template for safe execution with watchdog feeding
template<typename F>
bool safe_execute_with_watchdog(F&& func, const char* operation_name) {
  unsigned long start_time = millis();
  bool result = false;
  
  try {
    result = func();
  } catch (...) {
    Logger.error("Exception in %s", operation_name);
  }
  
  unsigned long duration = millis() - start_time;
  
  if (duration > 1000) {
    Logger.warn("Operation %s took %lu ms", operation_name, duration);
  }
  
  return result;
}

/**
 * Fixed LittleFS initialization with proper partition name
 * Updated: 2025-06-17 12:08:31
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
    
    // Format doesn't take a partition name parameter
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
 * Date: 2025-06-15 11:51:10
 * User: Chamil1983
 * 
 * Call this if SPIFFS operations fail during runtime
 */
bool emergency_spiffs_recovery() {
    Logger.warn("Attempting emergency SPIFFS recovery");
    
    // End the SPIFFS filesystem without checking if mounted
    // Arduino ESP32 doesn't provide a direct way to check if SPIFFS is mounted
    SPIFFS.end();
    Logger.info("SPIFFS ended for recovery");
    
    // Short delay to ensure cleanup
    delay(500);
    
    // Format the partition
    bool format_result = SPIFFS.format();
    if (!format_result) {
        Logger.error("SPIFFS format failed");
        return false;
    }
    
    Logger.info("SPIFFS formatted successfully");
    
    // Try to mount again
    bool mount_result = SPIFFS.begin(false);  // false = don't format again
    if (!mount_result) {
        Logger.error("SPIFFS remount failed after format");
        return false;
    }
    
    Logger.info("SPIFFS recovery successful");
    return true;
}

/**
 * Enhanced slideshow task initialization with safe WDT registration
 * Updated: 2025-06-17 10:25:58
 * User: Chamil1983
 */
void createSlideshowTask() {
    // Create slideshow task on core 0 with lower priority to avoid resource conflicts
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

    // Register slideshow task with watchdog ONLY if task creation succeeded
    if (slideshow_task_handle != NULL) {
        // Try to add the task to the watchdog, but don't crash if it fails
        esp_err_t err = ESP_OK;
        for (int retry = 0; retry < 3; retry++) {
            err = esp_task_wdt_add(slideshow_task_handle);
            if (err == ESP_OK) {
                Logger.info("Slideshow task added to watchdog successfully");
                break;
            } else {
                Logger.warn("Failed to add slideshow task to watchdog (retry %d): %d", 
                          retry + 1, err);
                delay(50);
            }
        }
        
        if (err != ESP_OK) {
            // Continue without watchdog for this task, it's not critical
            Logger.warn("Will continue without watchdog for slideshow task");
        }
    } else {
        Logger.error("Failed to create slideshow task");
    }
}

/**
 * Safe initialization function that retries critical components
 * Updated: 2025-06-17 10:25:58
 * User: Chamil1983
 */
bool initializeWithRetry(const char* componentName, bool (*initFunction)(), int maxRetries = 3) {
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
            delay(500 * (retry + 1)); // Progressive backoff
        }
    }
    
    if (!success) {
        Logger.error("%s initialization failed after %d attempts", componentName, maxRetries);
    }
    
    return success;
}

/**
 * Completely reworked setup() function for maximum stability
 * Updated: 2025-06-17 10:56:36
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
    
    // Initialize file system - use LittleFS instead of SPIFFS
    bool fsAvailable = initializeFileSystem();
    
    // Set up logger in a safe way
    safeLoggerInit(fsAvailable);
    
    Logger.info("====================================");
    Logger.info("ESP32P4 Image Gallery Starting");
    Logger.info("Date: 2025-06-17 10:56:36 UTC");
    Logger.info("User: Chamil1983");
    Logger.info("====================================");
    
    // Create mutexes
    sd_mutex = xSemaphoreCreateMutex();
    jpeg_mutex = xSemaphoreCreateMutex();
    ui_mutex = xSemaphoreCreateMutex();
        // Create LVGL mutex
    lvgl_mutex = xSemaphoreCreateMutex();
    if (lvgl_mutex == NULL) {
        Logger.error("Failed to create LVGL mutex");
    }
    Logger.info("System mutexes initialized");
    
    // Setup watchdog with much safer implementation
    Logger.info("Initializing enhanced watchdog system");
    if (setup_watchdog()) {
        Logger.info("Watchdog initialized successfully");
    } else {
        Logger.warn("Watchdog initialization failed - continuing without WDT");
    }
    
    // Reset ESP-IDF log levels to minimize spam
    esp_log_level_set("*", ESP_LOG_NONE);
    
    // Initialize components in sequence with proper error handling
    initializeWithRetry("LCD", setup_lcd, 2);
    initializeWithRetry("Touch", setup_touch, 2);
    initializeWithRetry("SD Card", setup_sd_card_reliable, 3);
    initializeWithRetry("WiFi", setup_wifi_ap, 2);
    initializeWithRetry("LVGL", setup_lvgl, 1);
    initializeWithRetry("UI", setup_ui, 1);
    initializeWithRetry("WebServer", setup_webserver, 2);
    
    // Create LVGL task with safety
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
 * Updated: 2025-06-17 11:58:18
 * User: Chamil1983
 */
void loop() {
    static uint32_t last_wdt_feed = 0;
    static uint32_t last_status_update = 0;
    static uint32_t last_dns_process = 0;
    static uint32_t last_wifi_check = 0;
    static uint32_t last_connection_report = 0;
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
            if (now - last_connection_report >= 60000) { // Report every minute
                Logger.info("WiFi status: %d clients connected", WiFi.softAPgetStationNum());
                last_connection_report = now;
            }
        } else if (wifi_ready && WiFi.softAPgetStationNum() == 0) {
            Logger.debug("WiFi AP running but no clients connected");
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
    
    // Check if system reset is requested
    if (reset_requested) {
        Logger.info("System reset requested");
        delay(200);
        ESP.restart();
    }
    
    // Short delay to prevent CPU hogging
    delay(5);
}

/**
 * Enhanced SD card writing function to replace direct File.write
 * Updated: 2025-06-17 11:58:18
 * User: Chamil1983
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
        size_t currentChunk = min(chunkSize, size - offset);
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
 * Add a special test page for diagnosing connectivity issues
 * Updated: 2025-06-17 11:22:17
 * User: Chamil1983
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
        response += "<li><strong>WiFi Channel:</strong> 1</li>";
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
 * Custom task watchdog reset function that doesn't fail on task not found
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
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
 * Function to check and update buttons on touch input
 * Updated: 2025-06-17 10:42:10
 * User: Chamil1983
 */
void check_buttons() {
    if (!touch_ready) return;
    
    // Check for touch events
    uint16_t x, y;
    bool touched = touch.getTouch(&x, &y);
    
    if (touched) {
        // Process the touch event
        handle_touch_event(x, y);
    }
}

/**
 * Enhanced system health check function with better error handling
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
void performSystemHealthCheck() {
    // Always use try/catch for safety
    try {
        Logger.debug("Performing system health check");
        
        // Check SD card - use timeout for semaphore
        if (!sd_ready) {
            // Use shorter timeout and don't block if can't get mutex
            if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                try {
                    Logger.warn("SD card not ready, attempting recovery");
                    emergency_sd_recovery();
                } catch (...) {
                    Logger.error("Exception during SD recovery");
                }
                xSemaphoreGive(sd_mutex);
            }
        }
        
        // Check WiFi - don't retry too aggressively
        static uint32_t last_wifi_retry = 0;
        if (!wifi_ready && millis() - last_wifi_retry > 60000) {  // Once per minute
            last_wifi_retry = millis();
            Logger.warn("WiFi not ready, attempting recovery");
            try {
                setup_wifi_ap();
            } catch (...) {
                Logger.error("Exception during WiFi recovery");
            }
        }
        
        // Report memory status
        Logger.debug("Memory - Heap: %u KB free, PSRAM: %u KB free", 
                   ESP.getFreeHeap() / 1024, 
                   ESP.getFreePsram() / 1024);
    } catch (...) {
        // Catch any exceptions to prevent crashes
        Serial.println("Exception in health check - continuing");
    }
}

/**
 * Safe logger initialization that can operate without SPIFFS
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
void safeLoggerInit(bool spiffsAvailable) {
    // First try normal initialization
    try {
        if (spiffsAvailable) {
            Logger.init(true, true, LOG_LEVEL_DEBUG);
        } else {
            Logger.init(true, false, LOG_LEVEL_DEBUG);
        }
        
        // Force output level for better debugging
        Logger.setLogLevel(LOG_LEVEL_DEBUG);
        Logger.enableSerialOutput(true);
    } catch (...) {
        // If logger initialization fails, set up a minimal fallback
        Serial.println("Logger initialization failed, using serial-only logging");
        
        // Try again with minimal settings
        try {
            Logger.init(true, false, LOG_LEVEL_DEBUG);
            Logger.setLogLevel(LOG_LEVEL_DEBUG);
            Logger.enableSerialOutput(true);
        } catch (...) {
            // If that still fails, we'll just have to use Serial directly
            Serial.println("CRITICAL: Logger completely failed to initialize");
        }
    }
}


/**
 * Safe task creation that handles errors gracefully
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
TaskHandle_t createTaskSafely(TaskFunction_t taskFunction, const char* name, 
                             uint32_t stackSize, void *param, UBaseType_t priority, 
                             BaseType_t coreID) {
    TaskHandle_t handle = NULL;
    
    // Try to create the task
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,
        name,
        stackSize,
        param,
        priority,
        &handle,
        coreID
    );
    
    if (result != pdPASS || handle == NULL) {
        Logger.error("Failed to create task: %s", name);
        return NULL;
    }
    
    Logger.info("Created task: %s on core %d", name, coreID);
    return handle;
}

/**
 * Safe task deletion helper function
 * Updated: 2025-06-17 10:25:58
 * User: Chamil1983
 */
void safeDeleteTask(TaskHandle_t taskHandle, const char* taskName) {
    if (taskHandle != NULL) {
        Logger.info("Safely deleting task: %s", taskName);
        
        // First remove from watchdog to prevent errors
        esp_err_t err = esp_task_wdt_delete(taskHandle);
        if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
            Logger.warn("Could not remove %s from watchdog: %d", taskName, err);
        }
        
        // Then delete the task
        vTaskDelete(taskHandle);
        
        // Log completion
        Logger.info("Task %s deleted", taskName);
    }
}

/**
 * Handle touch events on the screen with better debouncing
 * Updated: 2025-06-17 10:03:45
 * User: Chamil1983
 */
void handle_touch_event(uint16_t x, uint16_t y) {
    // Static variables for debouncing
    static uint32_t last_touch_time = 0;
    static uint16_t last_touch_x = 0;
    static uint16_t last_touch_y = 0;
    static bool touch_processed = false;
    
    uint32_t now = millis();
    
    // Enforce minimum time between touch events (500ms)
    if (now - last_touch_time < 500) {
        // Check if this is the same touch point (within small radius)
        int dx = abs(x - last_touch_x);
        int dy = abs(y - last_touch_y);
        if (dx < 50 && dy < 50 && touch_processed) {
            // Ignore this touch - it's likely the same touch event
            return;
        }
    } else {
        // Reset processed flag for new touch
        touch_processed = false;
    }
    
    // Store current touch information
    last_touch_time = now;
    last_touch_x = x;
    last_touch_y = y;
    
    // Screen dimensions
    int screenHeight = lcd.height();
    int screenWidth = lcd.width();
    
    Logger.debug("Touch at x=%d, y=%d (screen: %dx%d)", x, y, screenWidth, screenHeight);
    
    // Check touch zones - left third, right third, middle third, or top status bar
    if (y < screenHeight / 10) {
        // Top status bar - toggle slideshow status
        if (!touch_processed) {
            if (slideshow_active) {
                stopSlideshow();
            } else {
                startSlideshow(5000); // 5 seconds interval
            }
            touch_processed = true;
        }
    }
    // Left third of screen - previous image
    else if (x < screenWidth / 3) {
        if (!touch_processed) {
            if (image_count > 0) {
                int prevImage = (current_image > 0) ? (current_image - 1) : (image_count - 1);
                display_image(prevImage);
            }
            touch_processed = true;
        }
    }
    // Right third of screen - next image
    else if (x > (screenWidth * 2) / 3) {
        if (!touch_processed) {
            if (image_count > 0) {
                int nextImage = (current_image < image_count - 1) ? (current_image + 1) : 0;
                display_image(nextImage);
            }
            touch_processed = true;
        }
    }
    // Middle of screen - toggle slideshow with longer interval
    else {
        if (!touch_processed) {
            Logger.info("Touch detected in middle zone - toggling slideshow with longer interval");
            if (slideshow_active) {
                stopSlideshow();
            } else {
                startSlideshow(10000); // 10 second interval
            }
            touch_processed = true;
        }
    }
}

/**
 * Fixed slideshowTask with proper image cycling
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
void slideshowTask(void *parameter) {
    Logger.info("Slideshow task started");
    
    // Allow task to stabilize before entering main loop
    delay(500);
    Logger.info("Slideshow task initialized");
    
    // Initialize variables
    uint32_t last_image_change = 0;
    int last_displayed_image = -1;  // Track last displayed image
    bool mutex_was_taken = false;
    
    while (true) {
        uint32_t now = millis();
        
        // Process slideshow logic only if slideshow is active
        if (slideshow_active && now - last_image_change >= slideshow_interval) {
            Logger.debug("Slideshow advancing to next image");
            
            if (image_count > 0) {
                // IMPORTANT FIX: Calculate next image index with proper cycling
                int next_image = (current_image + 1) % image_count;
                
                // If it's the same as last displayed, try to move forward
                if (next_image == last_displayed_image && image_count > 1) {
                    next_image = (next_image + 1) % image_count;
                }
                
                // Clear mutex_was_taken flag before attempting to take mutex
                mutex_was_taken = false;
                
                // Try to get SD mutex with timeout
                if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                    mutex_was_taken = true;
                    
                    try {
                        // Display next image
                        display_image(next_image);
                        last_displayed_image = next_image;
                        
                        // Log the advancement
                        Logger.debug("Slideshow advanced to image %d", next_image);
                    } catch (...) {
                        Logger.error("Exception in slideshow task - continuing");
                    }
                    
                    // Release the mutex when done
                    xSemaphoreGive(sd_mutex);
                    mutex_was_taken = false;
                } else {
                    Logger.error("Failed to take SD mutex for JPEG loading");
                }
                
                // Safety check - release mutex if somehow still held
                if (mutex_was_taken) {
                    xSemaphoreGive(sd_mutex);
                }
            }
            
            // Update last image change time
            last_image_change = now;
        }
        
        // Feed watchdog to keep task active
        feed_watchdog();
        
        // Delay to prevent CPU hogging
        delay(slideshow_active ? 50 : 200);
    }
}

/**
 * Find next valid image, skipping corrupted ones
 * Updated: 2025-06-18 13:20:43
 * User: Chamil1983
 */
int findNextValidImage(int currentIndex) {
    if (image_count <= 0) {
        return -1; // No images available
    }
    
    // Start with the next image
    int nextIndex = (currentIndex + 1) % image_count;
    int startIndex = nextIndex; // Remember where we started
    
    // Try to find a valid image
    do {
        // Check if this image is valid without accessing SD card
        const char* filename = image_list[nextIndex];
        if (filename && strlen(filename) > 0 && 
            (strstr(filename, ".jpg") || strstr(filename, ".jpeg"))) {
            return nextIndex;
        }
        
        // Try next image
        nextIndex = (nextIndex + 1) % image_count;
    } while (nextIndex != startIndex); // Avoid infinite loop
    
    // If we got here, we couldn't find a valid image
    return currentIndex; // Return original index as fallback
}


/**
 * Enhanced startSlideshow function
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
void startSlideshow(uint32_t interval_ms) {
    if (interval_ms < 3000) interval_ms = 3000; // Minimum 3 seconds
    
    Logger.info("Starting slideshow with %d ms interval", interval_ms);
    
    // Set slideshow parameters
    slideshow_interval = interval_ms;
    
    // Enable slideshow
    slideshow_active = true;
}

/**
 * Enhanced stopSlideshow function
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
void stopSlideshow() {
    Logger.info("Stopping slideshow");
    
    // Disable slideshow
    slideshow_active = false;
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
 * Return true if rotation is needed, false otherwise
 */
bool check_image_needs_rotation(int img_width, int img_height) {
    // Screen is 800x1280 (portrait orientation)
    // Only rotate if image is in landscape (width > height)
    return (img_width > img_height);
}

/**
 * Enhanced LVGL task with proper error handling to fix core dumps
 * Updated: 2025-06-19 11:56:17
 * User: Chamil1983
 */
void lvgl_task(void *pvParameters) {
    Logger.info("LVGL task started on core %d", xPortGetCoreID());
    
    // Add this task to watchdog if supported
    esp_task_wdt_add(NULL);
    
    // Initialize timing variables
    uint32_t last_tick = millis();
    uint32_t last_watchdog_feed = millis();
    
    // Fixed scheduling to avoid hangs
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    
    while (true) {
        // Current time
        uint32_t now = millis();
        
        // Feed the watchdog periodically
        if (now - last_watchdog_feed >= 1000) {
            esp_task_wdt_reset();
            feed_watchdog();
            last_watchdog_feed = now;
        }
        
        // Run LVGL tasks safely
        try {
            // Calculate time elapsed
            uint32_t elapsed = now - last_tick;
            
            // Take UI mutex with timeout
            if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Update LVGL
                lv_tick_inc(elapsed);
                lv_timer_handler();
                
                // Release UI mutex
                xSemaphoreGive(ui_mutex);
            }
            
            // Update last tick time
            last_tick = now;
        } catch (...) {
            // Log any exceptions but continue
            Logger.error("Exception in LVGL task - continuing");
            
            // Make sure mutex is released in case of exception
            xSemaphoreGive(ui_mutex);
        }
        
        // Sleep to avoid CPU overload - use vTaskDelay for better scheduling
        vTaskDelay(xFrequency);
    }
}

/**
 * Improved image scaling function with screen ratio consideration
 * Updated: 2025-06-19 10:41:58
 * User: Chamil1983
 */
void calculateBestFit(uint16_t imageWidth, uint16_t imageHeight, uint16_t screenWidth, uint16_t screenHeight,
                     uint16_t& scaledWidth, uint16_t& scaledHeight, uint16_t& xOffset, uint16_t& yOffset) {
    // Calculate aspect ratio of image and screen
    float imageRatio = (float)imageWidth / imageHeight;
    float screenRatio = (float)screenWidth / screenHeight;
    
    // Determine if we need to fit by width or height
    if (imageRatio > screenRatio) {
        // Image is wider than screen (relative to height)
        scaledWidth = screenWidth;
        scaledHeight = (uint16_t)(screenWidth / imageRatio);
        xOffset = 0;
        yOffset = (screenHeight - scaledHeight) / 2;
    } else {
        // Image is taller than screen (relative to width)
        scaledHeight = screenHeight;
        scaledWidth = (uint16_t)(screenHeight * imageRatio);
        xOffset = (screenWidth - scaledWidth) / 2;
        yOffset = 0;
    }
}

/**
 * Enhanced render_jpeg_file with proper screen scaling
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
bool render_jpeg_file(const char* filename) {
    if (!filename || strlen(filename) == 0) {
        Logger.error("Invalid filename for rendering");
        return false;
    }
    
    Logger.info("Starting JPEG decoding for %s", filename);
    
    // Open file
    File jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
        Logger.error("Failed to open file: %s", filename);
        return false;
    }
    
    bool success = false;
    
    try {
        // Take JPEG mutex
        if (xSemaphoreTake(jpeg_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            Logger.error("Failed to take JPEG mutex for decoding");
            jpegFile.close();
            return false;
        }
        
        // Open JPEG for decoding
        success = jpeg.open(jpegFile, jpeg_draw_callback);
        
        if (success) {
            // Get image dimensions
            int width = jpeg.getWidth();
            int height = jpeg.getHeight();
            
            // Calculate scale option based on image and screen dimensions
            // CRITICAL: This handles both orientations to fit the screen properly
            int options = 0; // Default: no scaling or rotation
            
            // Calculate aspect ratios to decide on best orientation
            float img_aspect = (float)width / height;
            float screen_aspect_portrait = 800.0f / 1280.0f;
            float screen_aspect_landscape = 1280.0f / 800.0f;
            
            // Determine if the image needs rotation for best fit
            bool needs_rotation = false;
            
            if (img_aspect > 1.0f) {  // Landscape image
                if (img_aspect < screen_aspect_landscape) {
                    // Landscape image fits better in portrait orientation
                    needs_rotation = true;
                }
            } else {  // Portrait image
                if (img_aspect > screen_aspect_portrait) {
                    // Portrait image fits better in landscape orientation
                    needs_rotation = false;
                } else {
                    // Standard portrait image fits better in portrait mode
                    needs_rotation = false;
                }
            }
            
            // Set rotation option - 0=0°, 2=180°, 1=90°, 3=270°
            // For most landscape images on a portrait screen, we use 3 (270°)
            options = needs_rotation ? 3 : 0;
            
            // SCALE option bits:
            // 0 = full size, 1 = half size, 2 = quarter size
            // Add scale option based on image size to fit screen
            int scaleOption = 0;  // Default: full size
            
            if (width > 1600 || height > 1600) {
                scaleOption = 1;  // Half size for very large images
            } else if (width > 3200 || height > 3200) {
                scaleOption = 2;  // Quarter size for extremely large images
            }
            
            // Combine rotation and scale options
            options |= (scaleOption << 4);  // Scale in bits 4-5
            
            // Decode with calculated options
            success = jpeg.decode(0, 0, options);
            
            if (success) {
                // Log success with dimensions 
                if (needs_rotation) {
                    // Rotated dimensions are swapped
                    Logger.info("JPEG decoded successfully (rotated): %dx%d -> %dx%d", 
                              width, height, height, width);
                } else {
                    Logger.info("JPEG decoded successfully: %dx%d", width, height);
                }
            } else {
                Logger.error("JPEG decoding failed");
            }
            
            // Close JPEG decoder
            jpeg.close();
        } else {
            Logger.error("Failed to open JPEG for decoding");
        }
        
        // Release JPEG mutex
        xSemaphoreGive(jpeg_mutex);
        
    } catch (...) {
        Logger.error("Exception during JPEG decoding");
        
        // Make sure mutex is released and file closed
        xSemaphoreGive(jpeg_mutex);
        jpegFile.close();
        return false;
    }
    
    // Close the file
    jpegFile.close();
    
    return success;
}

/**
 * JPEG drawing callback function using direct LCD API calls
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
int jpeg_draw_callback(JPEGDRAW *pDraw) {
    if (!lcd_ready) return 0;  // Skip if LCD not ready
    
    // Logging for debug
    if (pDraw->x >= 800 || pDraw->y >= 1280 || 
        pDraw->x + pDraw->iWidth > 800 || pDraw->y + pDraw->iHeight > 1280) {
        
        Logger.warn("Draw coordinates out of bounds: x=%d, y=%d, w=%d, h=%d", 
                  pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
        // Continue anyway - the driver will clip it
    }
    
    // Use the lcd object directly (from your driver)
    // This is critical - using the lcd instance instead of an undefined tft object
    lcd.lcd_draw_bitmap(
        pDraw->x,                  // Starting X position
        pDraw->y,                  // Starting Y position
        pDraw->x + pDraw->iWidth,  // End X position
        pDraw->y + pDraw->iHeight, // End Y position
        (uint16_t*)pDraw->pPixels  // Pixel data
    );
    
    // Return 1 to indicate success
    return 1;
}


/**
 * Fixed display_image function to center images properly
 * Updated: 2025-06-19 12:09:31
 * User: Chamil1983
 */
void display_image(int index) {
    if (index < 0 || index >= image_count) {
        Logger.error("Invalid image index: %d", index);
        return;
    }
    
    // Log which image is being displayed
    Logger.info("Displaying image %d: %s", index, image_list[index]);
    
    // Update current image index
    current_image = index;
    
    // Clear screen for clean display
    clearLCDScreen();
    
    // Check if mutex is already held
    bool mutex_taken_externally = (xSemaphoreGetMutexHolder(sd_mutex) == xTaskGetCurrentTaskHandle());
    
    // Take SD mutex if not already held
    bool mutex_taken = false;
    if (!mutex_taken_externally) {
        mutex_taken = (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) == pdTRUE);
        if (!mutex_taken) {
            Logger.error("Failed to take SD mutex for JPEG loading");
            displayErrorMessage("Failed to access SD card");
            return;
        }
    }
    
    // Try to render the JPEG
    bool success = render_jpeg_file(image_list[index]);
    
    // Release mutex if we took it
    if (mutex_taken) {
        xSemaphoreGive(sd_mutex);
    }
    
    // Update UI with result
    updateStatusAfterImageLoad(index, success);
    
    if (success) {
        Logger.info("Successfully displayed image %d", index);
    } else {
        Logger.error("Failed to display image %d", index);
        displayErrorMessage("Image could not be displayed");
    }
}

/**
 * SD card recovery function when mutex operations fail
 * Updated: 2025-06-18 13:20:43
 * User: Chamil1983
 */
void emergency_sd_mutex_recovery() {
    Logger.warn("Attempting emergency SD mutex recovery");
    
    // First try normal semaphore give in case mutex is stuck
    xSemaphoreGive(sd_mutex);
    
    // Create a new mutex as a last resort
    SemaphoreHandle_t old_mutex = sd_mutex;
    sd_mutex = xSemaphoreCreateMutex();
    
    if (sd_mutex == NULL) {
        // Failed to create new mutex, restore old one
        sd_mutex = old_mutex;
        Logger.error("Failed to create new SD mutex in recovery");
    } else {
        // Successfully created new mutex
        vSemaphoreDelete(old_mutex);
        Logger.info("SD mutex recreated in recovery");
    }
}


/**
 * Display an error message on the LCD when image loading fails
 * Updated: 2025-06-16 10:45:51
 * User: Chamil1983
 */
void displayErrorMessage(const char* message) {
    if (!lcd_ready || !message) return;
    
    // Define colors
    uint16_t textColor = 0xFFFF;   // White
    uint16_t bgColor = 0x0000;     // Black
    uint16_t borderColor = 0xF800; // Red
    
    // Create a buffer for the error message backdrop
    int boxWidth = min((int)lcd.width() - 40, 320);
    int boxHeight = 60;
    int boxX = (lcd.width() - boxWidth) / 2;
    int boxY = (lcd.height() - boxHeight) / 2;
    
    // The LCD class doesn't support text rendering or pixel drawing directly
    // So we'll create a simple box by filling it with color using lcd_draw_bitmap
    
    // Create a buffer for a small part of the box
    const int stripHeight = 10;
    uint16_t* boxBuffer = (uint16_t*)malloc(boxWidth * stripHeight * sizeof(uint16_t));
    if (!boxBuffer) {
        Logger.error("Failed to allocate memory for error display");
        return;
    }
    
    // Draw the error box in strips
    for (int strip = 0; strip < boxHeight; strip += stripHeight) {
        int currentHeight = min(stripHeight, boxHeight - strip);
        
        // Fill the buffer with appropriate colors
        for (int y = 0; y < currentHeight; y++) {
            for (int x = 0; x < boxWidth; x++) {
                // Calculate actual y position in the box
                int actualY = strip + y;
                
                // Determine if this pixel is on the border (4px thick)
                bool isBorder = (actualY < 4 || actualY >= boxHeight - 4 ||
                                x < 4 || x >= boxWidth - 4);
                
                // Default to background color
                uint16_t pixelColor = bgColor;
                
                // If on border, use border color
                if (isBorder) {
                    pixelColor = borderColor;
                }
                // Otherwise check if we should draw error pattern
                else {
                    // Draw simple text in center - diagonal cross pattern to show error
                    int centerY = boxHeight / 2;
                    int centerX = boxWidth / 2;
                    
                    // Create a simple X pattern in the middle
                    if (actualY > centerY - 15 && actualY < centerY + 15) {
                        // First diagonal line (\)
                        if (abs((actualY - centerY) - (x - centerX)) < 3) {
                            pixelColor = textColor;
                        }
                        // Second diagonal line (/)
                        else if (abs((actualY - centerY) + (x - centerX)) < 3) {
                            pixelColor = textColor;
                        }
                    }
                }
                
                // Set the pixel color in the buffer
                boxBuffer[y * boxWidth + x] = pixelColor;
            }
        }
        
        // Draw this strip to the LCD
        lcd.lcd_draw_bitmap(boxX, boxY + strip, 
                          boxX + boxWidth, boxY + strip + currentHeight, 
                          boxBuffer);
        
        // Small delay between drawing operations
        delay(5);
    }
    
    // Free the buffer
    free(boxBuffer);
    
    // Wait a moment to ensure drawing completes
    delay(50);
    
    // Since text rendering isn't available, the message is shown as an X pattern
    // Log the error message instead
    Logger.error("Error displayed on screen: %s", message);
}

/**
 * Enhanced screen clearing function with better error handling
 * Updated: 2025-06-17 10:03:45
 * User: Chamil1983
 */
void clearLCDScreen() {
    if (!lcd_ready) return;
    
    // Track timing
    uint32_t startTime = millis();
    
    // Set LCD backlight to dim during transition
    lcd.example_bsp_set_lcd_backlight(50); // Dim during transition
    
    // Create a black buffer using PSRAM if available
    uint32_t bufferWidth = lcd.width();
    uint32_t bufferHeight = 15; // Process 15 rows at a time
    size_t bufferSize = bufferWidth * bufferHeight;
    
    // Try to allocate from PSRAM first
    uint16_t* blackBuffer = (uint16_t*)ps_malloc(bufferSize * sizeof(uint16_t));
    
    // Fallback to smaller heap buffer if needed
    if (!blackBuffer) {
        bufferHeight = 5; // Smaller height
        bufferSize = bufferWidth * bufferHeight;
        blackBuffer = (uint16_t*)malloc(bufferSize * sizeof(uint16_t));
        
        // Create a tiny buffer on stack if all else fails
        if (!blackBuffer) {
            static uint16_t stackBuffer[320]; // Static buffer as last resort
            blackBuffer = stackBuffer;
            bufferSize = 320;
            bufferHeight = bufferSize / bufferWidth;
            if (bufferHeight == 0) bufferHeight = 1;
        }
    }
    
    // Fill buffer with black (0x0000)
    for (size_t i = 0; i < bufferSize; i++) {
        blackBuffer[i] = 0x0000; // Pure black
    }
    
    // Clear screen in strips with proper error handling
    for (uint32_t y = 0; y < lcd.height(); y += bufferHeight) {
        uint32_t stripHeight = ((y + bufferHeight) > lcd.height()) ? 
                            (lcd.height() - y) : bufferHeight;
        
        // Safety check
        if (stripHeight == 0) continue;
        
        // Draw with exception handling
        try {
            lcd.lcd_draw_bitmap(0, y, bufferWidth, y + stripHeight, blackBuffer);
        } catch (...) {
            Logger.error("Exception during screen clearing at y=%lu", y);
        }
        
        // Critical: Wait for LCD operation to complete before next strip
        delay(5);
        
        // Feed watchdog periodically
        if (y % 100 == 0) {
            feed_watchdog();
        }
    }
    
    // Free buffer if allocated from heap
    if (blackBuffer != nullptr && 
        blackBuffer != (uint16_t*)0x3FFC0000 && // PSRAM address check
        bufferSize > 320) { // Not using stack buffer
        free(blackBuffer);
    }
    
    // Add final delay to ensure all LCD operations are complete
    delay(50);
    
    // Log timing information
    Logger.debug("Screen cleared in %lu ms", millis() - startTime);
}


/**
 * Enhanced image validation function that efficiently checks JPEGs
 * Updated: 2025-06-18 13:20:43
 * User: Chamil1983
 */
bool isValidJPEG(const char* filename) {
    if (!filename || strlen(filename) == 0) {
        return false;
    }
    
    // Check file extension
    if (!strstr(filename, ".jpg") && !strstr(filename, ".jpeg")) {
        return false;
    }
    
    // Use static buffer to avoid excessive allocations
    static uint8_t header[12];
    bool result = false;
    
    // Open file and check minimum size
    File file = SD_MMC.open(filename, FILE_READ);
    if (!file) {
        Logger.warn("Failed to open file for validation: %s", filename);
        return false;
    }
    
    size_t fileSize = file.size();
    if (fileSize < 100) {  // Minimum size for a valid JPEG
        file.close();
        Logger.warn("File too small to be valid JPEG: %s (%d bytes)", filename, fileSize);
        return false;
    }
    
    // Read header bytes (SOI marker: 0xFF 0xD8)
    if (file.read(header, 2) == 2) {
        if (header[0] == 0xFF && header[1] == 0xD8) {
            // It has a valid JPEG header
            
            // Now check for EOI marker at the end (0xFF 0xD9)
            if (fileSize > 2) {
                // Seek to end - 2 bytes
                if (file.seek(fileSize - 2)) {
                    if (file.read(header, 2) == 2) {
                        if (header[0] == 0xFF && header[1] == 0xD9) {
                            // Valid EOI marker
                            result = true;
                        } else {
                            Logger.warn("JPEG missing EOI marker - may be corrupted: %s", filename);
                            // We'll still try to decode it, so return true
                            result = true;
                        }
                    }
                }
            }
        } else {
            Logger.warn("Invalid JPEG header in file: %s", filename);
        }
    }
    
    file.close();
    return result;
}

/**
 * LCD setup function
 */
// Fixed setup_lcd function
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

// Fixed setup_touch function
// Fixed setup_touch function
bool setup_touch() {
  Logger.info("Setting up Touch...");
  
  touch_ready = false;
  
  try {
    // Initialize touch controller using begin() as defined in header
    touch.begin();
    Logger.info("Touch initialized successfully");
    touch_ready = true;
  } catch (...) {
    Logger.error("Exception during touch initialization");
    touch_ready = false;
  }
  
  return touch_ready;
}

/**
 * Completely new watchdog implementation that ignores task not found errors
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
bool setup_watchdog() {
    Serial.println("Setting up watchdog with critical error protection...");
    
    // Force disable any existing watchdog timer to start clean
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
 * Critical watchdog feed implementation that handles "task not found" errors
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
void feed_watchdog() {
    static uint32_t last_feed = 0;
    static uint32_t last_error_report = 0;
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
        esp_err_t err = esp_task_wdt_reset_user(main_task_handle);
        
        // Only log errors occasionally to avoid flooding
        if (err != ESP_OK && now - last_error_report > 10000) {
            // If task not found, try to re-add
            if (err == ESP_ERR_NOT_FOUND) {
                esp_task_wdt_add(main_task_handle);
            }
            
            last_error_report = now;
        }
    }
    
    // Also use the generic approach as a backup
    esp_task_wdt_reset();
}

/**
 * Complete SPIFFS replacement with failsafe operation
 * Updated: 2025-06-17 10:56:36
 * User: Chamil1983
 */
bool initializeSpiffs() {
    Serial.println("Initializing SPIFFS with critical error bypass...");
    
    // CRITICAL FIX: First, check if we're already getting the SPIFFS error
    bool spiffsWorking = false;
    
    // Completely disable SPIFFS if we've seen critical errors
    static bool spiffsDisabled = false;
    if (spiffsDisabled) {
        Serial.println("SPIFFS is permanently disabled due to persistent errors");
        return false;
    }
    
    // First attempt - try to mount without formatting
    if (SPIFFS.begin(false)) {
        Serial.println("SPIFFS mounted successfully without formatting");
        
        // Verify we can write to SPIFFS with a test
        File testFile = SPIFFS.open("/test.txt", "w");
        if (testFile) {
            testFile.println("Test data");
            testFile.close();
            
            // Verify we can read it back
            testFile = SPIFFS.open("/test.txt", "r");
            if (testFile) {
                String content = testFile.readString();
                testFile.close();
                SPIFFS.remove("/test.txt");
                
                if (content.indexOf("Test data") >= 0) {
                    Serial.println("SPIFFS read/write test successful");
                    return true;
                }
            }
        }
        
        // If we got here, the read/write test failed
        Serial.println("SPIFFS mounted but failed read/write test");
        SPIFFS.end();
    }
    
    // Second attempt - try with formatting
    Serial.println("First mount failed, trying with formatting...");
    if (SPIFFS.begin(true)) {
        Serial.println("SPIFFS mounted successfully after formatting");
        
        // Try write test again
        File testFile = SPIFFS.open("/test.txt", "w");
        if (testFile) {
            testFile.println("Test data");
            testFile.close();
            testFile = SPIFFS.open("/test.txt", "r");
            if (testFile) {
                String content = testFile.readString();
                testFile.close();
                SPIFFS.remove("/test.txt");
                
                if (content.indexOf("Test data") >= 0) {
                    Serial.println("SPIFFS read/write test successful after format");
                    return true;
                }
            }
        }
    }
    
    // If we got here, SPIFFS is not working even after formatting
    Serial.println("CRITICAL: SPIFFS failed to mount even after formatting");
    Serial.println("Disabling SPIFFS and continuing without file system");
    
    // Set flag to permanently disable SPIFFS
    spiffsDisabled = true;
    return false;
}

/**
 * Check flash partitions and print information
 */
void checkPartitions() {
  Serial.println("Checking partition information:");
  Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());
  
  Serial.println("\nPartition table:");
  Serial.println("Type | Subtype | Address | Size | Label");
  Serial.println("-----|---------|---------|------|-------");
  
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t* part = esp_partition_get(it);
    Serial.printf("%4d | %7d | %08x | %6dK | %s\n", 
                  part->type, part->subtype, part->address, 
                  part->size / 1024, part->label);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
  
  Serial.println();
}

/**
 * Enhanced WiFi setup with more reliable connection parameters
 * Updated: 2025-06-17 11:58:18
 * User: Chamil1983
 */
bool setup_wifi_ap() {
    Logger.info("Setting up WiFi in AP mode with maximum reliability...");
    
    // Complete reset of WiFi subsystem
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
            
            // Set TX power to maximum for better range
            int8_t power = 20;
            esp_wifi_set_max_tx_power(power);
            
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

/**
 * Setup SD card with robust error handling
 */
bool setup_sd_card_reliable() {
  Logger.info("Setting up SD Card with robust initialization...");
  
  sd_ready = false;
  
  try {
    // Wait for power stabilization
    Logger.info("Waiting for power stabilization...");
    delay(SD_POWER_STABILIZE_DELAY);
    
    // Reset SD pins to ensure clean state
    Logger.info("Resetting SD pins...");
    pinMode(SDMMC_CLK_PIN, INPUT);
    pinMode(SDMMC_CMD_PIN, INPUT);
    pinMode(SDMMC_D0_PIN, INPUT);
    pinMode(SDMMC_D1_PIN, INPUT);
    pinMode(SDMMC_D2_PIN, INPUT);
    pinMode(SDMMC_D3_PIN, INPUT);
    delay(100);
    
    // Force SD deinitialization to ensure clean start
    Logger.info("Ensuring SD_MMC is deinitialized...");
    SD_MMC.end();
    delay(500);
    
    // Set pins for SD card interface
    Logger.info("Configuring SD pins...");
    SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                   SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
    
    // Try ultra-conservative settings first (1-bit mode, very slow speed)
    Logger.info("SD init attempt 1: 1-bit mode at 125KHz (ultra conservative)");
    if (SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING)) {
      // Successfully initialized with conservative settings
      sd_ready = validateSDCard();
      if (sd_ready) {
        Logger.info("SD card initialized successfully in ultra-conservative mode");
        return true;
      }
    }
    
    // If conservative approach failed, try more normal settings
    Logger.info("SD init attempt 2: 1-bit mode at default speed");
    if (SD_MMC.begin("/sdcard", true)) {
      sd_ready = validateSDCard();
      if (sd_ready) {
        Logger.info("SD card initialized successfully with 1-bit mode");
        return true;
      }
    }
    
    // Last attempt with 4-bit mode (most compatible with good cards)
    Logger.info("SD init attempt 3: 4-bit mode");
    if (SD_MMC.begin()) {
      sd_ready = validateSDCard();
      if (sd_ready) {
        Logger.info("SD card initialized successfully with 4-bit mode");
        return true;
      }
    }
    
    // If all attempts failed, log error
    Logger.error("All SD card initialization attempts failed");
    sd_ready = false;
    
  } catch (...) {
    Logger.error("Exception during SD card initialization");
    sd_ready = false;
  }
  
  return sd_ready;
}

/**
 * Validate SD card is working properly
 */
bool validateSDCard() {
  Logger.info("Validating SD card...");
  
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
  Logger.info("Setting up LVGL...");
  
  lvgl_ready = false;
  
  // Initialize LVGL
  lv_init();
  
  // Allocate display buffer
  const uint32_t buf_size = LCD_H_RES * 40;  // Buffer for 40 rows
  Logger.debug("Allocating LVGL buffers: %d bytes each", buf_size * sizeof(lv_color_t));
  
  buf = (lv_color_t *)ps_malloc(buf_size * sizeof(lv_color_t));
  buf1 = (lv_color_t *)ps_malloc(buf_size * sizeof(lv_color_t));
  
  if (!buf || !buf1) {
    Logger.error("Failed to allocate LVGL buffers");
    return false;
  }
  
  // Initialize the display buffer
  lv_disp_draw_buf_init(&draw_buf, buf, buf1, buf_size);
  
  // Display driver initialization
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  
  // Set display driver properties
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = [](lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    // Convert LVGL color buffer to 16-bit RGB565 format
    lcd.lcd_draw_bitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *)color_p);
    
    // Indicate to LVGL that the flush is done
    lv_disp_flush_ready(disp);
  };
  
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  
  // Input device driver initialization for touch
if (touch_ready) {
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  
  // Fix the read_cb function to use the correct function name and return type
  indev_drv.read_cb = [](lv_indev_drv_t *drv, lv_indev_data_t *data) -> void {
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
    
    uint16_t touch_x, touch_y;
    
    // Use the correct function name
    if (touch.getTouch(&touch_x, &touch_y)) {
      // Convert touch coordinates to LVGL coordinates if needed
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
  
  Logger.info("LVGL initialized successfully");
  lvgl_ready = true;
  
  return lvgl_ready;
}

/**
 * Setup UI elements
 */
bool setup_ui() {
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
 * Updated: 2025-06-17 11:42:54
 * User: Chamil1983
 */
bool setup_webserver() {
    Logger.info("Setting up WebServer with enhanced connectivity...");
    
    try {
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
        
// Improved slideshow control handler with proper mutex handling
// Fixed slideshow control handler
server.on("/slideshow", HTTP_POST, []() {
    String action = server.arg("action");
    Logger.info("Slideshow control request: %s", action.c_str());
    
    // CRITICAL: Stop any currently running slideshow first
    if (slideshow_active && action == "start") {
        stopSlideshow();
        delay(500); // Give time for slideshow task to settle
    }
    
    if (action == "start") {
        uint32_t interval = 5000; // Default 5 seconds
        
        // Check if interval was provided
        if (server.hasArg("interval")) {
            interval = server.arg("interval").toInt();
            // Enforce minimum interval for stability
            if (interval < 3000) interval = 3000;
        }
        
        // Start with first image for better UX
        current_image = 0;
        
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
        
        // Handle file uploads
        server.on("/upload", HTTP_POST, []() {
            server.send(200, "text/plain", "Upload complete");
        }, handle_upload);
        
        // System info page
        server.on("/system", HTTP_GET, []() {
            String response = "<!DOCTYPE html><html><head><title>ESP32P4 System Info</title>";
            response += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
            response += "<style>";
            response += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
            response += "h1, h2 {color: #3b82f6;}";
            response += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
            response += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer;}";
            response += "table {width: 100%; border-collapse: collapse;}";
            response += "th, td {text-align: left; padding: 8px; border-bottom: 1px solid #3b82f6;}";
            response += "th {color: #3b82f6;}";
            response += "</style></head><body>";
            
            response += "<h1>ESP32P4 System Information</h1>";
            response += "<div class='card'>";
            response += "<h2>Memory</h2>";
            response += "<table>";
            response += "<tr><th>Type</th><th>Total (KB)</th><th>Free (KB)</th><th>Used (KB)</th></tr>";
            response += "<tr><td>Heap</td><td>" + String(ESP.getHeapSize() / 1024) + "</td>";
            response += "<td>" + String(ESP.getFreeHeap() / 1024) + "</td>";
            response += "<td>" + String((ESP.getHeapSize() - ESP.getFreeHeap()) / 1024) + "</td></tr>";
            response += "<tr><td>PSRAM</td><td>" + String(ESP.getPsramSize() / 1024) + "</td>";
            response += "<td>" + String(ESP.getFreePsram() / 1024) + "</td>";
            response += "<td>" + String((ESP.getPsramSize() - ESP.getFreePsram()) / 1024) + "</td></tr>";
            response += "</table>";
            response += "</div>";
            
            response += "<div class='card'>";
            response += "<h2>CPU & System</h2>";
            response += "<table>";
            response += "<tr><th>Parameter</th><th>Value</th></tr>";
            response += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
            response += "<tr><td>Flash Size</td><td>" + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB</td></tr>";
            response += "<tr><td>Flash Speed</td><td>" + String(ESP.getFlashChipSpeed() / 1000000) + " MHz</td></tr>";
            response += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
            response += "<tr><td>SDK Version</td><td>" + String(ESP.getSdkVersion()) + "</td></tr>";
            response += "</table>";
            response += "</div>";
            
            response += "<div class='card'>";
            response += "<h2>Network</h2>";
            response += "<table>";
            response += "<tr><th>Parameter</th><th>Value</th></tr>";
            response += "<tr><td>WiFi Mode</td><td>AP</td></tr>";
            response += "<tr><td>SSID</td><td>" + String(WIFI_AP_SSID) + "</td></tr>";
            response += "<tr><td>IP Address</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
            response += "<tr><td>Connected Clients</td><td>" + String(WiFi.softAPgetStationNum()) + "</td></tr>";
            response += "<tr><td>Channel</td><td>1</td></tr>";
            response += "</table>";
            response += "</div>";
            
            response += "<div class='card'>";
            response += "<h2>Storage</h2>";
            response += "<table>";
            response += "<tr><th>Parameter</th><th>Value</th></tr>";
            response += "<tr><td>SD Card</td><td>" + String(sd_ready ? "Connected" : "Not available") + "</td></tr>";
            if (sd_ready) {
                uint64_t cardSize = SD_MMC.cardSize() / 1024 / 1024;
                response += "<tr><td>SD Card Size</td><td>" + String((uint32_t)cardSize) + " MB</td></tr>";
                response += "<tr><td>Image Count</td><td>" + String(image_count) + "</td></tr>";
            }
            response += "</table>";
            response += "</div>";
            
            response += "<div class='card'>";
            response += "<h2>Component Status</h2>";
            response += "<table>";
            response += "<tr><th>Component</th><th>Status</th></tr>";
            response += "<tr><td>LCD</td><td>" + String(lcd_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>Touch</td><td>" + String(touch_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>SD Card</td><td>" + String(sd_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>WiFi</td><td>" + String(wifi_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>LVGL</td><td>" + String(lvgl_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>UI</td><td>" + String(ui_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>Server</td><td>" + String(server_ready ? "OK" : "Not Available") + "</td></tr>";
            response += "<tr><td>Slideshow</td><td>" + String(slideshow_active ? "Running" : "Stopped") + "</td></tr>";
            response += "</table>";
            response += "</div>";
            
            response += "<p><a href='/' class='btn'>Back to Gallery</a> <a href='/test' class='btn'>Connection Test</a></p>";
            response += "</body></html>";
            
            server.send(200, "text/html", response);
        });
        
        // Add test page handler for connectivity diagnostics
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
            response += "<li><strong>WiFi Channel:</strong> 1</li>";
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
        
        // Start the web server
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
 * Updates the upload form HTML with enhanced validation and error handling
 * Updated: 2025-06-16 10:45:51
 * User: Chamil1983
 */
String updateUploadForm() {
  String html = "<div class='card'>";
  html += "<h2>Upload Image</h2>";
  html += "<p><b>Important:</b> Upload only JPEG images smaller than 1MB for best results</p>";
  html += "<form id='upload-form' class='upload-form' method='post' action='/upload' enctype='multipart/form-data'>";
  html += "<div class='file-input-wrapper'>";
  html += "<button type='button' class='btn'>Choose File</button>";
  html += "<input type='file' name='image' accept='.jpg,.jpeg' id='file-input'>";
  html += "</div>";
  html += "<div id='file-info' style='margin: 10px 0;'><span id='file-selected'>No file selected</span></div>";
  html += "<button type='submit' id='upload-button' class='btn btn-success'>Upload</button>";
  html += "</form>";
  html += "<div id='status' class='info' style='display:none;'></div>";
  html += "<div id='progress-container' style='display:none;'><div id='progress-bar'></div></div>";
  html += "</div>";

  html += "<script>";
  html += "document.addEventListener('DOMContentLoaded', function() {";
  html += "  const form = document.getElementById('upload-form');";
  html += "  const fileInput = document.getElementById('file-input');";
  html += "  const fileSelected = document.getElementById('file-selected');";
  html += "  const status = document.getElementById('status');";
  html += "  const progressBar = document.getElementById('progress-bar');";
  html += "  const progressContainer = document.getElementById('progress-container');";
  html += "  const uploadButton = document.getElementById('upload-button');";

  html += "  // Ensure file input change event works correctly";
  html += "  if (fileInput) {";
  html += "    fileInput.addEventListener('click', function(e) {";
  html += "      console.log('File input clicked');";
  html += "    });";
  
  html += "    fileInput.addEventListener('change', function(e) {";
  html += "      console.log('File input changed', this.files);";
  html += "      if (this.files && this.files.length > 0) {";
  html += "        const file = this.files[0];";
  html += "        fileSelected.textContent = file.name + ' (' + formatBytes(file.size) + ')';";
  html += "        fileSelected.style.color = '#16a34a'; // Green text";
  html += "        console.log('File selected:', file.name, file.size);";
  
  html += "        // Validate file type";
  html += "        if (!file.name.toLowerCase().endsWith('.jpg') && ";
  html += "            !file.name.toLowerCase().endsWith('.jpeg')) {";
  html += "          status.textContent = 'Warning: File does not have .jpg extension';";
  html += "          status.className = 'warning';";
  html += "          status.style.display = 'block';";
  html += "        } else if (file.size > 3000000) { // 3MB";
  html += "          status.textContent = 'Warning: File is large ('+formatBytes(file.size)+') and may fail to upload';";
  html += "          status.className = 'warning';";
  html += "          status.style.display = 'block';";
  html += "        } else {";
  html += "          status.style.display = 'none';";
  html += "        }";
  html += "      } else {";
  html += "        fileSelected.textContent = 'No file selected';";
  html += "        fileSelected.style.color = '';";
  html += "        status.style.display = 'none';";
  html += "      }";
  html += "    });";
  html += "  }";

  html += "  // Set up form submission handling";
  html += "  if (form) {";
  html += "    form.addEventListener('submit', function(e) {";
  html += "      e.preventDefault();";
  
  html += "      if (!fileInput || !fileInput.files || !fileInput.files.length) {";
  html += "        alert('Please select a file to upload');";
  html += "        return;";
  html += "      }";
  
  html += "      const file = fileInput.files[0];";
  
  html += "      // Show progress UI";
  html += "      status.textContent = 'Preparing upload...';";
  html += "      status.style.display = 'block';";
  html += "      status.className = 'info';";
  html += "      progressContainer.style.display = 'block';";
  html += "      progressBar.style.width = '0%';";
  html += "      uploadButton.disabled = true;";
  html += "      fileInput.disabled = true;";
  
  html += "      // Create FormData";
  html += "      const formData = new FormData();";
  html += "      formData.append('image', file);";
  html += "      console.log('Uploading file:', file.name, file.size);";
  
  html += "      const xhr = new XMLHttpRequest();";
  
  html += "      // Set up upload progress handler";
  html += "      xhr.upload.addEventListener('progress', function(e) {";
  html += "        if (e.lengthComputable) {";
  html += "          const percent = Math.round((e.loaded / e.total) * 100);";
  html += "          progressBar.style.width = percent + '%';";
  html += "          status.textContent = `Uploading: ${percent}% (${formatBytes(e.loaded)} of ${formatBytes(e.total)})`;";
  html += "          console.log(`Upload progress: ${percent}%`);";
  html += "        }";
  html += "      });";
  
  html += "      // Set up completion handler";
  html += "      xhr.addEventListener('load', function() {";
  html += "        if (xhr.status === 200) {";
  html += "          progressBar.style.width = '100%';";
  html += "          status.textContent = 'Upload complete! Processing...';";
  html += "          status.className = 'success';";
  
  html += "          // Add processing animation";
  html += "          let dots = '';";
  html += "          const processingInterval = setInterval(function() {";
  html += "            dots = dots.length >= 3 ? '' : dots + '.';";
  html += "            status.textContent = 'Upload complete! Processing' + dots;";
  html += "          }, 500);";
  
  html += "          // Refresh page after delay";
  html += "          setTimeout(function() {";
  html += "            clearInterval(processingInterval);";
  html += "            window.location.reload();";
  html += "          }, 3000);";
  html += "        } else {";
  html += "          status.textContent = 'Upload failed: ' + xhr.responseText;";
  html += "          status.className = 'error';";
  html += "          uploadButton.disabled = false;";
  html += "          fileInput.disabled = false;";
  html += "        }";
  html += "      });";
  
  html += "      // Set up error handler";
  html += "      xhr.addEventListener('error', function() {";
  html += "        status.textContent = 'Upload failed due to network error';";
  html += "        status.className = 'error';";
  html += "        uploadButton.disabled = false;";
  html += "        fileInput.disabled = false;";
  html += "      });";
  
  html += "      // Set up timeout handler";
  html += "      xhr.timeout = 30000; // 30 seconds";
  html += "      xhr.addEventListener('timeout', function() {";
  html += "        status.textContent = 'Upload timed out';";
  html += "        status.className = 'error';";
  html += "        uploadButton.disabled = false;";
  html += "        fileInput.disabled = false;";
  html += "      });";
  
  html += "      // Open connection and send data";
  html += "      xhr.open('POST', '/upload', true);";
  html += "      xhr.send(formData);";
  html += "    });";
  html += "  }";
  
  html += "  // Helper function to format file size";
  html += "  function formatBytes(bytes) {";
  html += "    if (bytes === 0) return '0 Bytes';";
  html += "    const k = 1024;";
  html += "    const sizes = ['Bytes', 'KB', 'MB', 'GB'];";
  html += "    const i = Math.floor(Math.log(bytes) / Math.log(k));";
  html += "    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];";
  html += "  }";
  html += "});";
  html += "</script>";

  return html;
}

/**
 * Enhanced file upload handler with improved error handling and timeouts
 * Updated: 2025-06-17 11:58:18
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
        
        // Print diagnostic info
        Logger.debug("Received %d bytes of upload data", upload.currentSize);
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
 * Emergency SD card recovery
 */
bool emergency_sd_recovery() {
  Logger.info("Performing comprehensive emergency SD card recovery");
  
  // End any existing SD session
  SD_MMC.end();
  
  // Wait for full discharge and card reset
  delay(2000);
  
  // Reset pins individually with careful timing
  const uint8_t sdPins[] = {
    SDMMC_CLK_PIN, SDMMC_CMD_PIN, 
    SDMMC_D0_PIN, SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN
  };
  
  // First set all pins to input mode
  for (int i = 0; i < 6; i++) {
    pinMode(sdPins[i], INPUT);
  }
  delay(300);
  
  // Now drive them low to discharge
  for (int i = 0; i < 6; i++) {
    pinMode(sdPins[i], OUTPUT);
    digitalWrite(sdPins[i], LOW);
    delay(50); // Short delay between pins
  }
  delay(500);
  
  // Release pins in reverse order with pull-ups
  for (int i = 5; i >= 0; i--) {
    pinMode(sdPins[i], INPUT_PULLUP);
    delay(50);
  }
  delay(500);
  
  // Reset SD host controller
  Logger.info("Reinitializing SD host controller");
  
  // Configure with explicit pins
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  // Try ultra-conservative initialization
  bool success = SD_MMC.begin("/sdcard", false, false, 80000); // Even lower speed (80KHz)
  
  if (!success) {
    Logger.warn("First SD recovery attempt failed, trying with different settings");
    delay(1000);
    
    // Try 1-bit mode with slightly higher speed
    success = SD_MMC.begin("/sdcard", true, false, 125000);
  }
  
  if (success) {
    Logger.info("SD card recovery successful");
    sd_ready = true;
    
    // Test a simple write operation
    File testFile = SD_MMC.open("/recovery_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.print("RECOVERY");
      testFile.close();
      
      // Very small delay to ensure SD card operation completes
      delay(100);
      
      // Read back to verify
      testFile = SD_MMC.open("/recovery_test.txt", FILE_READ);
      if (testFile) {
        char buf[10] = {0};
        int bytesRead = testFile.read((uint8_t*)buf, 8);
        testFile.close();
        
        if (bytesRead == 8 && strcmp(buf, "RECOVERY") == 0) {
          Logger.info("SD card test write successful");
        } else {
          Logger.warn("SD card write test failed, but continuing");
        }
      }
      
      // Clean up test file
      SD_MMC.remove("/recovery_test.txt");
    }
    
    // Ensure images directory exists
    if (!SD_MMC.exists("/images")) {
      if (SD_MMC.mkdir("/images")) {
        Logger.info("Created /images directory");
      }
    }
    
    return true;
  } else {
    Logger.error("All SD card recovery attempts failed");
    sd_ready = false;
    return false;
  }
}

/**
 * Function to recover SD card for uploads
 */
bool recover_sd_card_for_upload() {
  Logger.info("Performing special SD recovery for uploads...");
  
  // First end any active SD session
  SD_MMC.end();
  
  // Wait for a complete discharge of the SD card interface
  delay(1000);
  
  // Reset pins one by one with timing gaps
  pinMode(SDMMC_CLK_PIN, INPUT);
  pinMode(SDMMC_CMD_PIN, INPUT);
  pinMode(SDMMC_D0_PIN, INPUT);
  pinMode(SDMMC_D1_PIN, INPUT);
  pinMode(SDMMC_D2_PIN, INPUT);
  pinMode(SDMMC_D3_PIN, INPUT);
  delay(100);
  
  // Set to outputs and drive low to discharge
  pinMode(SDMMC_CLK_PIN, OUTPUT);
  digitalWrite(SDMMC_CLK_PIN, LOW);
  pinMode(SDMMC_CMD_PIN, OUTPUT);
  digitalWrite(SDMMC_CMD_PIN, LOW);
  pinMode(SDMMC_D0_PIN, OUTPUT);
  digitalWrite(SDMMC_D0_PIN, LOW);
  pinMode(SDMMC_D1_PIN, OUTPUT);
  digitalWrite(SDMMC_D1_PIN, LOW);
  pinMode(SDMMC_D2_PIN, OUTPUT);
  digitalWrite(SDMMC_D2_PIN, LOW);
  pinMode(SDMMC_D3_PIN, OUTPUT);
  digitalWrite(SDMMC_D3_PIN, LOW);
  delay(200);
  
  // Reset to inputs with pullups
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  delay(200);
  
  // Configure SD MMC pins
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  // Try to initialize with ultra-conservative parameters
  bool success = SD_MMC.begin("/sdcard", false, false, 100000); // 1-bit mode, 100KHz speed
  
  if (success) {
    // Test if SD card is working by writing a small test file
    File testFile = SD_MMC.open("/sd_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.println("SD recovery test");
      testFile.close();
      SD_MMC.remove("/sd_test.txt");
      
      Logger.info("SD card recovery successful");
      sd_ready = true;
      return true;
    } else {
      Logger.error("SD card recovery - cannot write test file");
      sd_ready = false;
      return false;
    }
  } else {
    Logger.error("SD card recovery failed");
    sd_ready = false;
    return false;
  }
}

/**
 * General SD card recovery
 */
bool recover_sd_card() {
  Logger.info("Attempting SD card recovery...");
  
  // End any active SD session
  SD_MMC.end();
  delay(1000);
  
  // Reset pins to default state
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  delay(500);
  
  // Try to initialize with conservative settings
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  bool success = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING);
  
  if (success) {
    sd_ready = true;
    Logger.info("SD card recovery successful");
    return true;
  }
  
  Logger.error("SD card recovery failed");
  sd_ready = false;
  return false;
}

/**
 * Improved scan_images function with corruption detection and removal
 * Updated: 2025-06-17 10:03:45
 * User: Chamil1983
 */
void scan_images() {
    Logger.info("Scanning for images with enhanced validation...");
    
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
    
    // Track corrupt files so we can delete them
    const int MAX_CORRUPT_FILES = 20;
    char* corruptFiles[MAX_CORRUPT_FILES] = {nullptr};
    int corruptFileCount = 0;
    
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
    
    // First phase: Scan all files and mark corrupt ones for deletion
    Logger.info("Phase 1: Identifying valid and corrupt images...");
    File file = root.openNextFile();
    while (file && image_count < MAX_IMAGES) {
        if (!file.isDirectory()) {
            String filename = file.name();
            // Convert to lowercase for case-insensitive comparison
            String lowerFilename = filename;
            lowerFilename.toLowerCase();
            
            if (lowerFilename.endsWith(".jpg") || lowerFilename.endsWith(".jpeg")) {
                String fullPath = "/images/";
                fullPath += filename;
                
                // Check file size first
                size_t fileSize = file.size();
                if (fileSize < 1024) { // Minimum 1KB
                    Logger.warn("Marking small file for deletion: %s (%d bytes)", fullPath.c_str(), fileSize);
                    
                    // Add to corrupt files list
                    if (corruptFileCount < MAX_CORRUPT_FILES) {
                        corruptFiles[corruptFileCount] = strdup(fullPath.c_str());
                        corruptFileCount++;
                    }
                    
                    file = root.openNextFile();
                    continue;
                }
                
                // Perform full validation using our enhanced function
                file.close(); // Close first because our validator will reopen it
                bool isValid = isValidJPEG(fullPath.c_str());
                file = root.openNextFile(); // Re-open next file
                
                if (!isValid) {
                    Logger.warn("Marking corrupted file for deletion: %s", fullPath.c_str());
                    
                    // Add to corrupt files list
                    if (corruptFileCount < MAX_CORRUPT_FILES) {
                        corruptFiles[corruptFileCount] = strdup(fullPath.c_str());
                        corruptFileCount++;
                    }
                    
                    continue;
                }
                
                // If we get here, the file is valid - add it to our list
                image_list[image_count] = (char*)malloc(fullPath.length() + 1);
                if (image_list[image_count]) {
                    strcpy(image_list[image_count], fullPath.c_str());
                    Logger.debug("Found image %d: %s (%d bytes)", 
                              image_count, image_list[image_count], fileSize);
                    image_count++;
                } else {
                    Logger.error("Failed to allocate memory for image path");
                }
            }
        } else {
            file = root.openNextFile();
        }
    }
    
    // Close the directory
    root.close();
    
    // Second phase: Delete corrupt files
    if (corruptFileCount > 0) {
        Logger.info("Phase 2: Deleting %d corrupted files...", corruptFileCount);
        
        for (int i = 0; i < corruptFileCount; i++) {
            if (corruptFiles[i] != nullptr) {
                Logger.info("Deleting corrupted file: %s", corruptFiles[i]);
                
                if (SD_MMC.remove(corruptFiles[i])) {
                    Logger.info("Successfully deleted: %s", corruptFiles[i]);
                } else {
                    Logger.error("Failed to delete: %s", corruptFiles[i]);
                }
                
                free(corruptFiles[i]);
            }
        }
    }
    
    // Release the mutex
    xSemaphoreGive(sd_mutex);
    
    Logger.info("Found %d valid images, removed %d corrupted files", image_count, corruptFileCount);
    
    // Create a test image if no images were found
    if (image_count == 0) {
        Logger.warn("No images found, creating test pattern");
        createTestImage();
        scan_images(); // Rescan to pick up the test image
    }
}

/**
 * Update system information
 */
void update_system_info() {
  // Start with heap info
  ESP.getHeapSize();
  ESP.getFreeHeap();
  ESP.getMaxAllocHeap();
  
  // PSRAM info
  ESP.getPsramSize();
  ESP.getFreePsram();
  ESP.getMaxAllocPsram();
  
  // Other useful info
  ESP.getFlashChipSize();
  ESP.getCpuFreqMHz();
  
  // Update UI elements if needed
  last_system_update = millis();
}

// Create a simple test pattern image
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

// End of code
#pragma GCC pop_options
