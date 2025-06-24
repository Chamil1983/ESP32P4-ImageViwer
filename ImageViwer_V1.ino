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
#include <ArduinoJson.h>
#include <Update.h>
#include "JPEGDEC.h"
#include "lv_conf.h"
#include "pins_config.h"
#include "debug_logger.h"
#include "src/lcd/jd9365_lcd.h"
#include "src/touch/gsl3680_touch.h"

/*******************************************************
 * ESP32P4 Advanced Image Viewer - Version 3.0
 * Author: Chamil1983
 * Date: 2025-06-24
 * 
 * New features in V3:
 * - Enhanced UI with thumbnail gallery
 * - Image zoom and pan support
 * - Advanced image effects (grayscale, negative, etc.)
 * - Folder support for image organization
 * - Dual WiFi mode (AP and Station)
 * - Web interface improvements
 * - Image caching for faster loading
 * - Multi-file upload support
 * - Basic security features
 * - Image slideshow transitions
 * - Battery monitoring (if hardware supports)
 * - System health dashboard
 *******************************************************/

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
bool fs_ready = false;

// Watchdog settings
#define WDT_TIMEOUT_SECONDS 120  // 120 seconds watchdog timeout

// WiFi configuration
#define WIFI_AP_SSID "ESP32P4-Gallery"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP IPAddress(192, 168, 4, 1)
#define WIFI_AP_GATEWAY IPAddress(192, 168, 4, 1)
#define WIFI_AP_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_CHANNEL 6
#define MAX_WIFI_CLIENTS 4

// WiFi Station mode (to connect to existing WiFi)
#define WIFI_STA_ENABLED false  // Set to true to enable connection to existing WiFi
#define WIFI_STA_SSID "YourNetwork"
#define WIFI_STA_PASSWORD "YourPassword"

// Web Server and DNS settings
WebServer server(80);
DNSServer dnsServer;
bool dnsServerActive = false;

// Authentication settings
#define AUTH_ENABLED false  // Set to true to enable basic authentication
#define AUTH_USERNAME "admin"
#define AUTH_PASSWORD "admin"

// Hardware objects
jd9365_lcd lcd = jd9365_lcd(LCD_RST);
gsl3680_touch touch = gsl3680_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

// LVGL display buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

// Image gallery variables
#define MAX_IMAGES 500
#define MAX_FOLDERS 20
#define MAX_FILENAME_LENGTH 64
#define MAX_FILEPATH_LENGTH 128

// Image metadata structure
struct ImageInfo {
    char path[MAX_FILEPATH_LENGTH];  // Full path to the image
    char name[MAX_FILENAME_LENGTH];  // Just the filename
    char folder[MAX_FILENAME_LENGTH]; // Folder name
    uint32_t size;                   // File size in bytes
    uint16_t width;                  // Image width in pixels
    uint16_t height;                 // Image height in pixels
    bool isValid;                    // Whether the image is valid
    uint32_t lastAccessed;           // Last accessed timestamp
};

// Folder structure
struct FolderInfo {
    char name[MAX_FILENAME_LENGTH];
    int imageCount;
};

// Image management
ImageInfo *imageList = nullptr;
FolderInfo folderList[MAX_FOLDERS];
int imageCount = 0;  // Changed from volatile to fix min() function issues
int folderCount = 0;
int currentImage = 0;
int currentFolder = 0;

// Display and viewing state
#define VIEW_MODE_SINGLE 0    // Single image view
#define VIEW_MODE_GALLERY 1   // Thumbnail gallery view
#define VIEW_MODE_FOLDERS 2   // Folder view

int viewMode = VIEW_MODE_SINGLE;
bool imageNeedsRotation = false;
bool imageLoading = false;
float currentZoom = 1.0f;
int panOffsetX = 0;
int panOffsetY = 0;
bool isPanning = false;
int lastTouchX = 0;
int lastTouchY = 0;

// Image effects
#define EFFECT_NONE 0
#define EFFECT_GRAYSCALE 1
#define EFFECT_NEGATIVE 2
#define EFFECT_SEPIA 3
int currentEffect = EFFECT_NONE;

// Slideshow variables
bool slideshowActive = false;
uint32_t slideshowLastChange = 0;
uint32_t slideshowInterval = 5000;  // Default: 5 seconds
hw_timer_t *slideshowTimer = NULL;
portMUX_TYPE slideshowTimerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t slideshowTaskHandle = NULL;

// Transition effects
#define TRANSITION_NONE 0
#define TRANSITION_FADE 1
#define TRANSITION_SLIDE 2
#define TRANSITION_ZOOM 3
int currentTransition = TRANSITION_NONE;

// JPEG and image processing
#define DRAW_BUFFER_SIZE (32 * 1024)  // 32KB buffer
uint8_t *drawBuffer = nullptr;

// Cache management
#define CACHE_SIZE 3  // Number of images to cache
struct CacheEntry {
    char path[MAX_FILEPATH_LENGTH];
    uint8_t *data;
    size_t size;
    uint32_t lastUsed;
    bool valid;
};
CacheEntry imageCache[CACHE_SIZE];

// Upload variables
#define UPLOAD_BUFFER_SIZE 8192       // 8KB buffer
#define SDMMC_FREQ_PROBING 125000     // SD card probing frequency
#define RETRY_DELAY 1000              // Delay between retries
#define SD_POWER_STABILIZE_DELAY 2000 // Power stabilization delay
bool uploadActive = false;
uint32_t uploadStartTime = 0;
uint32_t uploadSize = 0;
char uploadFilename[MAX_FILENAME_LENGTH];
char uploadFolder[MAX_FILENAME_LENGTH];

// System timing variables
uint32_t lastUiUpdate = 0;
uint32_t lastSystemUpdate = 0;
uint32_t lastTouchEvent = 0;

// Mutex for thread safety
SemaphoreHandle_t sdMutex = NULL;
SemaphoreHandle_t jpegMutex = NULL;
SemaphoreHandle_t uiMutex = NULL;
SemaphoreHandle_t wifiMutex = NULL;

// Control flags
volatile bool wdtEnabled = false;
volatile bool resetRequested = false;
volatile bool deepSleepRequested = false;
volatile bool formatSdRequested = false;

// Task handles
TaskHandle_t lvglTaskHandle = NULL;
TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t backgroundTaskHandle = NULL;

// UI elements
lv_obj_t *mainScreen = nullptr;
lv_obj_t *galleryScreen = nullptr;
lv_obj_t *folderScreen = nullptr;
lv_obj_t *statusBar = nullptr;
lv_obj_t *imageView = nullptr;
lv_obj_t *imageDisplay = nullptr;
lv_obj_t *controlBar = nullptr;
lv_obj_t *statusLabel = nullptr;
lv_obj_t *wifiLabel = nullptr;
lv_obj_t *imageCounter = nullptr;
lv_obj_t *prevBtn = nullptr;
lv_obj_t *nextBtn = nullptr;
lv_obj_t *folderLabel = nullptr;
lv_obj_t *zoomBtn = nullptr;
lv_obj_t *effectBtn = nullptr;
lv_obj_t *galleryBtn = nullptr;
lv_obj_t *thumbnailView = nullptr;
lv_obj_t *folderView = nullptr;
lv_obj_t *sysInfo = nullptr;

// UI styling
lv_style_t styleBtn;
lv_style_t styleBtnPressed;
lv_style_t styleImage;
lv_style_t styleImageSelected;
lv_style_t styleFolder;
lv_style_t styleFolderSelected;
lv_style_t styleStatusBar;
lv_style_t styleControlBar;

// Function prototypes
bool initializeFileSystem();
bool initializeSpiffs();
bool setupWatchdog();
bool setupLcd();
bool setupTouch();
bool setupSdCardReliable();
bool validateSDCard();  // Added missing declaration
void createRequiredDirectories();  // Added missing declaration
bool setupWiFi();
bool setupLvgl();
bool setupUI();
bool setupWebserver();
bool setupCache();
bool recoverSdCard();
void scanImages();
void scanImagesInFolder(const char* folderPath, const char* folderName);  // Added missing declaration
void displayImage(int index);
bool renderJpegFile(const char* filename);
bool renderJpegFromMemory(uint8_t* jpegData, size_t jpegSize);  // Added missing declaration
int jpegDrawCallback(JPEGDRAW *pDraw);
void startSlideshow(uint32_t interval_ms);
void stopSlideshow();
void slideshowTask(void *parameter);
void lvglTask(void *pvParameters);
void networkTask(void *pvParameters);
void backgroundTask(void *pvParameters);
void handleTouchEvent(uint16_t x, uint16_t y);
void switchViewMode(int newMode);
void updateImageCounter(int index);
void updateStatusAfterImageLoad(int index, bool success);
void clearLcdScreen();
void handleImageZoom(float zoomFactor);
void handleImagePan(int deltaX, int deltaY);
void applyImageEffect(uint16_t* pixel, int effect);
bool isValidJpeg(const char* filename);
bool createFolder(const char* folderName);
bool createTestImage();  // Added missing declaration
void handleUpload();
void handleWebAuthentication();
void updateSystemInfo();
void feedWatchdog();
void safeLoggerInit();
void performSystemHealthCheck();
bool cacheImage(const char* path);
uint8_t* getCachedImage(const char* path, size_t* size);
void clearImageCache();
void updateBatteryStatus();

// Debug macros for consistent logging
#define DEBUG_ENABLED true

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

/**
 * Safe function execution template with watchdog feeding
 */
template<typename F>
bool safeExecuteWithWatchdog(F&& func, const char* operationName) {
  unsigned long startTime = millis();
  bool result = false;
  
  try {
    result = func();
  } catch (...) {
    Logger.error("Exception in %s", operationName);
  }
  
  unsigned long duration = millis() - startTime;
  
  if (duration > 1000) {
    Logger.warn("Operation %s took %lu ms", operationName, duration);
  }
  
  feedWatchdog();
  return result;
}

/**
 * Safe logger initialization
 */
void safeLoggerInit() {
    // First try normal initialization with serial only
    try {
        Logger.init(true, false, LOG_LEVEL_DEBUG);
        Logger.enableSerialOutput(true);
        Logger.setLogLevel(LOG_LEVEL_DEBUG);
    } catch (...) {
        // If logger initialization fails, set up minimal output
        Serial.println("Logger initialization failed, using serial-only logging");
        delay(100);
        Serial.flush();
    }
    
    Serial.println();
    Serial.println("ESP32P4 Advanced Image Viewer v3.0");
    Serial.println("Starting initialization...");
    Serial.flush();
}

/**
 * Initialize filesystem with proper partition management
 */
bool initializeFileSystem() {
    Serial.println("Initializing file system...");
    
    // Try mounting with storage partition name first
    if (LittleFS.begin(false, "storage")) {
        Serial.println("LittleFS mounted successfully on 'storage' partition");
        fs_ready = true;
        return true;
    }
    
    // If failed, try without specifying partition name
    if (LittleFS.begin(false)) {
        Serial.println("LittleFS mounted with default partition");
        fs_ready = true;
        return true;
    }
    
    // If still failed, try formatting
    Serial.println("LittleFS mount failed, formatting...");
    if (LittleFS.format()) {
        if (LittleFS.begin(false, "storage")) {
            Serial.println("LittleFS formatted and mounted successfully");
            fs_ready = true;
            return true;
        }
        
        if (LittleFS.begin(false)) {
            Serial.println("LittleFS formatted and mounted with default settings");
            fs_ready = true;
            return true;
        }
    }
    
    Serial.println("LittleFS initialization failed - continuing without file system");
    fs_ready = false;
    return false;
}

/**
 * Initialize SPIFFS for compatibility with older projects
 */
bool initializeSpiffs() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS initialization failed");
        return false;
    }
    Serial.println("SPIFFS initialized successfully");
    return true;
}


/**
 * Setup watchdog with improved error recovery
 */
bool setupWatchdog() {
    Serial.println("Setting up watchdog with critical error protection...");
    
    // Force disable any existing watchdog timer
    esp_task_wdt_deinit();
    delay(100);
    
    // Use conservative configuration
    esp_task_wdt_config_t wdtConfig = {
        .timeout_ms = 30000,       // 30 seconds timeout
        .idle_core_mask = 0,       // Don't watch idle cores
        .trigger_panic = false     // Don't panic on timeout
    };
    
    // Initialize watchdog
    esp_err_t err = esp_task_wdt_init(&wdtConfig);
    if (err != ESP_OK) {
        Serial.printf("Watchdog init failed: %d - continuing without WDT\n", err);
        wdtEnabled = false;
        return false;
    }
    
    // Get current task handle and subscribe to watchdog
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (currentTask != NULL) {
        esp_err_t addErr = esp_task_wdt_add(currentTask);
        if (addErr != ESP_OK) {
            Serial.printf("Could not add main task to WDT: %d - continuing anyway\n", addErr);
        } else {
            Serial.println("Main task subscribed to watchdog");
        }
    }
    
    mainTaskHandle = currentTask;
    wdtEnabled = (err == ESP_OK);
    
    Serial.printf("Watchdog initialized with %d second timeout\n", wdtConfig.timeout_ms / 1000);
    return (err == ESP_OK);
}


/**
 * Feed the watchdog to prevent resets
 */
void feedWatchdog() {
    static uint32_t lastFeed = 0;
    static uint32_t lastErrorReport = 0;
    uint32_t now = millis();
    
    // Don't feed too often
    if (now - lastFeed < 1000) {
        return;
    }
    
    lastFeed = now;
    
    // Skip if WDT is disabled
    if (!wdtEnabled) {
        return;
    }
    
    // Always use direct task handle approach
    if (mainTaskHandle != NULL) {
        esp_err_t err = esp_task_wdt_reset();
        
        // Only log errors occasionally
        if (err != ESP_OK && now - lastErrorReport > 10000) {
            // If task not found, try to re-add
            if (err == ESP_ERR_NOT_FOUND) {
                esp_task_wdt_add(mainTaskHandle);
            }
            
            lastErrorReport = now;
        }
    }
    
    // Also use the generic approach as a backup
    esp_task_wdt_reset();
}

/**
 * LCD setup function with error handling
 */
bool setupLcd() {
    Logger.info("Setting up LCD display...");
    
    lcd_ready = false;
    
    try {
        // Initialize LCD
        lcd.begin(); 
        
        // Start with lower brightness
        lcd.example_bsp_set_lcd_backlight(100);
        delay(50);
        
        // Gradually increase brightness
        for (int i = 100; i <= 255; i += 20) {
            lcd.example_bsp_set_lcd_backlight(i);
            delay(10);
        }
        
        // Final brightness setting
        lcd.example_bsp_set_lcd_backlight(255);
        
        Logger.info("LCD initialized successfully with backlight ON");
        lcd_ready = true;
    } catch (...) {
        Logger.error("Exception during LCD initialization");
        lcd_ready = false;
    }
    
    return lcd_ready;
}

/**
 * Touch controller setup
 */
bool setupTouch() {
    Logger.info("Setting up touch controller...");
    
    touch_ready = false;
    
    try {
        // Initialize touch controller
        touch.begin();
        
        // Verify touch controller is responding
        uint16_t x, y;
        bool touchAvailable = touch.getTouch(&x, &y);
        
        Logger.info("Touch initialization %s", touchAvailable ? "confirmed working" : "succeeded but not verified");
        touch_ready = true;
    } catch (...) {
        Logger.error("Exception during touch initialization");
        touch_ready = false;
    }
    
    return touch_ready;
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
    
    // Calculate card size in GB
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    float sizeGB = cardSize / 1024.0;
    Logger.info("SD Card Size: %.2f GB (%llu MB)", sizeGB, cardSize);
    
    // Test writing capability
    File testFile = SD_MMC.open("/sd_test.txt", FILE_WRITE);
    if (!testFile) {
        Logger.error("Failed to open test file for writing");
        return false;
    }
    
    // Write test data
    size_t bytesWritten = testFile.println("SD Card Test");
    testFile.close();
    
    if (bytesWritten == 0) {
        Logger.error("Failed to write to SD card");
        return false;
    }
    
    // Clean up test file
    SD_MMC.remove("/sd_test.txt");
    
    return true;
}

/**
 * Create required directories for file organization
 */
void createRequiredDirectories() {
    if (!sd_ready) return;
    
    // Create the main images directory if it doesn't exist
    if (!SD_MMC.exists("/images")) {
        if (SD_MMC.mkdir("/images")) {
            Logger.info("Created /images directory");
        } else {
            Logger.error("Failed to create /images directory");
        }
    }
    
    // Create default folders for organization
    const char* defaultFolders[] = {"/images/favorites", "/images/wallpapers", "/images/camera"};
    
    for (const char* folder : defaultFolders) {
        if (!SD_MMC.exists(folder)) {
            if (SD_MMC.mkdir(folder)) {
                Logger.info("Created directory: %s", folder);
            } else {
                Logger.warn("Failed to create directory: %s", folder);
            }
        }
    }
}

/**
 * Enhanced SD card initialization with multiple retry strategies
 */
bool setupSdCardReliable() {
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
        
        // Try ultra-conservative settings first
        Logger.info("SD init attempt 1: 1-bit mode at 125KHz (ultra conservative)");
        if (SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING)) {
            // Successfully initialized
            if (validateSDCard()) {
                Logger.info("SD card initialized successfully in ultra-conservative mode");
                createRequiredDirectories();
                sd_ready = true;
                return true;
            }
        }
        
        // Try with more normal settings
        Logger.info("SD init attempt 2: 1-bit mode at default speed");
        SD_MMC.end();
        delay(500);
        if (SD_MMC.begin("/sdcard", true)) {
            if (validateSDCard()) {
                Logger.info("SD card initialized successfully with 1-bit mode");
                createRequiredDirectories();
                sd_ready = true;
                return true;
            }
        }
        
        // Last attempt with 4-bit mode
        Logger.info("SD init attempt 3: 4-bit mode");
        SD_MMC.end();
        delay(500);
        if (SD_MMC.begin()) {
            if (validateSDCard()) {
                Logger.info("SD card initialized successfully with 4-bit mode");
                createRequiredDirectories();
                sd_ready = true;
                return true;
            }
        }
        
        Logger.error("All SD card initialization attempts failed");
        sd_ready = false;
    } catch (...) {
        Logger.error("Exception during SD card initialization");
        sd_ready = false;
    }
    
    return sd_ready;
}


/**
 * Setup WiFi - can operate in AP mode, Station mode, or both
 */
bool setupWiFi() {
    Logger.info("Setting up WiFi...");
    
    // Reset WiFi completely
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    
    // Improved WiFi configuration
    WiFi.persistent(false);      // Don't store in flash
    WiFi.setSleep(WIFI_PS_NONE); // Disable power saving
    
    // Determine which mode to use
    if (WIFI_STA_ENABLED) {
        // Use Station + AP mode
        Logger.info("Setting up dual mode WiFi (AP+STA)");
        WiFi.mode(WIFI_AP_STA);
        
        // Connect to existing WiFi
        Logger.info("Connecting to WiFi SSID: %s", WIFI_STA_SSID);
        WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
        
        // Wait for connection with timeout
        uint8_t attempt = 0;
        while (WiFi.status() != WL_CONNECTED && attempt < 20) {
            delay(500);
            Logger.debug("Connecting to WiFi... %d/20", ++attempt);
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            IPAddress localIP = WiFi.localIP();
            Logger.info("Connected to WiFi with IP: %s", localIP.toString().c_str());
        } else {
            Logger.warn("Failed to connect to WiFi, continuing with AP mode only");
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
        }
    } else {
        // AP mode only
        Logger.info("Setting up WiFi in AP mode only");
        WiFi.mode(WIFI_AP);
    }
    
    // Configure AP with static IP
    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    if (!WiFi.softAPConfig(localIP, gateway, subnet)) {
        Logger.error("Failed to configure AP IP settings");
        return false;
    }
    
    // Start the AP with improved parameters
    Logger.info("Starting AP with SSID: %s", WIFI_AP_SSID);
    bool success = false;
    
    for (int attempt = 1; attempt <= 3; attempt++) {
        Logger.info("AP start attempt %d...", attempt);
        
        success = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_CHANNEL, false, MAX_WIFI_CLIENTS);
        
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
            delay(1000 * attempt); // Progressive backoff
        }
    }
    
    if (success) {
        IPAddress apIP = WiFi.softAPIP();
        Logger.info("WiFi AP Mode active:");
        Logger.info("AP SSID: %s", WIFI_AP_SSID);
        Logger.info("AP IP Address: %s", apIP.toString().c_str());
        Logger.info("AP Channel: %d", WIFI_CHANNEL);
        Logger.info("AP Max Clients: %d", MAX_WIFI_CLIENTS);
        
        wifi_ready = true;
    } else {
        Logger.error("Failed to start WiFi AP after multiple attempts");
        wifi_ready = false;
    }
    
    return success;
}


/**
 * Setup LVGL graphics library
 */
bool setupLvgl() {
    Logger.info("Setting up LVGL...");
    
    lvgl_ready = false;
    
    // Initialize LVGL
    lv_init();
    
    // Allocate display buffer - use larger buffer for smoother rendering
    uint32_t bufSize = LCD_H_RES * 80;  // Buffer for 80 rows
    Logger.debug("Allocating LVGL buffers: %d bytes each", bufSize * sizeof(lv_color_t));
    
    buf = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));
    buf1 = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));
    
    if (!buf || !buf1) {
        // Fall back to smaller buffer if allocation fails
        bufSize = LCD_H_RES * 40;
        Logger.warn("Falling back to smaller LVGL buffers: %d bytes each", bufSize * sizeof(lv_color_t));
        
        if (buf) free(buf);
        if (buf1) free(buf1);
        
        buf = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));
        buf1 = (lv_color_t *)ps_malloc(bufSize * sizeof(lv_color_t));
        
        if (!buf || !buf1) {
            Logger.error("Failed to allocate LVGL buffers");
            return false;
        }
    }
    
    // Initialize the display buffer
    lv_disp_draw_buf_init(&draw_buf, buf, buf1, bufSize);
    
    // Display driver initialization
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    
    // Set display driver properties
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = [](lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
        uint32_t w = (area->x2 - area->x1 + 1);
        uint32_t h = (area->y2 - area->y1 + 1);
        
        // Convert LVGL color buffer to 16-bit RGB565 format for LCD
        lcd.lcd_draw_bitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *)color_p);
        
        // Indicate to LVGL that the flush is done
        lv_disp_flush_ready(disp);
    };
    
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    // Input device driver for touch
    if (touch_ready) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        
        indev_drv.read_cb = [](lv_indev_drv_t *drv, lv_indev_data_t *data) -> void {
            static lv_coord_t last_x = 0;
            static lv_coord_t last_y = 0;
            
            uint16_t touch_x, touch_y;
            
            if (touch.getTouch(&touch_x, &touch_y)) {
                data->state = LV_INDEV_STATE_PR;
                data->point.x = touch_x;
                data->point.y = touch_y;
                last_x = touch_x;
                last_y = touch_y;
                
                // Update last touch time
                lastTouchEvent = millis();
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
 * Setup the image cache system
 */
bool setupCache() {
    // Initialize cache entries
    for (int i = 0; i < CACHE_SIZE; i++) {
        imageCache[i].path[0] = '\0';
        imageCache[i].data = nullptr;
        imageCache[i].size = 0;
        imageCache[i].lastUsed = 0;
        imageCache[i].valid = false;
    }
    
    // Allocate memory for draw buffer
    drawBuffer = (uint8_t*)ps_malloc(DRAW_BUFFER_SIZE);
    if (!drawBuffer) {
        Logger.error("Failed to allocate JPEG draw buffer");
        return false;
    }
    
    Logger.info("Image cache system initialized with %d slots", CACHE_SIZE);
    return true;
}

/**
 * Switch between different view modes
 */
void switchViewMode(int newMode) {
    switch (newMode) {
        case VIEW_MODE_SINGLE:
            // Switch to single image view
            lv_disp_load_scr(mainScreen);
            viewMode = VIEW_MODE_SINGLE;
            break;
            
        case VIEW_MODE_GALLERY: {
            // Populate gallery with thumbnails
            lv_obj_clean(thumbnailView);
            
            // Create thumbnails
            int thumbnailSize = 150;
            int maxThumbs = std::min(imageCount, 20); // Limit thumbnails for performance
            
            for (int i = 0; i < maxThumbs; i++) {
                lv_obj_t *imgBtn = lv_btn_create(thumbnailView);
                lv_obj_set_size(imgBtn, thumbnailSize, thumbnailSize);
                lv_obj_add_style(imgBtn, &styleImage, 0);
                
                if (i == currentImage) {
                    lv_obj_add_style(imgBtn, &styleImageSelected, 0);
                }
                
                // Store image index in user data
                lv_obj_set_user_data(imgBtn, (void*)(intptr_t)i);
                
                lv_obj_t *imgName = lv_label_create(imgBtn);
                lv_label_set_text(imgName, imageList[i].name);
                lv_obj_align(imgName, LV_ALIGN_BOTTOM_MID, 0, -5);
                
                lv_obj_add_event_cb(imgBtn, [](lv_event_t *e) {
                    intptr_t idx = (intptr_t)lv_event_get_target(e)->user_data;
                    currentImage = (int)idx;
                    switchViewMode(VIEW_MODE_SINGLE);
                    displayImage(currentImage);
                }, LV_EVENT_CLICKED, NULL);
            }
            
            // Switch to gallery screen
            lv_disp_load_scr(galleryScreen);
            viewMode = VIEW_MODE_GALLERY;
            break;
        }
            
        case VIEW_MODE_FOLDERS: {
            // Populate folder view
            lv_obj_clean(folderView);
            
            // Create folder list
            for (int i = 0; i < folderCount; i++) {
                lv_obj_t *folderBtn = lv_btn_create(folderView);
                lv_obj_set_width(folderBtn, LCD_H_RES - 20);
                lv_obj_set_height(folderBtn, 60);
                lv_obj_add_style(folderBtn, &styleFolder, 0);
                
                if (i == currentFolder) {
                    lv_obj_add_style(folderBtn, &styleFolderSelected, 0);
                }
                
                // Store folder index in user data
                lv_obj_set_user_data(folderBtn, (void*)(intptr_t)i);
                
                lv_obj_t *folderName = lv_label_create(folderBtn);
                lv_label_set_text(folderName, folderList[i].name);
                lv_obj_align(folderName, LV_ALIGN_LEFT_MID, 10, 0);
                
                lv_obj_t *folderCount = lv_label_create(folderBtn);
                char countBuf[32];
                snprintf(countBuf, sizeof(countBuf), "%d images", folderList[i].imageCount);
                lv_label_set_text(folderCount, countBuf);
                lv_obj_align(folderCount, LV_ALIGN_RIGHT_MID, -10, 0);
                
                lv_obj_add_event_cb(folderBtn, [](lv_event_t *e) {
                    intptr_t idx = (intptr_t)lv_event_get_target(e)->user_data;
                    currentFolder = (int)idx;
                    switchViewMode(VIEW_MODE_GALLERY);
                }, LV_EVENT_CLICKED, NULL);
            }
            
            // Switch to folder screen
            lv_disp_load_scr(folderScreen);
            viewMode = VIEW_MODE_FOLDERS;
            break;
        }
    }
}

/**
 * Setup UI elements including styles
 */
bool setupUI() {
    Logger.info("Setting up UI...");
    
    ui_ready = false;
    
    if (!lvgl_ready) {
        Logger.error("Cannot setup UI - LVGL not initialized");
        return false;
    }
    
    // Define some nice colors for our UI
    lv_color_t primaryColor = lv_color_hex(0x2980b9);    // Blue
    lv_color_t secondaryColor = lv_color_hex(0x27ae60);  // Green
    lv_color_t accentColor = lv_color_hex(0xe74c3c);     // Red
    lv_color_t bgColor = lv_color_hex(0x1a1a1a);         // Dark gray
    lv_color_t textColor = lv_color_hex(0xffffff);       // White
    
    // Initialize styles
    
    // Status bar style
    lv_style_init(&styleStatusBar);
    lv_style_set_bg_color(&styleStatusBar, lv_color_hex(0x333333));
    lv_style_set_text_color(&styleStatusBar, textColor);
    lv_style_set_pad_all(&styleStatusBar, 5);
    
    // Control bar style
    lv_style_init(&styleControlBar);
    lv_style_set_bg_color(&styleControlBar, lv_color_hex(0x333333));
    lv_style_set_text_color(&styleControlBar, textColor);
    lv_style_set_pad_all(&styleControlBar, 5);
    
    // Button style
    lv_style_init(&styleBtn);
    lv_style_set_radius(&styleBtn, 8);
    lv_style_set_bg_color(&styleBtn, primaryColor);
    lv_style_set_bg_opa(&styleBtn, LV_OPA_COVER);
    lv_style_set_pad_all(&styleBtn, 10);
    lv_style_set_text_color(&styleBtn, textColor);
    
    // Pressed button style
    lv_style_init(&styleBtnPressed);
    lv_style_set_bg_color(&styleBtnPressed, lv_color_darken(primaryColor, 30));
    
    // Image style
    lv_style_init(&styleImage);
    lv_style_set_border_width(&styleImage, 2);
    lv_style_set_border_color(&styleImage, lv_color_hex(0x333333));
    lv_style_set_pad_all(&styleImage, 2);
    
    // Selected image style
    lv_style_init(&styleImageSelected);
    lv_style_set_border_width(&styleImageSelected, 3);
    lv_style_set_border_color(&styleImageSelected, secondaryColor);
    
    // Folder style
    lv_style_init(&styleFolder);
    lv_style_set_bg_color(&styleFolder, lv_color_hex(0x444444));
    lv_style_set_radius(&styleFolder, 5);
    lv_style_set_pad_all(&styleFolder, 10);
    lv_style_set_text_color(&styleFolder, textColor);
    
    // Selected folder style
    lv_style_init(&styleFolderSelected);
    lv_style_set_bg_color(&styleFolderSelected, secondaryColor);
    
    // Create screens
    
    // Main viewing screen
    mainScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(mainScreen, bgColor, 0);
    lv_disp_load_scr(mainScreen);
    
    // Gallery (thumbnail) screen
    galleryScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(galleryScreen, bgColor, 0);
    
    // Folder selection screen
    folderScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(folderScreen, bgColor, 0);
    
    // Create status bar at the top
    statusBar = lv_obj_create(mainScreen);
    lv_obj_set_size(statusBar, LCD_H_RES, 40);
    lv_obj_set_pos(statusBar, 0, 0);
    lv_obj_add_style(statusBar, &styleStatusBar, 0);
    
    // Status label on left side
    statusLabel = lv_label_create(statusBar);
    lv_label_set_text(statusLabel, "ESP32P4 Gallery");
    lv_obj_align(statusLabel, LV_ALIGN_LEFT_MID, 10, 0);
    
    // Image counter on right side
    imageCounter = lv_label_create(statusBar);
    lv_label_set_text(imageCounter, "Image: 0/0");
    lv_obj_align(imageCounter, LV_ALIGN_RIGHT_MID, -10, 0);
    
    // WiFi status
    wifiLabel = lv_label_create(statusBar);
    lv_label_set_text(wifiLabel, LV_SYMBOL_WIFI);
    lv_obj_align(wifiLabel, LV_ALIGN_TOP_RIGHT, -40, 5);
    
    // Create image view area
    imageView = lv_obj_create(mainScreen);
    lv_obj_set_size(imageView, LCD_H_RES, LCD_V_RES - 80);
    lv_obj_set_pos(imageView, 0, 40);
    lv_obj_set_style_bg_color(imageView, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(imageView, 0, 0);
    lv_obj_set_style_pad_all(imageView, 0, 0);
    
    // Create control bar at the bottom
    controlBar = lv_obj_create(mainScreen);
    lv_obj_set_size(controlBar, LCD_H_RES, 40);
    lv_obj_set_pos(controlBar, 0, LCD_V_RES - 40);
    lv_obj_add_style(controlBar, &styleControlBar, 0);
    
    // Previous button
    prevBtn = lv_btn_create(controlBar);
    lv_obj_set_size(prevBtn, 80, 30);
    lv_obj_add_style(prevBtn, &styleBtn, 0);
    lv_obj_add_style(prevBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(prevBtn, LV_ALIGN_LEFT_MID, 10, 0);
    
    lv_obj_t *prevLabel = lv_label_create(prevBtn);
    lv_label_set_text(prevLabel, LV_SYMBOL_LEFT);
    lv_obj_center(prevLabel);
    
    lv_obj_add_event_cb(prevBtn, [](lv_event_t *e) {
        if (imageCount > 0) {
            int prevImg = (currentImage > 0) ? (currentImage - 1) : (imageCount - 1);
            displayImage(prevImg);
        }
    }, LV_EVENT_CLICKED, NULL);
    
    // Next button
    nextBtn = lv_btn_create(controlBar);
    lv_obj_set_size(nextBtn, 80, 30);
    lv_obj_add_style(nextBtn, &styleBtn, 0);
    lv_obj_add_style(nextBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(nextBtn, LV_ALIGN_RIGHT_MID, -10, 0);
    
    lv_obj_t *nextLabel = lv_label_create(nextBtn);
    lv_label_set_text(nextLabel, LV_SYMBOL_RIGHT);
    lv_obj_center(nextLabel);
    
    lv_obj_add_event_cb(nextBtn, [](lv_event_t *e) {
        if (imageCount > 0) {
            int nextImg = (currentImage < imageCount - 1) ? (currentImage + 1) : 0;
            displayImage(nextImg);
        }
    }, LV_EVENT_CLICKED, NULL);
    
    // Slideshow button
    lv_obj_t *slideshowBtn = lv_btn_create(controlBar);
    lv_obj_set_size(slideshowBtn, 90, 30);
    lv_obj_add_style(slideshowBtn, &styleBtn, 0);
    lv_obj_add_style(slideshowBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(slideshowBtn, LV_ALIGN_CENTER, 0, 0);
    
    lv_obj_t *slideshowLabel = lv_label_create(slideshowBtn);
    lv_label_set_text(slideshowLabel, LV_SYMBOL_PLAY " Play");
    lv_obj_center(slideshowLabel);
    
    lv_obj_add_event_cb(slideshowBtn, [](lv_event_t *e) {
        if (slideshowActive) {
            stopSlideshow();
            lv_label_set_text((lv_obj_t*)lv_event_get_target(e)->user_data, LV_SYMBOL_PLAY " Play");
        } else {
            startSlideshow(5000);
            lv_label_set_text((lv_obj_t*)lv_event_get_target(e)->user_data, LV_SYMBOL_STOP " Stop");
        }
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_set_user_data(slideshowBtn, slideshowLabel);  // Store label pointer for updates
    
    // Gallery button
    galleryBtn = lv_btn_create(controlBar);
    lv_obj_set_size(galleryBtn, 40, 30);
    lv_obj_add_style(galleryBtn, &styleBtn, 0);
    lv_obj_add_style(galleryBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(galleryBtn, LV_ALIGN_LEFT_MID, 100, 0);
    
    lv_obj_t *galleryLabel = lv_label_create(galleryBtn);
    lv_label_set_text(galleryLabel, LV_SYMBOL_LIST);
    lv_obj_center(galleryLabel);
    
    lv_obj_add_event_cb(galleryBtn, [](lv_event_t *e) {
        switchViewMode(VIEW_MODE_GALLERY);
    }, LV_EVENT_CLICKED, NULL);
    
    // Zoom button
    zoomBtn = lv_btn_create(controlBar);
    lv_obj_set_size(zoomBtn, 40, 30);
    lv_obj_add_style(zoomBtn, &styleBtn, 0);
    lv_obj_add_style(zoomBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(zoomBtn, LV_ALIGN_RIGHT_MID, -100, 0);
    
    lv_obj_t *zoomLabel = lv_label_create(zoomBtn);
    lv_label_set_text(zoomLabel, LV_SYMBOL_PLUS);
    lv_obj_center(zoomLabel);
    
    lv_obj_add_event_cb(zoomBtn, [](lv_event_t *e) {
        handleImageZoom(currentZoom < 2.0 ? 2.0 : 1.0);
    }, LV_EVENT_CLICKED, NULL);
    
    // Effect button
    effectBtn = lv_btn_create(controlBar);
    lv_obj_set_size(effectBtn, 40, 30);
    lv_obj_add_style(effectBtn, &styleBtn, 0);
    lv_obj_add_style(effectBtn, &styleBtnPressed, LV_STATE_PRESSED);
    lv_obj_align(effectBtn, LV_ALIGN_RIGHT_MID, -150, 0);
    
    lv_obj_t *effectLabel = lv_label_create(effectBtn);
    lv_label_set_text(effectLabel, LV_SYMBOL_SETTINGS);
    lv_obj_center(effectLabel);
    
    lv_obj_add_event_cb(effectBtn, [](lv_event_t *e) {
        currentEffect = (currentEffect + 1) % 4; // Cycle through effects
        if (imageCount > 0) {
            displayImage(currentImage); // Redisplay with new effect
        }
    }, LV_EVENT_CLICKED, NULL);
    
    // Set up gallery screen
    thumbnailView = lv_obj_create(galleryScreen);
    lv_obj_set_size(thumbnailView, LCD_H_RES, LCD_V_RES - 40);
    lv_obj_set_pos(thumbnailView, 0, 40);
    lv_obj_set_flex_flow(thumbnailView, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(thumbnailView, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(thumbnailView, 5, 0);
    lv_obj_set_style_pad_row(thumbnailView, 5, 0);
    lv_obj_set_style_pad_column(thumbnailView, 5, 0);
    
    // Status bar for gallery view (clone of main status bar)
    lv_obj_t *galleryStatusBar = lv_obj_create(galleryScreen);
    lv_obj_set_size(galleryStatusBar, LCD_H_RES, 40);
    lv_obj_set_pos(galleryStatusBar, 0, 0);
    lv_obj_add_style(galleryStatusBar, &styleStatusBar, 0);
    
    lv_obj_t *galleryStatusLabel = lv_label_create(galleryStatusBar);
    lv_label_set_text(galleryStatusLabel, "Gallery View");
    lv_obj_align(galleryStatusLabel, LV_ALIGN_LEFT_MID, 10, 0);
    
    // Back button for gallery
    lv_obj_t *galleryBackBtn = lv_btn_create(galleryStatusBar);
    lv_obj_set_size(galleryBackBtn, 60, 30);
    lv_obj_add_style(galleryBackBtn, &styleBtn, 0);
    lv_obj_align(galleryBackBtn, LV_ALIGN_RIGHT_MID, -10, 0);
    
    lv_obj_t *galleryBackLabel = lv_label_create(galleryBackBtn);
    lv_label_set_text(galleryBackLabel, "Back");
    lv_obj_center(galleryBackLabel);
    
    lv_obj_add_event_cb(galleryBackBtn, [](lv_event_t *e) {
        switchViewMode(VIEW_MODE_SINGLE);
    }, LV_EVENT_CLICKED, NULL);
    
    // Set up folder screen
    folderView = lv_obj_create(folderScreen);
    lv_obj_set_size(folderView, LCD_H_RES, LCD_V_RES - 40);
    lv_obj_set_pos(folderView, 0, 40);
    lv_obj_set_flex_flow(folderView, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(folderView, 5, 0);
    
    // Status bar for folder view
    lv_obj_t *folderStatusBar = lv_obj_create(folderScreen);
    lv_obj_set_size(folderStatusBar, LCD_H_RES, 40);
    lv_obj_set_pos(folderStatusBar, 0, 0);
    lv_obj_add_style(folderStatusBar, &styleStatusBar, 0);
    
    lv_obj_t *folderStatusLabel = lv_label_create(folderStatusBar);
    lv_label_set_text(folderStatusLabel, "Folder View");
    lv_obj_align(folderStatusLabel, LV_ALIGN_LEFT_MID, 10, 0);
    
    // Back button for folders
    lv_obj_t *folderBackBtn = lv_btn_create(folderStatusBar);
    lv_obj_set_size(folderBackBtn, 60, 30);
    lv_obj_add_style(folderBackBtn, &styleBtn, 0);
    lv_obj_align(folderBackBtn, LV_ALIGN_RIGHT_MID, -10, 0);
    
    lv_obj_t *folderBackLabel = lv_label_create(folderBackBtn);
    lv_label_set_text(folderBackLabel, "Back");
    lv_obj_center(folderBackLabel);
    
    lv_obj_add_event_cb(folderBackBtn, [](lv_event_t *e) {
        switchViewMode(VIEW_MODE_GALLERY);
    }, LV_EVENT_CLICKED, NULL);
    
    Logger.info("UI initialized successfully");
    ui_ready = true;
    
    return ui_ready;
}



/**
 * Setup web server with enhanced features
 */
bool setupWebserver() {
    Logger.info("Setting up WebServer...");
    
    try {
        // Basic server configuration
        server.enableCORS(true);
        server.enableCrossOrigin(true);
        
        // Root route - main interface
        server.on("/", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            String response = "<!DOCTYPE html><html><head>";
            response += "<title>ESP32P4 Advanced Gallery</title>";
            response += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
            response += "<link rel='icon' href='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAACXBIWXMAAAsTAAALEwEAmpwYAAABLElEQVQ4y61TO04DMRSclQhFKpI0dBR0CGq4ABUHACUXQKLiHnAFWiQukJKKKjVpKGho4QK7dOFFNshKyJJMYfvt+vtmxs94SZLsA7gFsI7iZZH8JulLmkn6SOk7ZcpFM38/1wo9yVqRMvrW9xHoUn6rCqCN3lWAVJJfwGvMvQDTKoA1Xmf8GvZJ3lq6S+fo4IVwUAVowpFH76E+EzYJdys+HVYBmnDgERHhJ+EfgIOl3+GBAEA48N0GXXHid3A1372f+5/EJAcugL7v1InsNvpFcubV9/F3Y/UJ4IQIIO9FxuaLSA+E5qslkrJPOPbqwv8A8K3kaV7IuQfPxezz3pqZXZG8CQZUsh7JXknOJM1jc06SUYETkmNJD5K+i9Y47P5Vcgr6ogAAAABJRU5ErkJggg=='>";
            response += "<style>";
            response += "body {font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px; max-width: 1200px; margin: 0 auto;}";
            response += "h1, h2 {color: #3b82f6;}";
            response += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);}";
            response += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; transition: all 0.3s;}";
            response += ".btn:hover {background: #2563eb;}";
            response += ".btn-danger {background: #ef4444;}";
            response += ".btn-danger:hover {background: #dc2626;}";
            response += ".gallery {display: flex; flex-wrap: wrap; gap: 10px; margin: 20px 0;}";
            response += ".thumbnail {width: 150px; height: 150px; object-fit: cover; border-radius: 4px; box-shadow: 0 2px 4px rgba(0,0,0,0.2);}";
            response += ".thumb-container {position: relative; width: 150px; cursor: pointer;}";
            response += ".thumb-container img {transition: transform 0.2s;}";
            response += ".thumb-container:hover img {transform: scale(1.05);}";
            response += ".thumb-name {font-size: 0.8em; text-align: center; margin-top: 5px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;}";
            response += ".controls {display: flex; gap: 10px; margin: 20px 0; flex-wrap: wrap;}";
            response += ".status {margin-top: 10px; padding: 10px; background: #1e293b; border-radius: 4px;}";
            response += ".folder-grid {display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 15px; margin: 20px 0;}";
            response += ".folder {background: #475569; padding: 15px; border-radius: 8px; cursor: pointer; transition: all 0.2s;}";
            response += ".folder:hover {background: #64748b;}";
            response += ".folder-icon {font-size: 2em; margin-bottom: 10px;}";
            response += ".folder-info {display: flex; justify-content: space-between; margin-top: 10px; font-size: 0.9em; color: #cbd5e1;}";
            response += "input, select {padding: 8px; border-radius: 4px; border: none; margin-right: 10px;}";
            response += ".tabs {display: flex; margin-bottom: 20px; border-bottom: 1px solid #475569;}";
            response += ".tab {padding: 10px 15px; cursor: pointer; opacity: 0.7;}";
            response += ".tab.active {border-bottom: 3px solid #3b82f6; opacity: 1;}";
            response += ".tab-content {display: none;}";
                        response += ".tab-content.active {display: block;}";
            response += "@media (max-width: 600px) { .controls { flex-direction: column; } .thumbnail { width: 120px; height: 120px; }}";
            response += "@keyframes fadeIn { from { opacity: 0; } to { opacity: 1; }}";
            response += ".fade-in { animation: fadeIn 0.5s ease-in;}";
            response += ".upload-progress { width: 100%; height: 10px; background: #1e293b; border-radius: 5px; margin-top: 10px;}";
            response += ".upload-bar { height: 100%; width: 0%; background: #3b82f6; border-radius: 5px; transition: width 0.2s;}";
            response += ".stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; }";
            response += ".stat-item { background: #475569; padding: 15px; border-radius: 8px; text-align: center; }";
            response += ".stat-value { font-size: 1.5em; font-weight: bold; margin: 10px 0; color: #3b82f6; }";
            response += "</style></head><body>";
            
            // Header with diagnostic info
            response += "<h1>ESP32P4 Advanced Gallery</h1>";
            response += "<div class='status'>Server IP: " + WiFi.softAPIP().toString() + " | ";
            response += "Clients: " + String(WiFi.softAPgetStationNum()) + " | ";
            response += "Signal: OK</div>";
            
            // Tabs navigation
            response += "<div class='tabs'>";
            response += "<div class='tab active' onclick='showTab(\"gallery\")'>Gallery</div>";
            response += "<div class='tab' onclick='showTab(\"upload\")'>Upload</div>";
            response += "<div class='tab' onclick='showTab(\"folders\")'>Folders</div>";
            response += "<div class='tab' onclick='showTab(\"system\")'>System</div>";
            response += "</div>";
            
            // Gallery tab
            response += "<div id='gallery-tab' class='tab-content active'>";
            response += "<div class='card'>";
            response += "<h2>Image Gallery</h2>";
            
            response += "<div class='controls'>";
            response += "<button class='btn' onclick='prevImage()'>Previous</button>";
            response += "<button class='btn' onclick='toggleSlideshow()' id='slideshowBtn'>";
            response += slideshowActive ? "Stop Slideshow" : "Start Slideshow";
            response += "</button>";
            response += "<button class='btn' onclick='nextImage()'>Next</button>";
            
            response += "<select id='transition-select' onchange='setTransition(this.value)'>";
            response += "<option value='0'" + String(currentTransition == TRANSITION_NONE ? " selected" : "") + ">No Transition</option>";
            response += "<option value='1'" + String(currentTransition == TRANSITION_FADE ? " selected" : "") + ">Fade</option>";
            response += "<option value='2'" + String(currentTransition == TRANSITION_SLIDE ? " selected" : "") + ">Slide</option>";
            response += "<option value='3'" + String(currentTransition == TRANSITION_ZOOM ? " selected" : "") + ">Zoom</option>";
            response += "</select>";
            
            response += "</div>";
            
            response += "<div class='gallery'>";
            
            if (imageCount > 0 && sd_ready) {
                // Cast volatile int to regular int for type compatibility
                int imgCount = (int)imageCount;
                int maxDisplay = min(imgCount, 20);  // Display up to 20 thumbnails
                
                for (int i = 0; i < maxDisplay; i++) {
                    response += "<div class='thumb-container' onclick='showImage(" + String(i) + ")'>";
                    response += "<img src='/thumb?index=" + String(i) + "' class='thumbnail'" +
                               (i == currentImage ? " style='border: 3px solid #3b82f6;'" : "") +
                               " alt='Image " + String(i+1) + "'>";
                    response += "<div class='thumb-name'>" + String(imageList[i].name) + "</div>";
                    response += "</div>";
                }
                
                if (imgCount > maxDisplay) {
                    response += "<div class='thumb-container'><p>+" + String(imgCount - maxDisplay) + " more images</p></div>";
                }
            } else {
                response += "<p>No images available. Please upload some using the form in the Upload tab.</p>";
            }
            
            response += "</div>"; // gallery
            response += "</div>"; // card
            response += "</div>"; // gallery-tab
            
            // Upload tab
            response += "<div id='upload-tab' class='tab-content'>";
            response += "<div class='card'>";
            response += "<h2>Upload Images</h2>";
            
            response += "<form method='POST' action='/upload' enctype='multipart/form-data' id='uploadForm'>";
            response += "<div style='margin-bottom: 15px;'>";
            response += "<label for='folder-select'>Folder: </label>";
            response += "<select id='folder-select' name='folder'>";
            response += "<option value='images'>Root</option>";
            
            // Add folder options
            for (int i = 0; i < folderCount; i++) {
                response += "<option value='" + String(folderList[i].name) + "'>" + String(folderList[i].name) + "</option>";
            }
            
            response += "</select>";
            response += "<button type='button' class='btn' onclick='showNewFolderDialog()' style='margin-left: 10px;'>New Folder</button>";
            response += "</div>";
            
            response += "<input type='file' name='image' accept='image/jpeg' multiple id='file-input'>";
            response += "<button type='submit' class='btn'>Upload</button>";
            response += "</form>";
            
            response += "<div id='uploadStatus' class='status' style='display:none;'></div>";
            response += "<div id='progress-container' style='display:none;'>";
            response += "<div class='upload-progress'><div id='progress-bar' class='upload-bar'></div></div>";
            response += "</div>";
            
            response += "<div id='new-folder-dialog' style='display:none; margin-top: 15px; padding: 15px; border: 1px solid #475569; border-radius: 8px;'>";
            response += "<h3>Create New Folder</h3>";
            response += "<input type='text' id='new-folder-name' placeholder='Folder name'>";
            response += "<button class='btn' onclick='createFolder()'>Create</button>";
            response += "<button class='btn' onclick='hideNewFolderDialog()' style='background:#64748b;'>Cancel</button>";
            response += "</div>";
            
            response += "</div>"; // card
            response += "</div>"; // upload-tab
            
            // Folders tab
            response += "<div id='folders-tab' class='tab-content'>";
            response += "<div class='card'>";
            response += "<h2>Image Folders</h2>";
            
            response += "<div class='folder-grid'>";
            
            // Add root folder
            response += "<div class='folder' onclick='openFolder(\"images\")'>";
            response += "<div class='folder-icon'>📁</div>";
            response += "<div>Root</div>";
            int rootImageCount = 0;
            for (int i = 0; i < imageCount; i++) {
                if (strncmp(imageList[i].folder, "images", 6) == 0) rootImageCount++;
            }
            response += "<div class='folder-info'><span>" + String(rootImageCount) + " images</span></div>";
            response += "</div>";
            
            // Add all folders
            for (int i = 0; i < folderCount; i++) {
                response += "<div class='folder' onclick='openFolder(\"" + String(folderList[i].name) + "\")'>";
                response += "<div class='folder-icon'>📁</div>";
                response += "<div>" + String(folderList[i].name) + "</div>";
                response += "<div class='folder-info'>";
                response += "<span>" + String(folderList[i].imageCount) + " images</span>";
                response += "</div>";
                response += "</div>";
            }
            
            response += "</div>"; // folder-grid
            response += "</div>"; // card
            response += "</div>"; // folders-tab
            
            // System tab
            response += "<div id='system-tab' class='tab-content'>";
            response += "<div class='card'>";
            response += "<h2>System Information</h2>";
            
            response += "<div class='stats'>";
            response += "<div class='stat-item'>";
            response += "<div>Memory</div>";
            response += "<div class='stat-value'>" + String(ESP.getFreeHeap() / 1024) + " KB</div>";
            response += "<div>Free Heap</div>";
            response += "</div>";
            
            response += "<div class='stat-item'>";
            response += "<div>PSRAM</div>";
            response += "<div class='stat-value'>" + String(ESP.getFreePsram() / 1024) + " KB</div>";
            response += "<div>Free PSRAM</div>";
            response += "</div>";
            
            response += "<div class='stat-item'>";
            response += "<div>Images</div>";
            response += "<div class='stat-value'>" + String(imageCount) + "</div>";
            response += "<div>Total Images</div>";
            response += "</div>";
            
            response += "<div class='stat-item'>";
            response += "<div>Uptime</div>";
            unsigned long uptimeSec = millis() / 1000;
            unsigned long uptimeMin = uptimeSec / 60;
            unsigned long uptimeHour = uptimeMin / 60;
            uptimeMin %= 60;
            response += "<div class='stat-value'>" + String(uptimeHour) + ":" + (uptimeMin < 10 ? "0" : "") + String(uptimeMin) + "</div>";
            response += "<div>Hours:Minutes</div>";
            response += "</div>";
            
            response += "</div>"; // stats
            
            response += "<h3>Storage</h3>";
            
            if (sd_ready) {
                uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
                uint64_t totalSpace = cardSize;
                uint64_t usedSpace = 0;
                
                // Calculate used space (this is approximate)
                for (int i = 0; i < imageCount; i++) {
                    usedSpace += imageList[i].size;
                }
                usedSpace /= (1024 * 1024); // Convert to MB
                
                float usedPercent = (float)usedSpace / (float)totalSpace * 100.0;
                
                response += "<div style='margin: 20px 0;'>";
                response += "<div>SD Card: " + String((uint32_t)cardSize) + " MB total, " + String((uint32_t)usedSpace) + " MB used</div>";
                response += "<div style='height: 20px; background: #1e293b; border-radius: 10px; overflow: hidden; margin: 10px 0;'>";
                response += "<div style='height: 100%; width: " + String(usedPercent) + "%; background: #3b82f6;'></div>";
                response += "</div>";
                response += "</div>";
            } else {
                response += "<div class='status' style='color: #ef4444;'>SD Card not available</div>";
            }
            
            response += "<h3>System Control</h3>";
            response += "<div class='controls'>";
            response += "<button class='btn' onclick='restartDevice()'>Restart Device</button>";
            response += "<button class='btn btn-danger' onclick='if(confirm(\"Really format SD card? All data will be lost!\")) formatSD()'>Format SD Card</button>";
            response += "</div>";
            
            response += "</div>"; // card
            response += "</div>"; // system-tab
            
            // JavaScript for interactivity
            response += "<script>";
            // Tab switching
            response += "function showTab(tabId) {";
            response += "  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));";
            response += "  document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));";
            response += "  document.getElementById(tabId + '-tab').classList.add('active');";
            response += "  document.querySelector(`.tab[onclick*='${tabId}']`).classList.add('active');";
            response += "}";
            
            // Image navigation
            response += "function prevImage() { fetch('/navigate?direction=prev', {method: 'POST'}).then(res => res.text()).then(data => console.log(data)); }";
            response += "function nextImage() { fetch('/navigate?direction=next', {method: 'POST'}).then(res => res.text()).then(data => console.log(data)); }";
            response += "function showImage(idx) { fetch('/image?index=' + idx, {method: 'GET'}).then(res => res.text()).then(data => console.log(data)); }";
            
            // Slideshow control
            response += "function toggleSlideshow() {";
            response += "  const btn = document.getElementById('slideshowBtn');";
            response += "  const action = btn.innerText.includes('Start') ? 'start' : 'stop';";
            response += "  fetch('/slideshow?action=' + action, {method: 'POST'})";
            response += "    .then(res => res.text())";
            response += "    .then(data => {";
            response += "      console.log(data);";
            response += "      btn.innerText = action === 'start' ? 'Stop Slideshow' : 'Start Slideshow';";
            response += "    });";
            response += "}";
            
            // Transition effect
            response += "function setTransition(type) {";
            response += "  fetch('/transition?type=' + type, {method: 'POST'})";
            response += "    .then(res => res.text())";
            response += "    .then(data => console.log(data));";
            response += "}";
            
            // Folder management
            response += "function showNewFolderDialog() { document.getElementById('new-folder-dialog').style.display = 'block'; }";
            response += "function hideNewFolderDialog() { document.getElementById('new-folder-dialog').style.display = 'none'; }";
            response += "function createFolder() {";
            response += "  const folderName = document.getElementById('new-folder-name').value;";
            response += "  if(!folderName) return alert('Please enter a folder name');";
            response += "  fetch('/folder/create?name=' + encodeURIComponent(folderName), {method: 'POST'})";
            response += "    .then(res => res.text())";
            response += "    .then(data => {";
            response += "      alert(data);";
            response += "      location.reload();";
            response += "    });";
            response += "}";
            response += "function openFolder(name) { location.href = '/folder?name=' + encodeURIComponent(name); }";
            
            // System controls
            response += "function restartDevice() {";
            response += "  if(confirm('Really restart the device?')) {";
            response += "    fetch('/system/restart', {method: 'POST'})";
            response += "      .then(res => res.text())";
            response += "      .then(data => alert('Device is restarting...'));";
            response += "  }";
            response += "}";
            response += "function formatSD() {";
            response += "  fetch('/system/format-sd', {method: 'POST'})";
            response += "    .then(res => res.text())";
            response += "    .then(data => alert(data));";
            response += "}";
            
            // Upload form handling
            response += "document.addEventListener('DOMContentLoaded', function() {";
            response += "  const uploadForm = document.getElementById('uploadForm');";
            response += "  if (!uploadForm) return;";
            
            response += "  uploadForm.addEventListener('submit', function(e) {";
            response += "    e.preventDefault();";
            response += "    const formData = new FormData(this);";
            response += "    const status = document.getElementById('uploadStatus');";
            response += "    const progressBar = document.getElementById('progress-bar');";
            response += "    const progressContainer = document.getElementById('progress-container');";
            
            response += "    status.textContent = 'Uploading...';";
            response += "    status.style.display = 'block';";
            response += "    progressContainer.style.display = 'block';";
            response += "    progressBar.style.width = '0%';";
            
            response += "    const xhr = new XMLHttpRequest();";
            response += "    xhr.open('POST', '/upload', true);";
            
            response += "    xhr.upload.addEventListener('progress', function(e) {";
            response += "      if (e.lengthComputable) {";
            response += "        const percent = Math.round((e.loaded / e.total) * 100);";
            response += "        progressBar.style.width = percent + '%';";
            response += "        status.textContent = `Uploading: ${percent}%`;";
            response += "      }";
            response += "    });";
            
            response += "    xhr.addEventListener('load', function() {";
            response += "      if (xhr.status === 200) {";
            response += "        status.textContent = 'Upload complete!';";
            response += "        setTimeout(() => location.reload(), 1000);";
            response += "      } else {";
            response += "        status.textContent = 'Upload failed: ' + xhr.statusText;";
            response += "      }";
            response += "    });";
            
            response += "    xhr.addEventListener('error', function() {";
            response += "      status.textContent = 'Network error during upload';";
            response += "    });";
            
            response += "    xhr.send(formData);";
            response += "  });";
            
            response += "});";
            response += "</script>";
            
            response += "</body></html>";
            
            server.send(200, "text/html", response);
        });
        
        // Authentication check handler
        server.on("/auth/check", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            server.send(200, "text/plain", "Authenticated");
        });
        
        // Navigation handler
        server.on("/navigate", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            String direction = server.arg("direction");
            Logger.info("Navigation request: %s", direction.c_str());
            
            if (imageCount <= 0) {
                server.send(400, "text/plain", "No images available");
                return;
            }
            
            if (direction == "prev") {
                int prevImage = (currentImage > 0) ? (currentImage - 1) : (imageCount - 1);
                displayImage(prevImage);
                server.send(200, "text/plain", "Displaying previous image");
            } else if (direction == "next") {
                int nextImage = (currentImage < imageCount - 1) ? (currentImage + 1) : 0;
                displayImage(nextImage);
                server.send(200, "text/plain", "Displaying next image");
            } else {
                server.send(400, "text/plain", "Invalid direction");
            }
        });
        
        // Image display handler
        server.on("/image", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            int index = server.arg("index").toInt();
            if (index >= 0 && index < imageCount) {
                displayImage(index);
                server.send(200, "text/plain", "Displaying image " + String(index + 1) + " of " + String(imageCount));
            } else {
                server.send(400, "text/plain", "Invalid image index");
            }
        });
        
        // Slideshow control handler
        server.on("/slideshow", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            String action = server.arg("action");
            uint32_t interval = 5000; // Default 5 seconds
            
            if (server.hasArg("interval")) {
                interval = server.arg("interval").toInt();
                if (interval < 1000) interval = 1000; // Minimum 1 second
            }
            
            if (action == "start") {
                startSlideshow(interval);
                server.send(200, "text/plain", "Slideshow started with " + String(interval) + "ms interval");
            }
            else if (action == "stop") {
                stopSlideshow();
                server.send(200, "text/plain", "Slideshow stopped");
            }
            else {
                server.send(400, "text/plain", "Invalid action");
            }
        });
        
        // Transition effect setter
        server.on("/transition", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            if (server.hasArg("type")) {
                int type = server.arg("type").toInt();
                if (type >= 0 && type <= 3) { // Check valid transition type
                    currentTransition = type;
                    server.send(200, "text/plain", "Transition set to " + String(type));
                } else {
                    server.send(400, "text/plain", "Invalid transition type");
                }
            } else {
                server.send(400, "text/plain", "Missing transition type");
            }
        });
        
        // Folder creation handler
        server.on("/folder/create", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            if (server.hasArg("name")) {
                String name = server.arg("name");
                // Sanitize folder name
                name.replace(" ", "_");
                name.replace("/", "_");
                
                String folderPath = "/images/" + name;
                
                if (createFolder(folderPath.c_str())) {
                    // Re-scan images to update folders
                    scanImages();
                    server.send(200, "text/plain", "Folder created successfully");
                } else {
                    server.send(500, "text/plain", "Failed to create folder");
                }
            } else {
                server.send(400, "text/plain", "Missing folder name");
            }
        });
        
        // Folder view handler
        server.on("/folder", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            String folderName = server.arg("name");
            if (folderName == "") folderName = "images"; // Default to root
            
            // Generate HTML for folder view (simplified for brevity)
            String response = "<!DOCTYPE html><html><head>";
            response += "<title>ESP32P4 Gallery - Folder: " + folderName + "</title>";
            response += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
            // Include the same CSS as the root page
            // ...
            
            response += "</head><body>";
            response += "<h1>Folder: " + folderName + "</h1>";
            response += "<p><a href='/'>Back to Gallery</a></p>";
            
            response += "<div class='gallery'>";
            
            // List images in this folder
            int folderImageCount = 0;
            for (int i = 0; i < imageCount; i++) {
                if (strncmp(imageList[i].folder, folderName.c_str(), folderName.length()) == 0) {
                    response += "<div class='thumb-container' onclick='showImage(" + String(i) + ")'>";
                    response += "<img src='/thumb?index=" + String(i) + "' class='thumbnail' alt='Image'>";
                    response += "<div class='thumb-name'>" + String(imageList[i].name) + "</div>";
                    response += "</div>";
                    folderImageCount++;
                }
            }
            
            if (folderImageCount == 0) {
                response += "<p>No images in this folder</p>";
            }
            
            response += "</div>";
            response += "</body></html>";
            
            server.send(200, "text/html", response);
        });
        
        // Thumbnail handler
        server.on("/thumb", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            int index = server.arg("index").toInt();
            
            if (index < 0 || index >= imageCount || !sd_ready) {
                server.send(404, "text/plain", "Image not found");
                return;
            }
            
            File file = SD_MMC.open(imageList[index].path, FILE_READ);
            if (!file) {
                server.send(404, "text/plain", "Image file not found");
                return;
            }
            
            server.sendHeader("Content-Disposition", "inline; filename=thumb.jpg");
            server.sendHeader("Cache-Control", "max-age=31536000");
            
            size_t fileSize = file.size();
            server.setContentLength(fileSize);
            server.send(200, "image/jpeg", "");
            
            // Send in chunks
            uint8_t buffer[1024];
            size_t remaining = fileSize;
            
            WiFiClient client = server.client();
            
            while (remaining > 0 && client.connected()) {
                size_t toRead = remaining < 1024 ? remaining : 1024;
                file.read(buffer, toRead);
                client.write(buffer, toRead);
                remaining -= toRead;
                
                // Feed watchdog during transmission
                if (remaining % 8192 == 0) {
                    feedWatchdog();
                }
            }
            
            file.close();
        });
        
        // System restart handler
        server.on("/system/restart", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            server.send(200, "text/plain", "Device is restarting...");
            delay(500);
            resetRequested = true;
        });
        
        // SD format handler
        server.on("/system/format-sd", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            formatSdRequested = true;
            server.send(200, "text/plain", "SD card format requested. This will take a moment...");
        });
        
        // Upload handler
        server.on("/upload", HTTP_POST, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            server.send(200, "text/plain", "Upload complete");
        }, handleUpload);
        
        // System info API for AJAX updates
        server.on("/api/system", HTTP_GET, []() {
            if (AUTH_ENABLED && !server.authenticate(AUTH_USERNAME, AUTH_PASSWORD)) {
                return server.requestAuthentication();
            }
            
            String json = "{";
            json += "\"memory\":" + String(ESP.getFreeHeap() / 1024) + ",";
            json += "\"psram\":" + String(ESP.getFreePsram() / 1024) + ",";
            json += "\"uptime\":" + String(millis() / 1000) + ",";
            json += "\"imageCount\":" + String(imageCount) + ",";
            json += "\"folderCount\":" + String(folderCount) + ",";
            json += "\"slideshowActive\":" + String(slideshowActive ? "true" : "false") + ",";
            json += "\"currentImage\":" + String(currentImage) + ",";
            json += "\"wifiClients\":" + String(WiFi.softAPgetStationNum());
            json += "}";
            
            server.send(200, "application/json", json);
        });
        
        // Add test page for connectivity diagnostics
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
            response += "<li><strong>WiFi Channel:</strong> " + String(WIFI_CHANNEL) + "</li>";
            response += "<li><strong>Server Uptime:</strong> " + String(millis() / 1000) + " seconds</li>";
            response += "<li><strong>Image Count:</strong> " + String(imageCount) + "</li>";
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
        });
        
        // Simple ping endpoint for testing connection speed
        server.on("/ping", HTTP_GET, []() {
            server.send(200, "text/plain", "pong");
        });
        
        // File not found handler
        server.onNotFound([]() {
            server.send(404, "text/plain", "404: Not Found");
        });
        
        // Start the server
        server.begin();
        Logger.info("WebServer started on port 80");
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
 * Create a new folder on the SD card
 */
bool createFolder(const char* folderName) {
    if (!sd_ready) return false;
    
    // Acquire mutex for SD card access
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Logger.error("Failed to get SD mutex for folder creation");
        return false;
    }
    
    bool success = SD_MMC.mkdir(folderName);
    
    xSemaphoreGive(sdMutex);
    
    if (success) {
        Logger.info("Created folder: %s", folderName);
    } else {
        Logger.error("Failed to create folder: %s", folderName);
    }
    
    return success;
}

/**
 * Enhanced image upload handler
 */
void handleUpload() {
    static File uploadFile;
    static uint32_t uploadStartTime;
    static bool uploadError;
    static String uploadPath;
    static size_t totalReceived;
    
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        // New upload starting
        uploadStartTime = millis();
        uploadError = false;
        totalReceived = 0;
        
        // Get the folder from form data
        String folder = server.arg("folder");
        if (folder.isEmpty()) folder = "images"; // Default to root folder
        
        // Generate filename with timestamp to avoid duplicates
        String filename = upload.filename;
        
        // Sanitize filename
        filename.replace(" ", "_");
        filename.replace("&", "_");
        filename.replace("?", "_");
        filename.replace("+", "_");
        
        // Create a unique path by adding a timestamp prefix
        int timestamp = millis() % 100000; // Use last 5 digits of timestamp
        uploadPath = "/" + folder + "/img_" + String(timestamp) + "_" + filename;
        
        Logger.info("Starting new upload: %s -> %s", filename.c_str(), uploadPath.c_str());
        
        // Make sure SD card mutex is free
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            Logger.error("Failed to acquire SD mutex for upload");
            uploadError = true;
            return;
        }
        
        // Check if SD card is available
        if (!sd_ready) {
            Logger.error("SD card not ready for upload");
            xSemaphoreGive(sdMutex);
            uploadError = true;
            return;
        }
        
        // Create parent folder if it doesn't exist (strip filename)
        int lastSlash = uploadPath.lastIndexOf('/');
        if (lastSlash > 0) {
            String parentFolder = uploadPath.substring(0, lastSlash);
            if (!SD_MMC.exists(parentFolder.c_str())) {
                if (!SD_MMC.mkdir(parentFolder.c_str())) {
                    Logger.warn("Failed to create parent folder %s", parentFolder.c_str());
                }
            }
        }
        
        // Open file for writing
        uploadFile = SD_MMC.open(uploadPath.c_str(), FILE_WRITE);
        
        if (!uploadFile) {
            Logger.error("Failed to create file: %s", uploadPath.c_str());
            xSemaphoreGive(sdMutex);
            uploadError = true;
            return;
        }
        
        Logger.info("File created for upload: %s", uploadPath.c_str());
    }
    else if (upload.status == UPLOAD_FILE_WRITE && !uploadError) {
        // Writing file data
        if (!uploadFile) {
            Logger.error("Upload file not open");
            uploadError = true;
            return;
        }
        
        // Write data
        size_t written = uploadFile.write(upload.buf, upload.currentSize);
        
        // Check if write was successful
        if (written != upload.currentSize) {
            Logger.error("SD Write error: %d of %d bytes written", 
                      written, upload.currentSize);
            uploadError = true;
        }
        
        totalReceived += written;
        
        // Feed the watchdog periodically during large uploads
        if (totalReceived % 65536 == 0) {
            feedWatchdog();
        }
    }
    else if (upload.status == UPLOAD_FILE_END && !uploadError) {
        // Upload finished
        if (uploadFile) {
            // Get final file size
            size_t fileSize = uploadFile.size();
            
            // Close the file
            uploadFile.close();
            
            // Release SD mutex
            xSemaphoreGive(sdMutex);
            
            uint32_t uploadDuration = millis() - uploadStartTime;
            
            Logger.info("Upload complete: %s (%d bytes in %d ms)", 
                      uploadPath.c_str(), fileSize, uploadDuration);
            
            // Validate the uploaded file
            if (fileSize == 0) {
                Logger.error("Uploaded file has zero size - likely corrupt");
                SD_MMC.remove(uploadPath.c_str());
            }
            else if (isValidJpeg(uploadPath.c_str())) {
                // Re-scan for images to include the new one
                scanImages();
                
                // Find and display the new image
                for (int i = 0; i < imageCount; i++) {
                    if (String(imageList[i].path) == uploadPath) {
                        displayImage(i);
                        break;
                    }
                }
            }
            else {
                Logger.warn("Uploaded file is not a valid JPEG");
                SD_MMC.remove(uploadPath.c_str());
            }
        }
    }
    else if (upload.status == UPLOAD_FILE_ABORTED || uploadError) {
        // Upload aborted or error occurred
        Logger.error("Upload aborted or failed");
        
        if (uploadFile) {
            uploadFile.close();
            
            // Release SD mutex if we have it
            xSemaphoreGive(sdMutex);
            
            // Delete the partial file
            if (uploadPath.length() > 0) {
                SD_MMC.remove(uploadPath.c_str());
            }
        }
    }
}

/**
 * Function to check if a file is a valid JPEG
 */
bool isValidJpeg(const char* filename) {
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or null filename");
        return false;
    }
    
    // Check if file exists
    if (!SD_MMC.exists(filename)) {
        Logger.error("File not found: %s", filename);
        return false;
    }
    
    File jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
        Logger.error("Failed to open file: %s", filename);
        return false;
    }
    
    // Check file size - reject if too small or suspiciously large
    size_t fileSize = jpegFile.size();
    if (fileSize < 1024) { // Minimum 1KB for a valid JPEG
        Logger.error("File too small to be valid JPEG: %s (%d bytes)", filename, fileSize);
        jpegFile.close();
        return false;
    }
    
    if (fileSize > 10000000) { // 10MB max
        Logger.warn("File too large for reliable display: %s (%d bytes)", filename, fileSize);
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
    
    // Check for JPEG signature (FF D8 FF)
    if (header[0] != 0xFF || header[1] != 0xD8 || header[2] != 0xFF) {
        Logger.error("Invalid JPEG header: %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Check for EOI marker (FF D9) at end of file
    if (fileSize > 2) {
        uint8_t footer[2];
        jpegFile.seek(fileSize - 2);
        if (jpegFile.readBytes((char*)footer, 2) == 2) {
            if (footer[0] != 0xFF || footer[1] != 0xD9) {
                Logger.warn("JPEG missing EOI marker - may be corrupted: %s", filename);
                // Don't fail just for this, some valid files might not end properly
            }
        }
    }
    
    jpegFile.close();
    return true;
}

/**
 * Cache an image in memory for faster access
 */
bool cacheImage(const char* path) {
    if (!sd_ready || !path) return false;
    
    // Check if already in cache
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (imageCache[i].valid && strcmp(imageCache[i].path, path) == 0) {
            // Already cached, just update timestamp
            imageCache[i].lastUsed = millis();
            return true;
        }
    }
    
    // Find the oldest entry to replace
    int oldestIndex = 0;
    uint32_t oldestTime = 0xFFFFFFFF;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (!imageCache[i].valid) {
            // Found an empty slot
            oldestIndex = i;
            break;
        }
        
        if (imageCache[i].lastUsed < oldestTime) {
            oldestTime = imageCache[i].lastUsed;
            oldestIndex = i;
        }
    }
    
    // Free previous data if any
    if (imageCache[oldestIndex].valid && imageCache[oldestIndex].data) {
        free(imageCache[oldestIndex].data);
        imageCache[oldestIndex].data = nullptr;
    }
    
    // Take SD mutex
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Logger.error("Failed to acquire SD mutex for image caching");
        return false;
    }
    
    // Open the file
    File file = SD_MMC.open(path, FILE_READ);
    if (!file) {
        Logger.error("Failed to open image file for caching: %s", path);
        xSemaphoreGive(sdMutex);
        return false;
    }
    
    // Get file size
    size_t fileSize = file.size();
    if (fileSize == 0 || fileSize > 5000000) { // Don't cache empty or overly large files
        Logger.warn("File size inappropriate for caching: %d bytes", fileSize);
        file.close();
        xSemaphoreGive(sdMutex);
        return false;
    }
    
    // Allocate memory
    uint8_t* buffer = (uint8_t*)ps_malloc(fileSize);
    if (!buffer) {
        Logger.error("Failed to allocate memory for image cache");
        file.close();
        xSemaphoreGive(sdMutex);
        return false;
    }
    
    // Read file data
    size_t bytesRead = file.read(buffer, fileSize);
    file.close();
    xSemaphoreGive(sdMutex);
    
    if (bytesRead != fileSize) {
        Logger.error("Failed to read complete file for caching");
        free(buffer);
        return false;
    }
    
    // Update cache entry
    strncpy(imageCache[oldestIndex].path, path, MAX_FILEPATH_LENGTH);
    imageCache[oldestIndex].data = buffer;
    imageCache[oldestIndex].size = fileSize;
    imageCache[oldestIndex].lastUsed = millis();
    imageCache[oldestIndex].valid = true;
    
    Logger.info("Cached image %s (%d bytes)", path, fileSize);
    return true;
}

/**
 * Get a cached image if available
 */
uint8_t* getCachedImage(const char* path, size_t* size) {
    if (!path || !size) return nullptr;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (imageCache[i].valid && strcmp(imageCache[i].path, path) == 0) {
            // Update access time
            imageCache[i].lastUsed = millis();
            *size = imageCache[i].size;
            return imageCache[i].data;
        }
    }
    
    // Not found
    return nullptr;
}

/**
 * Clear image cache
 */
void clearImageCache() {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (imageCache[i].valid && imageCache[i].data) {
            free(imageCache[i].data);
        }
        imageCache[i].data = nullptr;
        imageCache[i].path[0] = '\0';
        imageCache[i].size = 0;
        imageCache[i].lastUsed = 0;
        imageCache[i].valid = false;
    }
    Logger.info("Image cache cleared");
}

/**
 * Scan for images on SD card with folder organization
 */
void scanImages() {
    Logger.info("Scanning for images with folder organization...");
    
    if (!sd_ready) {
        Logger.error("Cannot scan for images - SD card not ready");
        imageCount = 0;
        return;
    }
    
    // Acquire SD mutex
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Logger.error("Failed to get SD mutex for image scan");
        return;
    }
    
    // Free existing image list memory
    if (imageList != nullptr) {
        free(imageList);
    }
    
    // Allocate memory for image list
    imageList = (ImageInfo*)ps_calloc(MAX_IMAGES, sizeof(ImageInfo));
    if (!imageList) {
        Logger.error("Failed to allocate memory for image list");
        xSemaphoreGive(sdMutex);
        return;
    }
    
    imageCount = 0;
    folderCount = 0;
    
    // Reset folder list
    for (int i = 0; i < MAX_FOLDERS; i++) {
        folderList[i].name[0] = '\0';
        folderList[i].imageCount = 0;
    }
    
    // Make sure the main images directory exists
    if (!SD_MMC.exists("/images")) {
        if (SD_MMC.mkdir("/images")) {
            Logger.info("Created /images directory");
        } else {
            Logger.error("Failed to create /images directory");
            xSemaphoreGive(sdMutex);
            return;
        }
    }
    
    // First scan root images directory
    scanImagesInFolder("/images", "images");
    
    // Now scan all subdirectories
    File root = SD_MMC.open("/images");
    if (!root || !root.isDirectory()) {
        Logger.error("Failed to open /images directory");
        root.close();
        xSemaphoreGive(sdMutex);
        return;
    }
    
    File entry = root.openNextFile();
    while (entry && folderCount < MAX_FOLDERS) {
        if (entry.isDirectory()) {
            String folderPath = entry.name();
            String folderName = folderPath.substring(folderPath.lastIndexOf('/') + 1);
            
            // Register the folder
            if (folderCount < MAX_FOLDERS) {
                strncpy(folderList[folderCount].name, folderName.c_str(), MAX_FILENAME_LENGTH);
                folderList[folderCount].imageCount = 0;
                folderCount++;
            }
            
            // Scan images in this folder
            scanImagesInFolder(folderPath.c_str(), folderName.c_str());
        }
        entry.close();
        entry = root.openNextFile();
    }
    
    root.close();
    xSemaphoreGive(sdMutex);
    
    // Update folder image counts
    for (int i = 0; i < imageCount; i++) {
        for (int j = 0; j < folderCount; j++) {
            if (strcmp(imageList[i].folder, folderList[j].name) == 0) {
                folderList[j].imageCount++;
                break;
            }
        }
    }
    
    Logger.info("Found %d images in %d folders", imageCount, folderCount);
    
    // Create a test image if no images were found
    if (imageCount == 0) {
        Logger.warn("No images found, creating test pattern");
        createTestImage();
        scanImages(); // Rescan to pick up the test image
    }
}


/**
 * Scan images in a specific folder
 */
void scanImagesInFolder(const char* folderPath, const char* folderName) {
    File dir = SD_MMC.open(folderPath);
    if (!dir || !dir.isDirectory()) {
        Logger.error("Failed to open directory: %s", folderPath);
        return;
    }
    
    File file = dir.openNextFile();
    while (file && imageCount < MAX_IMAGES) {
        if (!file.isDirectory()) {
            String filename = file.name();
            String lowerFilename = filename;
            lowerFilename.toLowerCase();
            
            if (lowerFilename.endsWith(".jpg") || lowerFilename.endsWith(".jpeg")) {
                // Extract just the filename without path
                String baseName = filename.substring(filename.lastIndexOf('/') + 1);
                
                // Check file size
                size_t fileSize = file.size();
                if (fileSize < 1024) {
                    Logger.warn("Skipping small file: %s (%d bytes)", filename.c_str(), fileSize);
                    file.close();
                    file = dir.openNextFile();
                    continue;
                }
                
                // Validate JPEG
                file.close();
                bool isValid = isValidJpeg(filename.c_str());
                file = dir.openNextFile(); // Reopen next file
                
                if (!isValid) {
                    Logger.warn("Skipping invalid JPEG: %s", filename.c_str());
                    continue;
                }
                
                // Store the image info
                strncpy(imageList[imageCount].path, filename.c_str(), MAX_FILEPATH_LENGTH);
                strncpy(imageList[imageCount].name, baseName.c_str(), MAX_FILENAME_LENGTH);
                strncpy(imageList[imageCount].folder, folderName, MAX_FILENAME_LENGTH);
                imageList[imageCount].size = fileSize;
                imageList[imageCount].isValid = true;
                imageList[imageCount].lastAccessed = 0;
                
                // Set initial width and height to 0, they'll be populated when image is first displayed
                imageList[imageCount].width = 0;
                imageList[imageCount].height = 0;
                
                Logger.debug("Found image %d: %s in folder %s", 
                          imageCount, imageList[imageCount].name, folderName);
                imageCount++;
            }
        }
        
        file.close();
        file = dir.openNextFile();
    }
    
    dir.close();
}


/**
 * Create a simple test pattern image
 */
bool createTestImage() {
    if (!sd_ready) return false;
    
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    
    bool success = false;
    
    try {
        // Make sure directory exists
        if (!SD_MMC.exists("/images")) {
            SD_MMC.mkdir("/images");
        }
        
        // Create a simple test file with color bars
        File testFile = SD_MMC.open("/images/test_pattern.jpg", FILE_WRITE);
        if (testFile) {
            // This is a minimal valid JPEG file (JFIF header)
            // Hard-coded color bar test pattern
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
                // Rest of JPEG data would follow
            };
            
            testFile.write(jpegHeader, sizeof(jpegHeader));
            
            // Generate some colorful test pattern data
            for (int i = 0; i < 5000; i++) {
                uint8_t colorData = (i % 256);
                testFile.write(&colorData, 1);
            }
            
            // Add JPEG footer (EOI marker)
            uint8_t jpegFooter[] = {0xFF, 0xD9};
            testFile.write(jpegFooter, sizeof(jpegFooter));
            
            testFile.close();
            Logger.info("Created test pattern image");
            success = true;
        }
    } catch (...) {
        Logger.error("Exception in createTestImage()");
    }
    
    xSemaphoreGive(sdMutex);
    return success;
}

/**
 * Display image (continued function from earlier)
 */
void displayImage(int index) {
    if (!sd_ready || index < 0 || index >= imageCount) {
        Logger.error("Cannot display image - invalid index %d or SD not ready", index);
        return;
    }
    
    // Prevent concurrent image loading
    static SemaphoreHandle_t displaySemaphore = NULL;
    if (displaySemaphore == NULL) {
        displaySemaphore = xSemaphoreCreateMutex();
    }
    
    // Try to take the display semaphore with timeout
    if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
        Logger.warn("Image loading already in progress - ignoring request");
        return;
    }
    
    // Feed watchdog before starting image display
    feedWatchdog();
    
    // Set flags
    imageLoading = true;
    
    // Store current backlight level for transition effects
    uint8_t originalBrightness = 255;
    
    // Update current image index
    currentImage = index;
    
    // Reset zoom and pan when changing images
    currentZoom = 1.0f;
    panOffsetX = 0;
    panOffsetY = 0;
    
    // Update UI elements
    updateImageCounter(index);
    
    // Apply transition effect
    switch (currentTransition) {
        case TRANSITION_FADE:
            // Dim backlight for fade effect
            for (int brightness = 255; brightness >= 50; brightness -= 10) {
                lcd.example_bsp_set_lcd_backlight(brightness);
                delay(10);
            }
            break;
            
        case TRANSITION_SLIDE:
        case TRANSITION_ZOOM:
            // These transitions are managed by LVGL and would be implemented there
            // Here we just dim the screen slightly
            lcd.example_bsp_set_lcd_backlight(150);
            break;
            
        default:
            // Simple dimming for no transition
            lcd.example_bsp_set_lcd_backlight(100);
            break;
    }
    
    // Log display attempt
    Logger.info("Displaying image %d: %s", index, imageList[index].path);
    
    // Clear screen
    clearLcdScreen();
    
    // Check if image is in cache
    size_t cachedSize = 0;
    uint8_t* cachedData = getCachedImage(imageList[index].path, &cachedSize);
    
    bool success = false;
    
    if (cachedData && cachedSize > 0) {
        Logger.info("Using cached image data");
        
        // Try to acquire JPEG mutex
        if (xSemaphoreTake(jpegMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Use cached data for decoding
            success = renderJpegFromMemory(cachedData, cachedSize);
            xSemaphoreGive(jpegMutex);
        } else {
            Logger.error("Failed to take JPEG mutex for cached display");
        }
    } else {
        // Not in cache, load from SD card
        success = renderJpegFile(imageList[index].path);
        
        // If successful, cache the image for future use
        if (success) {
            cacheImage(imageList[index].path);
        }
    }
    
    // Update the image's last accessed time
    imageList[index].lastAccessed = millis();
    
    // Apply transition effect for appearance
    switch (currentTransition) {
        case TRANSITION_FADE:
            // Fade in
            for (int brightness = 50; brightness <= 255; brightness += 10) {
                lcd.example_bsp_set_lcd_backlight(brightness);
                delay(10);
            }
            break;
            
        default:
            // Restore full brightness
            lcd.example_bsp_set_lcd_backlight(255);
            break;
    }
    
    // Update UI status
    updateStatusAfterImageLoad(index, success);
    
    // Done loading
    imageLoading = false;
    
    // Release the display semaphore
    xSemaphoreGive(displaySemaphore);
    
    // Feed watchdog after completion
    feedWatchdog();
}

/**
 * Render JPEG from SD card file
 */
bool renderJpegFile(const char* filename) {
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or invalid filename");
        return false;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    // Try to acquire JPEG mutex
    if (xSemaphoreTake(jpegMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Logger.error("Failed to take JPEG mutex for display");
        return false;
    }
    
    // Try to acquire SD mutex
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Logger.error("Failed to take SD mutex for JPEG loading");
        xSemaphoreGive(jpegMutex);
        return false;
    }
    
    File jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
        Logger.error("Failed to open JPEG file: %s", filename);
        xSemaphoreGive(sdMutex);
        xSemaphoreGive(jpegMutex);
        return false;
    }
    
    size_t fileSize = jpegFile.size();
    if (fileSize < 1024) {
        Logger.error("File too small: %s (%d bytes)", filename, fileSize);
        jpegFile.close();
        xSemaphoreGive(sdMutex);
        xSemaphoreGive(jpegMutex);
        return false;
    }
    
    Logger.info("Starting JPEG decoding for %s (%u bytes)", filename, fileSize);
    
    // Initialize flags for image orientation
    imageNeedsRotation = false; // Default to no rotation
    
    // Try decoding at full resolution first
    Logger.info("Decoding image at full resolution");
    
    // Store any JPEG decoding errors
    const char* errorMsg = nullptr;
    
    // Set up error recovery
    bool decodeFailed = false;
    
    // Try to decode the image with progressively lower quality if needed
    if (jpeg.open(jpegFile, jpegDrawCallback)) {
        // Get image dimensions to determine if rotation is needed
        int imgWidth = jpeg.getWidth();
        int imgHeight = jpeg.getHeight();
        
        if (imgWidth <= 0 || imgHeight <= 0) {
            Logger.error("Invalid image dimensions: %dx%d", imgWidth, imgHeight);
            jpeg.close();
            jpegFile.seek(0);
            decodeFailed = true;
        } else {
            // Store dimensions in image info for future reference
            for (int i = 0; i < imageCount; i++) {
                if (strcmp(imageList[i].path, filename) == 0) {
                    imageList[i].width = imgWidth;
                    imageList[i].height = imgHeight;
                    break;
                }
            }
            
            // Set rotation flag based on image orientation
            // For a portrait screen (800x1280), rotate if image is landscape
            imageNeedsRotation = (imgWidth > imgHeight);
            
            // Attempt full resolution decode
            if (jpeg.decode(0, 0, 0)) {
                if (imageNeedsRotation) {
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
        
        if (jpeg.open(jpegFile, jpegDrawCallback)) {
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
        
        if (jpeg.open(jpegFile, jpegDrawCallback)) {
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
    xSemaphoreGive(sdMutex);
    xSemaphoreGive(jpegMutex);
    
    // Log the time taken to decode
    uint32_t decodeTime = millis() - startTime;
    Logger.info("JPEG decoding took %lu ms, result: %s", 
               decodeTime, success ? "success" : "failed");
    
    // If decode failed, capture error details
    if (!success && errorMsg) {
        Logger.error("JPEG decode error: %s for file: %s", errorMsg, filename);
    }
    
    return success;
}


/**
 * Render JPEG from memory buffer (for cached images)
 */
bool renderJpegFromMemory(uint8_t* jpegData, size_t jpegSize) {
    if (!jpegData || jpegSize == 0) {
        Logger.error("Invalid JPEG data or size");
        return false;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    // Try decoding at full resolution first
    Logger.info("Decoding cached JPEG image (%u bytes)", jpegSize);
    
    if (jpeg.openRAM(jpegData, jpegSize, jpegDrawCallback)) {
        // Get image dimensions
        int imgWidth = jpeg.getWidth();
        int imgHeight = jpeg.getHeight();
        
        if (imgWidth <= 0 || imgHeight <= 0) {
            Logger.error("Invalid cached image dimensions: %dx%d", imgWidth, imgHeight);
            jpeg.close();
            return false;
        }
        
        // Set rotation flag based on image orientation
        imageNeedsRotation = (imgWidth > imgHeight);
        
        // Decode the image
        if (jpeg.decode(0, 0, 0)) {
            if (imageNeedsRotation) {
                Logger.info("Cached JPEG decoded successfully (rotated): %dx%d -> %dx%d",
                            imgWidth, imgHeight, imgHeight, imgWidth);
            } else {
                Logger.info("Cached JPEG decoded successfully: %dx%d", imgWidth, imgHeight);
            }
            success = true;
        } else {
            Logger.warn("Full resolution cached decode failed, trying half resolution");
            
            // Try with lower resolution
            jpeg.close();
            if (jpeg.openRAM(jpegData, jpegSize, jpegDrawCallback)) {
                if (jpeg.decode(0, 0, JPEG_SCALE_HALF)) {
                    Logger.info("Cached JPEG decoded successfully at half resolution");
                    success = true;
                }
            }
        }
    } else {
        Logger.error("Failed to open cached JPEG for decoding");
    }
    
    // Always close the JPEG object
    jpeg.close();
    
    // Log the time taken to decode
    uint32_t decodeTime = millis() - startTime;
    Logger.info("Cached JPEG decoding took %lu ms, result: %s", 
               decodeTime, success ? "success" : "failed");
    
    return success;
}

/**
 * Function to recover SD card if it's not working
 */
bool recoverSdCard() {
    Logger.info("Attempting to recover SD card...");
    
    // End any existing SD connections
    SD_MMC.end();
    delay(500);
    
    // Try to reinitialize with safe parameters
    if (SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING)) {
        if (validateSDCard()) {
            Logger.info("SD card recovered successfully");
            sd_ready = true;
            createRequiredDirectories();
            return true;
        }
    }
    
    Logger.error("Failed to recover SD card");
    sd_ready = false;
    return false;
}

/**
 * JPEG draw callback with enhanced image processing features
 */
int jpegDrawCallback(JPEGDRAW *pDraw) {
    if (!lcd_ready || !pDraw || !pDraw->pPixels) {
        // Safety check
        return 0;
    }
    
    // Static timing control to prevent LCD driver overload
    static uint32_t lastDrawTime = 0;
    uint32_t currentTime = millis();
    
    // Add small delay between draw operations to prevent LCD driver errors
    if (currentTime - lastDrawTime < 5) {
        delay(5);
    }
    
    // Verify draw dimensions are realistic
    if (pDraw->iWidth <= 0 || pDraw->iHeight <= 0 ||
        pDraw->iWidth > 1280 || pDraw->iHeight > 1280) {
        Logger.error("Invalid JPEG draw dimensions: %dx%d", pDraw->iWidth, pDraw->iHeight);
        return 0;
    }
    
    // Make local copies of key variables to prevent race conditions
    int imgWidth = jpeg.getWidth();
    int imgHeight = jpeg.getHeight();
    
    // Skip drawing if invalid dimensions
    if (imgWidth <= 0 || imgHeight <= 0 || imgWidth > 5000 || imgHeight > 5000) {
        Logger.error("Invalid JPEG dimensions: %dx%d", imgWidth, imgHeight);
        return 0;
    }
    
    // Local copy of flags to prevent race condition
    bool needsRotation = imageNeedsRotation;
    float zoom = currentZoom;
    int offsetX = panOffsetX;
    int offsetY = panOffsetY;
    
    // Process with effect if needed
    if (currentEffect != EFFECT_NONE) {
        uint16_t* pixels = (uint16_t*)pDraw->pPixels;
        for (int i = 0; i < pDraw->iWidth * pDraw->iHeight; i++) {
            applyImageEffect(&pixels[i], currentEffect);
        }
    }
    
    // Create a local failure flag
    bool drawFailed = false;
    
    // When not rotating, simpler direct drawing path
    if (!needsRotation) {
        // Calculate centering offsets
        int centerX = (lcd.width() - imgWidth * zoom) / 2;
        int centerY = (lcd.height() - imgHeight * zoom) / 2;
        
        // Apply user pan offsets
        centerX += offsetX;
        centerY += offsetY;
        
        // Calculate final drawing coordinates
        int x = pDraw->x * zoom + centerX;
        int y = pDraw->y * zoom + centerY;
        int width = pDraw->iWidth * zoom;
        int height = pDraw->iHeight * zoom;
        
        // Safety check - ensure drawing within reasonable boundaries
        if (x > -width && y > -height && 
            x < lcd.width() && y < lcd.height()) {
            
            // Try to draw with error handling
            try {
                // If zoom is close to 1.0, use direct drawing for better performance
                if (zoom > 0.95 && zoom < 1.05) {
                    lcd.lcd_draw_bitmap(x, y, 
                                       x + pDraw->iWidth, 
                                       y + pDraw->iHeight, 
                                       (uint16_t*)pDraw->pPixels);
                } else {
                    // Zoomed drawing using more complex method
                    // This would typically use an intermediate buffer and scaling algorithm
                    // For simplicity in this example, we'll just use the LCD driver's scaling
                    lcd.lcd_draw_bitmap(x, y, 
                                       x + width, 
                                       y + height, 
                                       (uint16_t*)pDraw->pPixels);
                }
            } catch (...) {
                Logger.error("Exception in LCD drawing");
                drawFailed = true;
            }
        }
    } else {
        // Handling rotation - more complex with additional memory allocation
        int rotatedWidth = imgHeight;
        int rotatedHeight = imgWidth;
        
        // Calculate centering with zoom for rotated image
        int centerX = (lcd.width() - rotatedWidth * zoom) / 2;
        int centerY = (lcd.height() - rotatedHeight * zoom) / 2;
        
        // Apply user pan offsets
        centerX += offsetX;
        centerY += offsetY;
        
        // Calculate position in rotated space
        int rotX = centerX + pDraw->y * zoom;
        int rotY = centerY + (imgWidth - pDraw->x - pDraw->iWidth) * zoom;
        
        // Calculate dimensions of rotated block
        int rotWidth = pDraw->iHeight;
        int rotHeight = pDraw->iWidth;
        
        // Apply zoom to dimensions
        int scaledRotWidth = rotWidth * zoom;
        int scaledRotHeight = rotHeight * zoom;
        
        // Safety check - verify rotation dimensions
        if (rotWidth <= 0 || rotHeight <= 0 || rotWidth * rotHeight > 100000) {
            Logger.error("Invalid rotation dimensions: %dx%d", rotWidth, rotHeight);
            return 0;
        }
        
        // Allocate rotation buffer - using heap to support larger blocks
        uint16_t* rotatedBlock = (uint16_t*)ps_malloc(rotWidth * rotHeight * sizeof(uint16_t));
        if (!rotatedBlock) {
            Logger.error("Failed to allocate rotation buffer");
            return 0;
        }
        
        // Rotate the block with safety bounds checking
        uint16_t* pixels = (uint16_t*)pDraw->pPixels;
        for (int y = 0; y < pDraw->iHeight; y++) {
            for (int x = 0; x < pDraw->iWidth; x++) {
                // For 90° rotation: new_x = y, new_y = width - 1 - x
                int newX = y;
                int newY = pDraw->iWidth - 1 - x;
                
                // Strict bounds checking
                if (newX >= 0 && newX < rotWidth && 
                    newY >= 0 && newY < rotHeight &&
                    y < pDraw->iHeight && x < pDraw->iWidth) {
                    
                    rotatedBlock[newY * rotWidth + newX] = pixels[y * pDraw->iWidth + x];
                }
            }
        }
        
        // Draw if coordinates are within reasonable boundaries
        if (rotX > -scaledRotWidth && rotY > -scaledRotHeight && 
            rotX < lcd.width() && rotY < lcd.height()) {
            
            // Try to draw with error handling
            try {
                lcd.lcd_draw_bitmap(rotX, rotY, 
                                  rotX + scaledRotWidth, 
                                  rotY + scaledRotHeight, 
                                  rotatedBlock);
            } catch (...) {
                Logger.error("Exception in rotated LCD drawing");
                drawFailed = true;
            }
        }
        
        // Clean up memory
        free(rotatedBlock);
    }
    
    // Update timing tracker
    lastDrawTime = currentTime;
    
    // Return 0 (failure) on drawing errors, otherwise 1 (success)
    return drawFailed ? 0 : 1;
}

/**
 * Apply image effect to pixel
 */
void applyImageEffect(uint16_t* pixel, int effect) {
    // Extract RGB565 components
    uint8_t r = (*pixel & 0xF800) >> 8;
    uint8_t g = (*pixel & 0x07E0) >> 3;
    uint8_t b = (*pixel & 0x001F) << 3;
    
    switch (effect) {
        case EFFECT_GRAYSCALE: {
            // Grayscale conversion (luminance method)
            uint8_t gray = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
            r = gray;
            g = gray;
            b = gray;
            break;
        }
        
        case EFFECT_NEGATIVE: {
            // Negative/invert
            r = 255 - r;
            g = 255 - g;
            b = 255 - b;
            break;
        }
        
        case EFFECT_SEPIA: {
            // Sepia tone
            uint8_t outR = min(255, (int)(r * 0.393f + g * 0.769f + b * 0.189f));
            uint8_t outG = min(255, (int)(r * 0.349f + g * 0.686f + b * 0.168f));
            uint8_t outB = min(255, (int)(r * 0.272f + g * 0.534f + b * 0.131f));
            r = outR;
            g = outG;
            b = outB;
            break;
        }
        
        default:
            // No effect
            return;
    }
    
    // Convert back to RGB565
    *pixel = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

/**
 * Clear LCD screen with improved method
 */
void clearLcdScreen() {
    if (!lcd_ready) return;
    
    // Track timing
    uint32_t startTime = millis();
    
    // Set LCD backlight to dim during transition
    lcd.example_bsp_set_lcd_backlight(50);
    
    // Create a black buffer using PSRAM if available
    uint32_t bufferWidth = lcd.width();
    uint32_t bufferHeight = 20; // Process 20 rows at a time
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
            static uint16_t stackBuffer[400]; // Static buffer as last resort
            blackBuffer = stackBuffer;
            bufferSize = 400;
            bufferHeight = bufferSize / bufferWidth;
            if (bufferHeight == 0) bufferHeight = 1;
        }
    }
    
    // Fill buffer with black (0x0000)
    for (size_t i = 0; i < bufferSize; i++) {
        blackBuffer[i] = 0x0000;
    }
    
    // Clear screen in strips with proper error handling
    for (uint32_t y = 0; y < lcd.height(); y += bufferHeight) {
        uint32_t stripHeight = ((y + bufferHeight) > lcd.height()) ? 
                            (lcd.height() - y) : bufferHeight;
        
        // Safety check
        if (stripHeight == 0) continue;
        
        try {
            lcd.lcd_draw_bitmap(0, y, bufferWidth, y + stripHeight, blackBuffer);
        } catch (...) {
            Logger.error("Exception during screen clearing at y=%lu", y);
        }
        
        // Critical: Wait for LCD operation to complete before next strip
        delay(2);
        
        // Feed watchdog periodically
        if (y % 100 == 0) {
            feedWatchdog();
        }
    }
    
    // Free buffer if allocated from heap
    if (blackBuffer != nullptr && 
        blackBuffer != (uint16_t*)0x3FFC0000 && // PSRAM address check
        bufferSize > 400) { // Not using stack buffer
        free(blackBuffer);
    }
    
    // Add final delay to ensure all LCD operations are complete
    delay(50);
    
    Logger.debug("Screen cleared in %lu ms", millis() - startTime);
}

/**
 * Handle image zoom operations
 */
void handleImageZoom(float zoomFactor) {
    // Constrain zoom between 0.5x and 4x
    zoomFactor = max(0.5f, min(4.0f, zoomFactor));
    
    // If zoom hasn't changed significantly, do nothing
    if (abs(zoomFactor - currentZoom) < 0.05f) return;
    
    Logger.info("Zooming image: %.2f -> %.2f", currentZoom, zoomFactor);
    
    // Update zoom level
    currentZoom = zoomFactor;
    
    // Redisplay current image with new zoom
    if (imageCount > 0) {
        displayImage(currentImage);
    }
}

/**
 * Handle image pan operations
 */
void handleImagePan(int deltaX, int deltaY) {
    // Only allow panning when zoomed in
    if (currentZoom <= 1.0f) {
        panOffsetX = 0;
        panOffsetY = 0;
        return;
    }
    
    // Update pan offsets
    panOffsetX += deltaX;
    panOffsetY += deltaY;
    
    // Constrain pan limits based on zoom factor
    int maxOffset = (currentZoom - 1.0f) * 400; // Approximate constraint
    panOffsetX = constrain(panOffsetX, -maxOffset, maxOffset);
    panOffsetY = constrain(panOffsetY, -maxOffset, maxOffset);
    
    // Redisplay current image with new pan
    if (imageCount > 0) {
        displayImage(currentImage);
    }
}

/**
 * Update image counter display
 */
void updateImageCounter(int index) {
    if (!ui_ready || imageCounter == nullptr) return;
    
    if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char counterText[32];
        snprintf(counterText, sizeof(counterText), "Image: %d/%d", index + 1, imageCount);
        lv_label_set_text(imageCounter, counterText);
        xSemaphoreGive(uiMutex);
    }
}

/**
 * Update status after loading an image
 */
void updateStatusAfterImageLoad(int index, bool success) {
    if (success) {
        Logger.info("Successfully displayed image %d", index);
    } else {
        Logger.error("Failed to display image %d", index);
    }
    
    // Update UI elements
    if (ui_ready && statusLabel != nullptr) {
        if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!success) {
                lv_label_set_text(statusLabel, "Load Error");
            } else if (slideshowActive) {
                lv_label_set_text(statusLabel, "Slideshow Active");
            } else {
                // Display image info if successful
                if (index >= 0 && index < imageCount) {
                    char statusText[64];
                    ImageInfo *img = &imageList[index];
                    if (img->width > 0 && img->height > 0) {
                        snprintf(statusText, sizeof(statusText), 
                                "%s (%dx%d)", img->name, img->width, img->height);
                    } else {
                        snprintf(statusText, sizeof(statusText), "%s", img->name);
                    }
                    lv_label_set_text(statusLabel, statusText);
                } else {
                    lv_label_set_text(statusLabel, "ESP32P4 Gallery");
                }
            }
            xSemaphoreGive(uiMutex);
        }
    }
}

/**
 * Start slideshow with the specified interval
 */
void startSlideshow(uint32_t interval_ms) {
    if (interval_ms < 2000) interval_ms = 2000; // Minimum 2 seconds
    
    Logger.info("Starting slideshow with %d ms interval", interval_ms);
    
    // Set slideshow interval
    slideshowInterval = interval_ms;
    
    // Enable slideshow
    slideshowActive = true;
    
    // Create slideshow task if needed and not already running
    if (slideshowTaskHandle == NULL) {
        xTaskCreatePinnedToCore(
            slideshowTask,         // Function
            "Slideshow",           // Name
            4096,                  // Stack size
            NULL,                  // Parameter
            1,                     // Priority (1 is low)
            &slideshowTaskHandle,  // Handle
            0                      // Core ID (0)
        );
    }
    
    // Update UI to show slideshow is active
    if (ui_ready && statusLabel != nullptr) {
        if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_label_set_text(statusLabel, "Slideshow Active");
            xSemaphoreGive(uiMutex);
        }
    }
}

/**
 * Stop slideshow
 */
void stopSlideshow() {
    Logger.info("Stopping slideshow");
    
    // Disable slideshow
    slideshowActive = false;
    
    // Update UI to show slideshow is stopped
    if (ui_ready && statusLabel != nullptr) {
        if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_label_set_text(statusLabel, "ESP32P4 Gallery");
            xSemaphoreGive(uiMutex);
        }
    }
}

/**
 * Slideshow task function
 */
void slideshowTask(void *parameter) {
    Logger.info("Slideshow task started");
    
    // Initialize variables
    uint32_t lastImageChange = 0;
    
    while (true) {
        uint32_t now = millis();
        
        // Process slideshow logic only if active
        if (slideshowActive && imageCount > 0 && now - lastImageChange >= slideshowInterval) {
            Logger.debug("Slideshow advancing to next image");
            
            // Calculate next image index with wrap-around
            int nextImage = (currentImage + 1) % imageCount;
            
            // Try to safely display the next image
            try {
                // Get SD mutex with timeout to avoid deadlocks
                if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    // Display next image
                    displayImage(nextImage);
                    
                    // Release SD mutex
                    xSemaphoreGive(sdMutex);
                    
                    Logger.debug("Slideshow advanced to image %d", nextImage);
                } else {
                    Logger.warn("Slideshow couldn't acquire SD mutex, skipping cycle");
                }
            } catch (...) {
                Logger.error("Exception in slideshow task - continuing");
                xSemaphoreGive(sdMutex); // Make sure to release mutex in case of exception
            }
            
            // Update last change time
            lastImageChange = now;
        }
        
        // Feed watchdog
        feedWatchdog();  // Fixed: Changed from feed_watchdog to feedWatchdog
        
        // Delay to prevent CPU hogging
        delay(100);
    }
}


/**
 * LVGL task function for UI handling
 */
void lvglTask(void *pvParameters) {
    Logger.info("LVGL task started");
    
    while (true) {
        // Protect with mutex
        if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lv_timer_handler(); // Handle LVGL tasks
            xSemaphoreGive(uiMutex);
        }
        
        // Update UI periodically
        uint32_t now = millis();
        if (now - lastUiUpdate > 1000) { // Once per second
            if (xSemaphoreTake(uiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update WiFi status
                if (wifiLabel != nullptr) {
                    if (wifi_ready) {
                        int clients = WiFi.softAPgetStationNum();
                        if (clients > 0) {
                            char wifiText[32];
                            snprintf(wifiText, sizeof(wifiText), "%s %d", LV_SYMBOL_WIFI, clients);
                            lv_label_set_text(wifiLabel, wifiText);  // Fixed: Use char array instead of String
                        } else {
                            lv_label_set_text(wifiLabel, LV_SYMBOL_WIFI);
                        }
                    } else {
                        lv_label_set_text(wifiLabel, LV_SYMBOL_CLOSE);
                    }
                }
                xSemaphoreGive(uiMutex);
            }
            lastUiUpdate = now;
        }
        
        // Feed watchdog
        feedWatchdog();  // Fixed: Changed from feed_watchdog to feedWatchdog
        
        // Run at ~100Hz
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/**
 * Network task for handling WiFi and webserver
 */
void networkTask(void *pvParameters) {
    Logger.info("Network task started");
    
    while (true) {
        // Process DNS server
        if (dnsServerActive) {
            try {
                dnsServer.processNextRequest();
            } catch (...) {
                Logger.warn("Exception in DNS server processing");
            }
        }
        
        // Handle web server clients
        if (server_ready) {
            try {
                server.handleClient();
            } catch (...) {
                Logger.warn("Exception in webserver client handling");
            }
        }
        
        // Periodically check WiFi status
        uint32_t now = millis();
        static uint32_t lastWifiCheck = 0;
        
        if (now - lastWifiCheck > 10000) { // Every 10 seconds
            if (wifi_ready) {
                if (WiFi.getMode() == WIFI_AP_STA && WiFi.status() != WL_CONNECTED) {
                    // STA connection lost, try to reconnect
                    if (WIFI_STA_ENABLED) {
                        WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
                    }
                }
            }
            lastWifiCheck = now;
        }
        
        // Feed watchdog
        feedWatchdog();  // Fixed: Changed from feed_watchdog to feedWatchdog
        
        // Short delay
        delay(5);
    }
}


/**
 * Background task for system monitoring and maintenance
 */
void backgroundTask(void *pvParameters) {
    Logger.info("Background task started");
    
    while (true) {
        uint32_t now = millis();
        
        // Perform system health check periodically
        static uint32_t lastHealthCheck = 0;
        if (now - lastHealthCheck > 30000) { // Every 30 seconds
            performSystemHealthCheck();
            lastHealthCheck = now;
        }
        
        // Format SD card if requested
        if (formatSdRequested) {
            Logger.info("SD card format requested, performing format...");
            formatSdRequested = false;
            
            // Take SD mutex
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                // Close all open files and end the SD card session
                SD_MMC.end();
                delay(1000);
                
                // Try to initialize with format
                if (SD_MMC.begin("/sdcard", true, true)) {
                    Logger.info("SD card formatted successfully");
                    sd_ready = true;
                    
                    // Create required directories
                    createRequiredDirectories();
                    
                    // Scan for images
                    scanImages();
                } else {
                    Logger.error("SD card format failed");
                    sd_ready = false;
                }
                
                xSemaphoreGive(sdMutex);
            } else {
                Logger.error("Failed to acquire SD mutex for formatting");
            }
        }
        
        // Handle system reset if requested
        if (resetRequested) {
            Logger.info("System reset requested");
            delay(500);
            ESP.restart();
        }
        
        // Feed watchdog
        feedWatchdog();  // Fixed: Changed from feed_watchdog to feedWatchdog
        
        // Delay to prevent CPU hogging
        delay(1000);
    }
}


/**
 * Perform system health check
 */
void performSystemHealthCheck() {
    Logger.debug("Performing system health check");
    
    // Check SD card health
    if (!sd_ready) {
        Logger.warn("SD card not ready, attempting recovery");
        recoverSdCard();
    }
    
    // Check WiFi status
    if (!wifi_ready) {
        Logger.warn("WiFi not ready, attempting restart");
        setupWiFi();
    }
    
    // Check for low memory
    if (ESP.getFreeHeap() < 20000) {
        Logger.warn("Low memory detected: %d bytes free", ESP.getFreeHeap());
        // Consider clearing caches or other cleanup
    }
    
    // Report system stats
    Logger.info("System status - Heap: %u KB, PSRAM: %u KB, Images: %d, Uptime: %lu sec",
              ESP.getFreeHeap() / 1024,
              ESP.getFreePsram() / 1024,
              imageCount,
              millis() / 1000);
}

/**
 * Handle touch screen events
 */
void handleTouchEvent(uint16_t x, uint16_t y) {
    static uint32_t lastTouchTime = 0;
    static uint16_t lastTouchX = 0;
    static uint16_t lastTouchY = 0;
    static bool isPanning = false;
    
    uint32_t now = millis();
    
    // Enforce minimum time between touch events (debounce)
    if (now - lastTouchTime < 100) {
        return;
    }
    
    // Screen dimensions
    int screenHeight = lcd.height();
    int screenWidth = lcd.width();
    
    // Calculate delta from last touch
    int deltaX = x - lastTouchX;
    int deltaY = y - lastTouchY;
    
    // Store current touch information
    lastTouchTime = now;
    lastTouchX = x;
    lastTouchY = y;
    
    // Skip handling if UI has its own touch processing (through LVGL)
    if (ui_ready) {
        // Let LVGL handle the touch through its own input system
        return;
    }
    
    Logger.debug("Touch at x=%d, y=%d (screen: %dx%d)", x, y, screenWidth, screenHeight);
    
    // Check for pan gesture (when zoomed in)
    if (currentZoom > 1.0f && abs(deltaX) > 10 || abs(deltaY) > 10) {
        if (isPanning) {
            handleImagePan(deltaX, deltaY);
        }
        isPanning = true;
        return;
    } else {
        isPanning = false;
    }
    
    // Check touch zones
    if (y < screenHeight / 10) {
        // Top bar - toggle slideshow
        if (slideshowActive) {
            stopSlideshow();
        } else {
            startSlideshow(5000);
        }
    }
    else if (x < screenWidth / 3) {
        // Left third - previous image
        if (imageCount > 0) {
            int prevImage = (currentImage > 0) ? (currentImage - 1) : (imageCount - 1);
            displayImage(prevImage);
        }
    }
    else if (x > (screenWidth * 2) / 3) {
        // Right third - next image
        if (imageCount > 0) {
            int nextImage = (currentImage < imageCount - 1) ? (currentImage + 1) : 0;
            displayImage(nextImage);
        }
    }
    else {
        // Middle area - toggle zoom
        if (currentZoom > 1.0f) {
            // If already zoomed, reset to normal
            handleImageZoom(1.0f);
        } else {
            // Zoom in
            handleImageZoom(2.0f);
        }
    }
}

/**
 * Main setup function
 */
void setup() {
    // Initialize serial communication immediately
    Serial.begin(115200);
    delay(500);  // Give serial time to initialize
    
    // Initialize logger
    safeLoggerInit();
    
    Serial.println();
    Serial.println("=== ESP32P4 Advanced Gallery v3.0 ===");
    Serial.println("Starting initialization sequence...");
    Serial.println("Date: 2025-06-24 00:17:52 UTC");
    Serial.println("User: Chamil1983");
    Serial.flush();
    
    // Create mutexes
    sdMutex = xSemaphoreCreateMutex();
    jpegMutex = xSemaphoreCreateMutex();
    uiMutex = xSemaphoreCreateMutex();
    wifiMutex = xSemaphoreCreateMutex();
    
    // Setup watchdog
    setupWatchdog();
    
    // Reset ESP-IDF log levels to minimize spam
    esp_log_level_set("*", ESP_LOG_NONE);
    
    // Initialize components in sequence with proper error handling
    bool fs_ok = safeExecuteWithWatchdog([]() -> bool { return initializeFileSystem(); }, "file system");
    bool lcd_ok = safeExecuteWithWatchdog([]() -> bool { return setupLcd(); }, "LCD");
    bool touch_ok = safeExecuteWithWatchdog([]() -> bool { return setupTouch(); }, "touch");
    bool sd_ok = safeExecuteWithWatchdog([]() -> bool { return setupSdCardReliable(); }, "SD card");
    bool wifi_ok = safeExecuteWithWatchdog([]() -> bool { return setupWiFi(); }, "WiFi");
    bool lvgl_ok = safeExecuteWithWatchdog([]() -> bool { return setupLvgl(); }, "LVGL");
    bool ui_ok = safeExecuteWithWatchdog([]() -> bool { return setupUI(); }, "UI");
    bool cache_ok = safeExecuteWithWatchdog([]() -> bool { return setupCache(); }, "cache");
    bool web_ok = safeExecuteWithWatchdog([]() -> bool { return setupWebserver(); }, "webserver");
    
    // Scan images if SD card is working
    if (sd_ready) {
        Logger.info("Scanning for images...");
        scanImages();
        
        if (imageCount > 0) {
            Logger.info("Loading first image");
            displayImage(0);
        }
    }
    
    // Create tasks
    xTaskCreatePinnedToCore(
        lvglTask,              // Function
        "LVGL Task",           // Name
        8192,                  // Stack size
        NULL,                  // Parameters
        2,                     // Priority (higher than others)
        &lvglTaskHandle,       // Handle
        1                      // Core ID (UI on core 1)
    );
    
    xTaskCreatePinnedToCore(
        networkTask,           // Function
        "Network Task",        // Name
        8192,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority
        &networkTaskHandle,    // Handle
        0                      // Core ID (Network on core 0)
    );
    
    xTaskCreatePinnedToCore(
        backgroundTask,        // Function
        "Background Task",     // Name
        4096,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority
        &backgroundTaskHandle, // Handle
        0                      // Core ID (Background on core 0)
    );
    
    // Log initialization results
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
 * Main loop function
 */
void loop() {
    // Most functionality is handled in tasks
    // Main loop is kept minimal to ensure responsiveness
    
    // Check touch input
    if (touch_ready) {
        uint16_t x, y;
        if (touch.getTouch(&x, &y)) {
            // Process touch event
            handleTouchEvent(x, y);
            lastTouchEvent = millis();
        }
    }
    
    // Feed watchdog
    feedWatchdog();  // Fixed: Changed from feed_watchdog to feedWatchdog
    
    // Short delay to prevent CPU hogging
    delay(10);
}
// End of code
#pragma GCC pop_options