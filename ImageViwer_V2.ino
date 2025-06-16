#pragma GCC push_options
#pragma GCC optimize("O2")  // Using O2 for better stability

// Include required libraries
#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <FS.h>
#include <SPIFFS.h>
#include "JPEGDEC.h"  // Using the specified library
#include <esp_task_wdt.h>
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

// Forward declarations
bool setup_lcd();
bool setup_touch();
bool setup_sd_card_reliable();
void process_pending_operations();
bool setup_watchdog();
void updateImageCounter(int index);
bool initializeSpiffs();
bool emergency_sd_recovery();
void checkPartitions();
void feed_watchdog();
bool setup_wifi_ap();
bool setup_lvgl();
bool setup_ui();
bool setup_webserver();
bool validateSDCard();
void update_system_info();
void updateStatusAfterImageLoad(int index, bool success);
void scan_images();
bool recover_sd_card_for_upload();
void display_image(int index);
void handle_upload();
bool recover_sd_card();
void diagnose_sd_card();
bool render_jpeg_file(const char *filename);
void lvgl_task(void *pvParameters);
int jpeg_draw_callback(JPEGDRAW *pDraw);
void clearLCDScreen();
void slideshowTask(void *parameter);
void startSlideshow(uint32_t interval_ms = 5000);
void stopSlideshow();
void toggleSlideshow(uint32_t interval_ms = 5000);
void handle_touch_event(uint16_t x, uint16_t y);
void check_buttons();
uint32_t max_u32(uint32_t a, uint32_t b);
bool createTestImage();
bool check_image_needs_rotation(int img_width, int img_height);
bool isValidJPEG(const char* filename);
bool emergency_spiffs_recovery();
void displayErrorMessage(const char* message);

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
 * Setup Function
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println();
  Serial.println("=== ESP32P4 Gallery Debug Output ===");
  
    // Try to initialize SPIFFS
    bool spiffsAvailable = initializeSpiffs();
    
    // Initialize the logger with appropriate settings
    // Only enable flash logging if SPIFFS is available
    Logger.init(true, spiffsAvailable, LOG_LEVEL_DEBUG);
    
    // Force output level for better debugging
    Logger.setLogLevel(LOG_LEVEL_DEBUG);
    Logger.enableSerialOutput(true);
    
    // Log SPIFFS status
    if (spiffsAvailable) {
        Logger.info("SPIFFS is available for logging");
    } else {
        Logger.warn("SPIFFS is unavailable - logging to serial only");
    }
  
  // Check flash partitions
  checkPartitions();
  
  // Log startup information
  Logger.info("====================================");
  Logger.info("ESP32P4 Image Gallery Starting");
  Logger.info("Date: 2025-06-15 05:30:35 UTC");
  Logger.info("User: Chamil1983");
  Logger.info("====================================");
  
  // Initialize mutexes
  DEBUG_INIT("Mutexes");
  sd_mutex = xSemaphoreCreateMutex();
  jpeg_mutex = xSemaphoreCreateMutex();
  ui_mutex = xSemaphoreCreateMutex();
  DEBUG_SUCCESS("Mutexes");
  
  // Setup watchdog
  setup_watchdog();
  
  Logger.info("Starting component initialization sequence:");
  
  // 1. Initialize LCD
  DEBUG_INIT("LCD");
  if (setup_lcd()) {
    DEBUG_SUCCESS("LCD");
    Logger.info("LCD ready");
  } else {
    DEBUG_FAIL("LCD", "Failed to initialize");
    // Continue anyway, as some functions might still work
  }
  
  // 2. Initialize touch
  DEBUG_INIT("Touch");
  if (setup_touch()) {
    DEBUG_SUCCESS("Touch");
    Logger.info("Touch ready");
  } else {
    DEBUG_FAIL("Touch", "Failed to initialize");
    // Continue anyway
  }
  
  // 3. Initialize SD card with robust approach
  DEBUG_INIT("SD Card");
  if (setup_sd_card_reliable()) {
    DEBUG_SUCCESS("SD Card");
    Logger.info("SD ready");
  } else {
    DEBUG_FAIL("SD Card", "Failed to initialize");
    // Continue anyway
  }
  
  // Log watchdog status
  Logger.info("Task watchdog panic is enabled for crash detection");
  Logger.info("System configured to handle crashes via watchdog");
  
  // 4. Initialize WiFi in AP mode
  DEBUG_INIT("WiFi");
  if (setup_wifi_ap()) {
    DEBUG_SUCCESS("WiFi");
    Logger.info("WiFi ready");
  } else {
    DEBUG_FAIL("WiFi", "Failed to initialize");
    // Continue anyway
  }
  
  // 5. Initialize LVGL
  DEBUG_INIT("LVGL");
  if (setup_lvgl()) {
    DEBUG_SUCCESS("LVGL");
    Logger.info("LVGL ready");
  } else {
    DEBUG_FAIL("LVGL", "Failed to initialize");
    // Critical failure, but continue with limited functionality
  }
  
  // 6. Initialize UI
  DEBUG_INIT("UI");
  if (setup_ui()) {
    DEBUG_SUCCESS("UI");
    Logger.info("UI ready");
  } else {
    DEBUG_FAIL("UI", "Failed to initialize");
    // Critical failure, but continue with limited functionality
  }
  
  // 7. Initialize web server
  DEBUG_INIT("WebServer");
  if (setup_webserver()) {
    DEBUG_SUCCESS("WebServer");
    Logger.info("WebServer ready");
  } else {
    DEBUG_FAIL("WebServer", "Failed to initialize");
    // Continue anyway
  }
  
  // 8. Start LVGL task on Core 1
  xTaskCreatePinnedToCore(
    lvgl_task,           // Function
    "LVGL Task",         // Name
    8192,                // Stack size
    NULL,                // Parameters
    1,                   // Priority (1 is low)
    &lvgl_task_handle,   // Task handle
    1                    // Core ID (Core 1)
  );
  Logger.info("LVGL task started on Core 1");
  
  // 9. Scan for images on SD card
  Logger.info("Scanning for images...");
  scan_images();
  Logger.info("Found %d images", image_count);
  Serial.printf("Found %d images\n", image_count);
  
  // 10. Load first image if available
  if (image_count > 0) {
    Logger.info("Loading first image");
    display_image(0);
  }
  
  // Create slideshow task on core 0
  xTaskCreatePinnedToCore(
    slideshowTask,         // Function
    "Slideshow Task",      // Name
    4096,                  // Stack size
    NULL,                  // Parameters
    1,                     // Priority
    &slideshow_task_handle,// Task handle
    0                      // Core ID (Core 0)
  );

  // Register slideshow task with watchdog
if (slideshow_task_handle != NULL) {
  esp_task_wdt_add(slideshow_task_handle);
  Logger.info("Slideshow task added to watchdog");
}
  
  Logger.info("Initialization complete!");
  Serial.printf("STATUS: LCD:%s Touch:%s SD:%s WiFi:%s LVGL:%s UI:%s Server:%s\n", 
                lcd_ready ? "OK" : "FAIL",
                touch_ready ? "OK" : "FAIL",
                sd_ready ? "OK" : "FAIL",
                wifi_ready ? "OK" : "FAIL",
                lvgl_ready ? "OK" : "FAIL",
                ui_ready ? "OK" : "FAIL",
                server_ready ? "OK" : "FAIL");
  
  // Update system information
  update_system_info();
}

/**
 * Main loop
 */
// Fixed loop function touch handling
void loop() {
  // Process any pending operations
  process_pending_operations();
  
  // Feed watchdog
  if (wdt_enabled) {
    feed_watchdog();
  }
  
  // Handle WebServer
  if (server_ready) {
    server.handleClient();
  }
  
  // Check for reset request
  if (reset_requested) {
    Logger.info("Reset requested. Restarting system...");
    delay(1000);
    ESP.restart();
  }
  
  // Check touch events if touch is ready
  if (touch_ready) {
    uint16_t touch_x, touch_y;
    
    // Use getTouch instead of read_touch
    if (touch.getTouch(&touch_x, &touch_y)) {
      // Handle the touch event
      handle_touch_event(touch_x, touch_y);
    }
  }
  
  // Update system info periodically
  uint32_t now = millis();
  if (now - last_system_update > 5000) {
    update_system_info();
    last_system_update = now;
  }
  
  // Small delay to avoid CPU hogging
  delay(10);
}
/**
 * Handle touch events
 */
// Handle touch events with slideshow control
void handle_touch_event(uint16_t x, uint16_t y) {
  // Get screen dimensions
  uint16_t screen_height = lcd.height();
  uint16_t screen_width = lcd.width();
  
  // Define touch zones (as percentages of screen)
  const int topZoneHeight = screen_height / 10;      // Top 10% of screen
  const int bottomZoneHeight = screen_height / 10;   // Bottom 10% of screen
  
  // Debug touch coordinates for troubleshooting
  static uint32_t lastLogTime = 0;
  if (millis() - lastLogTime > 1000) { // Limit log frequency
    Logger.debug("Touch at x=%d, y=%d (screen: %dx%d)", x, y, screen_width, screen_height);
    lastLogTime = millis();
  }
  
  // Top area - toggles slideshow
  if (y < topZoneHeight) {
    Logger.info("Touch detected in top zone - toggling slideshow");
    toggleSlideshow(8000); // 8-second default interval
    return;
  }
  
  // Bottom area - navigation controls
  if (y > screen_height - bottomZoneHeight) {
    // Left side - previous image
    if (x < screen_width / 2) {
      Logger.info("Touch detected in bottom-left zone - previous image");
      stopSlideshow(); // Stop slideshow when manually navigating
      
      if (current_image > 0) {
        display_image(current_image - 1);
      } else if (image_count > 0) {
        display_image(image_count - 1); // Wrap around to last image
      }
    } 
    // Right side - next image
    else {
      Logger.info("Touch detected in bottom-right zone - next image");
      stopSlideshow(); // Stop slideshow when manually navigating
      
      if (current_image < image_count - 1) {
        display_image(current_image + 1);
      } else if (image_count > 0) {
        display_image(0); // Wrap around to first image
      }
    }
    return;
  }
  
  // Middle area - toggle slideshow with different interval (longer in middle)
  Logger.info("Touch detected in middle zone - toggling slideshow with longer interval");
  toggleSlideshow(10000); // 10-second interval in middle zone
}

/**
 * Enhanced slideshow task with better error handling
 * Updated: 2025-06-16 10:32:38
 * User: Chamil1983
 */
void slideshowTask(void *parameter) {
    // Initialize variables
    uint32_t lastChangeTime = millis();
    bool firstRun = true;
    int consecutiveErrors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 3; // Stop slideshow if too many errors
    
    // Use a lower priority to prevent competing with critical tasks
    vTaskPrioritySet(NULL, 1);
    
    Logger.info("Slideshow task started");
    
    // Try to add task to watchdog
    esp_err_t err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        Logger.warn("Failed to add slideshow task to watchdog: %d", err);
    }
    
    while (true) {
        // Always reset watchdog first
        esp_task_wdt_reset();
        
        // First run initialization
        if (firstRun) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            lastChangeTime = millis();
            firstRun = false;
            Logger.info("Slideshow task initialized");
        }
        
        // Only process if slideshow is active and we have images
        if (slideshow_active && image_count > 1) {
            uint32_t currentTime = millis();
            
            // Check if it's time to change images
            if (currentTime - lastChangeTime >= slideshow_interval) {
                Logger.info("Slideshow timer triggered after %lu ms", 
                           currentTime - lastChangeTime);
                
                // Don't try to change image if one is already loading
                if (!image_loading) {
                    // Calculate next image with bounds checking
                    int nextImage = (current_image + 1) % image_count;
                    int startImage = nextImage; // Remember where we started
                    bool foundValid = false;
                    int skippedImages = 0;
                    
                    // Try to find a valid image, but limit the number of attempts
                    do {
                        // Make a local copy of the filename
                        char filename[100] = {0};
                        if (image_list[nextImage]) {
                            strncpy(filename, image_list[nextImage], sizeof(filename) - 1);
                        }
                        
                        // Pre-validate the image file
                        if (SD_MMC.exists(filename) && isValidJPEG(filename)) {
                            foundValid = true;
                            break;
                        } else {
                            Logger.warn("Slideshow skipping invalid image %d: %s", 
                                       nextImage, filename);
                            
                            // Move to next image
                            nextImage = (nextImage + 1) % image_count;
                            skippedImages++;
                            
                            // Avoid infinite loop if all images are invalid
                            if (skippedImages >= image_count) {
                                Logger.error("All images appear to be invalid, pausing slideshow");
                                foundValid = false;
                                break;
                            }
                        }
                    } while (nextImage != startImage);
                    
                    bool displaySuccess = false;
                    
                    if (foundValid) {
                        Logger.info("Slideshow advancing to image %d", nextImage);
                        
                        // Now safely display the image
                        try {
                            // Track previous image for error handling
                            int previousImage = current_image;
                            
                            // Display the new image
                            display_image(nextImage);
                            
                            // Check if display was successful by comparing current_image
                            if (current_image == nextImage) {
                                displaySuccess = true;
                                consecutiveErrors = 0; // Reset error counter
                            } else {
                                // Image display failed
                                displaySuccess = false;
                                consecutiveErrors++;
                            }
                        } catch (...) {
                            Logger.error("Exception during slideshow image display");
                            displaySuccess = false;
                            consecutiveErrors++;
                        }
                        
                        if (displaySuccess) {
                            Logger.info("Slideshow advanced to image %d successfully", nextImage);
                        } else {
                            Logger.error("Slideshow failed to advance to image %d", nextImage);
                        }
                    } else {
                        Logger.error("No valid images found for slideshow");
                        consecutiveErrors++;
                    }
                    
                    // If we've had too many consecutive errors, pause slideshow
                    if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                        Logger.error("Too many consecutive errors (%d), pausing slideshow", 
                                    consecutiveErrors);
                        slideshow_active = false;
                        
                        // Update UI to show slideshow is paused
                        if (ui_ready && status_label != nullptr) {
                            if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                lv_label_set_text(status_label, "Slideshow: PAUSED (errors)");
                                xSemaphoreGive(ui_mutex);
                            }
                        }
                        
                        // Reset error counter
                        consecutiveErrors = 0;
                    }
                } else {
                    Logger.warn("Slideshow skipping cycle - image loading in progress");
                }
                
                // Update timestamp regardless of outcome
                lastChangeTime = currentTime;
            }
        } else {
            // Reset timer if slideshow is inactive
            lastChangeTime = millis();
            // Also reset error counter when inactive
            consecutiveErrors = 0;
        }
        
        // Use consistent delay between checks
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * Improved slideshow start function with proper task management
 */
void startSlideshow(uint32_t interval_ms) {
    Logger.info("Attempting to start slideshow");
    
    if (image_count <= 1) {
        Logger.warn("Slideshow not started - need at least 2 images (found %d)", image_count);
        return;
    }
    
    // Use reasonable interval limits
    slideshow_interval = max(interval_ms, (uint32_t)5000);  // Minimum 5 seconds (safer)
    slideshow_interval = min(slideshow_interval, (uint32_t)30000); // Maximum 30 seconds
    
    // Create slideshow task if it doesn't exist
    if (slideshow_task_handle == NULL) {
        Logger.info("Creating slideshow task");
        
        // Create task with proper stack size and low priority
        BaseType_t result = xTaskCreatePinnedToCore(
            slideshowTask,         // Function
            "Slideshow",           // Name
            4096,                  // Stack size
            NULL,                  // Parameters
            1,                     // Priority (low)
            &slideshow_task_handle,// Task handle
            0                      // Run on Core 0
        );
        
        if (result != pdPASS || slideshow_task_handle == NULL) {
            Logger.error("Failed to create slideshow task");
            return;
        }
    }
    
    // Set flag to activate slideshow
    slideshow_active = true;
    
    Logger.info("Slideshow started with %lu ms interval", slideshow_interval);
    
    // Update UI
    if (ui_ready && status_label != nullptr) {
        if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_label_set_text(status_label, "Slideshow: ON");
            xSemaphoreGive(ui_mutex);
        }
    }
}

/**
 * Stop the slideshow
 */
// Improved slideshow stop function
void stopSlideshow() {
  if (!slideshow_active) {
    // Already stopped
    return;
  }
  
  // Just set the flag to false - no need to terminate the task
  slideshow_active = false;
  Logger.info("Slideshow stopped");
  
  // Update UI
  if (ui_ready && status_label != nullptr) {
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      lv_label_set_text(status_label, "Slideshow: OFF");
      xSemaphoreGive(ui_mutex);
    }
  }
}

/**
 * Toggle slideshow on/off
 */
// Toggle slideshow on/off
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
 * LVGL task function 
 */
void lvgl_task(void *pvParameters) {
  while (true) {
    // Protect with mutex
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lv_timer_handler(); // Handle LVGL tasks
      xSemaphoreGive(ui_mutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Run at ~100Hz
  }
}

/**
 * Enhanced JPEG rendering with better error handling
 * Updated: 2025-06-16 10:32:38
 * User: Chamil1983
 */
bool render_jpeg_file(const char* filename) {
    if (!sd_ready || !filename) {
        Logger.error("SD card not ready or invalid filename");
        return false;
    }
    
    uint32_t startTime = millis();
    bool success = false;
    
    // Perform robust validation before attempting decode
    if (!isValidJPEG(filename)) {
        Logger.error("JPEG validation failed: %s", filename);
        return false;
    }
    
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
    
    // Try decoding at full resolution first
    Logger.info("Decoding image at full resolution");
    
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
            // For a portrait screen (800x1280), rotate if image is landscape
            image_needs_rotation = (imgWidth > imgHeight);
            
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
 * Enhanced JPEG draw callback with better memory safety and tearing prevention
 * Updated: 2025-06-16 10:32:38
 * User: Chamil1983
 */
int jpeg_draw_callback(JPEGDRAW *pDraw) {
    if (!lcd_ready || !pDraw) return 0;
    
    // Static timing control to prevent LCD driver overload
    static uint32_t lastDrawTime = 0;
    uint32_t currentTime = millis();
    
    // Add small delay between draw operations to prevent LCD driver errors
    if (currentTime - lastDrawTime < 5) {
        delay(5);
    }
    
    // Get image dimensions
    int imgWidth = jpeg.getWidth();
    int imgHeight = jpeg.getHeight();
    
    // Skip drawing if invalid dimensions
    if (imgWidth <= 0 || imgHeight <= 0) {
        return 0;
    }
    
    // Check if rotation is needed based on flag
    bool needsRotation = image_needs_rotation;
    
    if (!needsRotation) {
        // No rotation needed - standard centered drawing
        
        // Calculate centering offsets
        int centerX = (lcd.width() - imgWidth) / 2;
        int centerY = (lcd.height() - imgHeight) / 2;
        
        // Ensure non-negative values
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        // Calculate drawing coordinates
        int x = pDraw->x + centerX;
        int y = pDraw->y + centerY;
        
        // Check boundaries before drawing
        if (x >= 0 && y >= 0 && 
            x + pDraw->iWidth <= lcd.width() && 
            y + pDraw->iHeight <= lcd.height()) {
            
            // Draw the bitmap directly
            lcd.lcd_draw_bitmap(x, y, 
                               x + pDraw->iWidth, 
                               y + pDraw->iHeight, 
                               (uint16_t*)pDraw->pPixels);
        }
    } else {
        // Rotation needed - rotated dimensions
        int rotatedWidth = imgHeight;
        int rotatedHeight = imgWidth;
        
        // Center the rotated image
        int centerX = (lcd.width() - rotatedWidth) / 2;
        int centerY = (lcd.height() - rotatedHeight) / 2;
        
        // Ensure non-negative values
        centerX = max(0, centerX);
        centerY = max(0, centerY);
        
        // Calculate position in rotated space
        int rotX = centerX + pDraw->y;
        int rotY = centerY + (imgWidth - pDraw->x - pDraw->iWidth);
        
        // Calculate dimensions of rotated block
        int rotWidth = pDraw->iHeight;
        int rotHeight = pDraw->iWidth;
        
        // Prepare buffer for rotated block
        uint16_t stackBuf[64]; // Small stack buffer (128 bytes)
        uint16_t* rotatedBlock = nullptr;
        
        // Use appropriate buffer based on size
        if (rotWidth * rotHeight <= 64) {
            // Small enough for stack buffer
            rotatedBlock = stackBuf;
        } else {
            // Need heap allocation - try PSRAM first
            rotatedBlock = (uint16_t*)ps_calloc(rotWidth * rotHeight, sizeof(uint16_t));
            
            if (!rotatedBlock) {
                // Fall back to heap
                rotatedBlock = (uint16_t*)calloc(rotWidth * rotHeight, sizeof(uint16_t));
                
                if (!rotatedBlock) {
                    // If still can't allocate, skip this block
                    Logger.error("Failed to allocate memory for block rotation (%d bytes)",
                               rotWidth * rotHeight * sizeof(uint16_t));
                    return 0;
                }
            }
        }
        
        // Rotate the block with safety bounds checking
        uint16_t* pixels = (uint16_t*)pDraw->pPixels;
        for (int y = 0; y < pDraw->iHeight; y++) {
            for (int x = 0; x < pDraw->iWidth; x++) {
                // For 90° rotation: new_x = y, new_y = width - 1 - x
                int newX = y;
                int newY = pDraw->iWidth - 1 - x;
                
                // Bounds checking to prevent buffer overflow
                if (newX >= 0 && newX < rotWidth && 
                    newY >= 0 && newY < rotHeight &&
                    y < pDraw->iHeight && x < pDraw->iWidth) {
                    rotatedBlock[newY * rotWidth + newX] = pixels[y * pDraw->iWidth + x];
                }
            }
        }
        
        // Draw the rotated block within screen boundaries
        if (rotX >= 0 && rotY >= 0 && 
            rotX + rotWidth <= lcd.width() && 
            rotY + rotHeight <= lcd.height()) {
            lcd.lcd_draw_bitmap(rotX, rotY, 
                              rotX + rotWidth, 
                              rotY + rotHeight, 
                              rotatedBlock);
        }
        
        // Free allocated memory if not stack buffer
        if (rotatedBlock != stackBuf && rotatedBlock != nullptr) {
            free(rotatedBlock);
        }
    }
    
    // Update timing tracker
    lastDrawTime = currentTime;
    
    // Return success to continue processing
    return 1;
}


/**
 * Enhanced display_image function with smooth transitions
 * Updated: 2025-06-16 10:41:36
 * User: Chamil1983
 */
void display_image(int index) {
    if (!sd_ready || index < 0 || index >= image_count) {
        Logger.error("Cannot display image - invalid index %d or SD not ready", index);
        return;
    }
    
    // Check if we're already loading an image
    if (image_loading) {
        Logger.warn("Image loading already in progress - ignoring request");
        return;
    }
    
    // Feed watchdog before starting image display process
    feed_watchdog();
    
    // Set flag to indicate we're loading an image
    image_loading = true;
    
    // Store and use default backlight level since there's no getter method
    uint8_t originalBrightness = 255; // Default to full brightness
    
    // Update current image index and UI
    current_image = index;
    
    // Log display attempt
    Logger.info("Displaying image %d: %s", index, image_list[index]);
    updateImageCounter(index);
    
    // Dim backlight during transition
    lcd.example_bsp_set_lcd_backlight(50); // Dim to 50/255
    
    // Clear the screen completely with enhanced clearing function
    clearLCDScreen();
    
    // Make a local copy of the filename for safety
    char localFilename[100];
    strncpy(localFilename, image_list[index], sizeof(localFilename) - 1);
    localFilename[sizeof(localFilename) - 1] = 0; // Ensure null termination
    
    // Add a short delay to ensure screen clearing is complete
    delay(50);
    
    // Try to acquire JPEG mutex with a reasonable timeout
    if (xSemaphoreTake(jpeg_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Logger.error("Failed to take JPEG mutex for display");
        image_loading = false;
        // Restore backlight
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        return;
    }
    
    // Track timing
    uint32_t startTime = millis();
    bool success = false;
    
    // Safely decode and render JPEG
    try {
        success = render_jpeg_file(localFilename);
        
        if (!success) {
            Logger.error("Failed to render image %d: %s", index, localFilename);
        }
    } catch (...) {
        Logger.error("Exception during JPEG rendering");
        success = false;
    }
    
    // Always release the mutex
    xSemaphoreGive(jpeg_mutex);
    
    // Log timing information
    uint32_t renderTime = millis() - startTime;
    if (renderTime > 500) {
        Logger.warn("JPEG rendering took %lu ms", renderTime);
    } else {
        Logger.debug("JPEG rendering took %lu ms", renderTime);
    }
    
    // Fade the backlight back in if image rendering was successful
    if (success) {
        // Wait for any pending operations to complete
        delay(20);
        
        // Gradually restore backlight with smooth fade-in effect
        const int fadeStep = 10;
        const int fadeDelay = 10; // ms between steps
        
        for (int brightness = 50; brightness <= originalBrightness; brightness += fadeStep) {
            lcd.example_bsp_set_lcd_backlight(brightness);
            delay(fadeDelay);
        }
        
        // Ensure we're at the target brightness
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        
        Logger.info("Successfully displayed image %d", index);
    } else {
        // Immediately restore backlight on failure
        lcd.example_bsp_set_lcd_backlight(originalBrightness);
        
        // Display a "broken image" icon or message on screen
        displayErrorMessage("Image could not be displayed");
        
        Logger.error("Failed to display image %d", index);
    }
    
    // Update status in UI
    updateStatusAfterImageLoad(index, success);
    
    // Done loading
    image_loading = false;
    
    // Feed watchdog after completion
    feed_watchdog();
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
 * Enhanced screen clearing function with double buffering
 * to prevent tearing between image transitions
 * Updated: 2025-06-16 10:41:36
 * User: Chamil1983
 */
void clearLCDScreen() {
    if (!lcd_ready) return;
    
    // Set LCD backlight to dim during transition
    // There's no getter method, so we'll just dim and restore later
    lcd.example_bsp_set_lcd_backlight(50); // Dim for transition
    
    // Create a black buffer - use PSRAM for better performance
    size_t bufferSize = lcd.width() * 20; // 20 lines at a time
    uint16_t* blackBuffer = NULL;
    static uint16_t staticSmallBuffer[320]; // Fixed static buffer as fallback
    
    // Try PSRAM first
    blackBuffer = (uint16_t*)ps_malloc(bufferSize * sizeof(uint16_t));
    if (!blackBuffer) {
        // Fall back to a smaller static buffer if allocation fails
        blackBuffer = staticSmallBuffer;
        bufferSize = 320; // Reduced size
    }
    
    // Fill buffer with black (0x0000)
    for (size_t i = 0; i < bufferSize; i++) {
        blackBuffer[i] = 0x0000;
    }
    
    // Clear the screen in horizontal strips for better performance
    int stripHeight = bufferSize / lcd.width();
    if (stripHeight < 1) stripHeight = 1;
    
    // Track timing to ensure operations are properly paced
    uint32_t clearStart = millis();
    
    // Clear screen from top to bottom
    for (int y = 0; y < lcd.height(); y += stripHeight) {
        // Handle last strip which might be smaller
        int height = ((y + stripHeight) > lcd.height()) ? 
                     (lcd.height() - y) : stripHeight;
                     
        // Draw black rectangle for the entire width
        lcd.lcd_draw_bitmap(0, y, lcd.width(), y + height, blackBuffer);
        
        // Critical: Wait for LCD operation to complete before next strip
        delay(5); // Adjust based on LCD controller timing
        
        // Feed watchdog periodically
        if (y % 100 == 0) {
            feed_watchdog();
        }
    }
    
    // Free allocated buffer if we used PSRAM
    if (blackBuffer != NULL && blackBuffer != staticSmallBuffer) {
        free(blackBuffer);
    }
    
    // Add final delay to ensure all LCD operations are complete
    delay(50);
    
    // Log the clearing time
    Logger.debug("Screen cleared in %lu ms", millis() - clearStart);
    
    // Note: We don't restore backlight here - that happens after new image is drawn
}


/**
 * Enhanced JPEG file validation with comprehensive checks
 * Updated: 2025-06-16 10:32:38
 * User: Chamil1983
 */
bool isValidJPEG(const char* filename) {
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
    
    if (fileSize > 5000000) { // 5MB max
        Logger.warn("File too large for reliable display: %s (%d bytes)", filename, fileSize);
        jpegFile.close();
        return false;
    }
    
    // Check JPEG header (SOI marker: 0xFF 0xD8)
    uint8_t header[4];
    if (jpegFile.readBytes((char*)header, 4) != 4) {
        Logger.error("Couldn't read header from: %s", filename);
        jpegFile.close();
        return false;
    }
    
    if (header[0] != 0xFF || header[1] != 0xD8) {
        Logger.error("Invalid JPEG header (not FF D8): %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Most valid JPEGs have FF as the first byte of the third byte pair
    // This isn't guaranteed but helps filter obviously corrupt files
    if (header[2] != 0xFF) {
        Logger.warn("Suspicious JPEG structure in: %s", filename);
        jpegFile.close();
        return false;
    }
    
    // Scan for key JPEG markers to validate file integrity
    // We'll scan a limited number of bytes to avoid excessive processing
    const size_t MAX_SCAN_SIZE = 1024; // Scan first 1KB for markers
    const size_t scanSize = min(fileSize, MAX_SCAN_SIZE);
    
    // Reset file position
    jpegFile.seek(0);
    
    // Read chunk for scanning
    uint8_t* buffer = (uint8_t*)malloc(scanSize);
    if (!buffer) {
        Logger.error("Failed to allocate scan buffer");
        jpegFile.close();
        return false;
    }
    
    if (jpegFile.readBytes((char*)buffer, scanSize) != scanSize) {
        Logger.error("Failed to read scan data from: %s", filename);
        free(buffer);
        jpegFile.close();
        return false;
    }
    
    // Look for common JPEG markers (FF xx where xx != 00)
    bool foundAppMarker = false;  // APP0-APP15 (0xE0-0xEF)
    bool foundDQT = false;        // Define Quantization Table (0xDB)
    
    for (size_t i = 0; i < scanSize - 1; i++) {
        // Look for marker pattern (0xFF followed by non-zero)
        if (buffer[i] == 0xFF && buffer[i+1] != 0x00 && buffer[i+1] != 0xFF) {
            // Check marker type
            uint8_t markerType = buffer[i+1];
            
            // APP0-APP15 markers (0xE0-0xEF)
            if (markerType >= 0xE0 && markerType <= 0xEF) {
                foundAppMarker = true;
            }
            
            // Define Quantization Table (0xDB)
            if (markerType == 0xDB) {
                foundDQT = true;
            }
            
            // If we found both required markers, we can stop scanning
            if (foundAppMarker && foundDQT) {
                break;
            }
        }
    }
    
    free(buffer);
    
    // Check for required JPEG segments
    bool isValid = foundAppMarker && foundDQT;
    
    // Optionally check file end for JPEG EOI marker (0xFF 0xD9)
    // Not all valid JPEGs have this (e.g., if they were truncated but still viewable)
    if (isValid && fileSize > 2) {
        jpegFile.seek(fileSize - 2);
        uint8_t footer[2];
        if (jpegFile.readBytes((char*)footer, 2) == 2) {
            if (footer[0] != 0xFF || footer[1] != 0xD9) {
                Logger.warn("JPEG missing EOI marker - may be corrupted: %s", filename);
                // Don't invalidate just for missing EOI - some cameras create these
            }
        }
    }
    
    jpegFile.close();
    
    if (!isValid) {
        Logger.error("File failed JPEG structure validation: %s", filename);
    }
    
    return isValid;
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
 * Watchdog setup function
 */
bool setup_watchdog() {
  Logger.info("Setting up watchdog system with crash recovery...");
  
  try {
    // Configure watchdog timer
    esp_task_wdt_config_t wdt_config;
    wdt_config.timeout_ms = WDT_TIMEOUT_SECONDS * 1000;
    wdt_config.idle_core_mask = (1 << 0); // Don't watch idle cores
    wdt_config.trigger_panic = true; // Panic and reset on timeout
    
    esp_task_wdt_deinit(); // Deinitialize first to avoid conflicts
    esp_task_wdt_init(&wdt_config);
    
    // Add main task to watchdog
    main_task_handle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(main_task_handle);
    
    Logger.info("Watchdog reconfigured with %d second timeout and panic enabled", WDT_TIMEOUT_SECONDS);
    Logger.info("Main task added to watchdog with crash detection");
    
    wdt_enabled = true;
    return true;
  } catch (...) {
    Logger.error("Exception during watchdog setup");
    wdt_enabled = false;
    return false;
  }
}

/**
 * Feed the watchdog to prevent timeout
 */
void feed_watchdog() {
  if (wdt_enabled) {
    esp_task_wdt_reset();
  }
}

/**
 * Improved SPIFFS initialization with error handling and recovery
 * Date: 2025-06-15 11:47:49
 * User: Chamil1983
 */
bool initializeSpiffs() {
    Logger.info("Initializing SPIFFS...");
    
    // Define SPIFFS configuration parameters for Arduino ESP32
    // Arduino ESP32 uses a simpler initialization approach
    
    // Try to mount with retries
    bool mounted = false;
    int retries = 0;
    const int MAX_RETRIES = 3;
    
    while (retries < MAX_RETRIES && !mounted) {
        // In Arduino ESP32, we use SPIFFS.begin() instead of esp_vfs_spiffs_register
        mounted = SPIFFS.begin(true);  // true = format on failure
        
        if (mounted) {
            // Successfully mounted
            size_t total = SPIFFS.totalBytes();
            size_t used = SPIFFS.usedBytes();
            
            Logger.info("SPIFFS mounted successfully");
            Logger.info("SPIFFS: %d total, %d used, %d free", total, used, total - used);
            return true;
        } else {
            Logger.error("SPIFFS mount failed on attempt %d", retries + 1);
            
            // Try with explicit formatting for the last attempt
            if (retries == MAX_RETRIES - 1) {
                Logger.warn("Final attempt - formatting SPIFFS partition");
                // Arduino ESP32 format is handled by begin(true), but we can force it
                SPIFFS.format();
            }
            
            delay(1000);  // Wait before retry
            retries++;
        }
    }
    
    // If we reached here, all mount attempts failed
    Logger.error("SPIFFS mount failed after %d attempts", MAX_RETRIES);
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
 * Setup WiFi in Access Point mode
 */
bool setup_wifi_ap() {
  Logger.info("Setting up WiFi in AP mode...");
  
  wifi_ready = false;
  
  try {
    // Disconnect from any existing networks
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    
    // Configure AP with static IP
    if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
      Logger.error("WiFi AP Config Failed");
      return false;
    }
    
    // Start AP with SSID, password, channel, and max connections
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_CHANNEL, 0, MAX_WIFI_CLIENTS)) {
      Logger.error("WiFi AP Start Failed");
      return false;
    }
    
    // Log AP information
    Logger.info("WiFi AP Mode active");
    Logger.info("AP SSID: %s", WIFI_AP_SSID);
    Logger.info("AP IP Address: %s", WiFi.softAPIP().toString().c_str());
    Logger.info("AP Channel: %d", WIFI_CHANNEL);
    Logger.info("AP Max Clients: %d", MAX_WIFI_CLIENTS);
    
    wifi_ready = true;
  } catch (...) {
    Logger.error("Exception during WiFi initialization");
    wifi_ready = false;
  }
  
  return wifi_ready;
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
 * Setup and configure the web server
 * @return true if successful, false otherwise
 */
bool setup_webserver() {
  Logger.info("Setting up WebServer...");
  
  if (!wifi_ready) {
    Logger.error("Cannot setup WebServer - WiFi not ready");
    return false;
  }
  
  try {
    // Basic routes
    server.on("/", HTTP_GET, []() {
      Logger.debug("Serving main page");
      
      String html = "<html><head><title>ESP32P4 Gallery</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer;}";
      html += ".btn-danger {background: #dc2626;}";
      html += ".btn-success {background: #16a34a;}";
      html += ".upload-form {padding: 20px 0;}";
      html += "#status {padding: 10px; margin-top: 10px; border-radius: 4px;}";
      html += ".success {background: rgba(22, 163, 74, 0.2); color: #16a34a;}";
      html += ".error {background: rgba(220, 38, 38, 0.2); color: #dc2626;}";
      html += ".warning {background: rgba(234, 179, 8, 0.2); color: #eab308;}";
      html += ".info {background: rgba(59, 130, 246, 0.2); color: #3b82f6;}";
      html += "#progress-container {width: 100%; background-color: #1e293b; height: 20px; border-radius: 10px; margin: 10px 0; display: none;}";
      html += "#progress-bar {width: 0%; background-color: #16a34a; height: 100%; border-radius: 10px; transition: width 0.3s;}";
      html += ".file-input-wrapper {position: relative; overflow: hidden; display: inline-block; margin-bottom: 10px;}";
      html += ".file-input-wrapper input[type=file] {font-size: 100px; position: absolute; left: 0; top: 0; opacity: 0; cursor: pointer; width: 100%; height: 100%;}";
      html += ".file-input-wrapper button {display: inline-block; cursor: pointer;}";
      html += ".file-input-wrapper:hover button {background: #2563eb;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 Image Gallery</h1>";
      
      // System info card
      html += "<div class='card'>";
      html += "<h2>System Information</h2>";
      html += "<p>IP: " + WiFi.softAPIP().toString() + "</p>";
      html += "<p>Images: " + String(image_count) + "</p>";
      html += "<p>SD Card Status: " + String(sd_ready ? "Connected" : "Disconnected") + "</p>";
      html += "<p>Uptime: " + String(millis() / 1000) + " sec</p>";
      html += "<p>Slideshow: " + String(slideshow_active ? "Active" : "Inactive") + "</p>";
      html += "<button class='btn' onclick='window.location.href=\"/system\"'>System Info</button> ";
      html += "<button class='btn' onclick='window.location.href=\"/fix-sd-upload\"'>Upload Troubleshooter</button> ";
      html += "<button class='btn' onclick='window.location.href=\"/system-status\"'>System Monitor</button>";
      html += "</div>";
      
      // Upload card with enhanced javascript
      html += updateUploadForm();
      
      // Slideshow controls
      html += "<div class='card'>";
      html += "<h2>Slideshow Controls</h2>";
      if (slideshow_active) {
        html += "<p>Status: <span style='color:#16a34a'>Active</span> - " + String(slideshow_interval/1000) + " seconds per image</p>";
        html += "<button class='btn btn-danger' onclick='stopSlideshow()'>Stop Slideshow</button> ";
      } else {
        html += "<p>Status: <span style='color:#dc2626'>Inactive</span></p>";
        html += "<button class='btn btn-success' onclick='startSlideshow(5)'>Start Slideshow (5s)</button> ";
        html += "<button class='btn btn-success' onclick='startSlideshow(10)'>Start Slideshow (10s)</button> ";
        html += "<button class='btn btn-success' onclick='startSlideshow(30)'>Start Slideshow (30s)</button>";
      }
      html += "</div>";
      
      // Image navigation
      html += "<div class='card'>";
      html += "<h2>Image Navigation</h2>";
      html += "<p>Current Image: " + String(current_image + 1) + " of " + String(image_count) + "</p>";
      html += "<button class='btn' onclick='prevImage()'>Previous Image</button> ";
      html += "<button class='btn' onclick='nextImage()'>Next Image</button>";
      html += "</div>";
      
      // Actions card
      html += "<div class='card'>";
      html += "<h2>Actions</h2>";
      html += "<button class='btn btn-success' onclick='refreshSD()'>Refresh SD Card</button> ";
      html += "<button class='btn btn-danger' onclick='restartSystem()'>Restart System</button>";
      html += "</div>";
      
      // JavaScript for API interactions
      html += "<script>";
      
      html += "function startSlideshow(seconds) {";
      html += "  fetch('/slideshow?action=start&interval=' + (seconds * 1000), { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    window.location.reload();";
      html += "  });";
      html += "}";

      html += "function stopSlideshow() {";
      html += "  fetch('/slideshow?action=stop', { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    window.location.reload();";
      html += "  });";
      html += "}";

      html += "function prevImage() {";
      html += "  fetch('/navigate?direction=prev', { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    const status = document.getElementById('status');";
      html += "    status.style.display = 'block';";
      html += "    status.textContent = data;";
      html += "    status.className = 'success';";
      html += "  });";
      html += "}";
      
      html += "function nextImage() {";
      html += "  fetch('/navigate?direction=next', { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    const status = document.getElementById('status');";
      html += "    status.style.display = 'block';";
      html += "    status.textContent = data;";
      html += "    status.className = 'success';";
      html += "  });";
      html += "}";

      html += "function refreshSD() {";
      html += "  fetch('/refresh-sd', { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    const status = document.getElementById('status');";
      html += "    status.style.display = 'block';";
      html += "    status.textContent = data;";
      html += "    status.className = 'success';";
      html += "  });";
      html += "}";
      
      html += "function restartSystem() {";
      html += "  if(confirm('Are you sure you want to restart the system?')) {";
      html += "    fetch('/restart', { method: 'POST' })";
      html += "    .then(() => {";
      html += "      const status = document.getElementById('status');";
      html += "      status.style.display = 'block';";
      html += "      status.textContent = 'Restarting...';";
      html += "      status.className = 'success';";
      html += "    });";
      html += "  }";
      html += "}";
      
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });

    // Slideshow control endpoint
    server.on("/slideshow", HTTP_POST, []() {
      String action = server.arg("action");
      
      if (action == "start") {
        uint32_t interval = server.hasArg("interval") ? server.arg("interval").toInt() : 5000;
        startSlideshow(interval);
        server.send(200, "text/plain", "Slideshow started with " + String(interval/1000) + "s interval");
      } else if (action == "stop") {
        stopSlideshow();
        server.send(200, "text/plain", "Slideshow stopped");
      } else {
        server.send(400, "text/plain", "Invalid action. Use 'start' or 'stop'");
      }
    });
    
    // Navigation endpoint
    server.on("/navigate", HTTP_POST, []() {
      String direction = server.arg("direction");
      
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

    // Upload handler - just confirms upload completion
    server.on("/upload", HTTP_POST, []() {
      // Return a simple confirmation for AJAX response
      server.send(200, "text/plain", "File upload received");
    }, handle_upload);
    
    // System info page
    server.on("/system", HTTP_GET, []() {
      String html = "<html><head><title>System Info</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += "table {width: 100%; border-collapse: collapse; margin-bottom: 20px;}";
      html += "th, td {padding: 8px; text-align: left; border-bottom: 1px solid #334155;}";
      html += "th {background-color: #334155;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; text-decoration: none; display: inline-block;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 System Information</h1>";
      html += "<a href='/' class='btn'>Back to Gallery</a><br><br>";
      
      // Hardware info
      html += "<div class='card'>";
      html += "<h2>Hardware</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Model</td><td>ESP32P4</td></tr>";
      html += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
      html += "<tr><td>Flash Size</td><td>" + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB</td></tr>";
      html += "<tr><td>PSRAM Size</td><td>" + String(ESP.getPsramSize() / (1024 * 1024)) + " MB</td></tr>";
      html += "<tr><td>Free Heap</td><td>" + String(ESP.getFreeHeap() / 1024) + " KB</td></tr>";
      html += "<tr><td>Free PSRAM</td><td>" + String(ESP.getFreePsram() / 1024) + " KB</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Storage info
      html += "<div class='card'>";
      html += "<h2>Storage</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      
      if (sd_ready) {
        uint64_t total = 0;
        uint64_t used = 0;
        
        if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          total = SD_MMC.totalBytes();
          used = SD_MMC.usedBytes();
          xSemaphoreGive(sd_mutex);
        }
        
        uint64_t free = total - used;
        
        uint8_t cardType = SD_MMC.cardType();
        String cardTypeStr = "Unknown";
        if (cardType == CARD_MMC) cardTypeStr = "MMC";
        else if (cardType == CARD_SD) cardTypeStr = "SDSC";
        else if (cardType == CARD_SDHC) cardTypeStr = "SDHC";
        
        html += "<tr><td>SD Card</td><td>Connected (" + cardTypeStr + ")</td></tr>";
        html += "<tr><td>Total Space</td><td>" + String(total / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Used Space</td><td>" + String(used / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Free Space</td><td>" + String(free / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Image Count</td><td>" + String(image_count) + "</td></tr>";
      } else {
        html += "<tr><td>SD Card</td><td>Not connected</td></tr>";
      }
      html += "</table>";
      html += "<button class='btn' onclick='retrySD()'>Retry SD Card</button>";
      html += "</div>";
      
      // Network info
      html += "<div class='card'>";
      html += "<h2>Network</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>WiFi Mode</td><td>Access Point</td></tr>";
      html += "<tr><td>SSID</td><td>" + String(WIFI_AP_SSID) + "</td></tr>";
      html += "<tr><td>IP Address</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
      html += "<tr><td>Clients</td><td>" + String(WiFi.softAPgetStationNum()) + " connected</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Slideshow info
      html += "<div class='card'>";
      html += "<h2>Slideshow Status</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Slideshow Active</td><td>" + String(slideshow_active ? "Yes" : "No") + "</td></tr>";
      if (slideshow_active) {
        html += "<tr><td>Interval</td><td>" + String(slideshow_interval / 1000) + " seconds</td></tr>";
      }
      html += "</table>";
      html += "<button class='btn' onclick='toggleSlideshow()'>Toggle Slideshow</button>";
      html += "</div>";
      
      // Software info
      html += "<div class='card'>";
      html += "<h2>Software</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Version</td><td>1.0.8</td></tr>";
      html += "<tr><td>Build Date</td><td>2025-06-15 11:06:12 UTC</td></tr>";
      html += "<tr><td>User</td><td>Chamil1983</td></tr>";
      html += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
      html += "</table>";
      html += "</div>";
      
      html += "<script>";
      html += "function retrySD() {";
      html += "  fetch('/retry-sd', { method: 'POST' })";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      alert(data);";
      html += "      location.reload();";
      html += "    });";
      html += "}";
      html += "function toggleSlideshow() {";
      html += "  fetch('/slideshow?action=" + String(slideshow_active ? "stop" : "start") + "', { method: 'POST' })";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      alert(data);";
      html += "      location.reload();";
      html += "    });";
      html += "}";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });
    
    // SD Card actions
    server.on("/refresh-sd", HTTP_POST, []() {
      // Try to recover SD card first
      if (!sd_ready) {
        recover_sd_card();
      }
      
      if (sd_ready) {
        scan_images();
        if (image_count > 0) {
          display_image(0);
        }
        server.send(200, "text/plain", "SD Card refreshed. Found " + String(image_count) + " images.");
      } else {
        server.send(500, "text/plain", "SD card not available. Check connections.");
      }
    });
    
    server.on("/retry-sd", HTTP_POST, []() {
      if (recover_sd_card()) {
        scan_images();
        server.send(200, "text/plain", "SD card recovery successful. Found " + String(image_count) + " images.");
      } else {
        server.send(500, "text/plain", "SD card recovery failed. Check connections.");
      }
    });
    
    server.on("/restart", HTTP_POST, []() {
      server.send(200, "text/plain", "Restarting system...");
      reset_requested = true; // Safer than direct reset
    });
    
    server.on("/image", HTTP_GET, []() {
      if (server.hasArg("index")) {
        int index = server.arg("index").toInt();
        if (index >= 0 && index < image_count) {
          display_image(index);
          server.send(200, "text/plain", "Displaying image " + String(index + 1) + " of " + String(image_count));
        } else {
          server.send(400, "text/plain", "Invalid image index");
        }
      } else {
        server.send(400, "text/plain", "Missing index parameter");
      }
    });

// Add to setup_webserver() function
server.on("/recover-spiffs", HTTP_POST, []() {
    bool success = emergency_spiffs_recovery();
    if (success) {
        server.send(200, "text/plain", "SPIFFS recovery successful. System logs may be reset.");
    } else {
        server.send(500, "text/plain", "SPIFFS recovery failed. Consider restarting the device.");
    }
});

    // SD Upload helper page
    server.on("/fix-sd-upload", HTTP_GET, []() {
      String html = "<html><head><title>SD Card Upload Fix</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; margin: 5px;}";
      html += ".btn-success {background: #16a34a;}";
      html += ".status {padding: 10px; margin: 10px 0; border-radius: 4px;}";
      html += ".success {background: rgba(22, 163, 74, 0.2); color: #16a34a;}";
      html += ".error {background: rgba(220, 38, 38, 0.2); color: #dc2626;}";
      html += "</style></head><body>";
      
      html += "<h1>SD Card Upload Troubleshooter</h1>";
      
      html += "<div class='card'>";
      html += "<h2>Current SD Card Status</h2>";
      html += "<p>SD Card: " + String(sd_ready ? "Connected" : "Disconnected") + "</p>";
      html += "<p>Image Count: " + String(image_count) + "</p>";
      html += "</div>";
      
      html += "<div class='card'>";
      html += "<h2>SD Card Recovery</h2>";
      html += "<p>If you're having trouble uploading images, try these recovery options:</p>";
      html += "<button class='btn btn-success' onclick='recoverSD()'>Recover SD Card (Gentle)</button> ";
      html += "<button class='btn' onclick='resetSD()'>Reset SD Card (Full)</button>";
      html += "<p id='status' class='status' style='display:none;'></p>";
      html += "</div>";
      
      html += "<div class='card'>";
      html += "<h2>Upload Recommendations</h2>";
      html += "<ul>";
      html += "<li>Use smaller images (less than 1MB) for more reliable uploads</li>";
      html += "<li>Wait 10-15 seconds between uploads</li>";
      html += "<li>If upload fails, try the recovery options above</li>";
      html += "<li>Make sure the SD card is properly seated</li>";
      html += "</ul>";
      html += "<a href='/' class='btn'>Return to Gallery</a>";
      html += "</div>";
      
      html += "<script>";
      html += "function recoverSD() {";
      html += "  document.getElementById('status').textContent = 'Recovering SD card...';";
      html += "  document.getElementById('status').className = 'status';";
      html += "  document.getElementById('status').style.display = 'block';";
      html += "  fetch('/recover-sd-upload', { method: 'POST' })";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      document.getElementById('status').textContent = data;";
      html += "      document.getElementById('status').className = data.includes('failed') ? 'status error' : 'status success';";
      html += "    });";
      html += "}";
      html += "function resetSD() {";
      html += "  if(confirm('This will reset the SD card interface completely. Continue?')) {";
      html += "    document.getElementById('status').textContent = 'Resetting SD card...';";
      html += "    document.getElementById('status').className = 'status';";
      html += "    document.getElementById('status').style.display = 'block';";
      html += "    fetch('/reset-sd-upload', { method: 'POST' })";
      html += "      .then(response => response.text())";
      html += "      .then(data => {";
      html += "        document.getElementById('status').textContent = data;";
      html += "        document.getElementById('status').className = data.includes('failed') ? 'status error' : 'status success';";
      html += "      });";
      html += "  }";
      html += "}";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });

    // Recovery endpoint for uploads
    server.on("/recover-sd-upload", HTTP_POST, []() {
      if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        bool success = recover_sd_card_for_upload();
        xSemaphoreGive(sd_mutex);
        
        if (success) {
          scan_images();
          server.send(200, "text/plain", "SD card recovered successfully. Found " + 
                      String(image_count) + " images. Upload should work now.");
        } else {
          server.send(200, "text/plain", "SD card recovery failed. Try the full reset option.");
        }
      } else {
        server.send(200, "text/plain", "Could not get exclusive access to SD card. Try again.");
      }
    });

    // Full reset endpoint for uploads
    server.on("/reset-sd-upload", HTTP_POST, []() {
      if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        // Full reset process
        SD_MMC.end();
        delay(2000);
        
        // Reset pins
        pinMode(SDMMC_CLK_PIN, OUTPUT);
        pinMode(SDMMC_CMD_PIN, OUTPUT);
        pinMode(SDMMC_D0_PIN, OUTPUT);
        pinMode(SDMMC_D1_PIN, OUTPUT);
        pinMode(SDMMC_D2_PIN, OUTPUT);
        pinMode(SDMMC_D3_PIN, OUTPUT);
        
        digitalWrite(SDMMC_CLK_PIN, LOW);
        digitalWrite(SDMMC_CMD_PIN, LOW);
        digitalWrite(SDMMC_D0_PIN, LOW);
        digitalWrite(SDMMC_D1_PIN, LOW);
        digitalWrite(SDMMC_D2_PIN, LOW);
        digitalWrite(SDMMC_D3_PIN, LOW);
        delay(1000);
        
        // Reset to initial state
        pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
        pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
        delay(1000);
        
        // Reconfigure pins
        SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                      SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
        
        // Super safe initialization at lowest speed
        bool success = SD_MMC.begin("/sdcard", false, false, 125000);
        xSemaphoreGive(sd_mutex);
        
        if (success) {
          sd_ready = true;
          scan_images();
          server.send(200, "text/plain", "SD card reset successful. Found " + 
                      String(image_count) + " images. Upload should work now.");
        } else {
          sd_ready = false;
          server.send(200, "text/plain", "SD card reset failed. Please check physical connections or try restarting the device.");
        }
      } else {
        server.send(200, "text/plain", "Could not get exclusive access to SD card. Try again or restart the device.");
      }
    });
    
    // System status page
    server.on("/system-status", HTTP_GET, []() {
      String html = "<html><head><meta http-equiv='refresh' content='5'><title>System Status</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += "table {width: 100%; border-collapse: collapse; margin-bottom: 20px;}";
      html += "th, td {padding: 8px; text-align: left; border-bottom: 1px solid #334155;}";
      html += "th {background-color: #334155;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; text-decoration: none; display: inline-block;}";
      html += ".metric-good {color: #16a34a;}";
      html += ".metric-warning {color: #eab308;}";
      html += ".metric-critical {color: #dc2626;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 System Status</h1>";
      html += "<p>Auto-refreshes every 5 seconds</p>";
      html += "<a href='/' class='btn'>Back to Gallery</a><br><br>";
      
      // Memory usage
      html += "<div class='card'>";
      html += "<h2>Memory Usage</h2>";
      html += "<table>";
      html += "<tr><th>Metric</th><th>Current</th><th>Total</th><th>Used %</th></tr>";
      
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t totalHeap = ESP.getHeapSize();
      uint32_t usedHeap = totalHeap - freeHeap;
      float heapUsedPercent = 100.0f * (float)usedHeap / (float)totalHeap;
      
      String heapClass = heapUsedPercent < 70.0f ? "metric-good" : (heapUsedPercent < 90.0f ? "metric-warning" : "metric-critical");
      
      html += "<tr><td>Heap Memory</td><td>" + String(freeHeap / 1024) + " KB free</td>";
      html += "<td>" + String(totalHeap / 1024) + " KB</td>";
      html += "<td class='" + heapClass + "'>" + String(heapUsedPercent, 1) + "%</td></tr>";
      
      uint32_t freePsram = ESP.getFreePsram();
      uint32_t totalPsram = ESP.getPsramSize();
      uint32_t usedPsram = totalPsram - freePsram;
      float psramUsedPercent = 100.0f * (float)usedPsram / (float)totalPsram;
      
      String psramClass = psramUsedPercent < 50.0f ? "metric-good" : (psramUsedPercent < 80.0f ? "metric-warning" : "metric-critical");
      
      html += "<tr><td>PSRAM</td><td>" + String(freePsram / 1024) + " KB free</td>";
      html += "<td>" + String(totalPsram / 1024) + " KB</td>";
      html += "<td class='" + psramClass + "'>" + String(psramUsedPercent, 1) + "%</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // System uptime and activity
      html += "<div class='card'>";
      html += "<h2>System Activity</h2>";
      html += "<table>";
      html += "<tr><th>Metric</th><th>Value</th></tr>";
      
      // Format uptime nicely
      uint32_t uptime = millis() / 1000; // in seconds
      uint32_t uptimeDays = uptime / (24 * 3600);
      uptime %= (24 * 3600);
      uint32_t uptimeHours = uptime / 3600;
      uptime %= 3600;
      uint32_t uptimeMinutes = uptime / 60;
      uint32_t uptimeSeconds = uptime % 60;
      
      String uptimeStr = "";
      if (uptimeDays > 0) uptimeStr += String(uptimeDays) + " days, ";
      uptimeStr += String(uptimeHours) + ":" + 
                   (uptimeMinutes < 10 ? "0" : "") + String(uptimeMinutes) + ":" +
                   (uptimeSeconds < 10 ? "0" : "") + String(uptimeSeconds);
      
      html += "<tr><td>Uptime</td><td>" + uptimeStr + "</td></tr>";
      html += "<tr><td>Current Image</td><td>" + String(current_image + 1) + " of " + String(image_count) + "</td></tr>";
      html += "<tr><td>Slideshow Status</td><td>" + String(slideshow_active ? "Active (" + String(slideshow_interval / 1000) + "s interval)" : "Inactive") + "</td></tr>";
      html += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
      html += "<tr><td>WiFi Clients</td><td>" + String(WiFi.softAPgetStationNum()) + "</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Last 10 log entries
      html += "<div class='card'>";
      html += "<h2>Recent Logs</h2>";
      
      std::vector<String> logs = Logger.getRecentLogs();
      html += "<pre style='background:#1e293b; padding:10px; color:#f8fafc; overflow:auto; height:200px;'>";
      
      // Display most recent logs first (reversed)
      for (int i = logs.size() - 1; i >= 0 && i >= logs.size() - 10; i--) {
        String log = logs[i];
        
        // Add color based on log level
        if (log.indexOf("[ERROR]") >= 0 || log.indexOf("[CRITICAL]") >= 0) {
          html += "<span style='color:#dc2626;'>" + log + "</span>\n";
        } else if (log.indexOf("[WARN]") >= 0) {
          html += "<span style='color:#eab308;'>" + log + "</span>\n";
        } else if (log.indexOf("[INFO]") >= 0) {
          html += "<span style='color:#38bdf8;'>" + log + "</span>\n";
        } else {
          html += log + "\n";
        }
      }
      
      html += "</pre>";
      html += "</div>";
      
      html += "</body></html>";
      server.send(200, "text/html", html);
    });
    
    // Start web server
    server.begin();
    Logger.info("WebServer started on port 80");
    server_ready = true;
    
    return true;
  } catch (...) {
    Logger.error("Exception in WebServer setup");
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
 * Enhanced handle_upload function with proper file validation
 */
void handle_upload() {
    static File uploadFile;
    static size_t uploadSize = 0;
    static uint32_t lastWdtFeed = 0;
    static bool uploadInProgress = false;
    static bool uploadFailed = false;
    static const size_t WRITE_BUFFER_SIZE = 1024;
    static uint8_t* writeBuffer = NULL;
    static size_t bufferPos = 0;
    
    HTTPUpload& upload = server.upload();
    
    // Feed watchdog frequently
    uint32_t now = millis();
    if (now - lastWdtFeed > 1000) {
        feed_watchdog();
        lastWdtFeed = now;
    }
    
    if (upload.status == UPLOAD_FILE_START) {
        // Reset all variables
        uploadSize = 0;
        bufferPos = 0;
        uploadInProgress = true;
        uploadFailed = false;
        
        // Allocate write buffer from PSRAM if available
        if (writeBuffer != NULL) {
            free(writeBuffer);
            writeBuffer = NULL;
        }
        
        writeBuffer = (uint8_t*)ps_malloc(WRITE_BUFFER_SIZE);
        if (!writeBuffer) {
            // Fall back to regular heap
            writeBuffer = (uint8_t*)malloc(WRITE_BUFFER_SIZE);
            if (!writeBuffer) {
                Logger.error("Failed to allocate upload buffer");
                uploadFailed = true;
                return;
            }
        }
        
        // Generate safe filename with timestamp
        String safeFilename = upload.filename;
        safeFilename.replace(" ", "_");
        safeFilename.replace("/", "_");
        
        // Ensure file has .jpg extension
        if (!safeFilename.endsWith(".jpg") && !safeFilename.endsWith(".JPG") && 
            !safeFilename.endsWith(".jpeg") && !safeFilename.endsWith(".JPEG")) {
            safeFilename += ".jpg";
        }
        
        // Create full path with timestamp
        snprintf(upload_filename, sizeof(upload_filename), 
                "/images/img_%u_%s", millis() % 100000, safeFilename.c_str());
        
        Logger.info("Starting new upload: %s -> %s", 
                  upload.filename.c_str(), upload_filename);
        upload_start_time = millis();
        
        // Try to recover SD card if not working
        if (!sd_ready) {
            Logger.warn("SD card not ready before upload, attempting recovery");
            emergency_sd_recovery();
        }
        
        // Take mutex with retry
        int retryCount = 0;
        while (retryCount < 5 && !xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000))) {
            Logger.warn("Failed to get SD mutex for upload, retry %d", retryCount + 1);
            feed_watchdog();
            retryCount++;
        }
        
        if (retryCount >= 5) {
            Logger.error("Failed to get SD mutex after 5 retries");
            uploadFailed = true;
            uploadInProgress = false;
            free(writeBuffer);
            writeBuffer = NULL;
            return;
        }
        
        // Make sure images directory exists
        if (!SD_MMC.exists("/images")) {
            if (!SD_MMC.mkdir("/images")) {
                delay(100);
                if (!SD_MMC.mkdir("/images")) {
                    Logger.error("Failed to create images directory");
                    xSemaphoreGive(sd_mutex);
                    uploadFailed = true;
                    uploadInProgress = false;
                    free(writeBuffer);
                    writeBuffer = NULL;
                    return;
                }
            }
        }
        
        // Delete any existing file with the same name
        if (SD_MMC.exists(upload_filename)) {
            SD_MMC.remove(upload_filename);
        }
        
        // Open file for writing
        uploadFile = SD_MMC.open(upload_filename, FILE_WRITE);
        
        if (!uploadFile) {
            Logger.error("Failed to create file: %s", upload_filename);
            
            // Try with an alternate filename
            String altFilename = String("/images/upload_") + String(millis()) + ".jpg";
            uploadFile = SD_MMC.open(altFilename.c_str(), FILE_WRITE);
            
            if (!uploadFile) {
                Logger.error("Alternate filename also failed");
                xSemaphoreGive(sd_mutex);
                uploadFailed = true;
                uploadInProgress = false;
                free(writeBuffer);
                writeBuffer = NULL;
                return;
            }
            
            strncpy(upload_filename, altFilename.c_str(), sizeof(upload_filename) - 1);
        }
        
        Logger.info("File created successfully for upload: %s", upload_filename);
    }
    else if (upload.status == UPLOAD_FILE_WRITE && uploadInProgress && !uploadFailed) {
        // Process upload data in chunks
        size_t remaining = upload.currentSize;
        size_t position = 0;
        
        Logger.debug("Received %d bytes of upload data", remaining);
        
        while (remaining > 0) {
            // How many bytes can fit in the buffer
            size_t bytesToCopy = min(remaining, WRITE_BUFFER_SIZE - bufferPos);
            
            // Copy bytes to write buffer
            memcpy(writeBuffer + bufferPos, upload.buf + position, bytesToCopy);
            bufferPos += bytesToCopy;
            position += bytesToCopy;
            remaining -= bytesToCopy;
            
            // If buffer is full or this is the last chunk, write to SD
            if (bufferPos == WRITE_BUFFER_SIZE || remaining == 0) {
                if (uploadFile) {
                    size_t bytesWritten = uploadFile.write(writeBuffer, bufferPos);
                    
                    if (bytesWritten != bufferPos) {
                        Logger.error("SD Write error: %d of %d bytes written", bytesWritten, bufferPos);
                        uploadFailed = true;
                        break;
                    }
                    
                    // Update counters
                    uploadSize += bytesWritten;
                    
                    // Reset buffer
                    bufferPos = 0;
                    
                    // Periodic flush
                    if (uploadSize % 32768 == 0) { // Every 32KB
                        uploadFile.flush();
                    }
                } else {
                    Logger.error("Upload file not open during write");
                    uploadFailed = true;
                    break;
                }
            }
            
            // Feed watchdog in long loops
            if (position % 16384 == 0) { // Every 16KB
                feed_watchdog();
            }
        }
    }
    else if (upload.status == UPLOAD_FILE_END && uploadInProgress) {
        // Write any remaining data
        if (bufferPos > 0 && uploadFile && !uploadFailed) {
            uploadFile.write(writeBuffer, bufferPos);
        }
        
        // Free buffer memory
        if (writeBuffer != NULL) {
            free(writeBuffer);
            writeBuffer = NULL;
        }
        
        // Finalize file
        if (uploadFile) {
            // Final flush and close
            uploadFile.flush();
            uploadFile.close();
            
            // Log upload statistics
            float duration = (millis() - upload_start_time) / 1000.0;
            float speed = uploadSize / (1024.0 * duration);
            Logger.info("Upload complete: %s, %u bytes, %.1f KB/s", 
                       upload_filename, uploadSize, speed);
            
            // Validate the uploaded file
            bool isValid = isValidJPEG(upload_filename);
            
            // If file is invalid, delete it
            if (!isValid) {
                Logger.warn("Uploaded file is not a valid JPEG, deleting: %s", upload_filename);
                SD_MMC.remove(upload_filename);
                uploadFailed = true;
            }
            
            // Release SD mutex
            xSemaphoreGive(sd_mutex);
            
            // Wait for SD card operations to finish
            delay(200);
            
            // Only update the image list if upload was successful
            if (!uploadFailed && uploadSize > 0) {
                // Rescan for images
                if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                    scan_images();
                    
                    // Find our newly uploaded image
                    int newImageIndex = -1;
                    for (int i = 0; i < image_count; i++) {
                        if (strcmp(image_list[i], upload_filename) == 0) {
                            newImageIndex = i;
                            break;
                        }
                    }
                    
                    // Display the new image
                    if (newImageIndex >= 0) {
                        current_image = newImageIndex;
                        display_image(current_image);
                    }
                    
                    xSemaphoreGive(sd_mutex);
                }
            }
        } else {
            // Release mutex if we got this far
            xSemaphoreGive(sd_mutex);
        }
        
        // Reset upload state
        uploadInProgress = false;
        bufferPos = 0;
    }
    else if (upload.status == UPLOAD_FILE_ABORTED) {
        Logger.error("Upload aborted");
        
        // Free buffer memory
        if (writeBuffer != NULL) {
            free(writeBuffer);
            writeBuffer = NULL;
        }
        
        // Close file if open
        if (uploadFile) {
            uploadFile.close();
            
            // Delete partial file
            if (SD_MMC.exists(upload_filename)) {
                SD_MMC.remove(upload_filename);
            }
        }
        
        // Release mutex
        if (uploadInProgress) {
            xSemaphoreGive(sd_mutex);
        }
        
        // Reset state
        uploadInProgress = false;
        uploadFailed = true;
        bufferPos = 0;
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
 * Improved scan_images function with corruption detection
 */
void scan_images() {
    Logger.info("Scanning for images...");
    
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
    
    // Scan for jpg files
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
                
                // Validate size - skip too small files
                size_t fileSize = file.size();
                if (fileSize < 1024) { // Minimum 1KB
                    Logger.warn("Skipping small file: %s (%d bytes)", fullPath.c_str(), fileSize);
                    file = root.openNextFile();
                    continue;
                }
                
                // Quick check for valid JPEG header
                bool isValidHeader = false;
                uint8_t header[2];
                if (file.readBytes((char*)header, 2) == 2) {
                    isValidHeader = (header[0] == 0xFF && header[1] == 0xD8);
                }
                file.seek(0); // Reset file position
                
                if (!isValidHeader) {
                    Logger.warn("Skipping file with invalid JPEG header: %s", fullPath.c_str());
                    file = root.openNextFile();
                    continue;
                }
                
                // Allocate memory for the path string
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
        }
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