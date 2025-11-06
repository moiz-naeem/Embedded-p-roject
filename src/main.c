
#include <stdio.h>
#include <string.h>
#include <math.h> 
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include "tkjhat/sdk.h"
#include "tusb.h"

#include "usbSerialDebug/helper.h"
// Exercise 4. Include the libraries necessaries to use the usb-serial-debug, and tinyusb
// Tehtävä 4 . Lisää usb-serial-debugin ja tinyusbin käyttämiseen tarvittavat kirjastot.



#define DEFAULT_STACK_SIZE 1024
#define CDC_ITF_TX      1
#define MAX_MESSAGE_LENGTH 256


typedef enum {
    STATE_IDLE,
    STATE_DETECTING_POSITION,
    STATE_SENDING_SYMBOL,
    STATE_RECEIVING,
    STATE_DISPLAYING_MESSAGE,
    STATE_ERROR
} AppState;

typedef enum {
    POSITION_FLAT,      
    POSITION_TILTED,    
    POSITION_VERTICAL, 
    POSITION_UNKNOWN
} DevicePosition;

typedef struct {
    char symbol;       
    uint32_t timestamp;
} MorseSymbol;

static AppState currentState = STATE_IDLE;
static SemaphoreHandle_t stateMutex = NULL;
static QueueHandle_t symbolQueue = NULL;
static QueueHandle_t receiveQueue = NULL;


static char outgoingMessage[MAX_MESSAGE_LENGTH];
static char receivedMessage[MAX_MESSAGE_LENGTH];
static uint16_t messageIndex = 0;
static uint16_t receivedIndex = 0;


static float accel_x, accel_y, accel_z;
static float gyro_x, gyro_y, gyro_z;
static float temperature;


#define FLAT_THRESHOLD_Z_MIN 0.8f
#define FLAT_THRESHOLD_Z_MAX 1.2f
#define TILTED_THRESHOLD_X 0.6f
#define VERTICAL_THRESHOLD_Y 0.8f
#define POSITION_STABLE_TIME_MS 300



/**
 * @brief Safely change the application state
 */
static void setState(AppState newState) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

/**
 * @brief Get current application state
 */
static AppState getState(void) {
    AppState state;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

/**
 * @brief Detect device position based on IMU data
 */
static DevicePosition detectPosition(float ax, float ay, float az) {
    // Flat position (dot): device lying flat on table, Z-axis ~1g
    if (az > FLAT_THRESHOLD_Z_MIN && az < FLAT_THRESHOLD_Z_MAX && 
        fabs(ax) < 0.3f && fabs(ay) < 0.3f) {
        return POSITION_FLAT;
    }
    
    // Tilted position (dash): device tilted ~45-90 degrees on X-axis
    if (fabs(ax) > TILTED_THRESHOLD_X) {
        return POSITION_TILTED;
    }
    
    // Vertical position (space/send): device standing on edge, Y-axis dominant
    if (fabs(ay) > VERTICAL_THRESHOLD_Y) {
        return POSITION_VERTICAL;
    }
    
    return POSITION_UNKNOWN;
}

/**
 * @brief Add symbol to outgoing message buffer
 */
static bool addSymbolToMessage(char symbol) {
    if (messageIndex >= MAX_MESSAGE_LENGTH - 3) { // Leave room for "  \n"
        return false;
    }
    outgoingMessage[messageIndex++] = symbol;
    return true;
}

/**
 * @brief Send complete message via USB CDC
 */
static void sendMessageToWorkstation(void) {
    // Add message terminator: two spaces and newline
    if (messageIndex > 0) {
        outgoingMessage[messageIndex++] = ' ';
        outgoingMessage[messageIndex++] = ' ';
        outgoingMessage[messageIndex++] = '\n';
        outgoingMessage[messageIndex] = '\0';
        
        // Send via CDC interface 1 (data)
        if (tud_cdc_n_connected(CDC_ITF_TX)) {
            tud_cdc_n_write_str(CDC_ITF_TX, outgoingMessage);
            tud_cdc_n_write_flush(CDC_ITF_TX);
            
            // Feedback: blink LED and beep
            blink_red_led(2);
            buzzer_play_tone(1000, 100);
            
            // Debug via CDC 0
            usb_serial_print("__Message sent successfully__\n");
        }
        
        // Reset buffer
        messageIndex = 0;
        memset(outgoingMessage, 0, MAX_MESSAGE_LENGTH);
    }
}

/**
 * @brief Display received symbol on LCD and play feedback
 */
static void displayReceivedSymbol(char symbol) {
    char displayBuffer[32];
    
    if (symbol == '.') {
        snprintf(displayBuffer, sizeof(displayBuffer), "DOT");
        buzzer_play_tone(800, 100);  // Short beep for dot
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_red_led_status(false);
    } else if (symbol == '-') {
        snprintf(displayBuffer, sizeof(displayBuffer), "DASH");
        buzzer_play_tone(800, 300);  // Long beep for dash
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(300));
        set_red_led_status(false);
    } else if (symbol == ' ') {
        snprintf(displayBuffer, sizeof(displayBuffer), "SPACE");
        buzzer_play_tone(600, 50);   // Different tone for space
    } else {
        snprintf(displayBuffer, sizeof(displayBuffer), "???");
    }
    
    // Display on LCD
    write_text(displayBuffer);
}

// ==================== BUTTON INTERRUPT HANDLER ====================
static volatile bool button1Pressed = false;
static volatile bool button2Pressed = false;

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        button1Pressed = true;

    } else if (gpio == BUTTON2) {
        button2Pressed = true;

    }
}

// ==================== FREERTOS TASKS ====================

/**
 * @brief Task for detecting device position and generating Morse symbols
 */
static void imu_sensor_task(void *arg) {
    (void)arg;
    
    init_ICM42670();
    vTaskDelay(pdMS_TO_TICKS(100));
    ICM42670_start_with_default_values();
    
    DevicePosition lastPosition = POSITION_UNKNOWN;
    DevicePosition currentPosition;
    TickType_t positionStartTime = 0;
    bool positionStable = false;
    
    for (;;) {
        AppState state = getState();
        
        // Only detect positions when in appropriate state
        if (state == STATE_IDLE || state == STATE_DETECTING_POSITION) {
            // Read IMU data
            ICM42670_read_sensor_data(&accel_x, &accel_y, &accel_z,
                                     &gyro_x, &gyro_y, &gyro_z, &temperature);
            
            currentPosition = detectPosition(accel_x, accel_y, accel_z);
            
            // Check if position changed
            if (currentPosition != lastPosition) {
                lastPosition = currentPosition;
                positionStartTime = xTaskGetTickCount();
                positionStable = false;
                setState(STATE_DETECTING_POSITION);
            } else {
                // Position is same, check if stable long enough
                TickType_t elapsed = xTaskGetTickCount() - positionStartTime;
                if (elapsed > pdMS_TO_TICKS(POSITION_STABLE_TIME_MS) && !positionStable) {
                    positionStable = true;
                    
                    // Generate symbol based on position
                    MorseSymbol symbol;
                    symbol.timestamp = xTaskGetTickCount();
                    
                    if (currentPosition == POSITION_FLAT) {
                        symbol.symbol = '.';
                        usb_serial_print("__Detected: DOT__\n");
                    } else if (currentPosition == POSITION_TILTED) {
                        symbol.symbol = '-';
                        usb_serial_print("__Detected: DASH__\n");
                    } else if (currentPosition == POSITION_VERTICAL) {
                        symbol.symbol = ' ';
                        usb_serial_print("__Detected: SPACE__\n");
                    } else {
                        symbol.symbol = '\0';
                    }
                    
                    // Send to queue if valid symbol
                    if (symbol.symbol != '\0') {
                        xQueueSend(symbolQueue, &symbol, 0);
                        setState(STATE_SENDING_SYMBOL);
                        
                        // Brief feedback
                        toggle_red_led();
                        vTaskDelay(pdMS_TO_TICKS(50));
                        toggle_red_led();
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz sampling
    }
}

/**
 * @brief Task for sending symbols to workstation
 */
static void transmit_task(void *arg) {
    (void)arg;
    MorseSymbol symbol;
    
    for (;;) {
        // Wait for symbol from queue
        if (xQueueReceive(symbolQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Add to message buffer
            if (addSymbolToMessage(symbol.symbol)) {
                // Add space between symbols (Morse protocol requirement)
                if (symbol.symbol != ' ') {
                    addSymbolToMessage(' ');
                }
                
                // Send individual symbol via CDC 1 for real-time feedback
                char symbolStr[3] = {symbol.symbol, ' ', '\0'};
                if (tud_cdc_n_connected(CDC_ITF_TX)) {
                    tud_cdc_n_write_str(CDC_ITF_TX, symbolStr);
                    tud_cdc_n_write_flush(CDC_ITF_TX);
                }
                buzzer_play_tone(1000, 100);
                usb_serial_print("__Symbol added to message__\n");
            } else {
                usb_serial_print("__Message buffer full!__\n");
                buzzer_play_tone(1000, 1000);
            }
            
            setState(STATE_IDLE);
        }
        
        // Check button presses for sending complete message
        if (button2Pressed) {
            button2Pressed = false;
            usb_serial_print("__Sending message...__\n");
            sendMessageToWorkstation();
            clear_display();
            buzzer_play_tone(1000, 100);
            write_text("SENT!");
            vTaskDelay(pdMS_TO_TICKS(500));
            clear_display();
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task for receiving messages from workstation
 */
static void receive_task(void *arg) {
    (void)arg;
    char rxBuffer[128];
    
    for (;;) {
        // Check if data available on CDC 1
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rxBuffer, sizeof(rxBuffer) - 1);
            if (count > 0) {
                rxBuffer[count] = '\0';
                
                setState(STATE_RECEIVING);
                usb_serial_print("__Received data__\n");
                
                // Process each character
                for (uint32_t i = 0; i < count; i++) {
                    char c = rxBuffer[i];
                    
                    // Valid Morse symbols
                    if (c == '.' || c == '-' || c == ' ') {
                        // Send to display queue
                        xQueueSend(receiveQueue, &c, 0);
                        
                        // Add to received message buffer
                        if (receivedIndex < MAX_MESSAGE_LENGTH - 1) {
                            receivedMessage[receivedIndex++] = c;
                        }
                    }
                    
                    // Check for message end (two spaces or newline)
                    if (c == '\n' || 
                        (receivedIndex >= 2 && 
                         receivedMessage[receivedIndex-1] == ' ' && 
                         receivedMessage[receivedIndex-2] == ' ')) {
                        
                        receivedMessage[receivedIndex] = '\0';
                        usb_serial_print("__Complete message received__\n");
                        
                        // Display complete message notification
                        clear_display();
                        write_text("MSG END");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        clear_display();
                        
                        // Reset buffer
                        receivedIndex = 0;
                        memset(receivedMessage, 0, MAX_MESSAGE_LENGTH);
                    }
                }
                
                setState(STATE_IDLE);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task for displaying received symbols
 */
static void display_task(void *arg) {
    (void)arg;
    char symbol;

    
    for (;;) {
        if (xQueueReceive(receiveQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE) {
            setState(STATE_DISPLAYING_MESSAGE);
            displayReceivedSymbol(symbol);
            setState(STATE_IDLE);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void usb_task(void *arg) {
    (void)arg;
    while (1) {
        tud_task();
    }
}

static void startup_task(void *arg) {
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(500));
    usb_serial_print("__System starting...__\n");
    write_text("READY");
    buzzer_play_tone(1000, 100);
    vTaskDelay(pdMS_TO_TICKS(1000));  // ADD THIS LINE - Show READY for 1 sec
    clear_display();
    
    usb_serial_print("__System initialized__\n");
    
    vTaskDelete(NULL);
}
// Exercise 4: Uncomment the following line to activate the TinyUSB library.  
// Tehtävä 4:  Poista seuraavan rivin kommentointi aktivoidaksesi TinyUSB-kirjaston. 

/*
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}*/

int main() {

    // Exercise 4: Comment the statement stdio_init_all(); 
    //             Instead, add AT THE END OF MAIN (before vTaskStartScheduler();) adequate statements to enable the TinyUSB library and the usb-serial-debug.
    //             You can see hello_dual_cdc for help
    //             In CMakeLists.txt add the cfg-dual-usbcdc
    //             In CMakeLists.txt deactivate pico_enable_stdio_usb
    // Tehtävä 4:  Kommentoi lause stdio_init_all();
    //             Sen sijaan lisää MAIN LOPPUUN (ennen vTaskStartScheduler();) tarvittavat komennot aktivoidaksesi TinyUSB-kirjaston ja usb-serial-debugin.
    //             Voit katsoa apua esimerkistä hello_dual_cdc.
    //             Lisää CMakeLists.txt-tiedostoon cfg-dual-usbcdc
    //             Poista CMakeLists.txt-tiedostosta käytöstä pico_enable_stdio_usb



    init_hat_sdk();
    sleep_ms(200); //Wait some time so initialization of USB and hat is done.
    init_display();
    clear_display();
    write_text("HAT-SDK");
    sleep_ms(300);
    init_red_led();
    gpio_put(LED1, 1);
    sleep_ms(5000);
    gpio_put(LED1, 0);
    sleep_ms(2000);
    // Pattern: 1 long blink = Started main
    

    // Exercise 1: Initialize the button and the led and define an register the corresponding interrupton.
    //             Interruption handler is defined up as btn_fxn
    // Tehtävä 1:  Alusta painike ja LEd ja rekisteröi vastaava keskeytys.
    //             Keskeytyskäsittelijä on määritelty yläpuolella nimellä btn_fxn
    

 
    init_rgb_led();
    rgb_led_write(255, 0, 40);
    sleep_ms(1000);
    rgb_led_write(0, 250, 50);
    sleep_ms(1000);
    rgb_led_write(0, 0, 255);
    sleep_ms(1000);
    rgb_led_write(0, 255, 0);
    sleep_ms(1000);
    
    stop_rgb_led();


    init_button1();
    init_button2();
    
    init_buzzer();
    buzzer_play_tone(1000, 2000);
    clear_display();
    write_text("SENSORS");
    sleep_ms(2000);
    // Register button interrupts
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    
    clear_display();
    write_text("BUTTONS");
    sleep_ms(2000);
    // Welcome message
    //clear_display();
    //write_text("READY");
    //buzzer_play_tone(1000, 100);
    //vTaskDelay(pdMS_TO_TICKS(500));
    //clear_display();
    
    // Create synchronization primitives
    // Create synchronization primitives
    stateMutex = xSemaphoreCreateMutex();

    if (stateMutex == NULL)
    {
        clear_display();
        write_text("MUTEX-FAIL");
        sleep_ms(5000);
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    clear_display();
    write_text("MUTEX-OK");
    sleep_ms(2000);

    symbolQueue = xQueueCreate(10, sizeof(MorseSymbol));

    if (symbolQueue == NULL)
    {
        clear_display();
        write_text("SYMQ-FAIL");
        sleep_ms(5000);
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    clear_display();
    write_text("SYMQ-OK");
    sleep_ms(2000);

    receiveQueue = xQueueCreate(10, sizeof(char));

    if (receiveQueue == NULL)
    {
        clear_display();
        write_text("RECQ-FAIL");
        sleep_ms(5000);
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    clear_display();
    write_text("RECQ-OK");
    sleep_ms(2000);

    if (stateMutex == NULL || symbolQueue == NULL || receiveQueue == NULL) {
        clear_display();
        write_text("NULL");
        sleep_ms(2000);
        while(1) {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    for(int i = 0; i < 4; i++) {
        gpio_put(LED1, 1);
        sleep_ms(1000);
        gpio_put(LED1, 0);
        sleep_ms(1000);
    }
    
    // Create tasks
    TaskHandle_t hStartup, hIMU, hTransmit, hReceive, hDisplay, hUSB = NULL;
    
    


    xTaskCreate(usb_task, "usb", 1024, NULL, 4, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif

    clear_display();
    write_text("USB");
    sleep_ms(2000);


    xTaskCreate(startup_task, "startup", 512, NULL, 3, &hStartup);

    clear_display();
    write_text("STARTUP");
    sleep_ms(2000);

    xTaskCreate(imu_sensor_task, "imu", DEFAULT_STACK_SIZE, NULL, 2, &hIMU);
 
    clear_display();
    write_text("IMU");
    sleep_ms(2000);

    xTaskCreate(transmit_task, "transmit", DEFAULT_STACK_SIZE, NULL, 2, &hTransmit);

    clear_display();
    write_text("TRANSMIT");
    sleep_ms(2000);

    xTaskCreate(receive_task, "receive", DEFAULT_STACK_SIZE, NULL, 2, &hReceive);
  
    clear_display();
    write_text("RECEIVE");
    sleep_ms(2000);

    xTaskCreate(display_task, "display", DEFAULT_STACK_SIZE, NULL, 2, &hDisplay);
    
    clear_display();
    write_text("DISPLAY");
    sleep_ms(2000);
    
    for(int i = 0; i < 5; i++) {
        gpio_put(LED1, 1);
        sleep_ms(2000);
        gpio_put(LED1, 0);
        sleep_ms(2000);
    }

    
    //stdio_init_all();


    //Uncomment this lines if you want to wait till the serial monitor is connected
    //while (!stdio_usb_connected()){
      //  sleep_ms(10);
    //} 
    tusb_init();

    clear_display();
    write_text("TUSB");
    sleep_ms(2000);

    usb_serial_init();
    
    clear_display();
    write_text("SERIAL");
    sleep_ms(2000);
    
    // Start scheduler
    vTaskStartScheduler();



    clear_display();
    write_text("DANG");
    sleep_ms(2000);

    gpio_put(LED1, 1);
    while(1) {
        sleep_ms(1000);
    }
    
    return 0;
}

