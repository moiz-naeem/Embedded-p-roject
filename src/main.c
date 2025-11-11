
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
#include "morse_translator.h"
#include "wifi.h"

// Exercise 4. Include the libraries necessaries to use the usb-serial-debug, and tinyusb
// Tehtävä 4 . Lisää usb-serial-debugin ja tinyusbin käyttämiseen tarvittavat kirjastot.



#define DEFAULT_STACK_SIZE 2048
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
// Queue handles - non-static so wifi.c can access them
QueueHandle_t symbolQueue = NULL;
QueueHandle_t receiveQueue = NULL;


static char outgoingMessage[MAX_MESSAGE_LENGTH];
static char receivedMessage[MAX_MESSAGE_LENGTH];
static uint16_t messageIndex = 0;
static uint16_t receivedIndex = 0;

// Translation buffers
static char currentMorsePattern[10];  // Current letter being decoded (max morse = 5 chars)
static uint8_t patternIndex = 0;
static char translatedWord[32];       // Translated letters accumulate here
static uint8_t wordIndex = 0;


#define FLAT_THRESHOLD_Z_MIN 0.8f
#define FLAT_THRESHOLD_Z_MAX 1.2f
#define TILTED_THRESHOLD_X 0.6f
#define VERTICAL_THRESHOLD_Y 0.8f
#define POSITION_STABLE_TIME_MS 300



/**
 * @brief safely change the application state
 */
static void setState(AppState newState) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

/**
 * @brief get current application state
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
 * @brief detect device position based on IMU data
 */
static DevicePosition detectPosition(float ax, float ay, float az) {

    if (az > FLAT_THRESHOLD_Z_MIN && az < FLAT_THRESHOLD_Z_MAX && 
        fabs(ax) < 0.3f && fabs(ay) < 0.3f) {
        return POSITION_FLAT;
    }
    

    if (fabs(ax) > TILTED_THRESHOLD_X) {
        return POSITION_TILTED;
    }
    

    if (fabs(ay) > VERTICAL_THRESHOLD_Y) {
        return POSITION_VERTICAL;
    }
    
    return POSITION_UNKNOWN;
}

/**
 * @brief add symbol to outgoing message buffer
 */
static bool addSymbolToMessage(char symbol) {
    if (messageIndex >= MAX_MESSAGE_LENGTH - 3) { // leavingroom for "  \n"
        return false;
    }
    outgoingMessage[messageIndex++] = symbol;
    return true;
}

/**
 * @brief translate outgoing morse message to plain text
 */
static void translateOutgoingMessage(const char *morse_msg, char *translated, uint16_t max_len) {
    char pattern[10];
    uint8_t pattern_idx = 0;
    uint16_t trans_idx = 0;
    
    for (int i = 0; morse_msg[i] != '\0' && trans_idx < max_len - 1; i++) {
        char c = morse_msg[i];
        
        if (c == '.' || c == '-') {
            // Accumulate morse pattern
            if (pattern_idx < sizeof(pattern) - 1) {
                pattern[pattern_idx++] = c;
            }
        } else if (c == ' ') {
            // Translate pattern to letter
            if (pattern_idx > 0) {
                pattern[pattern_idx] = '\0';
                char letter = morse_to_letter(pattern);
                translated[trans_idx++] = letter;
                pattern_idx = 0;
            }
        }
    }
    
    translated[trans_idx] = '\0';
}

/**
 * @brief send complete message via USB CDC
 */
static void sendMessageToWorkstation(void) {
    // add message terminator, two spaces and newline
    if (messageIndex > 0) {
        outgoingMessage[messageIndex++] = ' ';
        outgoingMessage[messageIndex++] = ' ';
        outgoingMessage[messageIndex++] = '\n';
        outgoingMessage[messageIndex] = '\0';
        
        // Translate morse to plain text and display on LCD
        char translatedMsg[32];
        translateOutgoingMessage(outgoingMessage, translatedMsg, sizeof(translatedMsg));
        
        if (strlen(translatedMsg) > 0) {
            // Using display conflicts with IMU sensor
            // Just play distinctive tone and print to serial
            buzzer_play_tone(1200, 100);  // Preview tone for outgoing
            buzzer_play_tone(1500, 100);  // Second tone = sending translated message
            
            // Debug output
            if (usb_serial_connected()) {
                usb_serial_print(">>SENDING: ");
                usb_serial_print(translatedMsg);
                usb_serial_print("<<\n");
            }
        }
        
        // senf via CDC interface 1 (data)
        if (tud_cdc_n_connected(CDC_ITF_TX)) {
            tud_cdc_n_write_str(CDC_ITF_TX, outgoingMessage);
            tud_cdc_n_write_flush(CDC_ITF_TX);
            

            blink_red_led(2);
            buzzer_play_tone(6000, 100);
            
            // CDC 0 for debug
            usb_serial_print("__Message sent via USB__\n");
        }
        
        // Also send via WiFi UDP (broadcast to all devices on network)
        if (wifi_send_message(outgoingMessage)) {
            usb_serial_print("__Message sent via WiFi__\n");
            buzzer_play_tone(7000, 100);  // Higher tone for WiFi success
        } else {
            usb_serial_print("__WiFi send failed__\n");
        }
        
        // buffer reset
        messageIndex = 0;
        memset(outgoingMessage, 0, MAX_MESSAGE_LENGTH);
    }
}

/**
 * @brief display received symbol on LCD and play feedback
 */
static void displayReceivedSymbol(char symbol) {
    char displayBuffer[32];
    
    if (symbol == '.') {
        snprintf(displayBuffer, sizeof(displayBuffer), "DOT");
        buzzer_play_tone(660, 100); 
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_red_led_status(false);
    } else if (symbol == '-') {
        snprintf(displayBuffer, sizeof(displayBuffer), "DASH");
        buzzer_play_tone(660, 300);  
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(300));
        set_red_led_status(false);
    } else if (symbol == ' ') {
        snprintf(displayBuffer, sizeof(displayBuffer), "SPACE");
  
    } else {
        snprintf(displayBuffer, sizeof(displayBuffer), "XXXXX");
    }
    clear_display();
    write_text(displayBuffer);
}

// BUTTON INTERRUPT HANDLER 
static volatile bool button1Pressed = false;
static volatile bool button2Pressed = false;

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        button1Pressed = true;

    } else if (gpio == BUTTON2) {
        button2Pressed = true;

    }
}



/**
 * @brief sask for detecting device position and generating morse symbols
 */
static void imu_sensor_task(void *arg) {
    (void)arg;

    float ax, ay, az, gx, gy, gz, t;

    while (!tud_mounted() || !tud_cdc_n_connected(0)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (usb_serial_connected()) {
        usb_serial_print("Initializing ICM-42670P...\n");
    }

    if (init_ICM42670() == 0) {
        if (usb_serial_connected()) {
            usb_serial_print("ICM-42670P initialized successfully!\n");
        }


        if (ICM42670_start_with_default_values() != 0) {
            if (usb_serial_connected()) {
                usb_serial_print("ICM-42670P failed to start with default values!\n");
            }

        }
    } else {
        if (usb_serial_connected()) {
            usb_serial_print("ICM-42670P initialization failed!\n");
        }
        for (int i = 0; i < 3; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
            if (init_ICM42670() == 0) {
                usb_serial_print("Retry success: ICM-42670P initialized!\n");
                ICM42670_start_with_default_values();
                break;
            } else if (i == 2) {
                usb_serial_print("IMU not responding. Entering safe idle.\n");
                for (;;) {
                    toggle_red_led();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }
    }

    //position state variables initialization
    DevicePosition lastPosition = POSITION_UNKNOWN;
    DevicePosition currentPosition = POSITION_UNKNOWN;
    TickType_t positionStartTime = 0;
    bool positionStable = false;

    usb_serial_print("IMU sensor task running.\n");

    for (;;) {
        AppState state = getState();

        if (state == STATE_IDLE || state == STATE_DETECTING_POSITION) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                currentPosition = detectPosition(ax, ay, az);  // position detection from accelerometer readings
            } else {
                usb_serial_print("IMU read error\n");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            
            

            if (currentPosition != lastPosition) {
                lastPosition = currentPosition;
                positionStartTime = xTaskGetTickCount();
                positionStable = false;
                setState(STATE_DETECTING_POSITION);
            } else {
                TickType_t elapsed = xTaskGetTickCount() - positionStartTime;
                if (elapsed > pdMS_TO_TICKS(POSITION_STABLE_TIME_MS) && !positionStable) {
                    positionStable = true;

                    MorseSymbol symbol;
                    symbol.timestamp = xTaskGetTickCount();

                    switch (currentPosition) {
                        case POSITION_FLAT:
                            symbol.symbol = '.';
                            usb_serial_print("__Detected: DOT__\n");
                            break;
                        case POSITION_TILTED:
                            symbol.symbol = '-';
                            usb_serial_print("__Detected: DASH__\n");
                            break;
                        case POSITION_VERTICAL:
                            symbol.symbol = ' ';
                            usb_serial_print("__Detected: SPACE__\n");
                            break;
                        default:
                            symbol.symbol = '\0';
                            break;
                    }

                    if (symbol.symbol != '\0') {
                        if (xQueueSend(symbolQueue, &symbol, 0) == pdTRUE) {
                            setState(STATE_SENDING_SYMBOL);
                            toggle_red_led();
                            vTaskDelay(pdMS_TO_TICKS(50));
                            toggle_red_led();
                        } else {
                            usb_serial_print("Symbol queue full, skipping.\n");
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

/**
 * @brief for sending symbols to workstation
 */
static void transmit_task(void *arg) {
    (void)arg;
    MorseSymbol symbol;
    
    for (;;) {
        if (xQueueReceive(symbolQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE) {
            
           
            if (addSymbolToMessage(symbol.symbol))  // message to buffer
            {
                // imp- space between symbols
                //if (symbol.symbol != ' ') {
                   // addSymbolToMessage(' ');
                //}
                //addSymbolToMessage(symbol.symbol);
                
                
                buzzer_play_tone(1000, 100);
                if (usb_serial_connected()) { 
                    usb_serial_print("__Symbol added to message__\n");
                }
            } else {
                if (usb_serial_connected()) { 
                    usb_serial_print("__Message buffer full!__\n");
                }
                buzzer_play_tone(1000, 1000);
            }
            
            setState(STATE_IDLE);
        }
        
        
        if (button2Pressed) // button presse checker for sendingcomplete message
        {
            button2Pressed = false;
            if (usb_serial_connected()) { 
                usb_serial_print("__Sending message...__\n");
            }
            sendMessageToWorkstation();
  
            buzzer_play_tone(1000, 100);
            if (usb_serial_connected()) {
                usb_serial_print("SENT!\n");
            }
            vTaskDelay(pdMS_TO_TICKS(200));

        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief for receiving messages from workstation
 */
static void receive_task(void *arg) {
    (void)arg;
    char rxBuffer[128];
    
    for (;;) {
        // checker for data availablility on CDC 1 
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rxBuffer, sizeof(rxBuffer) - 1);
            if (count > 0) {
                rxBuffer[count] = '\0';
                
                setState(STATE_RECEIVING);
                usb_serial_print("__Received data__\n");
                

                for (uint32_t i = 0; i < count; i++) {
                    char c = rxBuffer[i];
                    
         
                    if (c == '.' || c == '-' || c == ' ') {
    
                        xQueueSend(receiveQueue, &c, 0);
                        
                        
                        if (receivedIndex < MAX_MESSAGE_LENGTH - 1) // push to received message buffer
                        {
                            receivedMessage[receivedIndex++] = c;
                        }
                    }
                    
                    // check for message end - wo spaces or newline
                    if (c == '\n' || 
                        (receivedIndex >= 2 && 
                         receivedMessage[receivedIndex-1] == ' ' && 
                         receivedMessage[receivedIndex-2] == ' ')) {
                        
                        receivedMessage[receivedIndex] = '\0';
                        usb_serial_print("__Complete message received__\n");
                        
                        //complete message notification
                        usb_serial_print("MSG END");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                   
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
 * @brief for displaying received symbols
 */
static void display_task(void *arg) {
    (void)arg;
    char symbol;

    
    for (;;) {
        if (xQueueReceive(receiveQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE) {
            setState(STATE_DISPLAYING_MESSAGE);
            
            if (symbol == '.' || symbol == '-') {
                // Accumulate morse pattern for current letter
                if (patternIndex < sizeof(currentMorsePattern) - 1) {
                    currentMorsePattern[patternIndex++] = symbol;
                }
                
                // Still show the individual symbol
                displayReceivedSymbol(symbol);
                
            } else if (symbol == ' ') {
                // Space = end of letter, translate it
                if (patternIndex > 0) {
                    currentMorsePattern[patternIndex] = '\0';
                    char letter = morse_to_letter(currentMorsePattern);
                    
                    // Add letter to word buffer
                    if (wordIndex < sizeof(translatedWord) - 1) {
                        translatedWord[wordIndex++] = letter;
                    }
                    
                    // Debug: show letter on serial
                    if (usb_serial_connected()) {
                        char debug_msg[32];
                        snprintf(debug_msg, sizeof(debug_msg), "Translated: %c\n", letter);
                        usb_serial_print(debug_msg);
                    }
                    
                    // Reset pattern buffer for next letter
                    patternIndex = 0;
                    memset(currentMorsePattern, 0, sizeof(currentMorsePattern));
                }
                
                // Show space on display
                displayReceivedSymbol(symbol);
                
            } else if (symbol == '\n') {
                // Newline = end of word, display translation!
                translatedWord[wordIndex] = '\0';
                
                if (wordIndex > 0) {
                    // Flash the translated word on LCD
                    clear_display();
                    write_text(translatedWord);
                    buzzer_play_tone(1500, 300);  // Success tone
                    
                    // Debug output
                    if (usb_serial_connected()) {
                        usb_serial_print("==WORD: ");
                        usb_serial_print(translatedWord);
                        usb_serial_print("==\n");
                    }
                    
                    vTaskDelay(pdMS_TO_TICKS(2000));  // Show word for 2 seconds
                }
                
                // Reset word buffer for next word
                wordIndex = 0;
                patternIndex = 0;
                memset(translatedWord, 0, sizeof(translatedWord));
                memset(currentMorsePattern, 0, sizeof(currentMorsePattern));
            } else {
                // Unknown symbol, just display it
                displayReceivedSymbol(symbol);
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
            setState(STATE_IDLE);
        }
    
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
    
    
    while (!tud_mounted() || !tud_cdc_n_connected(0)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }


    if (usb_serial_connected()) {
        usb_serial_print("System READY\n");
    }
    
    buzzer_play_tone(1000, 100);
    
    
    vTaskDelete(NULL);
}
// Exercise 4: Uncomment the following line to activate the TinyUSB library.  
// Tehtävä 4:  Poista seuraavan rivin kommentointi aktivoidaksesi TinyUSB-kirjaston. 

/*
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              
                                 
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
    sleep_ms(200); 




    

    // Exercise 1: Initialize the button and the led and define an register the corresponding interrupton.
    //             Interruption handler is defined up as btn_fxn
    // Tehtävä 1:  Alusta painike ja LEd ja rekisteröi vastaava keskeytys.
    //             Keskeytyskäsittelijä on määritelty yläpuolella nimellä btn_fxn
    

    init_red_led(); 
    init_display();
    clear_display();
    init_rgb_led();
    stop_rgb_led();
    init_button1();
    init_button2();   
    init_buzzer();


    

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

    stateMutex = xSemaphoreCreateMutex();
    
    symbolQueue = xQueueCreate(10, sizeof(MorseSymbol));
    receiveQueue = xQueueCreate(50, sizeof(char));

    if (stateMutex == NULL || symbolQueue == NULL || receiveQueue == NULL) {
        usb_serial_print("MEM-FAIL");
        while(1) {
            gpio_put(LED1, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(LED1, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }


    //clear_display();
    //write_text("READY");
    //buzzer_play_tone(1000, 100);
    //vTaskDelay(pdMS_TO_TICKS(500));
    //clear_display();
  
    TaskHandle_t hStartup, hIMU, hTransmit, hReceive, hDisplay, hUSB = NULL;
    
    BaseType_t result;

    result = xTaskCreate(usb_task, "usb", 1024, NULL, 4, &hUSB);
   
    if (result != pdPASS) {

    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}
    
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    
    result = xTaskCreate(startup_task, "startup", 1536, NULL, 3, &hStartup);


    if (result != pdPASS) {

    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}

   result = xTaskCreate(imu_sensor_task, "imu", 1536, NULL, 2, &hIMU);
    if (result != pdPASS) {
    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}


    result = xTaskCreate(transmit_task, "transmit", 1024, NULL, 2, &hTransmit);

    if (result != pdPASS) {

    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}


    result = xTaskCreate(receive_task, "receive", 1024, NULL, 2, &hReceive);

    if (result != pdPASS) {

    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}


    result = xTaskCreate(display_task, "display", 1024, NULL, 2, &hDisplay);



    if (result != pdPASS) {

    while(1) { gpio_put(LED1, 1); sleep_ms(100); gpio_put(LED1, 0); sleep_ms(100); }
}
    
    // --- create wifi task for Pico W ---
    // use larger stack for cyw43 + lwIP (4KB)
    result = xTaskCreate(wifi_task, "wifi", 4096, NULL, 2, NULL);
    if (result != pdPASS) {
        // WiFi task creation failed.
    }


    


    
    //stdio_init_all();


    //Uncomment this lines if you want to wait till the serial monitor is connected
    //while (!stdio_usb_connected()){
      //  sleep_ms(10);
    //} 
    tusb_init();

    usb_serial_init();



    
    vTaskStartScheduler();

    gpio_put(LED1, 1);
    while(1) { sleep_ms(1000); }
    
    return 0;
}
void tud_cdc_rx_cb(uint8_t itf) {
    if (usb_serial_connected()) {
        usb_serial_print("Data available on CDC ");
        usb_serial_print(itf == 0 ? "0" : "1");
        usb_serial_print("\n");
}
}