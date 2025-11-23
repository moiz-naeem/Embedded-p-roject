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
#include "pico/cyw43_arch.h"
#include "usbSerialDebug/helper.h"
#include "morse_translator.h"
#include "wifi.h"

// Exercise 4. Include the libraries necessaries to use the usb-serial-debug, and tinyusb
// Tehtävä 4 . Lisää usb-serial-debugin ja tinyusbin käyttämiseen tarvittavat kirjastot.

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1
#define MAX_MESSAGE_LENGTH 256


typedef enum
{
    STATE_IDLE,
    STATE_DETECTING_POSITION,
    STATE_SENDING_SYMBOL,
    STATE_TRANSMITTING_MESSAGE,
    STATE_RECEIVING,
    STATE_DISPLAYING_MESSAGE,
    STATE_TRANSLATING_DISPLAY,
    STATE_ERROR
} AppState;

typedef enum
{
    MODE_SENDING,
    MODE_RECEIVING,
    MODE_IDLE
} OperationMode;

OperationMode currentMode = MODE_IDLE;
SemaphoreHandle_t modeMutex = NULL;

typedef enum
{
    POSITION_FLAT,
    POSITION_TILTED,
    POSITION_VERTICAL,
    POSITION_UNKNOWN
} DevicePosition;

typedef struct
{
    char symbol;
    uint32_t timestamp;
} MorseSymbol;

static AppState currentState = STATE_IDLE;
static SemaphoreHandle_t stateMutex = NULL;

QueueHandle_t symbolQueue = NULL;
QueueHandle_t receiveQueue = NULL;

static char outgoingMessage[MAX_MESSAGE_LENGTH];
static char receivedMessage[MAX_MESSAGE_LENGTH];
static uint16_t messageIndex = 0;
static uint16_t receivedIndex = 0;

static char currentMorsePattern[10];
static uint8_t patternIndex = 0;
static char translatedWord[32];
static uint8_t wordIndex = 0;

#define FLAT_THRESHOLD_Z_MIN 0.8f
#define FLAT_THRESHOLD_Z_MAX 1.2f
#define TILTED_THRESHOLD_X 0.6f
#define VERTICAL_THRESHOLD_Y 0.8f
#define POSITION_STABLE_TIME_MS 300

/**
 * @brief safely change the application state with mutex protection
 */
static void setState(AppState newState)
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

/**
 * @brief get current application state with mutex protection
 * returns STATE_IDLE if mutex acquisition fails
 */
static AppState getState(void)
{
    AppState state = STATE_IDLE;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

/**
 * @brief safely change operation mode with mutex protection
 */
static void setMode(OperationMode newMode)
{
    if (xSemaphoreTake(modeMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        currentMode = newMode;
        xSemaphoreGive(modeMutex);
    }
}

/**
 * @brief get current operation mode with mutex protection
 * returns MODE_IDLE if mutex acquisition fails
 */
static OperationMode getMode(void)
{
    OperationMode mode = MODE_IDLE;
    if (xSemaphoreTake(modeMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        mode = currentMode;
        xSemaphoreGive(modeMutex);
    }
    return mode;
}

/**
 * @brief check if its safeto detect IMU positions
 * prevents IMU operation during receiving to avoid I2C conflicts
 */
static bool canDetectIMU(void)
{
    OperationMode mode = getMode();
    AppState state = getState();

    if (mode == MODE_RECEIVING || state == STATE_RECEIVING || 
        state == STATE_DISPLAYING_MESSAGE || state == STATE_TRANSLATING_DISPLAY)
    {
        return false;
    }

    return (mode == MODE_SENDING || mode == MODE_IDLE) &&
           (state == STATE_IDLE || state == STATE_DETECTING_POSITION);
}

/**
 * @brief clear all sending buffets when receivng interrupts sending
 * prevents mixing of messages
 */
static void clearSendingBuffers(void)
{
    messageIndex = 0;
    memset(outgoingMessage, 0, MAX_MESSAGE_LENGTH);

    MorseSymbol dummy;
    while (xQueueReceive(symbolQueue, &dummy, 0) == pdTRUE)
    {
    }
    if (usb_serial_connected())
    {
        usb_serial_print("__Sending buffers cleared__\n");
    }
}

/**
 * @brief detect device positoon based on IMU accelerometer data
 */
static DevicePosition detectPosition(float ax, float ay, float az)
{
    if (az > FLAT_THRESHOLD_Z_MIN && az < FLAT_THRESHOLD_Z_MAX &&
        fabs(ax) < 0.3f && fabs(ay) < 0.3f)
    {
        return POSITION_FLAT;
    }

    if (fabs(ax) > TILTED_THRESHOLD_X)
    {
        return POSITION_TILTED;
    }

    if (fabs(ay) > VERTICAL_THRESHOLD_Y)
    {
        return POSITION_VERTICAL;
    }

    return POSITION_UNKNOWN;
}

/**
 * @brief add symbol to outgoing message buffer
 */
static bool addSymbolToMessage(char symbol)
{
    if (messageIndex >= MAX_MESSAGE_LENGTH - 3)
    {
        return false;
    }
    outgoingMessage[messageIndex++] = symbol;
    return true;
}

/**
 * @brief translate outgoing morde message to plain text
 */
static void translateOutgoingMessage(const char *morse_msg, char *translated, uint16_t max_len)
{
    char pattern[10];
    uint8_t pattern_idx = 0;
    uint16_t trans_idx = 0;

    for (int i = 0; morse_msg[i] != '\0' && trans_idx < max_len - 1; i++)
    {
        char c = morse_msg[i];

        if (c == '.' || c == '-')
        {
            if (pattern_idx < sizeof(pattern) - 1)
            {
                pattern[pattern_idx++] = c;
            }
        }
        else if (c == ' ')
        {
            if (pattern_idx > 0)
            {
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
 * @brief display received symbol on LCD and play audio feedback

static void displayReceivedSymbol(char symbol)
{
    char displayBuffer[32];

    setState(STATE_DISPLAYING_MESSAGE);

    if (symbol == '.')
    {
        snprintf(displayBuffer, sizeof(displayBuffer), "DOT");
        clear_display();
        write_text(displayBuffer);
        //buzzer_play_tone(660, 100);
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_red_led_status(false);
    }
    else if (symbol == '-')
    {
        snprintf(displayBuffer, sizeof(displayBuffer), "DASH");
        clear_display();
        write_text(displayBuffer);
        //buzzer_play_tone(660, 300);
        set_red_led_status(true);
        vTaskDelay(pdMS_TO_TICKS(300));
        set_red_led_status(false);
    }
    else if (symbol == ' ')
    {
        snprintf(displayBuffer, sizeof(displayBuffer), "SPACE");
        clear_display();
        write_text(displayBuffer);
        //buzzer_play_tone(660, 150);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    else
    {
        snprintf(displayBuffer, sizeof(displayBuffer), "???");
        clear_display();
        write_text(displayBuffer);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    clear_display();
}
 */
// BUTTON INTERRUPT HANDLER
static volatile bool button1Pressed = false;
static volatile bool button2Pressed = false;

static void btn_fxn(uint gpio, uint32_t eventMask)
{
    if (gpio == BUTTON1)
    {
        button1Pressed = true;
    }
    else if (gpio == BUTTON2)
    {
        button2Pressed = true;
    }
}

static void play_usb_receive_tone(void)
{
    buzzer_play_tone(523, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(659, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(784, 150);
}

static void play_wifi_receive_tone(void)
{
    buzzer_play_tone(880, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(1047, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(1319, 150);
}

/**
 * @brief task for detecting device position and generating morse symbols
 */
static void imu_sensor_task(void *arg)
{
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;

    DevicePosition lastPosition = POSITION_UNKNOWN;
    DevicePosition currentPosition = POSITION_UNKNOWN;
    TickType_t positionStartTime = 0;
    bool positionStable = false;

    usb_serial_print("IMU sensor task running.\n");

    for (;;)
    {
        if (!canDetectIMU())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        AppState state = getState();

        if (state == STATE_IDLE || state == STATE_DETECTING_POSITION)
        {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0)
            {
                currentPosition = detectPosition(ax, ay, az);
            }
            else
            {
                usb_serial_print("IMU read error\n");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (currentPosition != lastPosition)
            {
                lastPosition = currentPosition;
                positionStartTime = xTaskGetTickCount();
                positionStable = false;
                setState(STATE_DETECTING_POSITION);
            }
            else
            {
                TickType_t elapsed = xTaskGetTickCount() - positionStartTime;
                if (elapsed > pdMS_TO_TICKS(POSITION_STABLE_TIME_MS) && !positionStable)
                {
                    positionStable = true;

                    MorseSymbol symbol;
                    symbol.timestamp = xTaskGetTickCount();

                    switch (currentPosition)
                    {
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

                    if (symbol.symbol != '\0')
                    {
                        if (xQueueSend(symbolQueue, &symbol, 0) == pdTRUE)
                        {
                            setState(STATE_SENDING_SYMBOL);

                            set_red_led_status(true);
                            //buzzer_play_tone(800, 50);
                            vTaskDelay(pdMS_TO_TICKS(50));
                            set_red_led_status(false);

                            lastPosition = POSITION_UNKNOWN;
                            currentPosition = POSITION_UNKNOWN;
                        }
                        else
                        {
                            usb_serial_print("Symbol queue full, skipping.\n");
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/**
 * @brief task for buffeting symbols and sending messages
 */
static void transmit_task(void *arg)
{
    (void)arg;
    MorseSymbol symbol;

    for (;;)
    {
        if (getMode() == MODE_RECEIVING)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (xQueueReceive(symbolQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (addSymbolToMessage(symbol.symbol))
            {
                //buzzer_play_tone(1000, 100);
                if (usb_serial_connected())
                {
                    usb_serial_print("__Symbol added to message__\n");
                }
            }
            else
            {
                if (usb_serial_connected())
                {
                    usb_serial_print("__Message buffer full!__\n");
                }
                //buzzer_play_tone(1000, 1000);

                clearSendingBuffers();
            }

            setState(STATE_IDLE);
        }

        if (button1Pressed)
        {
            button1Pressed = false;

            setState(STATE_TRANSMITTING_MESSAGE);

            if (usb_serial_connected())
            {
                usb_serial_print("__Sending message via WiFi...__\n");
            }

            if (messageIndex > 0)
            {
                outgoingMessage[messageIndex++] = ' ';
                outgoingMessage[messageIndex++] = ' ';
                outgoingMessage[messageIndex++] = '\n';
                outgoingMessage[messageIndex] = '\0';

                char translatedMsg[32];
                translateOutgoingMessage(outgoingMessage, translatedMsg, sizeof(translatedMsg));

                if (strlen(translatedMsg) > 0)
                {
                     buzzer_play_tone(392, 200);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(523, 200);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(659, 200);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(784, 400);
                    if (usb_serial_connected())
                    {
                        usb_serial_print(">>WIFI SENDING: ");
                        usb_serial_print(translatedMsg);
                        usb_serial_print("<<\n");
                    }
                }

                if (wifi_send_message(outgoingMessage))
                {
                    usb_serial_print("__Message sent via WiFi__\n");
                    //buzzer_play_tone(7000, 200);
                    blink_red_led(3);
                }
                else
                {
                    usb_serial_print("__WiFi send failed__\n");
                    //buzzer_play_tone(300, 500);
                }

                messageIndex = 0;
                memset(outgoingMessage, 0, MAX_MESSAGE_LENGTH);
            }

            setState(STATE_IDLE);

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (button2Pressed)
        {
            button2Pressed = false;

            setState(STATE_TRANSMITTING_MESSAGE);

            if (usb_serial_connected())
            {
                usb_serial_print("__Sending message via USB...__\n");
            }

            if (messageIndex > 0)
            {
                outgoingMessage[messageIndex++] = ' ';
                outgoingMessage[messageIndex++] = ' ';
                outgoingMessage[messageIndex++] = '\n';
                outgoingMessage[messageIndex] = '\0';

                char translatedMsg[32];
                translateOutgoingMessage(outgoingMessage, translatedMsg, sizeof(translatedMsg));

                if (strlen(translatedMsg) > 0)
                {
                    buzzer_play_tone(659, 150);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(659, 150);
                    vTaskDelay(pdMS_TO_TICKS(100));

                    buzzer_play_tone(784, 150);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(698, 150);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    buzzer_play_tone(659, 300);

                    if (usb_serial_connected())
                    {
                        usb_serial_print(">>USB SENDING: ");
                        usb_serial_print(translatedMsg);
                        usb_serial_print("<<\n");
                    }
                }

                if (tud_cdc_n_connected(CDC_ITF_TX))
                {
                    tud_cdc_n_write_str(CDC_ITF_TX, outgoingMessage);
                    tud_cdc_n_write_flush(CDC_ITF_TX);

                    blink_red_led(2);
                    //buzzer_play_tone(6000, 200);

                    usb_serial_print("__Message sent via USB__\n");
                }
                else
                {
                    usb_serial_print("__USB not connected__\n");
                    //buzzer_play_tone(300, 500);
                }

                messageIndex = 0;
                memset(outgoingMessage, 0, MAX_MESSAGE_LENGTH);
            }

            setState(STATE_IDLE);

            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief task for receiving messages from workstation vis usb cdc
 * automaticaly interrupts sending mode when message arrives
 */
static void receive_task(void *arg)
{
    (void)arg;
    char rxBuffer[128];

    for (;;)
    {
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX))
        {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rxBuffer, sizeof(rxBuffer) - 1);
            if (count > 0)
            {
                rxBuffer[count] = '\0';

                clearSendingBuffers();

                setMode(MODE_RECEIVING);
                setState(STATE_RECEIVING);

                if (usb_serial_connected())
                {
                    usb_serial_print("__Receiving data via USB__\n");
                }

                for (uint32_t i = 0; i < count; i++)
                {
                    char c = rxBuffer[i];

                    if (c == '.' || c == '-' || c == ' ')
                    {
                        if (xQueueSend(receiveQueue, &c, 0) != pdTRUE)
                        {
                            usb_serial_print("Receive queue full!\n");
                            break;
                        }

                        if (receivedIndex < MAX_MESSAGE_LENGTH - 1)
                        {
                            receivedMessage[receivedIndex++] = c;
                        }
                    }

                    if (c == '\n' ||
                        (receivedIndex >= 2 &&
                         receivedMessage[receivedIndex - 1] == ' ' &&
                         receivedMessage[receivedIndex - 2] == ' '))
                    {
                        receivedMessage[receivedIndex] = '\0';
                        
                        if (usb_serial_connected())
                        {
                            usb_serial_print("__Complete message received via USB__\n");
                        }

                        char source_marker = 'U';
                        xQueueSend(receiveQueue, &source_marker, 0);
                        
                        char newline = '\n';
                        xQueueSend(receiveQueue, &newline, 0);

                        vTaskDelay(pdMS_TO_TICKS(100));

                        receivedIndex = 0;
                        memset(receivedMessage, 0, MAX_MESSAGE_LENGTH);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
/**
 * @brief task for displaying received symbols and translating morse
 * shows individua symbols then displays translated word for
 */

static void display_task(void *arg)
{
    (void)arg;
    char symbol;
    char message_source = 'U';

    for (;;)
    {
        if (xQueueReceive(receiveQueue, &symbol, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            setState(STATE_DISPLAYING_MESSAGE);

            if (symbol == 'U' || symbol == 'W')
            {
                message_source = symbol;
                continue;
            }

            if (symbol == '.' || symbol == '-')
            {
                if (patternIndex < sizeof(currentMorsePattern) - 1)
                {
                    currentMorsePattern[patternIndex++] = symbol;
                }
                
                if (usb_serial_connected())
                {
                    usb_serial_print(symbol == '.' ? "." : "-");
                }
            }
            else if (symbol == ' ')
            {
                if (patternIndex > 0)
                {
                    currentMorsePattern[patternIndex] = '\0';
                    char letter = morse_to_letter(currentMorsePattern);

                    if (wordIndex < sizeof(translatedWord) - 1)
                    {
                        translatedWord[wordIndex++] = letter;
                    }

                    if (usb_serial_connected())
                    {
                        char debug_msg[32];
                        snprintf(debug_msg, sizeof(debug_msg), " [%c] ", letter);
                        usb_serial_print(debug_msg);
                    }

                    patternIndex = 0;
                    memset(currentMorsePattern, 0, sizeof(currentMorsePattern));
                }
            }
            else if (symbol == '\n')
            {
                translatedWord[wordIndex] = '\0';

                if (wordIndex > 0)
                {
                    setState(STATE_TRANSLATING_DISPLAY);

                    if (message_source == 'W')
                    {
                        play_wifi_receive_tone();
                    }
                    else
                    {
                        play_usb_receive_tone();
                    }

                    clear_display();
                    write_text(translatedWord);

                    if (usb_serial_connected())
                    {
                        usb_serial_print("\n==WORD: ");
                        usb_serial_print(translatedWord);
                        if (message_source == 'W')
                        {
                            usb_serial_print(" (WiFi)");
                        }
                        else
                        {
                            usb_serial_print(" (USB)");
                        }
                        usb_serial_print("==\n");
                    }

                    vTaskDelay(pdMS_TO_TICKS(2000));
                    clear_display();
                }

                wordIndex = 0;
                patternIndex = 0;
                memset(translatedWord, 0, sizeof(translatedWord));
                memset(currentMorsePattern, 0, sizeof(currentMorsePattern));
                message_source = 'U';

                setState(STATE_IDLE);
                setMode(MODE_IDLE);
            }

            setState(STATE_IDLE);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void usb_task(void *arg)
{
    (void)arg;

    while (1)
    {
        tud_task();
    }
}

/**
 * @brief startup task for system initialization
 */
static void startup_task(void *arg)
{
    (void)arg;

    while (!tud_mounted() || !tud_cdc_n_connected(0))
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (usb_serial_connected())
    {
        usb_serial_print("System READY\n");
    }

    //buzzer_play_tone(1000, 100);

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

int main()
{
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
    init_veml6030();

    init_ICM42670();
    sleep_ms(200);
    ICM42670_start_with_default_values();
    sleep_ms(200);
    cyw43_arch_init();
    sleep_ms(200);


     
    

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

    stateMutex = xSemaphoreCreateMutex();
    modeMutex = xSemaphoreCreateMutex();

    symbolQueue = xQueueCreate(10, sizeof(MorseSymbol));
    receiveQueue = xQueueCreate(50, sizeof(char));

    if (stateMutex == NULL || modeMutex == NULL || symbolQueue == NULL || receiveQueue == NULL)
    {
        usb_serial_print("MEM-FAIL");
        while (1)
        {
            gpio_put(LED1, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(LED1, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    TaskHandle_t hStartup, hIMU, hTransmit, hReceive, hDisplay, hUSB = NULL;

    BaseType_t result;

    result = xTaskCreate(usb_task, "usb", 1024, NULL, 4, &hUSB);

    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

    result = xTaskCreate(startup_task, "startup", 1536, NULL, 3, &hStartup);

    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    result = xTaskCreate(imu_sensor_task, "imu", 1536, NULL, 2, &hIMU);
    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    result = xTaskCreate(transmit_task, "transmit", 1024, NULL, 2, &hTransmit);

    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    result = xTaskCreate(receive_task, "receive", 1024, NULL, 2, &hReceive);

    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    result = xTaskCreate(display_task, "display", 1024, NULL, 2, &hDisplay);

    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }

    result = xTaskCreate(wifi_task, "wifi", 4096, NULL, 2, NULL);
    if (result != pdPASS)
    {
        while (1)
        {
            gpio_put(LED1, 1);
            sleep_ms(100);
            gpio_put(LED1, 0);
            sleep_ms(100);
        }
    }
    tusb_init();
    usb_serial_init();

    vTaskStartScheduler();

    gpio_put(LED1, 1);
    while (1)
    {
        sleep_ms(1000);
    }

    return 0;
}

/**
 * @brief alled when data is available on cdc interface
 */
void tud_cdc_rx_cb(uint8_t itf)
{
}