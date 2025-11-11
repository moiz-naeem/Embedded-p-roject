/**
 * @file wifi.c
 * @brief WiFi connectivity implementation for Pico W morse code messenger
 * 
 * Implements WiFi connection management and UDP-based message transmission
 * 
 * Uses lwIP in NO_SYS=1 mode (bare-metal with background threading)
 * provided by pico_cyw43_arch_lwip_threadsafe_background library.
 */

#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"  // For LED control

// External queues from main.c
extern QueueHandle_t receiveQueue;
extern QueueHandle_t symbolQueue;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PSK  "YOUR_WIFI_PASSWORD"
#define UDP_PORT  50000

static struct udp_pcb *udp_pcb_handle = NULL;

/**
 * @brief lwIP UDP receive callback for incoming morse messages
 * @param arg User argument (unused)
 * @param pcb UDP protocol control block
 * @param p Received packet buffer
 * @param addr Source IP address of sender
 * @param port Source UDP port of sender
 * 
 * Filters incoming UDP packets for valid morse characters:
 * - '.' (dot)
 * - '-' (dash)
 * - ' ' (space - letter separator)
 * - '\n' (newline - word terminator)
 * 
 * Filtered characters are forwarded to the receive queue for
 * display and translation by the display_task.
 */
static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        char *data = (char *)p->payload;
        for (int i = 0; i < p->len; i++) {
            char c = data[i];
            if (c == '.' || c == '-' || c == ' ' || c == '\n') {
                if (receiveQueue) {
                    xQueueSend(receiveQueue, &c, 0);
                }
            }
        }
        pbuf_free(p);
    }
}

void wifi_task(void *pvParams) {
    (void)pvParams;

    // Short delay for system to be ready
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Visual feedback: WiFi task started
    blink_red_led(3);  // 3 quick blinks = WiFi task starting
    
    // Initialize CYW43439
    if (cyw43_arch_init()) {
        // Init failed - continuous fast beeping
        while(1) {
            buzzer_play_tone(200, 100);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    
    // Wireless initialized - short beep
    buzzer_play_tone(1000, 100);

    // Enable station mode
    cyw43_arch_enable_sta_mode();
    
    // Try to connect with retries
    int max_retries = 5;
    int retry_count = 0;
    bool connected = false;
    
    while (retry_count < max_retries && !connected) {
        // Blink LED during connection attempt
        toggle_red_led();
        
        // Connect to WiFi (10s timeout per attempt) - using WPA2_MIXED for WPA2/WPA3 compatibility
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, 
                                             CYW43_AUTH_WPA2_MIXED_PSK, 10000) == 0) {
            connected = true;
            // Success: solid LED for 1 second + success beeps
            set_red_led_status(true);
            buzzer_play_tone(2000, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
            buzzer_play_tone(2500, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_red_led_status(false);
        } else {
            retry_count++;
            toggle_red_led();  // Turn off LED
            if (retry_count < max_retries) {
                // Retry beep (low tone)
                buzzer_play_tone(500, 100);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }
    
    if (!connected) {
        // Failed after all retries - 5 error beeps
        for (int i = 0; i < 5; i++) {
            buzzer_play_tone(300, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    // Create UDP PCB
    udp_pcb_handle = udp_new();
    if (udp_pcb_handle == NULL) {
        // UDP creation failed - 3 error beeps
        for (int i = 0; i < 3; i++) {
            buzzer_play_tone(400, 150);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }

    // Bind to port
    if (udp_bind(udp_pcb_handle, IP_ADDR_ANY, UDP_PORT) != ERR_OK) {
        // Bind failed - 4 error beeps
        for (int i = 0; i < 4; i++) {
            buzzer_play_tone(400, 150);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }

    // Set receive callback
    udp_recv(udp_pcb_handle, udp_recv_callback, NULL);
    
    // UDP server ready - final success tone (high pitch)
    buzzer_play_tone(3000, 300);

    // Keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

exit:
    if (udp_pcb_handle) udp_remove(udp_pcb_handle);
    cyw43_arch_deinit();
    vTaskDelete(NULL);
}

// Helper to send morse messages
bool wifi_send_message(const char *msg) {
    if (udp_pcb_handle == NULL) return false;

    // Broadcast address
    ip_addr_t dest_addr;
    IP4_ADDR(&dest_addr, 255, 255, 255, 255);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg), PBUF_RAM);
    if (p == NULL) return false;

    memcpy(p->payload, msg, strlen(msg));
    err_t err = udp_sendto(udp_pcb_handle, p, &dest_addr, UDP_PORT);
    pbuf_free(p);

    return err == ERR_OK;
}
