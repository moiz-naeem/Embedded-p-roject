#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"

//from main.c for received characters
extern QueueHandle_t receiveQueue;

#define WIFI_SSID "DNA-WIFI-3E5A"
#define WIFI_PSK  "80885139441"
#define UDP_PORT  50000

static struct udp_pcb *udp_pcb_handle = NULL;

/**
 * 
 * filters incoming udp packets for valid morse characters
 * implements character limit to prevent queue overflow
 * handles queue full condition gracefully
 */
static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        if (receiveQueue != NULL) {
            char *data = (char *)p->payload;
            
            usb_serial_print("__Received data via WiFi__\n");
            

            int processed = 0;
            const int MAX_CHARS = 30;  
            
            for (int i = 0; i < p->len && processed < MAX_CHARS; i++) {
                char c = data[i];
                if (c == '.' || c == '-' || c == ' ' || c == '\n') {

                    if (xQueueSend(receiveQueue, &c, 0) == pdTRUE) {
                        processed++;
                    } else {
                        usb_serial_print("Queue full, dropping WiFi data\n");
                        break;  
                    }
                }
            }
            
            char source_marker = 'W';
            xQueueSend(receiveQueue, &source_marker, 0);
        }
        pbuf_free(p);
    }
}

/**
 * @brief
 * initializes cyw43 wifi chip
 * connects to wifi network with retry logic
 * creatrs udp server for receiving morse messages
 */
void wifi_task(void *pvParams) {
    (void)pvParams;

    vTaskDelay(pdMS_TO_TICKS(3000));
    
    blink_red_led(3);
    
    usb_serial_print("==WiFi task started==\n");

    cyw43_arch_enable_sta_mode();
    vTaskDelay(pdMS_TO_TICKS(100));
    int max_retries = 5;
    int retry_count = 0;
    bool connected = false;
    
    usb_serial_print("Connecting to WiFi: ");
    usb_serial_print(WIFI_SSID);
    usb_serial_print("\n");
    
    while (retry_count < max_retries && !connected) {
        char attempt_msg[32];
        snprintf(attempt_msg, sizeof(attempt_msg), "Attempt %d/%d...\n", retry_count + 1, max_retries);
        usb_serial_print(attempt_msg);

        toggle_red_led();
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, 
                                             CYW43_AUTH_WPA2_MIXED_PSK, 10000) == 0) {
            connected = true;

            set_red_led_status(true);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_red_led_status(false);
        } else {
            retry_count++;
            toggle_red_led();
            usb_serial_print("Connection failed, retrying...\n");
            if (retry_count < max_retries) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }
    
    if (connected) {
        usb_serial_print("==WiFi CONNECTED!==\n");
        usb_serial_print("IP Address: ");
        usb_serial_print(ip4addr_ntoa(netif_ip4_addr(netif_list)));
        usb_serial_print("\n");
        usb_serial_print("Gateway: ");
        usb_serial_print(ip4addr_ntoa(netif_ip4_gw(netif_list)));
        usb_serial_print("\n");
        
        blink_red_led(5);
    } else {
        usb_serial_print("==WiFi connection FAILED after all retries!==\n");
        for (int i = 0; i < 5; i++) {
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    usb_serial_print("Creating UDP socket...\n");
    udp_pcb_handle = udp_new();
    if (udp_pcb_handle == NULL) {
        usb_serial_print("==UDP socket creation FAILED!==\n");
        for (int i = 0; i < 3; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }

    usb_serial_print("Binding to port 50000...\n");
    if (udp_bind(udp_pcb_handle, IP_ADDR_ANY, UDP_PORT) != ERR_OK) {
        usb_serial_print("==UDP bind FAILED!==\n");
        for (int i = 0; i < 4; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }

    udp_recv(udp_pcb_handle, udp_recv_callback, NULL);
    
    usb_serial_print("==UDP server ready on port 50000==\n");
    usb_serial_print("==System ready for WiFi communication!==\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

exit:
    if (udp_pcb_handle) udp_remove(udp_pcb_handle);
    cyw43_arch_deinit();
    vTaskDelete(NULL);
}

/**
 * broadcasts message to all devices on network 
 */
bool wifi_send_message(const char *msg) {
    if (udp_pcb_handle == NULL) return false;

    ip_addr_t dest_addr;
    IP4_ADDR(&dest_addr, 255, 255, 255, 255);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg), PBUF_RAM);
    if (p == NULL) return false;

    memcpy(p->payload, msg, strlen(msg));
    
    err_t err = udp_sendto(udp_pcb_handle, p, &dest_addr, UDP_PORT);
    
    pbuf_free(p);

    return err == ERR_OK;
}


