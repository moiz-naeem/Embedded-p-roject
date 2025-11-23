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
    
    // visual feedback wifi task started
    blink_red_led(3);
    

    if (cyw43_arch_init()) {

        while(1) {
           // buzzer_play_tone(200, 100);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    //buzzer_play_tone(1000, 100);


    cyw43_arch_enable_sta_mode();

    int max_retries = 5;
    int retry_count = 0;
    bool connected = false;
    
    while (retry_count < max_retries && !connected) {

        toggle_red_led();
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, 
                                             CYW43_AUTH_WPA2_MIXED_PSK, 10000) == 0) {
            connected = true;

            set_red_led_status(true);
            //buzzer_play_tone(2000, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
            //buzzer_play_tone(2500, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_red_led_status(false);
        } else {
            retry_count++;
            toggle_red_led(); 
            if (retry_count < max_retries) {
                //buzzer_play_tone(500, 100);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }
    
    if (!connected) {
        for (int i = 0; i < 5; i++) {
            //buzzer_play_tone(300, 200);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        cyw43_arch_deinit();
        vTaskDelete(NULL);
        return;
    }

    udp_pcb_handle = udp_new();
    if (udp_pcb_handle == NULL) {
        for (int i = 0; i < 3; i++) {
          //  buzzer_play_tone(400, 150);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }


    if (udp_bind(udp_pcb_handle, IP_ADDR_ANY, UDP_PORT) != ERR_OK) {
        for (int i = 0; i < 4; i++) {
            //buzzer_play_tone(400, 150);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        goto exit;
    }

    udp_recv(udp_pcb_handle, udp_recv_callback, NULL);
    

    //buzzer_play_tone(3000, 300);


    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

exit:
    if (udp_pcb_handle) udp_remove(udp_pcb_handle);
    cyw43_arch_deinit();
    vTaskDelete(NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
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