#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

/**
 * @file wifi.h
 * @brief WiFi connectivity and UDP communication for Pico W
 * 
 * This module provides WiFi connection management and UDP-based
 * morse code message transmission for the Raspberry Pi Pico W.
 */

/**
 * @brief Initialize WiFi and start UDP server task
 * 
 * This FreeRTOS task handles:
 * - WiFi chip initialization (CYW43439)
 * - Connection to configured network with retries
 * - UDP server creation and binding
 * - Receiving morse code messages via UDP
 * 
 * The task provides audio/visual feedback during initialization:
 * - 3 LED blinks: WiFi task starting
 * - LED toggles: Connection attempts
 * - Solid LED + 2 beeps: Successfully connected
 * - High-pitched beep: UDP server ready
 * 
 * @param pvParams FreeRTOS task parameter (unused)
 * 
 * @note This task runs indefinitely once started
 * @note WiFi credentials are hardcoded in wifi.c (WIFI_SSID, WIFI_PSK)
 */
void wifi_task(void *pvParams);

/**
 * @brief Send a morse code message via WiFi UDP broadcast
 * 
 * Broadcasts a morse code message to all devices on the network
 * using UDP on port 50000. The message should be formatted as
 * dots (.), dashes (-), spaces ( ), and terminated with newline.
 * 
 * @param msg Null-terminated string containing morse code message
 *            Example: "... --- ...  \n" for SOS
 * 
 * @return true if message was sent successfully
 * @return false if UDP is not initialized or send failed
 * 
 * @note Message is broadcast to 255.255.255.255 (all devices on subnet)
 * @note Maximum message length depends on UDP packet size limits
 * 
 * @example
 *   if (wifi_send_message(".- -...  \n")) {
 *       // Message sent successfully
 *   }
 */
bool wifi_send_message(const char *msg);

#endif // WIFI_H
