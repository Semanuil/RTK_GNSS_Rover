#ifndef UART_GNSS_H
#define UART_GNSS_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// UART configuration for LC29HDA
#define UART_PORT_NUM      UART_NUM_2
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        17
#define UART_RX_PIN        16
#define UART_RTS_PIN       UART_PIN_NO_CHANGE
#define UART_CTS_PIN       UART_PIN_NO_CHANGE

#define BUF_SIZE           1024

// GNSS data structure
typedef struct {
    double latitude;      // Decimal degrees
    double longitude;     // Decimal degrees
    float altitude;       // Meters above sea level
    float accuracy;       // Horizontal accuracy in meters
    uint8_t satellites;   // Number of satellites
    uint8_t fix_quality;  // 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    uint32_t timestamp;   // System timestamp
} gnss_data_t;

// External queue handles
extern QueueHandle_t rtcm_queue;
extern QueueHandle_t gnss_queue;

// Function prototypes
esp_err_t uart_gnss_init(void);
void uart_gnss_rx_task(void *pvParameters);
void uart_gnss_tx_task(void *pvParameters);
bool parse_gga_sentence(const char* sentence, gnss_data_t* gnss_data);
bool parse_rmc_sentence(const char* sentence, gnss_data_t* gnss_data);

#endif // UART_GNSS_H