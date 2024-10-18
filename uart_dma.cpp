#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
// #include "hardware/uart.h"


#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define DEBUG_MODE 1


static int dma_chan;

void setup_uart_dma() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(UART_ID, true));
    dma_channel_configure(dma_chan, &c, &uart_get_hw(UART_ID)->dr, NULL, 0, false);
}

void uart_send_dma(const uint8_t* buffer, size_t length) {
    dma_channel_set_read_addr(dma_chan, buffer, false);
    dma_channel_set_trans_count(dma_chan, length, true);
}

void debug_print(const char* message) {
    if (DEBUG_MODE) {
        uart_puts(UART_ID, message);
        debug_print("Sending data via UART DMA\n");

    }
}

void uart_print(const char* message) {
    uart_puts(UART_ID, message);
}
