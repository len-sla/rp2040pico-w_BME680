#include "pico/runtime.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "lwip/tcp.h"
#include "pico/malloc.h"
#include "bme68x.h"
#include "hardware/rtc.h"
#include "lwip/apps/mqtt.h"
#include "hardware/watchdog.h"
#include <string>
#include <array>
#include "mqtt.hpp"
#include <pico/util/queue.h>
#include "mqtt_config.h"
#include "uart_dma.h"
#include "pico/multicore.h"
#include "pico/malloc.h"


bool is_message_processing_complete();

#define TCP_PORT 80
#define DEBUG_printf printf
#define POLL_TIME_S 5
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define DEBUG_MODE 1



#define HTTP_BUFFER_COUNT 5
#define HTTP_BUFFER_SIZE 1024
#define MQTT_BUFFER_COUNT 5
#define MQTT_BUFFER_SIZE 1024
#define MQTT_MAX_PACKET_SIZE 1024
#define CIRCULAR_BUFFER_SIZE 2048

#define QUEUE_LENGTH 40  // Define the length of the queue
#define MAX_ALLOCATIONS 100 //for mem ccheing






const char WIFI_SSID[] = "yuor id ";
const char WIFI_PASSWORD[] = "your password";

static mqtt_client_t* mqtt_client;
static ip_addr_t mqtt_server_ip;


// ============================= mqtt retry
static bool subscription_successful = false;

static void mqtt_subscribe_with_retry(mqtt_client_t *client);
static void mqtt_retry_cb(void *arg, err_t err);
static void mqtt_request_cb(void *arg, err_t err);




extern mqtt_client_t* mqtt_client;

typedef struct {
    int idx; // The idx from the MQTT message
    char cmd[20];
    char cmddata[100];
} queue_entry_t;




struct bme68x_dev bme;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data;

bool bme68x_initialized = false;

struct SensorData {
    float temperature;
    float pressure;
    float humidity;
    float gas;
    float cpu_temperature;
    int8_t rssi;
    uint64_t uptime;
} sensor_data;

struct TCP_SERVER_T {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
};

struct TCP_CONNECT_STATE_T {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[512];
    int header_len;
    int result_len;
    ip_addr_t *gw;
};

// Memory pools
static uint8_t http_buffers[HTTP_BUFFER_COUNT][HTTP_BUFFER_SIZE];
static bool http_buffer_used[HTTP_BUFFER_COUNT] = {false};
static uint8_t mqtt_buffers[MQTT_BUFFER_COUNT][MQTT_BUFFER_SIZE];
static bool mqtt_buffer_used[MQTT_BUFFER_COUNT] = {false};

// Circular buffer for large messages
std::array<uint8_t, CIRCULAR_BUFFER_SIZE> circular_buffer;
size_t circular_buffer_head = 0;
size_t circular_buffer_tail = 0;

uint8_t* allocate_http_buffer() {
    for (int i = 0; i < HTTP_BUFFER_COUNT; i++) {
        if (!http_buffer_used[i]) {
            http_buffer_used[i] = true;
            return http_buffers[i];
        }
    }
    return nullptr;
}

void uart_print(const char* message);

// ===================================
// #include "pico/malloc.h"
// #define MAX_ALLOCATIONS 100

typedef struct {
    void *ptr;
    size_t size;
} allocation_t;

static allocation_t allocations[MAX_ALLOCATIONS];
static int allocation_count = 0;
static size_t peak_memory_usage = 0;
static size_t current_memory_usage = 0;

void *tracked_malloc(size_t nmemb, size_t size) {
    size_t total_size = nmemb * size;
    void *ptr = malloc(total_size);
    if (ptr && allocation_count < MAX_ALLOCATIONS) {
        memset(ptr, 0, total_size);  // Zero-initialize the memory
        allocations[allocation_count].ptr = ptr;
        allocations[allocation_count].size = total_size;
        allocation_count++;
        current_memory_usage += total_size;
        if (current_memory_usage > peak_memory_usage) {
            peak_memory_usage = current_memory_usage;
        }
    }
    return ptr;
}


void tracked_free(void *ptr) {
    for (int i = 0; i < allocation_count; i++) {
        if (allocations[i].ptr == ptr) {
            current_memory_usage -= allocations[i].size;
            free(ptr);
            allocations[i] = allocations[allocation_count - 1];
            allocation_count--;
            break;
        }
    }
}


;


// ======================================





void free_http_buffer(uint8_t* buffer) {
    for (int i = 0; i < HTTP_BUFFER_COUNT; i++) {
        if (http_buffers[i] == buffer) {
            http_buffer_used[i] = false;
            return;
        }
    }
}

uint8_t* allocate_mqtt_buffer() {
    for (int i = 0; i < MQTT_BUFFER_COUNT; i++) {
        if (!mqtt_buffer_used[i]) {
            mqtt_buffer_used[i] = true;
            return mqtt_buffers[i];
        }
    }
    return nullptr;
}

void free_mqtt_buffer(uint8_t* buffer) {
    for (int i = 0; i < MQTT_BUFFER_COUNT; i++) {
        if (mqtt_buffers[i] == buffer) {
            mqtt_buffer_used[i] = false;
            return;
        }
    }
}

void circular_buffer_push(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        circular_buffer[circular_buffer_head] = data[i];
        circular_buffer_head = (circular_buffer_head + 1) % CIRCULAR_BUFFER_SIZE;
        if (circular_buffer_head == circular_buffer_tail) {
            circular_buffer_tail = (circular_buffer_tail + 1) % CIRCULAR_BUFFER_SIZE;
        }
    }
}

size_t circular_buffer_pop(uint8_t* data, size_t max_length) {
    size_t count = 0;
    while (circular_buffer_tail != circular_buffer_head && count < max_length) {
        data[count++] = circular_buffer[circular_buffer_tail];
        circular_buffer_tail = (circular_buffer_tail + 1) % CIRCULAR_BUFFER_SIZE;
    }
    return count;
}


// queue_t* queue_init();
// bool queue_try_remove(queue_t* queue, queue_entry_t* entry);



queue_t* queue_init() {
    queue_t* queue = new queue_t;
    queue_init(queue, sizeof(queue_entry_t), 10); // Adjust size as needed
    return queue;
}

bool queue_try_remove(queue_t* queue, queue_entry_t* entry) {
    return queue_try_remove(queue, entry);
}


float get_cpu_temperature() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    uint16_t raw = adc_read();
    float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    return 27 - (voltage - 0.706) / 0.001721;
}

uint64_t get_uptime_ms() {
    return to_ms_since_boot(get_absolute_time());
}



static void mqtt_subscribe_with_retry(mqtt_client_t *client) {
    static int retry_count = 0;
    const int max_retries = 5;

    
    if (!subscription_successful) {
        err_t err = mqtt_sub_unsub(client, "pico_w_347e_in", 0, mqtt_request_cb, NULL, 1);
        if (err != ERR_OK) {
            printf("MQTT subscribe request failed, retrying...\n");
            if (retry_count < max_retries) {
                retry_count++;
                mqtt_publish(client, "retry_topic", NULL, 0, 0, 0, mqtt_retry_cb, NULL);
            } else {
                printf("Max retries reached. MQTT subscription failed.\n");
                retry_count = 0;
            }
        } else {
            printf("MQTT subscription initiated\n");
            uart_print("MQTT subscription initiated-new-uart-func\n");
        }
    }
}


static void mqtt_retry_cb(void *arg, err_t err) {
    mqtt_subscribe_with_retry(mqtt_client);
}

void reset_mqtt_callbacks() {
    mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
}


static void mqtt_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) {
        printf("MQTT subscribe request failed\n");
        mqtt_subscribe_with_retry(mqtt_client);
    } else {
        printf("MQTT subscribe request successful\n");
        subscription_successful = true;
        mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL); // Reset the callbacks here
    }
}



void mqtt_publish_sensor_data() {
    if (!mqtt_client_is_connected(mqtt_client)) {
        printf("MQTT client not connected. Skipping publish.\n");
        return;
    }

    char payload[128];  // Increase size if more data is added in the future
    err_t err;

    // Publish temperature
    snprintf(payload, sizeof(payload), "{\"idx\":420,\"nvalue\":0,\"svalue\":\"%.2f\"}", sensor_data.temperature);
    err = mqtt_publish(mqtt_client, "domoticz/in", payload, strlen(payload), 0, 0, NULL, NULL);
    if (err != ERR_OK) {
        printf("Failed to publish temperature data. Error: %d\n", err);
    }
    cyw43_arch_poll();
    sleep_ms(100);  // Sleep to allow network stack time to handle communication

    // Publish pressure
    snprintf(payload, sizeof(payload), "{\"idx\":421,\"nvalue\":0,\"svalue\":\"%.2f\"}", sensor_data.pressure);
    err = mqtt_publish(mqtt_client, "domoticz/in", payload, strlen(payload), 0, 0, NULL, NULL);
    if (err != ERR_OK) {
        printf("Failed to publish pressure data. Error: %d\n", err);
    }
    cyw43_arch_poll();
    sleep_ms(100);

    // Publish humidity
    snprintf(payload, sizeof(payload), "{\"idx\":424,\"nvalue\":0,\"svalue\":\"%.2f\"}", sensor_data.humidity);
    err = mqtt_publish(mqtt_client, "domoticz/in", payload, strlen(payload), 0, 0, NULL, NULL);
    if (err != ERR_OK) {
        printf("Failed to publish humidity data. Error: %d\n", err);
    }
    cyw43_arch_poll();
    sleep_ms(100);

    // Publish gas
    snprintf(payload, sizeof(payload), "{\"idx\":423,\"nvalue\":0,\"svalue\":\"%.2f\"}", sensor_data.gas);
    err = mqtt_publish(mqtt_client, "domoticz/in", payload, strlen(payload), 0, 0, NULL, NULL);
    if (err != ERR_OK) {
        printf("Failed to publish gas data. Error: %d\n", err);
    }
    cyw43_arch_poll();
    sleep_ms(100);
}




void process_mqtt_message(const char* topic, const char* message) {
    printf("Received MQTT message on topic %s: %s\n", topic, message);
    // Add your processing logic here
}

void mqtt_init() {
    ip4addr_aton(MQTT_SERVER_IP, &mqtt_server_ip);

    mqtt_client = mqtt_client_new();
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w_client";
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASSWORD;
    ci.keep_alive = 60;

    mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

    err_t err = mqtt_client_connect(mqtt_client, &mqtt_server_ip, MQTT_SERVER_PORT, mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("MQTT client connect failed: %d\n", err);
    }
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

int8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_write_blocking(I2C_PORT, BME68X_I2C_ADDR_LOW, &reg_addr, 1, true);
    i2c_read_blocking(I2C_PORT, BME68X_I2C_ADDR_LOW, reg_data, len, false);
    return 0;
}

int8_t bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, len);
    i2c_write_blocking(I2C_PORT, BME68X_I2C_ADDR_LOW, buf, len + 1, false);
    return 0;
}


void init_bme68x() {
    bme.read = bme68x_i2c_read;
    bme.write = bme68x_i2c_write;
    bme.intf = BME68X_I2C_INTF;
    bme.delay_us = bme68x_delay_us;
    bme.intf_ptr = NULL;
    bme.amb_temp = 25;

    int8_t rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK) {
        printf("BME68X initialization failed with error code: %d\n", rslt);
        switch (rslt) {
            case BME68X_E_NULL_PTR:
                printf("Error: Null pointer\n");
                break;
            case BME68X_E_COM_FAIL:
                printf("Error: Communication failure\n");
                break;
            case BME68X_E_DEV_NOT_FOUND:
                printf("Error: Device not found\n");
                break;
            case BME68X_E_INVALID_LENGTH:
                printf("Error: Invalid length\n");
                break;
            default:
                printf("Unknown error\n");
        }
        bme68x_initialized = false;
        return;
    }

    printf("BME68X initialization successful\n");

    conf.filter = BME68X_FILTER_SIZE_3;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    rslt = bme68x_set_conf(&conf, &bme);
    if (rslt != BME68X_OK) {
        printf("Failed to set BME68X configuration, error code: %d\n", rslt);
        bme68x_initialized = false;
        return;
    }

    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    if (rslt != BME68X_OK) {
        printf("Failed to set BME68X heater configuration, error code: %d\n", rslt);
        bme68x_initialized = false;
        return;
    }

    bme68x_initialized = true;
    printf("BME68X configuration complete\n");
}



void read_bme68x_sensor() {
    if (!bme68x_initialized) {
        sensor_data.temperature = 0;
        sensor_data.pressure = 0;
        sensor_data.humidity = 0;
        sensor_data.gas = 0;
    } else {
        bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + heatr_conf.heatr_dur * 1000;
        bme.delay_us(del_period, bme.intf_ptr);

        uint8_t n_fields;
        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);

        sensor_data.temperature = data.temperature;
        sensor_data.pressure = data.pressure / 100.0f;
        sensor_data.humidity = data.humidity;
        sensor_data.gas = data.gas_resistance / 1000.0f;
    }

    sensor_data.cpu_temperature = get_cpu_temperature();
    
    int32_t rssi;
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    sensor_data.rssi = rssi;
    
    sensor_data.uptime = get_uptime_ms();
}


static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (state->server_pcb != NULL) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    state->complete = true;
    return ERR_OK;
}


static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        tcp_arg(tpcb, NULL);
        tcp_sent(tpcb, NULL);
        tcp_recv(tpcb, NULL);
        tcp_close(tpcb);
        free(con_state);
    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) {
        tcp_arg(tpcb, NULL);
        tcp_sent(tpcb, NULL);
        tcp_recv(tpcb, NULL);
        tcp_close(tpcb);
        free(con_state);
        return ERR_OK;
    }
    if (p->tot_len > 0) {
        const char http_ok[] = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
        con_state->header_len = sizeof(http_ok) - 1;
        memcpy(con_state->headers, http_ok, con_state->header_len);

        read_bme68x_sensor();

        int len = snprintf(con_state->result, sizeof(con_state->result),
                           "<html><body>\r\n"
                           "<h1>Pico_w_347e   Rev_20</h1>\r\n"
                           "<table border='1'>\r\n"
                           "<tr><th>Parameter</th><th>Value</th></tr>\r\n"
                           "<tr><td>BME680 Initialized</td><td>%s</td></tr>\r\n"
                           "<tr><td>Temperature</td><td>%.2f C</td></tr>\r\n"
                           "<tr><td>Pressure</td><td>%.2f hPa</td></tr>\r\n"
                           "<tr><td>Humidity</td><td>%.2f %%</td></tr>\r\n"
                           "<tr><td>Gas Resistance</td><td>%.2f kOhm</td></tr>\r\n"
                           "<tr><td>CPU Temperature</td><td>%.2f C</td></tr>\r\n"
                           "<tr><td>RSSI</td><td>%d dBm</td></tr>\r\n"
                           "<tr><td>Uptime</td><td>%llu ms</td></tr>\r\n"
                           "</table>\r\n"
                           "</body></html>\r\n",
                           bme68x_initialized ? "Yes" : "No",
                           sensor_data.temperature, sensor_data.pressure, sensor_data.humidity, sensor_data.gas,
                           sensor_data.cpu_temperature, sensor_data.rssi, sensor_data.uptime);
        con_state->result_len = len;

        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    tcp_arg(tpcb, con_state);
    tcp_sent(tpcb, tcp_server_sent);
    con_state->sent_len = 0;

    int retry_count = 0;
    const int max_retries = 5;
    while (retry_count < max_retries) {
        err_t err2 = tcp_write(tpcb, con_state->headers, con_state->header_len, TCP_WRITE_FLAG_COPY);
        if (err2 == ERR_OK) {
            err2 = tcp_write(tpcb, con_state->result, con_state->result_len, TCP_WRITE_FLAG_COPY);
            if (err2 == ERR_OK) {
                err2 = tcp_output(tpcb);
                if (err2 == ERR_OK) {
                    return ERR_OK;
                }
            }
        }
        retry_count++;
        sleep_ms(100);
    }

    DEBUG_printf("Failed to write data after %d retries\n", max_retries);
    return tcp_server_close(con_state->gw);
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\n");
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)tracked_malloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        DEBUG_printf("Failed to allocate connect state\n");
        return ERR_MEM;
    }
    con_state->pcb = client_pcb;
    con_state->gw = &state->gw;

    tcp_arg(client_pcb, con_state);
    tcp_recv(client_pcb, tcp_server_recv);

    return ERR_OK;
}


static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("Starting server on port %u\n", TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("Failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) {
        DEBUG_printf("Failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("Failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}




void initialize_context() {
    app_context.pQueue = (queue_t*)tracked_malloc(1, sizeof(queue_t));  // Allocate memory for the queue
    if (app_context.pQueue == NULL) {
        printf("Failed to allocate memory for the queue\n");
        exit(1);
    }
    
    queue_init(app_context.pQueue, sizeof(queue_entry_t), QUEUE_LENGTH);  // Initialize the queue
}

void message_processing_task();

int main() {
    stdio_init_all();
    watchdog_enable(8000, 1);
    // multicore_launch_core1(message_processing_task); // tu sobie przerzucilem

    setup_uart_dma();  // Initialize UART DMA

    if (cyw43_arch_init()) {
        printf("Failed to initialize\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        ip_addr_t ip = cyw43_state.netif[CYW43_ITF_STA].ip_addr;
        printf("IP Address: %s\n", ip4addr_ntoa(&ip));
        printf("Connect to http://%s:%d\n", ip4addr_ntoa(&ip), TCP_PORT);
    }

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    init_bme68x();

    TCP_SERVER_T *state = (TCP_SERVER_T*)tracked_malloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return 1;
    }

    if (!tcp_server_open(state)) {
        DEBUG_printf("failed to open server\n");
        return 1;
    }

    initialize_context();
    mqtt_setContext(&app_context);
    mqtt_init();

    int mqtt_connect_attempts = 0;
    while (!mqtt_client_is_connected(mqtt_client) && mqtt_connect_attempts < 5) {
        printf("Waiting for MQTT connection...\n");
        sleep_ms(1000);
        cyw43_arch_poll();
        mqtt_connect_attempts++;
    }

    if (!mqtt_client_is_connected(mqtt_client)) {
        printf("Failed to connect to MQTT server after 5 attempts\n");
    } else {
        printf("MQTT client connected and ready\n");
    }

    multicore_launch_core1(message_processing_task); // tak bylo orginalnie

    printf(" \nline719 Current memory usage: %zu bytes\n", current_memory_usage);
    printf(" line720- Peak memory usage: %zu bytes\n", peak_memory_usage);

    

    while (!state->complete) {
        watchdog_update();
        cyw43_arch_poll();
        read_bme68x_sensor();

        if (mqtt_client_is_connected(mqtt_client)) {
            mqtt_publish_sensor_data();
        } else {
            printf("MQTT client not connected. Skipping publish.\n");
        }

        // Use UART DMA for sending data==============not working====================
        // char buffer[256];
        // snprintf(buffer, sizeof(buffer), "Sensor Data: Temp=%.2f, Pressure=%.2f, Humidity=%.2f\n", 
        //          sensor_data.temperature, sensor_data.pressure, sensor_data.humidity);
        // uart_send_dma((const uint8_t*)buffer, strlen(buffer));
        // =================here it breaks and I see four SSSS====================

        // printf("\n line 741 Current memory usage: %zu bytes\n", current_memory_usage);
        // printf(" line 742 Peak memory usage: %zu bytes\n", peak_memory_usage);

        // Check if message processing is complete or queue is empty
        if (is_message_processing_complete()) {
            printf("line 45 aafter message processing Current memory usage: %zu bytes\n", current_memory_usage);
            // printf("A123456 Doing nothing but now could handle other tasks....\n");
            // sleep_ms(1500);

        }

        printf("\n line 753 Current memory usage: %zu bytes\n", current_memory_usage);
        printf(" line 754 Peak memory usage: %zu bytes\n", peak_memory_usage);


        // Updated sleep with watchdog updates
        for (int i = 0; i < POLL_TIME_S; i++) {
            sleep_ms(1000);  // Sleep for 1 second
            watchdog_update();  // Update the watchdog
        }
    }

    tracked_free(state);
    tracked_free(app_context.pQueue);
    mqtt_client_free(mqtt_client);
    cyw43_arch_deinit();
    return 0;
}
