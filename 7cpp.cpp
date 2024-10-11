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

#include <string>

#define TCP_PORT 80
#define DEBUG_printf printf
#define POLL_TIME_S 5
#define I2C_PORT i2c0
// #define I2C_SDA 4
// #define I2C_SCL 5
#define I2C_SDA 8
#define I2C_SCL 9

const char WIFI_SSID[] = "your wifi id";
const char WIFI_PASSWORD[] = "#your password";

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

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

int8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_write_blocking(I2C_PORT, BME68X_I2C_ADDR_LOW, &reg_addr, 1, true); // it works with sBME 680 connected to ground
    // i2c_write_blocking(I2C_PORT, BME68X_I2C_ADDR_HIGH, &reg_addr, 1, true);
    i2c_read_blocking(I2C_PORT, BME68X_I2C_ADDR_LOW, reg_data, len, false); // 
    // i2c_read_blocking(I2C_PORT, BME68X_I2C_ADDR_HIGH, reg_data, len, false);
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
        const char http_ok[] = "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n";
        con_state->header_len = sizeof(http_ok) - 1;
        memcpy(con_state->headers, http_ok, con_state->header_len);

        read_bme68x_sensor();

        int len = snprintf(con_state->result, sizeof(con_state->result),
                           "<html><body>\r\n"
                           "<h1>Raspberry Pi Pico W Sensor Data</h1>\r\n"
                           "<p>BME680 Initialized: %s</p>\r\n"
                           "<p>Temperature: %.2f C</p>\r\n"
                           "<p>Pressure: %.2f hPa</p>\r\n"
                           "<p>Humidity: %.2f %%</p>\r\n"
                           "<p>Gas Resistance: %.2f kOhm</p>\r\n"
                           "<p>CPU Temperature: %.2f C</p>\r\n"
                           "<p>RSSI: %d dBm</p>\r\n"
                           "<p>Uptime: %llu ms</p>\r\n"
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
    err_t err2 = tcp_write(tpcb, con_state->headers, con_state->header_len, TCP_WRITE_FLAG_COPY);
    if (err2 != ERR_OK) {
        DEBUG_printf("Failed to write header data %d\n", err);
        return tcp_server_close(con_state->gw);
    }
    err2 = tcp_write(tpcb, con_state->result, con_state->result_len, TCP_WRITE_FLAG_COPY);
    if (err2 != ERR_OK) {
        DEBUG_printf("Failed to write result data %d\n", err);
        return tcp_server_close(con_state->gw);
    }
    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\n");
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)calloc(1, sizeof(TCP_CONNECT_STATE_T));
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


int main() {
    stdio_init_all();

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
    // printf("I2C Initialization: %s\n", i2c_init_success ? "Success" : "Failed");
    
    

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // printf("SDA GPIO function: %d\n", gpio_get_function(I2C_SDA));
    // printf("SCL GPIO function: %d\n", gpio_get_function(I2C_SCL));

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    printf("SCL GPIO function: %d\n", gpio_get_function(I2C_SCL));
    printf("SDA GPIO function: %d\n", gpio_get_function(I2C_SDA));
    printf("SCL Pull-up: %s\n", gpio_is_pulled_up(I2C_SCL) ? "Enabled" : "Disabled");
    printf("SDA Pull-up: %s\n", gpio_is_pulled_up(I2C_SDA) ? "Enabled" : "Disabled");

    init_bme68x();
    // int8_t rslt = bme68x_init(&bme);
    // if (rslt != BME68X_OK) {
    //     printf("BME68X initialization failed with error code: %d\n", rslt);
    //     bme68x_initialized = false;
    //     return;
    // }



    TCP_SERVER_T *state = (TCP_SERVER_T*)calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return 1;
    }

    if (!tcp_server_open(state)) {
        DEBUG_printf("failed to open server\n");
        return 1;
    }

    while (!state->complete) {
        cyw43_arch_poll();
        sleep_ms(10);
    }

    free(state);
    cyw43_arch_deinit();
    return 0;
}
