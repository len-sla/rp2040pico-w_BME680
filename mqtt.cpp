#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "mqtt.hpp"
// #include <cstring>
#include "pico/cyw43_arch.h"
#include "mqtt_config.h"
#include "hardware/watchdog.h"




#define QUEUE_SIZE 20
#define THROTTLE_INTERVAL_MS 100

static context_t *pContext;


typedef struct {
    int idx; // The idx from the MQTT message
    char cmd[20];
    char cmddata[100];
} queue_entry_t;

typedef struct {
    queue_entry_t entries[QUEUE_SIZE];
    int head;
    int tail;
    int count;
} circular_queue_t;

circular_queue_t mqtt_queue;
bool circular_queue_is_empty(circular_queue_t* q);


void circular_queue_init(circular_queue_t* q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

bool is_message_processing_complete() {
    return circular_queue_is_empty(&mqtt_queue);
}

bool circular_queue_is_empty(circular_queue_t* q) {
    return q->count == 0;
}


bool circular_queue_push(circular_queue_t* q, queue_entry_t* entry) {
    if (q->count == QUEUE_SIZE) {
        // Overwrite oldest entry
        q->tail = (q->tail + 1) % QUEUE_SIZE;
    } else {
        q->count++;
    }
    memcpy(&q->entries[q->head], entry, sizeof(queue_entry_t));
    q->head = (q->head + 1) % QUEUE_SIZE;
    return true;
}

bool circular_queue_pop(circular_queue_t* q, queue_entry_t* entry) {
    printf("mqtt.cpp-lDEBUG: Attempting to pop from circular queue\n");
    if (q->count == 0) {
        printf("mqtt.cpp-lDEBUG: Queue is empty\n");
        return false;
    }
    memcpy(entry, &q->entries[q->tail], sizeof(queue_entry_t));
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count--;
    printf("mqtt.cpp-lDEBUG: Pop successful. New tail: %d, count: %d\n", q->tail, q->count);
    return true;
}
// ====================koniec do

void log_queue_stats() {
    printf("mqtt.cpp-lQueue stats: Count: %d, Head: %d, Tail: %d\n", 
           mqtt_queue.count, mqtt_queue.head, mqtt_queue.tail);
}

// Call this function periodically or when adding/removing from the queue
 
void message_processing_task() {
    queue_entry_t entry;
    const int MAX_EMPTY_CHECKS = 3;
    int empty_check_count = 0;

    while (true) {
        printf("mqtt.cpp-line82 DEBUG: Checking queue for messages\n");
        if (circular_queue_pop(&mqtt_queue, &entry)) {
            printf("DEBUG: Processing message. Command: %s, Data: %s\n", entry.cmd, entry.cmddata);
            
            // Add your message processing logic here
            // For example:
            // if (strcmp(entry.cmd, "set_led") == 0) {
            //     // Handle LED control
            // } else if (strcmp(entry.cmd, "get_sensor") == 0) {
            //     // Handle sensor data request
            // }
            
            printf("mqtt.cpp-lDEBUG: Finished processing message\n");
            log_queue_stats();
            empty_check_count = 0;  // Reset the counter when a message is processed
        } else {
            empty_check_count++;
            if (empty_check_count >= MAX_EMPTY_CHECKS) {
                printf("mqtt.cpp-lDEBUG: Queue empty for %d checks, moving to next task\n", MAX_EMPTY_CHECKS);
                // Add code here to move to the next task or step
                // For example, you could call a function to handle other periodic tasks
                // handle_periodic_tasks();
                empty_check_count = 0;  // Reset the counter
                sleep_ms(1000);  // Wait for a second before checking again
            }
        }
        watchdog_update();  // Update the watchdog timer
        sleep_ms(10);
    }
}



void mqtt_setContext(context_t *pCtx){
  pContext = pCtx;
}

void mqtt_sub_request_cb(void* arg, err_t result) {
  printf("mqtt.cpp-lSubscribe result: %d\n", result);
}

void mqtt_connect(mqtt_client_t* client) {
  struct mqtt_connect_client_info_t ci;
  err_t err;

  memset(&ci, 0, sizeof(ci));
  ci.client_id = "pico_w_client";
  ci.client_user = MQTT_USER;
  ci.client_pass = MQTT_PASSWORD;
  ci.keep_alive = 60;

  ip_addr_t mqtt_ip;
  ip4addr_aton(MQTT_SERVER_IP, &mqtt_ip);

  cyw43_arch_lwip_begin();
  err = mqtt_client_connect(client, &mqtt_ip, MQTT_SERVER_PORT, mqtt_connection_cb, 0, &ci);
  cyw43_arch_lwip_end();

  if (err != ERR_OK) {
    printf("mqtt.cpp-l mqtt_connect return %d\n", err);
  }
}


// =====================old

static absolute_time_t last_message_time;

void mqtt_incoming_data_cb(void* arg, const u8_t* data, u16_t len, u8_t flags) {
    printf("mqtt.cpp-DEBUG: Entering mqtt_incoming_data_cb. Data length: %d, Flags: %u\n", len, (unsigned int)flags);
    if (absolute_time_diff_us(last_message_time, get_absolute_time()) < THROTTLE_INTERVAL_MS * 1000) {
        return; // Throttle messages
    }
    last_message_time = get_absolute_time();

    printf("mqtt.cpp-Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);
    
    if (flags & MQTT_DATA_FLAG_LAST) {
        printf("mqtt.cpp-DEBUG: Processing last fragment of MQTT message\n");
        queue_entry_t entry = {};
        char request[100];
        
        if (len > 99) {
            len = 99;  // Avoid buffer overflow
        }
        
        strncpy(request, (const char*)data, len);
        request[len] = '\0';  // Null terminate

        printf("[mqtt] Received data: %s\n", request);
        
        int parsed_items = sscanf(request, "%19[^:]:%99s", entry.cmd, entry.cmddata);
        if (parsed_items != 2) {
            printf("[mqtt] Failed to parse request\n");
            return;
        }

        printf("[mqtt] %s %s\n", entry.cmd, entry.cmddata);
        printf("[mqtt] %p\n", (void*)&mqtt_queue);

        if (circular_queue_push(&mqtt_queue, &entry)) {
            printf("[mqtt] Added to queue\n");
        } else {
            printf("[mqtt] Failed to add to queue\n");
        }

        log_queue_stats();
    }
}




void mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status) {
  if (status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);
    err_t err = mqtt_subscribe(client, "pico_w_347e_in", 0, mqtt_sub_request_cb, arg);
    if (err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
    mqtt_connect(client);
  }
}

void mqtt_incoming_publish_cb(void* arg, const char* topic, u32_t tot_len) {
  printf("mqtt.cpp-DEBUG: -Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);
}
