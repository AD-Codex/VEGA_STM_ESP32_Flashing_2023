#ifndef __ping_h__
#define __ping_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "ping/ping_sock.h"
// uint8_t loss_count;
bool is_internet_available;
SemaphoreHandle_t internetMutex;

esp_ping_config_t ping_config;
ip_addr_t target_addr;
// void cmd_ping_on_ping_success(esp_ping_handle_t hdl, void *args);
// void cmd_ping_on_ping_timeout(esp_ping_handle_t hdl, void *args);
void cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *args);
esp_err_t initialize_ping(uint32_t interval_ms, uint32_t task_prio);

#ifdef __cplusplus
}
#endif

#endif /* __ping_h__ */
