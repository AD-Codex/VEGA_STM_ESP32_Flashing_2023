#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <mbedtls/base64.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#define STORAGE_NAMESPACE "storage"
	static const char *TAG_F = "flash";
	typedef struct
	{
		char SSID_key[32];
		char SSID[32];
		char password[32];
		char password_key[32];
	} flash_t;

	char STA_SSID[32];
	char STA_PASSWORD[64];

	uint8_t setup_flag = 1;
	uint8_t isConfigState = 1;
	uint8_t isChargerLocked = 0;

	esp_err_t save_key_value(char *key, char *value)
	{
		nvs_handle_t my_handle;
		esp_err_t err;

		// Open
		err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
			return err;

		// Write
		err = nvs_set_str(my_handle, key, value);
		if (err != ESP_OK)
			return err;

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		err = nvs_commit(my_handle);
		if (err != ESP_OK)
			return err;

		// Close
		nvs_close(my_handle);
		return ESP_OK;
	}

	esp_err_t load_key_value(char *key, char *value, size_t size)
	{
		nvs_handle_t my_handle;
		esp_err_t err;

		// Open
		err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
			return err;

		// Read
		size_t _size = size;
		err = nvs_get_str(my_handle, key, value, &_size);
		ESP_LOGI(TAG_F, "nvs_get_str err=%d", err);
		// if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
		if (err != ESP_OK)
			return err;
		ESP_LOGI(TAG_F, "err=%d key=[%s] value=[%s] _size=%d", err, key, value, _size);

		// Close
		nvs_close(my_handle);
		// return ESP_OK;
		return err;
	}

	esp_err_t load_int_value(char *key, uint8_t *value)
	{
		nvs_handle_t my_handle;
		esp_err_t err;

		// Open
		err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
			return err;

		// Read

		err = nvs_get_u8(my_handle, key, value);
		ESP_LOGI(TAG_F, "nvs_get_str err=%d", err);
		// if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
		if (err != ESP_OK)
			return err;
		ESP_LOGI(TAG_F, "err=%d key=[%s] value=%u", err, key, *value);

		// Close
		nvs_close(my_handle);
		// return ESP_OK;
		return err;
	}

	esp_err_t save_int_value(char *key, uint8_t value)
	{
		nvs_handle_t my_handle;
		esp_err_t err;

		// Open
		err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
		if (err != ESP_OK)
			return err;

		// Write
		err = nvs_set_u8(my_handle, key, value);
		if (err != ESP_OK)
			return err;

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		err = nvs_commit(my_handle);
		if (err != ESP_OK)
			return err;

		// Close
		nvs_close(my_handle);
		return ESP_OK;
	}
	esp_err_t save_long_to_eeprom(char *key,  uint32_t value)
	{
		nvs_handle_t handle;
		esp_err_t err;
		 err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
		if (err != ESP_OK)
		{
			// Handle error
			return err;
		}
		err = nvs_set_u32(handle,key, value);
		if (err != ESP_OK)
		{
			// Handle error
			nvs_close(handle);
			return err;
		}
		err = nvs_commit(handle);
		if (err != ESP_OK)
		{
			return err;// Handle error
		}
		nvs_close(handle);
		return ESP_OK;
	}

	esp_err_t read_long_from_eeprom(char *key, uint32_t *value)
	{
		nvs_handle_t handle;
		esp_err_t err;
		 err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
		if (err != ESP_OK)
		{
			// Handle error
			return err;
		}
		err = nvs_get_u32(handle, key, value);
		if (err != ESP_OK)
		{
			// Handle error
			return err;
		}
		nvs_close(handle);
		return err;
	}
#ifdef __cplusplus
}
#endif

#endif /*HTTP_SERVER_H*/