static const char *TAG_SPIFFS = "SPIFFS";

static void initialize_spiffs()
{
    ESP_LOGI(TAG_SPIFFS, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SPIFFS, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG_SPIFFS, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG_SPIFFS, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPIFFS, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_SPIFFS, "Partition size: total: %d, used: %d", total, used);
    }
}

static void write_bytes(char* path, uint8_t* data, int size)
{
    FILE* f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG_SPIFFS, "Failed to open file for writing");
        return;
    }
    fwrite(data , 1 , size , f);
    fclose(f);
    ESP_LOGI(TAG_SPIFFS, "File written");
}