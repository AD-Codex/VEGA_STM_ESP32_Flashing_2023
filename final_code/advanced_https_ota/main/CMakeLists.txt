
idf_component_register(SRCS  "advanced_https_ota_example2.cpp" "ping.c" "components/lora/include/lora.c"
                    "ble_helper/ble_api.c" "led_strip_rmt_ws2812.c"
                    INCLUDE_DIRS "." "./ble_helper/include/"
                    INCLUDE_DIRS "." "./components/lora/include"
                    # Embed the server root certificate into the final binary
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert_1.pem
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert_2.pem)
                    #EMBED_TXTFILES ${project_dir}/ca_cert_1.h)
#)

#target_include_directories(advanced_https_ota PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/main)