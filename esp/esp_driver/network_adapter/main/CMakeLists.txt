set(COMPONENT_SRCS "app_main.c" "slave_bt.c" "cmd.c" "stats.c")

if(CONFIG_ESP_HOSTED_RCP)
    list(APPEND COMPONENT_SRCS "esp_rcp.c")
    # esp_rcp.c uses esp_random() to bind each session to one firmware boot.
    list(APPEND COMPONENT_REQUIRES esp_hw_support)
    if(CONFIG_ESP_HOSTED_RCP_OPENTHREAD)
        list(APPEND COMPONENT_SRCS "esp_rcp_openthread.c")
        # esp_rcp_openthread.c directly includes esp_coexist.h and
        # esp_vfs_eventfd.h; OpenThread keeps both as private dependencies.
        list(APPEND COMPONENT_REQUIRES openthread esp_coex vfs)
    endif()
endif()

if(CONFIG_XWAVE_MESH)
    # xWave: ESP-WIFI-MESH mode with the ESP's own TCP/IP stack (docs/WiFi-Plan.md 7.1)
    list(APPEND COMPONENT_SRCS "xwave_mesh.c" "xwave_mesh_stubs.c")
    list(APPEND COMPONENT_REQUIRES esp_netif lwip esp_event esp_wifi app_update)
endif()

if(CONFIG_ESP_THREAD_RCP_UART)
    list(APPEND COMPONENT_SRCS "thread_rcp_uart.c")
    list(APPEND COMPONENT_REQUIRES openthread esp_coex vfs esp_event)
endif()
set(COMPONENT_ADD_INCLUDEDIRS "./include")

if(CONFIG_ESP_SDIO_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS sdio_slave_api.c)
elseif(CONFIG_ESP_SPI_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_slave_api.c)
elseif(CONFIG_ESP_SPI_HD_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS spi_hd_slave_api.c)
elseif(CONFIG_ESP_USB_HOST_INTERFACE)
    list(APPEND COMPONENT_SRCS usb/esp_usb_device.c)
endif()


register_component()

# Hosted firmware calls private 802.11 MLME/AP/mgmt APIs provided by the
# project-owned Wi-Fi archives in esp/esp_driver/lib, NOT the stock ESP-IDF
# prebuilt archives. Directly bind all ESP-IDF imported Wi-Fi archive targets
# to that immutable source-tree copy: a concurrent CI job may reset or replace
# a shared ESP-IDF checkout's components/esp_wifi/lib directory at any time.
# Never silently link the stock blobs or replace missing APIs with stubs.
idf_build_get_property(_hosted_wifi_target IDF_TARGET)
if(CONFIG_ESP_HOST_WIFI_ENABLED)
    set(_hosted_wifi_target "${CONFIG_ESP_WIFI_CONTROLLER_TARGET}_host")
endif()
get_filename_component(_hosted_wifi_lib_dir
    "${CMAKE_CURRENT_LIST_DIR}/../../lib/${_hosted_wifi_target}" ABSOLUTE)
# xWave: the mesh image links ESP-IDF's stock archives instead (the ones the
# submodule holds after `git submodule update --force`, see xwave-build.ps1):
# only they run the station/AP authentication and build the AP beacons inside
# the ESP; the hosted archives wait for the host to do both, so a mesh root
# linked with them never beacons and no node can find it (2026-11-05).
if(CONFIG_XWAVE_MESH_IMAGE)
    set(_hosted_wifi_blobs)
    message(STATUS "xWave mesh image: ESP-IDF's stock Wi-Fi archives")
else()
    set(_hosted_wifi_blobs core espnow mesh net80211 pp smartconfig wapi target)
endif()
foreach(_blob ${_hosted_wifi_blobs})
    if(TARGET esp_wifi_${_blob})
        set(_archive "${_hosted_wifi_lib_dir}/lib${_blob}.a")
        if(NOT EXISTS "${_archive}")
            message(FATAL_ERROR "ESP-Hosted Wi-Fi archive missing: ${_archive}")
        endif()
        set_property(TARGET esp_wifi_${_blob} PROPERTY IMPORTED_LOCATION "${_archive}")
        message(STATUS "ESP-Hosted Wi-Fi archive pinned: ${_blob} -> ${_archive}")
    endif()
endforeach()
if(CONFIG_ESP_WIFI_ENABLED AND NOT TARGET esp_wifi_net80211)
    message(FATAL_ERROR "ESP-IDF Wi-Fi net80211 imported target unavailable")
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE)

# Add directory of protocomm_priv.h to include paths
idf_component_get_property(protocomm_dir protocomm COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${protocomm_dir}/src/common")

# Redirect IDF's esp_supplicant_init/deinit (called unconditionally by
# esp_wifi's wifi_init.c) to the __wrap_ no-ops in cmd.c. This lets
# esp_wpa_main.o link for the RSN/WPA IE parser (registered as
# wpa_cb.wpa_parse_wpa_ie in cmd.c) while --gc-sections drops the rest of
# the supplicant state machine from the final image.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=esp_supplicant_init"
    "-Wl,--wrap=esp_supplicant_deinit")

# xWave: in the mesh image the Wi-Fi AP must not forward a plain station's
# frames to the other stations (a plain group-addressed frame on the mesh AP
# stops the mesh children); the library's forwarding call is taken over by
# __wrap_ieee80211_output in xwave_mesh.c, which explains it. And a device that
# admits a phone keeps its beacon interval at 100 ms (the mesh stack's
# stretching of it drops a plain station): __wrap_esp_wifi_set_beacon_interval.
if(CONFIG_XWAVE_MESH_IMAGE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=ieee80211_output"
        "-Wl,--wrap=esp_wifi_set_beacon_interval")
endif()
