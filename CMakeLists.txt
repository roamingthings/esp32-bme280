PROJECT(esp32-bme280 C CXX ASM)
set(CMAKE_VERBOSE_MAKEFILE on)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
ENABLE_LANGUAGE(ASM)

set(CMAKE_VERBOSE_MAKEFILE on)

SET(IDF_PATH $ENV{IDF_PATH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

file(GLOB_RECURSE USER_SOURCES "main/*.c" "main/*.cpp" "components/*.cpp" "components/*.c")

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${USER_SOURCES} ${PROJECT_SOURCES} ${CMSIS_SOURCES} ${STM32HAL_SOURCES})

INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMSIS_INCLUDE_DIRS}
        ${STM32HAL_INCLUDE_DIR}
        components/cpp_utils
        components/bme280
        "/usr/include"
        "/opt/xtensa-esp32-elf/xtensa-esp32-elf/include/c++/5.2.0"
        "/opt/xtensa-esp32-elf/lib/gcc/xtensa-esp32-elf/5.2.0/include"
        "${IDF_PATH}"
        "${IDF_PATH}/components/esp32/include"
        "${IDF_PATH}/components/newlib/include"
        "${IDF_PATH}/components/freertos/include"
        "${IDF_PATH}/components/nvs_flash/include"
        "${IDF_PATH}/components/driver/include"
        "${IDF_PATH}/components/log/include"
        "${IDF_PATH}/components/tcpip_adapter/include"
        "${IDF_PATH}/components/lwip/include/lwip"
        "${IDF_PATH}/components/vfs/include"
        "${IDF_PATH}/components/spi_flash/include"
        "${IDF_PATH}/components/mbedtls/include"
        "${IDF_PATH}/components/mdns/include"
        "${IDF_PATH}/components/json/include"
        "${IDF_PATH}/components/bt/include"
        "${IDF_PATH}/components/bt/bluedroid/bta/include"
        "${IDF_PATH}/components/bt/bluedroid/bta/sys/include"
        "${IDF_PATH}/components/bt/bluedroid/btcore/include"
        "${IDF_PATH}/components/bt/bluedroid/device/include"
        "${IDF_PATH}/components/bt/bluedroid/gki/include"
        "${IDF_PATH}/components/bt/bluedroid/hci/include"
        "${IDF_PATH}/components/bt/bluedroid/osi/include"
        "${IDF_PATH}/components/bt/bluedroid/btc/profile/esp/blufi/include"
        "${IDF_PATH}/components/bt/bluedroid/btc/profile/esp/include"
        "${IDF_PATH}/components/bt/bluedroid/btc/profile/std/sdp/include"
        "${IDF_PATH}/components/bt/bluedroid/btc/profile/std/include"
        "${IDF_PATH}/components/bt/bluedroid/stack/include"
        "${IDF_PATH}/components/bt/bluedroid/stack/l2cap/include"
        "${IDF_PATH}/components/bt/bluedroid/stack/sdp/include"
        "${IDF_PATH}/components/bt/bluedroid/stack/smp/include"
        "${IDF_PATH}/components/bt/bluedroid/api/include"
        "${IDF_PATH}/components/bt/bluedroid/btc/include"
        "${IDF_PATH}/components/soc/esp32/include"
        "${IDF_PATH}/components/aws_iot/include"
        "${IDF_PATH}/components/aws_iot/aws-iot-device-sdk-embedded-C/include"
        "${IDF_PATH}/components/fatfs/src"
        "${IDF_PATH}/components/wear_levelling/include"
)

add_custom_target(esp32-bme280 COMMAND make -C ${esp32-bme280_SOURCE_DIR}
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})
