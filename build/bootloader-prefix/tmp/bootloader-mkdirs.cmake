# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Lenovo/esp/v5.1.4/esp-idf/components/bootloader/subproject"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/tmp"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/src/bootloader-stamp"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/src"
  "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/workspace/VS_esp32/esp32c3/esp32c3/SUCESS/success/ESP32C3_ESPNow/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
