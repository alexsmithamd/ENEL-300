# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/marcus/esp/esp-idf-v5.5.3/components/bootloader/subproject"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/tmp"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/src/bootloader-stamp"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/src"
  "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/media/marcus/BCF853CFF8538714/Users/Marcus Meyer/Desktop/enel_300/ENEL-300/controller_board/esp_code/esp_now_controller/espnow/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
