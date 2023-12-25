# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/josefvagner/Documents/CRS/rpi_lora/pico-sdk/tools/pioasm"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pioasm"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/tmp"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src"
  "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/josefvagner/Documents/CRS/rpi_lora/build/pico-sdk/src/rp2_common/tinyusb/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
