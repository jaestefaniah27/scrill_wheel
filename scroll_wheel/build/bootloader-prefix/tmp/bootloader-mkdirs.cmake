# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/jaest/esp/v5.2/esp-idf/components/bootloader/subproject"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/tmp"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/src"
  "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/jaest/OneDrive/Documentos/ESP-32 Projects/scroll_wheel/scrill_wheel/scroll_wheel/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
