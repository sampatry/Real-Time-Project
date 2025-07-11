# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sam/esp/esp-idf/components/bootloader/subproject"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/tmp"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/src/bootloader-stamp"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/src"
  "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sam/esp/projects/Project/20250711/Real-Time-Project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
