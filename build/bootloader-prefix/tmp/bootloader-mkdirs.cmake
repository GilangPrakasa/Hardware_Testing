# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/eFishery/esp/v4.4.5/esp-idf/components/bootloader/subproject"
  "D:/@CODE/QC_FIRMWARE/build/bootloader"
  "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix"
  "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix/tmp"
  "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix/src/bootloader-stamp"
  "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix/src"
  "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/@CODE/QC_FIRMWARE/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
