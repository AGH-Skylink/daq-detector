cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only the components that main actually requires (not every ESP-IDF
# component). Besides being faster, this skips esp_lcd, whose RGB panel driver
# crashes GCC 15.2 (internal compiler error at -Og) on IDF v6.1.
idf_build_set_property(MINIMAL_BUILD ON)

project(daq-detector)
