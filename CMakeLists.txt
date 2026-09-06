cmake_minimum_required(VERSION 3.16)

set(ELF_LOADER_COMPONENT_DIR
    "${CMAKE_CURRENT_LIST_DIR}/components/espressif__elf_loader")
set(EXTRA_COMPONENT_DIRS "${ELF_LOADER_COMPONENT_DIR}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rd03d_radar)

include("${ELF_LOADER_COMPONENT_DIR}/elf_loader.cmake")
project_so(rd03d_radar)
