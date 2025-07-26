# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)


# Pull in the CMSIS DSP library from the ARM Github repo, specificaly v1.16.2.
set(CMSISCORE ${USERHOME}/.pico-sdk/sdk/2.1.1/src/rp2_common/cmsis/stub/CMSIS/Core)
set(DISABLEFLOAT16 ON)
include(FetchContent)
FetchContent_Declare(cmsisdsp
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
    GIT_TAG "v1.16.2"
    )
FetchContent_MakeAvailable(cmsisdsp)


project(dsp_audio_filter C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(dsp_audio_filter)

pico_generate_pio_header(dsp_audio_filter ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

target_sources(dsp_audio_filter PRIVATE dsp_audio_filter.c dynamicFilters.c)

pico_set_program_name(dsp_audio_filter "dsp_audio_filter")
pico_set_program_version(dsp_audio_filter "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(dsp_audio_filter 1)
pico_enable_stdio_usb(dsp_audio_filter 0)

# Add the standard library to the build including the CMSIS-DSP library
target_link_libraries(
    dsp_audio_filter
    PRIVATE 
        pico_stdlib
        hardware_clocks
        hardware_adc
        hardware_dma
        hardware_pio
        pico_multicore
        ${cmsisdsp_BINARY_DIR}/Source/libCMSISDSP.a
        )

# Add the standard include files to the build including the CMSIS-DSP header
# files
target_include_directories(dsp_audio_filter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
         ${cmsisdsp_SOURCE_DIR}/Include
         ${USERHOME}/.pico-sdk/sdk/2.1.1/src/rp2_common/cmsis/stub/CMSIS/Core/Include
)

pico_add_extra_outputs(dsp_audio_filter)

