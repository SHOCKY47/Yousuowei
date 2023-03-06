cmake_minimum_required(VERSION 3.21)
# Target definition
set(CMAKE_SYSTEM_NAME  Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)

# Perform compiler test with static library
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

#---------------------------------------------------------------------------------------
# Set compilers
#---------------------------------------------------------------------------------------
set(CMAKE_C_COMPILER armclang CACHE INTERNAL "C Compiler")
set(CMAKE_CXX_COMPILER armclang CACHE INTERNAL "C++ Compiler")
set(CMAKE_ASM_COMPILER armasm CACHE INTERNAL "ASM Compiler")
set(CMAKE_LINKER armlink)
set(CMAKE_OBJCOPY fromelf)

# Set suffix of binary
set(CMAKE_EXECUTABLE_SUFFIX_C        .axf)
set(CMAKE_EXECUTABLE_SUFFIX_CXX      .axf)
set(CMAKE_C_OUTPUT_EXTENSION         .o)
set(CMAKE_CXX_OUTPUT_EXTENSION       .o)
set(CMAKE_STATIC_LIBRARY_SUFFIX_C    .a)
set(CMAKE_STATIC_LIBRARY_SUFFIX_CXX  .a)

# Project config
project(dualcar C)

set(CMAKE_C_FLAGS "-xc -std=c99 --target=arm-arm-none-eabi -mcpu=cortex-m3 -c -fno-rtti -fsigned-char -fshort-enums  -D__MICROLIB -gdwarf-4 -O0 -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -MD")
set(CMAKE_ASM_FLAGS "--cpu Cortex-M3 -g --pd \"__MICROLIB SETA 1\"")

# Header directories
include_directories("${PROJECT_SOURCE_DIR}/include/cmsis")
include_directories("${PROJECT_SOURCE_DIR}/include/drivers")
include_directories("${PROJECT_SOURCE_DIR}/include/mm32f3277")
include_directories("${PROJECT_SOURCE_DIR}/include/zf_common")
include_directories("${PROJECT_SOURCE_DIR}/include/zf_driver")
include_directories("${PROJECT_SOURCE_DIR}/include/zf_device")
include_directories("${PROJECT_SOURCE_DIR}/include/fatfs")
include_directories("${PROJECT_SOURCE_DIR}/include/rtt")
include_directories("${PROJECT_SOURCE_DIR}/user/inc")

# Source directories
aux_source_directory(${PROJECT_SOURCE_DIR}/user/src PRJ_SRCS)

# Link Library
find_library(Seekfree_Lib seekfree "${PROJECT_SOURCE_DIR}/lib")
find_library(RTT_Lib rtt "${PROJECT_SOURCE_DIR}/lib")

add_link_options(--cpu Cortex-M3 --library_type=microlib)
add_link_options(--scatter ../config/mm32f327x_g9p.sct)
add_link_options(--strict --summary_stderr --map --xref --callgraph --symbols --info=stack,sizes,summarysizes,totals,unused,veneers)


# Output target
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
add_executable(${PROJECT_NAME} ${PRJ_SRCS})
target_link_libraries(${PROJECT_NAME} ${Seekfree_Lib} ${RTT_Lib})

# Optional: Create hex, bin and S-Record files after the build
set(AXF_FILE_NAME ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.axf)
set(HEX_FILE_NAME ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} --i32combined --output ${HEX_FILE_NAME} ${AXF_FILE_NAME}
)