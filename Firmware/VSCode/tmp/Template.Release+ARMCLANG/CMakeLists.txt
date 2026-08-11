cmake_minimum_required(VERSION 3.27)

# Roots
include("../roots.cmake")

set(CONTEXT Template.Release+ARMCLANG)
set(TARGET ${CONTEXT})
set(DNAME M258KG6AE)
set(DPACK Nuvoton::NuMicroM23_DFP@1.0.6)
set(DPACK_DIR "${CMSIS_PACK_ROOT}/Nuvoton/NuMicroM23_DFP/1.0.6")
set(OUT_DIR "${SOLUTION_ROOT}/out/Template/ARMCLANG/Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json)
set(COMPILE_COMMANDS ${OUT_DIR}/compile_commands.json)
set(COMPILE_MACROS ${OUT_DIR}/compile_macros.h)
set(BIN_FILE "Template.bin")
set(LD_SCRIPT "${SOLUTION_ROOT}/../../../Library/Device/Nuvoton/M251/Source/ARM/APROM.scf")
set(LD_SCRIPT_PP ${LD_SCRIPT})

# Processor Options
set(CPU Cortex-M23)

# Toolchain config map
set(COMPILER AC6)
include("toolchain.cmake")

# Setup project
project(${CONTEXT} LANGUAGES C)

# Enable color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Compilation database
add_custom_target(database DEPENDS ${COMPILE_COMMANDS} ${COMPILE_MACROS})
add_custom_command(OUTPUT ${COMPILE_COMMANDS}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_COMPILE_COMMANDS}" "${COMPILE_COMMANDS}"
  DEPENDS "${CMAKE_COMPILE_COMMANDS}"
)
add_custom_command(OUTPUT ${COMPILE_MACROS}
  COMMAND ${CPP} ${CPP_DUMP_MACROS}
)
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})

# Setup context
add_executable(${CONTEXT})
set_target_properties(${CONTEXT} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "Template")
set_target_properties(${CONTEXT} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR})
add_library(${CONTEXT}_GLOBAL INTERFACE)

# Includes
target_include_directories(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    "${SOLUTION_ROOT}/../../../Library/CMSIS/Core/Include"
    "${SOLUTION_ROOT}/../../../Library/StdDriver/inc"
    "${SOLUTION_ROOT}/../../../Library/Device/Nuvoton/M251/Include"
  >
  "${CMSIS_PACK_ROOT}/Nuvoton/NuMicroM23_DFP/1.0.6/Device/M251/Include"
)

# Defines
target_compile_definitions(${CONTEXT} PUBLIC
)

# Compile options
target_compile_options(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_CPU}"
    "SHELL:${CC_FLAGS}"
    "SHELL:-std=c11 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -fdata-sections -Wall"
    "SHELL:-D__MICROLIB"
    "SHELL:-Wconditional-uninitialized"
  >
)

# Compile Options Abstractions
add_library(${CONTEXT}_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "balanced" "on" "" "" CC_OPTIONS_FLAGS_${CONTEXT})
target_compile_options(${CONTEXT}_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_${CONTEXT}}"
  >
)

# Add groups and components
include("groups.cmake")
include("components.cmake")

target_link_libraries(${CONTEXT} PUBLIC
  Group_CMSIS
  Group_Library
  Group_User
)

# Linker options
target_link_options(${CONTEXT} PUBLIC
  "SHELL:${LD_CPU}"
  "SHELL:${_LS}\"${LD_SCRIPT_PP}\""
  "SHELL:--library_type=microlib --map --load_addr_map_info --symbols"
  "SHELL:--info sizes --info totals --strict --summary_stderr --info summarysizes"
)
set(LD_DEPS ${LD_SCRIPT})
set_target_properties(${CONTEXT} PROPERTIES LINK_DEPENDS "${LD_DEPS}")

# Bin Conversion
add_custom_command(TARGET ${CONTEXT} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ${ELF2BIN})
