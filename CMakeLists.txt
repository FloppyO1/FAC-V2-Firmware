# ---------------------------------------------------------------------------
# F.A.C. V2 firmware - CMake build (STM32Cube for VS Code)
#
# Mirrors the STM32CubeIDE project: every .c and .s under Core, Drivers,
# Middlewares and USB_DEVICE is compiled, which is exactly the source set of
# .cproject. The glob re-runs at every build (CONFIGURE_DEPENDS), so a new .c
# file is picked up without regenerating anything.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)	# gnu11, as in STM32CubeIDE

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME FAC_Firmware_V2)

# Lets clangd (and the extension) see the exact compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})

file(GLOB_RECURSE FAC_SOURCES CONFIGURE_DEPENDS
	"${CMAKE_SOURCE_DIR}/Core/*.c"
	"${CMAKE_SOURCE_DIR}/Core/*.s"
	"${CMAKE_SOURCE_DIR}/Drivers/*.c"
	"${CMAKE_SOURCE_DIR}/Middlewares/*.c"
	"${CMAKE_SOURCE_DIR}/USB_DEVICE/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FAC_SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
	Core/Inc
	Drivers/STM32F0xx_HAL_Driver/Inc
	Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
	Drivers/CMSIS/Device/ST/STM32F0xx/Include
	Drivers/CMSIS/Include
	USB_DEVICE/App
	USB_DEVICE/Target
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
	USE_HAL_DRIVER
	STM32F072xB
	$<$<CONFIG:Debug>:DEBUG>
)

# The linker script is not a source, so a change to it would not relink on its own
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
	LINK_DEPENDS "${CMAKE_SOURCE_DIR}/STM32F072CBTX_FLASH.ld"
)

# .hex and .bin next to the .elf, for STM32CubeProgrammer
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
	COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
