cmake_minimum_required(VERSION 3.27)
project(STM32-Template C CXX ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Tool path
set(TOOLROOT /l/arm/codesourcery/bin)


# Library path
set(LIBROOT /l/arm/STM32F10x_StdPeriph_Lib_V3.5.0)

# Tools
set(CMAKE_C_COMPILER ${TOOLROOT}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLROOT}/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${TOOLROOT}/arm-none-eabi-as)

set(CHIP "stm32f100")

set(DEVICE ${LIBROOT}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x)
set(CORE ${LIBROOT}/Libraries/CMSIS/CM3/CoreSupport)
set(PERIPH ${LIBROOT}/Libraries/stlib/STM32F10x_StdPeriph_Driver)

# This one is needed if we have to use a module like gpio so we add it to
# list of files to compile like this:
# ${PERIPH_SRC}/stm32f10x_gpio.c
set(PERIPH_SRC ${LIBROOT}/Libraries/stlib/STM32F10x_StdPeriph_Driver/src)


# Processor specific details
set(PTYPE STM32F10X_MD_VL)
add_definitions(-D${PTYPE})
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/stm32f100.ld)
set(PTYPE_STARTUP startup_stm32f10x.c)

include_directories(${DEVICE})
include_directories(${CORE})
include_directories(${PERIPH}/inc)
include_directories(${CMAKE_SOURCE_DIR}/inc/) #this is for the file stm32f10x_conf.h
# this one comes from the template provided by ST

# Include paths for library
include_directories(${CMAKE_SOURCE_DIR}/Library)
include_directories(${CMAKE_SOURCE_DIR}/Library/ff9/src)
include_directories(${CMAKE_SOURCE_DIR}/Library/ff9/src/option)
include_directories(${CMAKE_SOURCE_DIR}/src/uart/)


# We have to add the stm32 .c files we use manually as opposed to it being added
# Better for clarity, more laborious upfront :)
set(DISC_FILES
        lib/${PTYPE_STARTUP} #auto generated but available in this template
        ${DEVICE}/system_stm32f10x.c
)


# Compilation flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m3 -mthumb")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mthumb -mcpu=cortex-m3")
# Without these options it does not compile
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nano.specs -specs=nosys.specs")

# Defs
add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DUSE_FULL_ASSERT)

# Add our demos as a target for compilation
add_subdirectory(Demo)

