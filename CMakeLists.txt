cmake_minimum_required(VERSION 3.0)

project(firmware)

set(COMMON_FLAGS
                "-Wall -pedantic -Wextra -Winline -mcpu=cortex-m4 \
                 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
                 -fno-common -ffunction-sections -fdata-sections")

set(COMPILE_STD
                "-std=gnu11")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} ${COMPILE_STD}")

set(LD_SCRIPT_PATH "${CMAKE_SOURCE_DIR}/cmsis/ld/stm32f407.ld")
set(SYSTEM_SRC cmsis/startup_stm32f407.c)

set(LINKER_FLAGS "-T${LD_SCRIPT_PATH} -Wl,--gc-sections")
set(LINKER_FLAGS "${LINKER_FLAGS} -Wl,-Map=${PROJECT_NAME}.map")
set(LINKER_FLAGS "${LINKER_FLAGS} --specs=nosys.specs")

add_definitions(
                -DUSE_STDPERIPH_DRIVER
                -DHSE_VALUE=25000000
                -DSTM32F40_41xxx)

include_directories(
                    include
                    cmsis/include
                    cmsis/device
                    freertos/include
                    freertos/portable/GCC/ARM_CM4F
                    stm32f4xx_spd/inc)

set(FREERTOS_SRC
                freertos/portable/GCC/ARM_CM4F/port.c
                freertos/portable/MemMang/heap_1.c
                freertos/list.c
                freertos/queue.c
                freertos/tasks.c
                freertos/timers.c)

set(ST_SRC
          cmsis/device/system_stm32f4xx.c
          stm32f4xx_spd/src/misc.c
          stm32f4xx_spd/src/stm32f4xx_gpio.c
          stm32f4xx_spd/src/stm32f4xx_rcc.c
          stm32f4xx_spd/src/stm32f4xx_tim.c
          stm32f4xx_spd/src/stm32f4xx_spi.c
          stm32f4xx_spd/src/stm32f4xx_syscfg.c
          stm32f4xx_spd/src/stm32f4xx_exti.c)

set(FIRMWARE_SRC
                source/main.c
                source/led.c
                source/delay.c
                source/sx1276_cfg.c)

add_executable(${PROJECT_NAME} ${SYSTEM_SRC} ${ST_SRC} ${FREERTOS_SRC} ${FIRMWARE_SRC})

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${LINKER_FLAGS})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${PROJECT_NAME} -O binary ${PROJECT_NAME}.bin)