set(BOARD_NAME "dm-h723")
set(MCU_FAMILY "STM32H7xx")
set(MCU_MODEL "STM32H723xx")

# MCU-specific definitions
add_definitions(
    -DUSE_HAL_DRIVER
    -D${MCU_MODEL}
    -DUSE_PWR_LDO_SUPPLY
)

# 芯片架构和浮点配置
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

set(STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32h723xx.s")
# 使用绝对路径确保正确解析
# 使用 CMAKE_CURRENT_LIST_DIR 确保正确的目录解析
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32H723VGTX_FLASH.ld")

# HAL drivers path
set(HAL_DIR "${PROJECT_SOURCE_DIR}/ThirdParty/STM32H7xx_HAL_Driver")

# CMSIS
set(CMSIS_DEVICE "${PROJECT_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32H7xx")
set(CMSIS_INCLUDE "${PROJECT_SOURCE_DIR}/ThirdParty/CMSIS/Include")

# Math library for this chip
set(MATH_LIB "libarm_cortexM7lfsp_math.a")

# Optional: this board needs FreeRTOS?
set(BOARD_USE_FREERTOS ON)
