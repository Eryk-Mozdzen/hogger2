cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE "toolchain.cmake")

project(firmware-stm)

enable_language(ASM C)

set(TARGET ${CMAKE_PROJECT_NAME}.elf)

add_executable(${TARGET}
    "cubemx/startup_stm32u535xx.s"
    "cubemx/Core/Src/main.c"
    "cubemx/Core/Src/stm32u5xx_hal_msp.c"
    "cubemx/Core/Src/stm32u5xx_it.c"
    "cubemx/Core/Src/syscalls.c"
    "cubemx/Core/Src/sysmem.c"
    "cubemx/Core/Src/system_stm32u5xx.c"

    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_comp.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cordic.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_crc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cryp_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dac_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcache.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dcmi.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma2d.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dsi.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fmac.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxmmu.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gfxtim.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpu2d.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hcd.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_irda.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_jpeg.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_lptim.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ltdc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mdf.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_mmc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nand.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_nor.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_opamp_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ospi.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_otfdec.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pcd_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pka.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pssi.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_ramcfg.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rng_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rtc_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sai_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sdio.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smartcard_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_smbus_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sram.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tsc.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_usart_ex.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_wwdg.c"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_xspi.c"

	"src/main.c"
    "src/utils/tasks.c"
    "src/utils/mpack.c"
    "src/com/telemetry.c"
    "src/com/stream.c"
    "src/actuate/motor.c"
    "src/actuate/motors.c"
    "src/actuate/dynamixel.c"
    "src/actuate/servos.c"
    "src/measure/imu.c"
    "src/measure/flow.c"
    "src/control/controller.c"

    "../common/lrcp/stream.c"
    "../common/lrcp/frame.c"
    "../common/cmp/cmp.c"
)

target_include_directories(${TARGET} PRIVATE
    "cubemx/Core/Inc"
    "cubemx/Drivers/CMSIS/Include"
    "cubemx/Drivers/CMSIS/Device/ST/STM32U5xx/Include"
    "cubemx/Drivers/STM32U5xx_HAL_Driver/Inc"

	"src"
    "../common"
)

target_compile_definitions(${TARGET} PRIVATE
    USE_HAL_DRIVER
    STM32U535xx
)

target_compile_options(${TARGET} PRIVATE
	$<$<COMPILE_LANGUAGE:ASM>:
		-Os

        -x assembler-with-cpp

        -mthumb
        -mcpu=cortex-m33
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
	>

	$<$<COMPILE_LANGUAGE:C>:
		-Os -g
		-std=c17
		-Wall
        -Wextra
        -Wpedantic

        -Wno-unused-parameter

		-ffunction-sections
		-fdata-sections

        -mthumb
		-mcpu=cortex-m33
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
	>
)

target_link_options(${TARGET} PRIVATE
	-T ${CMAKE_SOURCE_DIR}/cubemx/STM32U535xx_FLASH.ld
	-Wl,-Map=${CMAKE_PROJECT_NAME}.map
	-Wl,--gc-sections
	-Wl,-cref
    -Wl,--start-group -lc -lm -Wl,--end-group

    -u _printf_float
    --specs=nano.specs

    -mthumb
	-mcpu=cortex-m33
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_link_libraries(${TARGET}
	m
)

add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -j .text -D ${TARGET} > ${CMAKE_PROJECT_NAME}.dump
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${CMAKE_PROJECT_NAME}.bin
	COMMAND ${CMAKE_SIZE} ${TARGET}
)
