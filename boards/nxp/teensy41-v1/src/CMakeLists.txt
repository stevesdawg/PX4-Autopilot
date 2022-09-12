px4_add_library(drivers_board
	adc.c
	autoleds.c
	encoder.c
	ethernet.c
	flexcan.c
	flexpwm.c
	gpio.c
	i2c.cpp
	imxrt_appinit.c
	imxrt_boot.c
	imxrt_bringup.c
	imxrt_flexspi_nor_boot.c
	imxrt_flexspi_nor_flash.c
	spi.cpp
	timer_config.cpp
	userleds.c
)

target_link_libraries(drivers_board
	PRIVATE
		arch_board_hw_info
		arch_io_pins
		drivers__led # drv_led_start
		nuttx_arch # sdio
		nuttx_drivers # sdio
		px4_layer
)

