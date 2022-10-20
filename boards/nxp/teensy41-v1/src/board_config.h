/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/teensy-4.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __SRC_TEENSY_4_H
#  define __SRC_TEENSY_4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#  include <px4_platform_common/px4_config.h>

#  include <stdint.h>
#  include <stdbool.h>

#  include <arch/irq.h>
#  include <nuttx/irq.h>
#  include <nuttx/config.h>
#  include <nuttx/compiler.h>
#  include <arch/board/board.h>

#  include "imxrt_gpio.h"
#  include "imxrt_iomuxc.h"
#  include "hardware/imxrt_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* i.MX RT 1060 GPIO Pin Definitions ****************************************/

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

/* LEDs */

/* There are two LED status indicators located on the Teensy 4.x board.
 * The functions of these LEDs include:
 *
 *   - RED LED (loading status)
 *      - dim:    ready
 *      - bright: writing
 *      - blink:  no USB
 *   - USER LED (D3)
 *
 * Only a single LED, D3, is under software control.
 */

#  define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT2 | GPIO_PIN3)  /* BO_03 */
#  define LED_DRIVER_PATH "/dev/userleds"

/* LPSPI3 CS:  GPIO_AD_B1_12 */
#  define IOMUX_LPSPI3_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#  define GPIO_LPSPI3_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT1 | GPIO_PIN28 | IOMUX_LPSPI3_CS)
/* LPSPI4 CS:  GPIO_B0_00  */
#  define IOMUX_LPSPI4_CS      (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#  define GPIO_LPSPI4_CS       (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT2 | GPIO_PIN0 | IOMUX_LPSPI4_CS)

// /* LCD dispay */
// #  define GPIO_LCD_RST        (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
//                               GPIO_PORT2 | GPIO_PIN18 | IOMUX_LPSPI4_CS)    /* B1_02 */
// #  define GPIO_LCD_CD         (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
//                               GPIO_PORT2 | GPIO_PIN19 | IOMUX_LPSPI4_CS)    /* B1_03 */

/* USB OTG ID Pin： GPIO_AD_B1_02 */
#define GPIO_USBOTG_ID  (GPIO_USB_OTG1_ID_1 | IOMUX_USBOTG_ID_DEFAULT)      /* AD_B1_02 */

/* Ethernet */
/* Ethernet Interrupt: GPIO_B0_15
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */
#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | \
                         GPIO_PORT2 | GPIO_PIN15)    /* B0_15 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO2_15

/* Ethernet Reset:  GPIO_B0_14
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */
#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT2 | GPIO_PIN14 | IOMUX_ENET_RST_DEFAULT) /* B0_14 */

/* Quadrature Encoder */
#define GPIO_ENC1_PHASE_A    (GPIO_XBAR1_INOUT09_1|IOMUX_ENC_DEFAULT|PADMUX_MUXMODE_ALT3)  /* EMC_07 */
#define GPIO_ENC1_PHASE_B    (GPIO_XBAR1_INOUT08_1|IOMUX_ENC_DEFAULT|PADMUX_MUXMODE_ALT3)  /* EMC_06 */
#define GPIO_ENC1_INDEX      (GPIO_XBAR1_INOUT10_1|IOMUX_ENC_DEFAULT|PADMUX_MUXMODE_ALT1)  /* B0_12 */

/* GPIO pins used by the GPIO subsystem */
#define BOARD_NGPIOIN   1   /* Amount of GPIO input pins */
#define BOARD_NGPIOOUT  1   /* Amount of GPIO output pins */

#define GPIO_IN1       (GPIO_INPUT | GPIO_PORT4 | GPIO_PIN4)    /* EMC_04 */
#define GPIO_OUT1      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                        GPIO_PORT4 | GPIO_PIN5)                 /* EMC_05 */

// BELOW ARE CONSTANTS THAT I ADDED
/* ADC */
#define ADC_IOMUX (IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n, p)             (GPIO_PORT1 | GPIO_PIN##p | ADC_IOMUX) //

/* Define GPIO pins used as ADC N.B. Channel numbers are for reference, */
#define PX4_ADC_GPIO  \
	/* BATTERY1_VOLTAGE       GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_GPIO(0,  27),  \
	/* BATTERY1_CURRENT       GPIO_AD_B0_12 GPIO1 Pin 12 */  ADC1_GPIO(1,  12),  \
	/* BATTERY2_VOLTAGE       GPIO_AD_B0_13 GPIO1 Pin 13 */  ADC1_GPIO(2,  13),  \
	/* HW_VER_SENSE           GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9,  20),  \
	/* SCALED_V5              GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_GPIO(10, 21), \
	/* SCALED_VDD_3V3_SENSORS GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_GPIO(11, 22), \
	/* HW_REV_SENSE           GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_GPIO(13, 24), \
	/* SPARE_1                GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_GPIO(14, 25)
	// /* BATTERY2_CURRENT       GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_GPIO(3,  14),
	// /* SPARE_2_CHANNEL        GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_GPIO(4,  15),
	// /* RSSI_IN                GPIO_AD_B1_10 GPIO1 Pin 26 */  ADC1_GPIO(15, 26),

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_CH(0)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* GPIO_AD_B0_12 GPIO1 Pin 12 */  ADC1_CH(1)
#define ADC_BATTERY2_VOLTAGE_CHANNEL        /* GPIO_AD_B0_13 GPIO1 Pin 13 */  ADC1_CH(2)
#define ADC_HW_VER_SENSE_CHANNEL            /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_CH(9)
#define ADC_SCALED_V5_CHANNEL               /* GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_CH(11)
#define ADC_HW_REV_SENSE_CHANNEL            /* GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_CH(13)
#define ADC1_SPARE_1_CHANNEL                /* GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_CH(14)
// #define ADC_BATTERY2_CURRENT_CHANNEL        /* GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_CH(3)
// #define ADC1_SPARE_2_CHANNEL                /* GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_CH(4)
// #define ADC_RSSI_IN_CHANNEL                 /* GPIO_AD_B1_10 GPIO1 Pin 26 */  ADC1_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
	//  (1 << ADC_BATTERY2_CURRENT_CHANNEL)       |
	//  (1 << ADC1_SPARE_2_CHANNEL)               |
	//  (1 << ADC_RSSI_IN_CHANNEL)                |

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define HW_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MAX | IOMUX_SLEW_FAST)

#define GPIO_HW_VER_REV_DRIVE /* GPIO_AD_B0_01 GPIO1_IO01   */  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | HW_IOMUX)
#define GPIO_HW_REV_SENSE     /* GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_GPIO(13, 24)
#define GPIO_HW_VER_SENSE     /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9,  20)
#define HW_INFO_INIT_PREFIX   "V5"
#define V500   HW_VER_REV(0x0,0x0) // FMUV5,                    Rev 0
#define V540   HW_VER_REV(0x4,0x0) // mini no can 2,3,          Rev 0

/* PWM Capture
 *
 * 2  PWM Capture inputs are supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  2
#define CAP_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define PIN_FLEXPWM2_PWMB0  /* P2:7  PWM2 B0 FMU_CAP1 */ (CAP_IOMUX | GPIO_FLEXPWM2_PWMB00_2)
#define PIN_FLEXPWM2_PWMB3  /* P3:3  PWM2 A1 FMU_CAP2 */ (CAP_IOMUX | GPIO_FLEXPWM2_PWMB03_3)

#define nARMED_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)
#define nARMED_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

// #define GPIO_nARMED_INIT     /* GPIO_SD_B1_01 GPIO3_IO1 */ (GPIO_PORT3 | GPIO_PIN1 | GPIO_INPUT | nARMED_INPUT_IOMUX)
// #define GPIO_nARMED          /* GPIO_SD_B1_01 GPIO3_IO1 */ (GPIO_PORT3 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | nARMED_OUTPUT_IOMUX)

// #define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
// #define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
// Input Capture not supported on MVP
#define BOARD_HAS_NO_CAPTURE

/* Tone alarm output */
// TODO: Fix Tone Alarm Timer and Channel. Cannot be GPIO B1_07.
#define TONE_ALARM_TIMER        2  /* GPT 2 */
#define TONE_ALARM_CHANNEL      3  /* GPIO_AD_B1_07 GPT2_COMPARE3 */
#define GPIO_BUZZER_1           /* GPIO_AD_B1_07  GPIO1_IO23  */ (GPIO_PORT1 | GPIO_PIN23  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         (GPIO_GPT2_COMPARE3_2 | GENERAL_OUTPUT_IOMUX)

/* High-resolution timer */
#define HRT_TIMER               1  /* use GPT1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

// TODO: Fix this IO Pin. Cannot be B1_06
#define HRT_PPM_CHANNEL         /* GPIO_B1_06 GPT1_CAPTURE2 */  2  /* use capture/compare channel 2 */
#define GPIO_PPM_IN             /* GPIO_B1_06 GPT1_CAPTURE2 */ (GPIO_GPT1_CAPTURE2_2 | GENERAL_INPUT_IOMUX)

#define RC_SERIAL_PORT          "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/* PWM input driver. Use FMU AUX5 pins attached to GPIO_EMC_33 GPIO3_IO19 FLEXPWM3_PWMA2 */

#define PWMIN_TIMER            /* FLEXPWM3_PWMA2 */  3
#define PWMIN_TIMER_CHANNEL    /* FLEXPWM3_PWMA2 */  2
#define GPIO_PWM_IN            /* GPIO_EMC_33 GPIO3_IO19 */ (GPIO_FLEXPWM3_PWMA02_1 | GENERAL_INPUT_IOMUX)

#define BOARD_NUMBER_BRICKS             0
#define BOARD_ENABLE_CONSOLE_BUFFER

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#  ifndef __ASSEMBLY__

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#    if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#    endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the i.MXRT1050 EVK.
 *
 ****************************************************************************/

void imxrt_spidev_initialize(void);

/****************************************************************************
 * Name: imxrt_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

// #ifdef CONFIG_IMXRT_FLEXCAN
int imxrt_can_setup(void);
// #endif

/****************************************************************************
 * Name: imxrt_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

// #ifdef CONFIG_IMXRT_FLEXPWM
int imxrt_pwm_setup(void);
// #endif

/****************************************************************************
 * Name: imxrt_adc_initialize
 *
 * Description:
 *   Initialize ADC drivers
 *
 ****************************************************************************/

// #ifdef CONFIG_IMXRT_ADC
int imxrt_adc_initialize(void);
// #endif

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

// #ifdef CONFIG_DEV_GPIO
int imxrt_gpio_initialize(void);
// #endif

/****************************************************************************
 * Name: imxrt_enc_initialize
 *
 * Description:
 *   Initialize the quadrature encoder driver for the given timer
 *
 ****************************************************************************/

// #ifdef CONFIG_IMXRT_ENC
int imxrt_enc_initialize(void);
// #endif

/****************************************************************************
 * Name: imxrt_i2c_setup
 *
 * Description:
 *  Choose which I2C driver should be initialize
 *
 ****************************************************************************/

// #ifdef CONFIG_IMXRT_LPI2C
void imxrt_i2c_setup(void);
// #endif

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

// #ifdef CONFIG_ARCH_LEDS
void imxrt_autoled_initialize(void);
// #endif

#  include <px4_platform_common/board_common.h>

#endif  /* __ASSEMBLY__ */
#endif  /* __SRC_TEENSY_4_H */
