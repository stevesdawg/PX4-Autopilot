/************************************************************************************
 * nuttx-configs/nxp_teensy41-v1/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __NUTTX_CONFIG_NXP_TEENSY41_V1_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_NXP_TEENSY41_V1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>

/* Do not include i.MXRT header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        600Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 100
 *                        ARM_PODF_DIVISOR   = 2
 *                        600Mhz  = (24Mhz * 100/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 MHz             = 600Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        600Mhz  = 600Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF,
 *                                     1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 600Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       150Mhz = 600Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16.6Mhz  = 150Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                  (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       75Mhz    = 600Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 * Set LPSPI PLL3 PFD0 to fOut    = (480Mhz / 12 * 18)
 *                        720Mhz  = (480Mhz / 12 * 18)
 *                         90Mhz  = (720Mhz / LSPI_PODF_DIVIDER)
 *
 * Set LPI2C PLL3 / 8 to   fOut   = (480Mhz / 8)
 *                         60Mhz  = (480Mhz / 8)
 *                         12Mhz  = (60Mhz / LSPI_PODF_DIVIDER)
 *
 * These clock frequencies can be verified via the CCM_CLKO1 pin and sending
 * the appropriate clock to it with something like;
 *
 *   putreg32( <Clk number> | CCM_CCOSR_CLKO1_EN ,   IMXRT_CCM_CCOSR);
 *   imxrt_config_gpio(GPIO_CCM_CLKO1);
 */

#define BOARD_XTAL_FREQUENCY       24000000
#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT   96
#define IMXRT_ARM_PODF_DIVIDER     2
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  9
#define IMXRT_SEMC_PODF_DIVIDER    8

#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_CAN_CLK_SELECT       CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80
#define IMXRT_CAN_PODF_DIVIDER     1

#define IMXRT_SYS_PLL_SELECT       CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20

#define BOARD_CPU_FREQUENCY \
  (BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

#define BOARD_GPT_FREQUENCY \
	(BOARD_CPU_FREQUENCY / IMXRT_IPG_PODF_DIVIDER) / IMXRT_PERCLK_PODF_DIVIDER

/* Define this to enable tracing */

#if 0
#  define IMXRT_TRACE_PODF_DIVIDER 1
#  define IMXRT_TRACE_CLK_SELECT   CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0
#endif

/* LED definitions **********************************************************/

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

/* LED index values for use with board_userled() */

#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)
#  define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT2 | GPIO_PIN3)  /* BO_03 */
#define BOARD_LED_RED 	0
#define BOARD_LED_GREEN 0
#define BOARD_LED_BLUE 	0

#define BOARD_LED1_BIT    (1 << BOARD_NLEDS)
#define BOARD_LED2_BIT    (1 << BOARD_NLEDS)
#define BOARD_LED3_BIT    (1 << BOARD_NLEDS)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1  /* Idle stack created      ON     */
#define LED_INIRQ         2  /* In an interrupt         N/C    */
#define LED_SIGNAL        2  /* In a signal handler     N/C    */
#define LED_ASSERTION     2  /* An assertion failed     N/C    */
#define LED_PANIC         3  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* SDIO *********************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning
 * for specific boards, but has been checked by scope on the EVKB to make
 * sure shapes are square with minimal ringing.
 */

#define PIN_USDHC1_D0     (GPIO_USDHC1_DATA0_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D1     (GPIO_USDHC1_DATA1_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D2     (GPIO_USDHC1_DATA3_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D3     (GPIO_USDHC1_DATA2_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_DCLK   (GPIO_USDHC1_CLK_1   | IOMUX_USDHC1_CLK_DEFAULT)
#define PIN_USDHC1_CMD    (GPIO_USDHC1_CMD_1   | IOMUX_USDHC1_CMD_DEFAULT)
#define PIN_USDHC1_CD     (PIN_USDHC1_D3)

/* 386 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* ETH Disambiguation *******************************************************/

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_1|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_B1_15 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_1|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_B1_14 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
                              IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */

/* PIO Disambiguation *******************************************************/

/* LPUARTs
 */
#define LPUART_IOMUX      (IOMUX_PULL_UP_22K | IOMUX_DRIVE_40OHM | IOMUX_SLEW_SLOW | IOMUX_SPEED_LOW | IOMUX_SCHMITT_TRIGGER)

/* GPS 1 */

#define GPIO_LPUART2_RX   (GPIO_LPUART2_RX_1 | LPUART_IOMUX) /* EVK J22-8 */ /* GPIO_AD_B1_03 */
#define GPIO_LPUART2_TX   (GPIO_LPUART2_TX_1 | LPUART_IOMUX) /* EVK J22-7 */ /* GPIO_AD_B1_02 */

/* N.B. Rev B schematic did not change the names of the nets. Just the silk screen renamed the ports
 * Such that Telem 2 had the real HW HS signals. The imx driver to dated does not support GOIO controlled
 * HS lines
 */

/* Telem 1 */

#define HS_INPUT_IOMUX  (IOMUX_CMOS_INPUT | IOMUX_SLEW_SLOW | IOMUX_DRIVE_HIZ  | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_47K)
#define HS_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_SLEW_FAST | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_PULL_KEEP)

#define GPIO_LPUART3_RX   (GPIO_LPUART3_RX_3 | LPUART_IOMUX) /* GPIO_B0_09 */
#define GPIO_LPUART3_TX   (GPIO_LPUART3_TX_3 | LPUART_IOMUX) /* GPIO_B0_08 */
#define GPIO_LPUART3_CTS  (GPIO_PORT3 | GPIO_PIN4  | GPIO_INPUT  | HS_INPUT_IOMUX)  /* GPIO_SD_B1_04 GPIO3_IO04 (GPIO only, no HW Flow control) */
#define GPIO_LPUART3_RTS  (GPIO_PORT4 | GPIO_PIN24 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | HS_OUTPUT_IOMUX) /* GPIO_EMC_24   GPIO4_IO24 (GPIO only, no HW Flow control) */

/* Telem 2 */

#define GPIO_LPUART4_RX   (GPIO_LPUART4_RX_2  | LPUART_IOMUX) /* GPIO_EMC_20 */
#define GPIO_LPUART4_TX   (GPIO_LPUART4_TX_2  | LPUART_IOMUX) /* GPIO_EMC_19 */
#define GPIO_LPUART4_CTS  (GPIO_LPUART4_CTS_1 | LPUART_IOMUX) /* GPIO_EMC_17 */
#define GPIO_LPUART4_RTS  (GPIO_LPUART4_RTS_1 | LPUART_IOMUX) /* GPIO_EMC_18 */

/* GPS2 */

#define GPIO_LPUART5_RX   (GPIO_LPUART5_RX_1 | LPUART_IOMUX)  /* GPIO_B1_13 */
#define GPIO_LPUART5_TX   (GPIO_LPUART5_TX_2 | LPUART_IOMUX)  /* GPIO_EMC_23 */

/* RC INPUT single wire mode on TX, RX is not used */

#define GPIO_LPUART6_RX   (GPIO_LPUART6_RX_2 | LPUART_IOMUX)  /* GPIO_EMC_26 */
#define GPIO_LPUART6_TX   (GPIO_LPUART6_TX_2 | LPUART_IOMUX)  /* GPIO_EMC_25 */

#define GPIO_LPUART7_RX   (GPIO_LPUART7_RX_1 | LPUART_IOMUX)  /* GPIO_EMC_32 */
#define GPIO_LPUART7_TX   (GPIO_LPUART7_TX_1 | LPUART_IOMUX)  /* GPIO_EMC_31 */

#define GPIO_LPUART8_RX   (GPIO_LPUART8_RX_2 | LPUART_IOMUX) /* GPIO_EMC_39 */
#define GPIO_LPUART8_TX   (GPIO_LPUART8_TX_2 | LPUART_IOMUX) /* GPIO_EMC_38 */

/* LPI2Cs */

#define LPI2C_IOMUX    (IOMUX_SPEED_MEDIUM | IOMUX_DRIVE_33OHM  | IOMUX_OPENDRAIN | GPIO_SION_ENABLE)
#define LPI2C_IO_IOMUX (IOMUX_SPEED_MAX | IOMUX_SLEW_FAST | IOMUX_DRIVE_33OHM  | IOMUX_OPENDRAIN | IOMUX_PULL_NONE)

#define GPIO_LPI2C1_SDA   (GPIO_LPI2C1_SDA_2 | LPI2C_IOMUX) /* EVK J24-9 R276  */ /* GPIO_AD_B1_01 */
#define GPIO_LPI2C1_SCL   (GPIO_LPI2C1_SCL_2 | LPI2C_IOMUX) /* EVK J24-10 R277 */ /* GPIO_AD_B1_00 */

#define GPIO_LPI2C1_SDA_RESET (GPIO_PORT1 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B1_01 GPIO1_IO17 */
#define GPIO_LPI2C1_SCL_RESET (GPIO_PORT1 | GPIO_PIN16 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B1_00 GPIO1_IO16 */

#define GPIO_LPI2C2_SDA   (GPIO_LPI2C2_SDA_1 | LPI2C_IOMUX) /* EVK J8-A25 */ /* GPIO_B0_05 */
#define GPIO_LPI2C2_SCL   (GPIO_LPI2C2_SCL_1 | LPI2C_IOMUX) /* EVK J8-A24 */ /* GPIO_B0_04 */

#define GPIO_LPI2C2_SDA_RESET (GPIO_PORT2 | GPIO_PIN5 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_B0_05 GPIO2_IO5 */
#define GPIO_LPI2C2_SCL_RESET (GPIO_PORT2 | GPIO_PIN4 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_B0_04 GPIO2_IO4 */

#define GPIO_LPI2C3_SDA   (GPIO_LPI2C3_SDA_2 | LPI2C_IOMUX) /* GPIO_EMC_21 */
#define GPIO_LPI2C3_SCL   (GPIO_LPI2C3_SCL_2 | LPI2C_IOMUX) /* GPIO_EMC_22 */

#define GPIO_LPI2C3_SDA_RESET (GPIO_PORT4 | GPIO_PIN21 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_EMC_21 GPIO4_IO21 */
#define GPIO_LPI2C3_SCL_RESET (GPIO_PORT4 | GPIO_PIN22 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_EMC_22 GPIO4_IO22 */

/* LPSPI */
#define LPSPI_IOMUX       (IOMUX_PULL_UP_100K | IOMUX_DRIVE_33OHM | IOMUX_SLEW_FAST | IOMUX_SPEED_MAX)

#define GPIO_LPSPI1_SCK   (GPIO_LPSPI1_SCK_1 | LPSPI_IOMUX) /* GPIO_EMC_27 */
#define GPIO_LPSPI1_MISO  (GPIO_LPSPI1_SDI_1 | LPSPI_IOMUX) /* GPIO_EMC_29 */
#define GPIO_LPSPI1_MOSI  (GPIO_LPSPI1_SDO_1 | LPSPI_IOMUX) /* GPIO_EMC_28 */

#define GPIO_LPSPI2_SCK   (GPIO_LPSPI2_SCK_1 | LPSPI_IOMUX) /* GPIO_EMC_00 */
#define GPIO_LPSPI2_MISO  (GPIO_LPSPI2_SDI_1 | LPSPI_IOMUX) /* GPIO_EMC_03 */
#define GPIO_LPSPI2_MOSI  (GPIO_LPSPI2_SDO_1 | LPSPI_IOMUX) /* GPIO_EMC_02 */

#define GPIO_LPSPI3_SCK   (GPIO_LPSPI3_SCK_1 | LPSPI_IOMUX) /* GPIO_AD_B1_15 */
#define GPIO_LPSPI3_MISO  (GPIO_LPSPI3_SDI_1 | LPSPI_IOMUX) /* GPIO_AD_B1_13 */
#define GPIO_LPSPI3_MOSI  (GPIO_LPSPI3_SDO_1 | LPSPI_IOMUX) /* GPIO_AD_B1_14 */

#define GPIO_LPSPI4_SCK   (GPIO_LPSPI4_SCK_1 | LPSPI_IOMUX) /* GPIO_B1_07 */
#define GPIO_LPSPI4_MISO  (GPIO_LPSPI4_SDI_1 | LPSPI_IOMUX) /* GPIO_B1_05 */
#define GPIO_LPSPI4_MOSI  (GPIO_LPSPI4_SDO_2 | LPSPI_IOMUX) /* GPIO_B0_02 */

/* CAN
 *
 * CAN1 is routed to transceiver.
 * CAN2 is routed to transceiver.
 * CAN3 is routed to transceiver.
 */
#define FLEXCAN_IOMUX     (IOMUX_PULL_UP_100K | IOMUX_DRIVE_40OHM | IOMUX_SLEW_FAST | IOMUX_SPEED_MEDIUM)

#define GPIO_FLEXCAN1_RX  (GPIO_FLEXCAN1_RX_2 | FLEXCAN_IOMUX)  /* GPIO_B0_03 */
#define GPIO_FLEXCAN1_TX  (GPIO_FLEXCAN1_TX_4 | FLEXCAN_IOMUX) /* GPIO_SD_B1_02 */
#define GPIO_FLEXCAN2_RX  (GPIO_FLEXCAN2_RX_1 | FLEXCAN_IOMUX) /* GPIO_AD_B0_03 */
#define GPIO_FLEXCAN2_TX  (GPIO_FLEXCAN2_TX_1 | FLEXCAN_IOMUX) /* GPIO_AD_B0_02 */
#define GPIO_FLEXCAN3_RX  (GPIO_FLEXCAN3_RX_1 | FLEXCAN_IOMUX) /* GPIO_AD_B0_11  */
#define GPIO_FLEXCAN3_TX  (GPIO_FLEXCAN3_TX_3 | FLEXCAN_IOMUX) /* GPIO_EMC_36 */

/* FlexPWM */

#define GPIO_FLEXPWM2_MOD1_A (GPIO_FLEXPWM2_PWMA00_1|IOMUX_PWM_DEFAULT) /* GPIO_EMC_06 */
#define GPIO_FLEXPWM2_MOD2_A (GPIO_FLEXPWM2_PWMA01_1|IOMUX_PWM_DEFAULT) /* GPIO_EMC_08 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __NUTTX_CONFIG_NXP_TEENSY41_V1_INCLUDE_BOARD_H */
