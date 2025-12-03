/**
 * @file boards/bossbrain_f427/board.h
 *
 * @author Boss Garage, 2025
 */
 #define BOARD_NAME "bossbrain_f427"

#define HAL_USE_SDC FALSE

#ifndef BOARD_IO_H
#define BOARD_IO_H

//Real-Time Clock (RTC) peripheral.
//Check schematic for RTC crystal and connections (e.g., 32.768 kHz to LSE pins)
#undef EFI_RTC
#define EFI_RTC FALSE

//Enables serial port over USB (via STM32 built-in USB device).
//Check schematic for USB connector (USB D- and D+)
#undef EFI_USB_SERIAL
#define EFI_USB_SERIAL TRUE

//Sets alternate function number for USB on STM32 pins.
//Check STM32F427 datasheet: PA11/PA12 usually need AF10 for USB.
//If your schematic wires PA11/PA12 to USB, keep as 10U.
#undef EFI_USB_AF
#define EFI_USB_AF 10U

//Defines the pins used for USB D- (DM) and D+ (DP).
//Check schematic: Are PA11 and PA12 connected to USB lines? If so, set here.
#undef EFI_USB_SERIAL_DM
#define EFI_USB_SERIAL_DM Gpio::A11

#undef EFI_USB_SERIAL_DP
#define EFI_USB_SERIAL_DP Gpio::A12

//Bluetooth configuration via console.
#undef EFI_BLUETOOTH_SETUP
#define EFI_BLUETOOTH_SETUP FALSE

//Enables built-in serial support on USART1 (TX on PA9, RX on PA10).
#undef STM32_SERIAL_USE_USART1
#define STM32_SERIAL_USE_USART1 FALSE

//Enables STM32 HAL UART driver for USART1 (TX on PA9, RX on PA10). 
#undef STM32_UART_USE_USART1
#define STM32_UART_USE_USART1 TRUE

//Sets the primary tuning serial/UART port (used by rusefi tuning/console software).
#undef TS_PRIMARY_UxART_PORT
#define TS_PRIMARY_UxART_PORT UARTD1

#undef SERIAL_SPEED
#define SERIAL_SPEED 115200

#undef EFI_CONSOLE_TX_BRAIN_PIN
#define EFI_CONSOLE_TX_BRAIN_PIN Gpio::A9

#undef EFI_CONSOLE_RX_BRAIN_PIN
#define EFI_CONSOLE_RX_BRAIN_PIN Gpio::A10

//Enables Oscilloscope feature (PC14/PC15 inputs).
#undef EFI_USE_OSC
#define EFI_USE_OSC TRUE

//Enables CAN bus support via CAN1 (PB5, PB6, PD0, PD1).
#undef EFI_CAN_SUPPORT
#define EFI_CAN_SUPPORT TRUE

//Enables file logging support to SD card.
#undef EFI_FILE_LOGGING
#define EFI_FILE_LOGGING FALSE

#undef EFI_STORAGE_SD
#define EFI_STORAGE_SD FALSE

//Defines which SD card device/instance is present.
//Usually SDCD1. If you have SD card on your board, wiring matches STM32’s SDIO peripheral.
#undef EFI_SDC_DEVICE
#define EFI_SDC_DEVICE SDCD1

//Enables virtual drive support for ini file.
#undef HAL_USE_USB_MSD
#define HAL_USE_USB_MSD FALSE

//ICU stands for Input Capture Unit, used for high-speed input capture (e.g., for RPM).
#undef EFI_ICU_INPUTS
#define EFI_ICU_INPUTS FALSE

//Enables PAL (Platform Abstraction Layer) for trigger inputs.
#undef HAL_TRIGGER_USE_PAL
#define HAL_TRIGGER_USE_PAL TRUE

//Enables logic analyzer feature (high-speed input capture on multiple pins).
#undef EFI_LOGIC_ANALYZER
#define EFI_LOGIC_ANALYZER FALSE

//Enables PAL driver for Vehicle Speed Sensor (VSS) inputs.
#undef HAL_VSS_USE_PAL
#define HAL_VSS_USE_PAL TRUE

//Defines the pin used for the critical error LED.
#undef LED_CRITICAL_ERROR_BRAIN_PIN
#define LED_CRITICAL_ERROR_BRAIN_PIN Gpio::G11

// Ignore USB VBUS pin (we're never a host, only a device)
#define BOARD_OTG_NOVBUSSENS TRUE

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK 32768U
#endif

#define STM32_LSEDRV (3U << 3U)

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD 300U

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */

 // Helper macros to define GPIO setup.
#define PIN_MODE_INPUT(n) (0U << ((n)*2U))
#define PIN_MODE_OUTPUT(n) (1U << ((n)*2U))
#define PIN_MODE_ALTERNATE(n) (2U << ((n)*2U))
#define PIN_MODE_ANALOG(n) (3U << ((n)*2U))
#define PIN_ODR_LOW(n) (0U << (n))
#define PIN_ODR_HIGH(n) (1U << (n))
#define PIN_OTYPE_PUSHPULL(n) (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n) (1U << (n))
#define PIN_OSPEED_VERYLOW(n) (0U << ((n)*2U))
#define PIN_OSPEED_LOW(n) (1U << ((n)*2U))
#define PIN_OSPEED_MEDIUM(n) (2U << ((n)*2U))
#define PIN_OSPEED_HIGH(n) (3U << ((n)*2U))
#define PIN_PUPDR_FLOATING(n) (0U << ((n)*2U))
#define PIN_PUPDR_PULLUP(n) (1U << ((n)*2U))
#define PIN_PUPDR_PULLDOWN(n) (2U << ((n)*2U))
#define PIN_AFIO_AF(n, v) ((v) << (((n) % 8U) * 4U))

// ----------------------------------------------------------------------
// GPIOA Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOA (PAx) pin:
// - Mode: Analog for ADC inputs, Output for lowside drivers, Alternate for UART/USB/SWD
// - Output type: Normally push-pull (safe for outputs/alt), since open-drain not required
// - Speed: High speed for outputs and alternate functions
// - Pull-up/Pull-down: Pull-up for RX (UART), floating for others
// - Output default value: Outputs start LOW (safe default)
// - Alternate function number: Sets STM32 peripheral multiplexing for UART, USB, SWD
// ----------------------------------------------------------------------

// GPIOA mode setup: set each pin's function (Analog, Output, Alternate)
#define VAL_GPIOA_MODER ( \
    PIN_MODE_ANALOG(0)    | /* PA0: ADC IN0 (BRV) */ \
    PIN_MODE_ANALOG(1)    | /* PA1: ADC IN1 */ \
    PIN_MODE_ANALOG(2)    | /* PA2: ADC IN2 */ \
    PIN_MODE_ANALOG(3)    | /* PA3: ADC IN3 */ \
    PIN_MODE_ANALOG(4)    | /* PA4: ADC IN4 */ \
    PIN_MODE_ANALOG(5)    | /* PA5: ADC IN5 */ \
    PIN_MODE_ANALOG(6)    | /* PA6: ADC IN6 */ \
    PIN_MODE_ANALOG(7)    | /* PA7: ADC IN7 */ \
    PIN_MODE_OUTPUT(8)    | /* PA8: Lowside 20 (output) */ \
    PIN_MODE_ALTERNATE(9) | /* PA9: UART TX (CPU) */ \
    PIN_MODE_ALTERNATE(10)| /* PA10: UART RX (CPU) */ \
    PIN_MODE_ALTERNATE(11)| /* PA11: USB_OTG- */ \
    PIN_MODE_ALTERNATE(12)| /* PA12: USB_OTG+ */ \
    PIN_MODE_ALTERNATE(13)| /* PA13: SWDIO */ \
    PIN_MODE_ALTERNATE(14)| /* PA14: SWCLK */ \
    PIN_MODE_OUTPUT(15)     /* PA15: Lowside 24 (output) */ \
)

// GPIOA output type setup: push-pull for all (safe default)
#define VAL_GPIOA_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOA output speed: high speed for outputs/altfunc (others can be high or medium)
#define VAL_GPIOA_OSPEEDR ( \
    PIN_OSPEED_HIGH(0)  | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4)  | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8)  | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10)| PIN_OSPEED_HIGH(11)| \
    PIN_OSPEED_HIGH(12) | PIN_OSPEED_HIGH(13)| PIN_OSPEED_HIGH(14)| PIN_OSPEED_HIGH(15))

// GPIOA pull-up/down setup: pull-up for RX, floating for analog/output
#define VAL_GPIOA_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_FLOATING(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_FLOATING(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_PULLUP(10)  | PIN_PUPDR_FLOATING(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOA default output value (ODR): all outputs start LOW
#define VAL_GPIOA_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOA alternate function registers:
// - AFRL: PA0–PA7 (all analog, so AF = 0)
// - AFRH: PA8–PA15, set AF for each alternate (see STM32F427 datasheet)
#define VAL_GPIOA_AFRL ( \
    PIN_AFIO_AF(0, 0U) | PIN_AFIO_AF(1, 0U) | PIN_AFIO_AF(2, 0U) | PIN_AFIO_AF(3, 0U) | \
    PIN_AFIO_AF(4, 0U) | PIN_AFIO_AF(5, 0U) | PIN_AFIO_AF(6, 0U) | PIN_AFIO_AF(7, 0U) \
)
#define VAL_GPIOA_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* PA8: output (no AF) */ \
    PIN_AFIO_AF(9, 7U)   | /* PA9: USART1_TX (AF7) */ \
    PIN_AFIO_AF(10, 7U)  | /* PA10: USART1_RX (AF7) */ \
    PIN_AFIO_AF(11, 10U) | /* PA11: USB_OTG_FS_DM (AF10) */ \
    PIN_AFIO_AF(12, 10U) | /* PA12: USB_OTG_FS_DP (AF10) */ \
    PIN_AFIO_AF(13, 0U)  | /* PA13: SWDIO (AF0) */ \
    PIN_AFIO_AF(14, 0U)  | /* PA14: SWCLK (AF0) */ \
    PIN_AFIO_AF(15, 0U)    /* PA15: output (no AF) */ \
)

// ----------------------------------------------------------------------
// GPIOB Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOB (PBx) pin:
// - Analog inputs for ADC channels
// - CAN2 RX/TX uses alternate function AF9
// - Outputs for stepper driver and Flash memory control
// - I2C lines use alternate function AF4
// - Pins not connected or ground are configured as safe inputs
// ----------------------------------------------------------------------

// GPIOB mode setup: set each pin's function
#define VAL_GPIOB_MODER ( \
    PIN_MODE_ANALOG(0)    | /* PB0: Analog IN 10 (ADC) */ \
    PIN_MODE_ANALOG(1)    | /* PB1: Analog IN 11 (ADC) */ \
    PIN_MODE_INPUT(2)     | /* PB2: Ground (safe input) */ \
    PIN_MODE_INPUT(3)     | /* PB3: not connected */ \
    PIN_MODE_INPUT(4)     | /* PB4: not connected */ \
    PIN_MODE_ALTERNATE(5) | /* PB5: CAN2 RX (AF9) */ \
    PIN_MODE_ALTERNATE(6) | /* PB6: CAN2 TX (AF9) */ \
    PIN_MODE_OUTPUT(7)    | /* PB7: output - stepper 2 step */ \
    PIN_MODE_OUTPUT(8)    | /* PB8: output - stepper 2 enable */ \
    PIN_MODE_OUTPUT(9)    | /* PB9: output - stepper 2 dir */ \
    PIN_MODE_ALTERNATE(10)| /* PB10: I2C BARO_SCL (AF4) */ \
    PIN_MODE_ALTERNATE(11)| /* PB11: I2C BARO_SDA (AF4) */ \
    PIN_MODE_OUTPUT(12)   | /* PB12: Flash CS (output) */ \
    PIN_MODE_OUTPUT(13)   | /* PB13: Flash CLK (output) */ \
    PIN_MODE_OUTPUT(14)   | /* PB14: Flash MISO (output) */ \
    PIN_MODE_OUTPUT(15)     /* PB15: Flash MOSI (output) */ \
)

// GPIOB output type setup: push-pull for all (safe default)
#define VAL_GPIOB_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOB output speed: high speed for outputs/altfunc
#define VAL_GPIOB_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12) | PIN_OSPEED_HIGH(13) | PIN_OSPEED_HIGH(14) | PIN_OSPEED_HIGH(15) \
)

// GPIOB pull-up/down setup: floating for analog/outputs, pull-up for I2C, default for others
#define VAL_GPIOB_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_PULLDOWN(2) | PIN_PUPDR_PULLDOWN(3) | \
    PIN_PUPDR_PULLDOWN(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_FLOATING(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_PULLUP(10)  | PIN_PUPDR_PULLUP(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOB default output value (ODR): all outputs start LOW
#define VAL_GPIOB_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOB alternate function registers:
// - AFRL: PB0–PB7
// - AFRH: PB8–PB15
#define VAL_GPIOB_AFRL ( \
    PIN_AFIO_AF(0, 0U) | /* Analog */ \
    PIN_AFIO_AF(1, 0U) | /* Analog */ \
    PIN_AFIO_AF(2, 0U) | /* Input */ \
    PIN_AFIO_AF(3, 0U) | /* Input */ \
    PIN_AFIO_AF(4, 0U) | /* Input */ \
    PIN_AFIO_AF(5, 9U) | /* CAN2 RX (AF9) */ \
    PIN_AFIO_AF(6, 9U) | /* CAN2 TX (AF9) */ \
    PIN_AFIO_AF(7, 0U)   /* Output (stepper 2 step) */ \
)
#define VAL_GPIOB_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* Output (stepper 2 enable) */ \
    PIN_AFIO_AF(9, 0U)   | /* Output (stepper 2 dir) */ \
    PIN_AFIO_AF(10, 4U)  | /* I2C BARO_SCL (AF4) */ \
    PIN_AFIO_AF(11, 4U)  | /* I2C BARO_SDA (AF4) */ \
    PIN_AFIO_AF(12, 0U)  | /* Output (Flash CS) */ \
    PIN_AFIO_AF(13, 0U)  | /* Output (Flash CLK) */ \
    PIN_AFIO_AF(14, 0U)  | /* Output (Flash MISO) */ \
    PIN_AFIO_AF(15, 0U)    /* Output (Flash MOSI) */ \
)

// ----------------------------------------------------------------------
// GPIOC Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOC (PCx) pin:
// - Analog inputs for ADC channels
// - Outputs for lowside drivers
// - SD card interface using alternate functions
// - Digital input and oscillator pins
// - Pins not connected are configured as safe inputs
// ----------------------------------------------------------------------

// GPIOC mode setup: set each pin's function
#define VAL_GPIOC_MODER ( \
    PIN_MODE_ANALOG(0)    | /* PC0: Analog IN 12 (ADC) */ \
    PIN_MODE_ANALOG(1)    | /* PC1: Analog IN 13 (ADC) */ \
    PIN_MODE_ANALOG(2)    | /* PC2: Analog IN 14 (ADC) */ \
    PIN_MODE_INPUT(3)     | /* PC3: not connected */ \
    PIN_MODE_ANALOG(4)    | /* PC4: Analog IN 8 (ADC) */ \
    PIN_MODE_ANALOG(5)    | /* PC5: Analog IN 9 (ADC) */ \
    PIN_MODE_OUTPUT(6)    | /* PC6: Output - Low side 18 */ \
    PIN_MODE_OUTPUT(7)    | /* PC7: Output - Low side 19 */ \
    PIN_MODE_ALTERNATE(8) | /* PC8: SD D0 (AF12) */ \
    PIN_MODE_ALTERNATE(9) | /* PC9: SD D1 (AF12) */ \
    PIN_MODE_ALTERNATE(10)| /* PC10: SD D2 (AF12) */ \
    PIN_MODE_ALTERNATE(11)| /* PC11: SD D3 (AF12) */ \
    PIN_MODE_ALTERNATE(12)| /* PC12: SD SCK (AF12) */ \
    PIN_MODE_INPUT(13)    | /* PC13: Digital input - 8 */ \
    PIN_MODE_ALTERNATE(14)| /* PC14: OSC32_IN (AF0, kept as alternate for RTC/crystal) */ \
    PIN_MODE_ALTERNATE(15)  /* PC15: OSC32_OUT (AF0, kept as alternate for RTC/crystal) */ \
)

// GPIOC output type setup: push-pull for all (safe default)
#define VAL_GPIOC_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOC output speed: high speed for outputs/altfunc
#define VAL_GPIOC_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12) | PIN_OSPEED_HIGH(13) | PIN_OSPEED_HIGH(14) | PIN_OSPEED_HIGH(15) \
)

// GPIOC pull-up/down setup: floating for analog/output/SD, input pin as floating
#define VAL_GPIOC_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_PULLDOWN(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_FLOATING(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_FLOATING(10) | PIN_PUPDR_FLOATING(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOC default output value (ODR): all outputs start LOW
#define VAL_GPIOC_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOC alternate function registers:
// - AFRL: PC0–PC7 (none used since all analog/output/input)
// - AFRH: PC8–PC15 for SD and OSC functions
#define VAL_GPIOC_AFRL ( \
    PIN_AFIO_AF(0, 0U) | /* Analog */ \
    PIN_AFIO_AF(1, 0U) | /* Analog */ \
    PIN_AFIO_AF(2, 0U) | /* Analog */ \
    PIN_AFIO_AF(3, 0U) | /* Input */ \
    PIN_AFIO_AF(4, 0U) | /* Analog */ \
    PIN_AFIO_AF(5, 0U) | /* Analog */ \
    PIN_AFIO_AF(6, 0U) | /* Output */ \
    PIN_AFIO_AF(7, 0U)   /* Output */ \
)
#define VAL_GPIOC_AFRH ( \
    PIN_AFIO_AF(8, 12U)  | /* SD D0 (AF12) */ \
    PIN_AFIO_AF(9, 12U)  | /* SD D1 (AF12) */ \
    PIN_AFIO_AF(10, 12U) | /* SD D2 (AF12) */ \
    PIN_AFIO_AF(11, 12U) | /* SD D3 (AF12) */ \
    PIN_AFIO_AF(12, 12U) | /* SD SCK (AF12) */ \
    PIN_AFIO_AF(13, 0U)  | /* Digital input */ \
    PIN_AFIO_AF(14, 0U)  | /* OSC32_IN (AF0) */ \
    PIN_AFIO_AF(15, 0U)    /* OSC32_OUT (AF0) */ \
)

// ----------------------------------------------------------------------
// GPIOD Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOD (PDx) pin:
// - CAN1 RX/TX uses alternate function AF9
// - SD card command uses alternate function AF12
// - Outputs for low side drivers
// - Pins not connected are configured as safe inputs
// ----------------------------------------------------------------------

// GPIOD mode setup: set each pin's function
#define VAL_GPIOD_MODER ( \
    PIN_MODE_ALTERNATE(0) | /* PD0: CAN1 RX (AF9) */ \
    PIN_MODE_ALTERNATE(1) | /* PD1: CAN1 TX (AF9) */ \
    PIN_MODE_ALTERNATE(2) | /* PD2: SD CMD (AF12) */ \
    PIN_MODE_OUTPUT(3)    | /* PD3: Output - Low side 23 */ \
    PIN_MODE_OUTPUT(4)    | /* PD4: Output - Low side 22 */ \
    PIN_MODE_OUTPUT(5)    | /* PD5: Output - Low side 21 */ \
    PIN_MODE_INPUT(6)     | /* PD6: not connected */ \
    PIN_MODE_INPUT(7)     | /* PD7: not connected */ \
    PIN_MODE_OUTPUT(8)    | /* PD8: Output - Low side 3 */ \
    PIN_MODE_OUTPUT(9)    | /* PD9: Output - Low side 4 */ \
    PIN_MODE_OUTPUT(10)   | /* PD10: Output - Low side 5 */ \
    PIN_MODE_OUTPUT(11)   | /* PD11: Output - Low side 6 */ \
    PIN_MODE_OUTPUT(12)   | /* PD12: Output - Low side 7 */ \
    PIN_MODE_OUTPUT(13)   | /* PD13: Output - Low side 8 */ \
    PIN_MODE_OUTPUT(14)   | /* PD14: Output - Low side 9 */ \
    PIN_MODE_OUTPUT(15)     /* PD15: Output - Low side 10 */ \
)

// GPIOD output type setup: push-pull for all (safe default)
#define VAL_GPIOD_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOD output speed: high speed for outputs/altfunc
#define VAL_GPIOD_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12) | PIN_OSPEED_HIGH(13) | PIN_OSPEED_HIGH(14) | PIN_OSPEED_HIGH(15) \
)

// GPIOD pull-up/down setup: floating for outputs/altfunc, floating for inputs
#define VAL_GPIOD_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_FLOATING(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_PULLDOWN(6) | PIN_PUPDR_PULLDOWN(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_FLOATING(10) | PIN_PUPDR_FLOATING(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOD default output value (ODR): all outputs start LOW
#define VAL_GPIOD_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOD alternate function registers:
// - AFRL: PD0–PD7
// - AFRH: PD8–PD15
#define VAL_GPIOD_AFRL ( \
    PIN_AFIO_AF(0, 9U) |  /* CAN1 RX (AF9) */ \
    PIN_AFIO_AF(1, 9U) |  /* CAN1 TX (AF9) */ \
    PIN_AFIO_AF(2, 12U)|  /* SD CMD (AF12) */ \
    PIN_AFIO_AF(3, 0U) |  /* Output */ \
    PIN_AFIO_AF(4, 0U) |  /* Output */ \
    PIN_AFIO_AF(5, 0U) |  /* Output */ \
    PIN_AFIO_AF(6, 0U) |  /* Input */ \
    PIN_AFIO_AF(7, 0U)    /* Input */ \
)
#define VAL_GPIOD_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* Output */ \
    PIN_AFIO_AF(9, 0U)   | /* Output */ \
    PIN_AFIO_AF(10, 0U)  | /* Output */ \
    PIN_AFIO_AF(11, 0U)  | /* Output */ \
    PIN_AFIO_AF(12, 0U)  | /* Output */ \
    PIN_AFIO_AF(13, 0U)  | /* Output */ \
    PIN_AFIO_AF(14, 0U)  | /* Output */ \
    PIN_AFIO_AF(15, 0U)    /* Output */ \
)

// ----------------------------------------------------------------------
// GPIOE Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOE (PEx) pin:
// - UART8 RX/TX uses alternate function AF8
// - Digital inputs for triggers and generic signals
// - High side drivers set as outputs
// ----------------------------------------------------------------------

// GPIOE mode setup: set each pin's function
#define VAL_GPIOE_MODER ( \
    PIN_MODE_ALTERNATE(0) | /* PE0: UART8 RX (AF8) */ \
    PIN_MODE_ALTERNATE(1) | /* PE1: UART8 TX (AF8) */ \
    PIN_MODE_INPUT(2)     | /* PE2: Digital input - trigger 1 */ \
    PIN_MODE_INPUT(3)     | /* PE3: Digital input - trigger 2 */ \
    PIN_MODE_INPUT(4)     | /* PE4: Digital input - trigger 3 */ \
    PIN_MODE_INPUT(5)     | /* PE5: Digital input - trigger 4 */ \
    PIN_MODE_INPUT(6)     | /* PE6: Digital input - 7 */ \
    PIN_MODE_OUTPUT(7)    | /* PE7: Output - High side 9 */ \
    PIN_MODE_OUTPUT(8)    | /* PE8: Output - High side 12 */ \
    PIN_MODE_OUTPUT(9)    | /* PE9: Output - High side 11 */ \
    PIN_MODE_OUTPUT(10)   | /* PE10: Output - High side 10 */ \
    PIN_MODE_OUTPUT(11)   | /* PE11: Output - High side 5 */ \
    PIN_MODE_OUTPUT(12)   | /* PE12: Output - High side 4 */ \
    PIN_MODE_OUTPUT(13)   | /* PE13: Output - High side 3 */ \
    PIN_MODE_OUTPUT(14)   | /* PE14: Output - High side 2 */ \
    PIN_MODE_OUTPUT(15)     /* PE15: Output - High side 1 */ \
)

// GPIOE output type setup: push-pull for all (safe default)
#define VAL_GPIOE_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOE output speed: high speed for outputs/altfunc
#define VAL_GPIOE_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12) | PIN_OSPEED_HIGH(13) | PIN_OSPEED_HIGH(14) | PIN_OSPEED_HIGH(15) \
)

// GPIOE pull-up/down setup: floating for inputs/outputs
#define VAL_GPIOE_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_FLOATING(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_FLOATING(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_FLOATING(10) | PIN_PUPDR_FLOATING(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOE default output value (ODR): all outputs start LOW
#define VAL_GPIOE_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOE alternate function registers:
// - AFRL: PE0–PE7
// - AFRH: PE8–PE15
#define VAL_GPIOE_AFRL ( \
    PIN_AFIO_AF(0, 8U) | /* UART8 RX (AF8) */ \
    PIN_AFIO_AF(1, 8U) | /* UART8 TX (AF8) */ \
    PIN_AFIO_AF(2, 0U) | /* Input */ \
    PIN_AFIO_AF(3, 0U) | /* Input */ \
    PIN_AFIO_AF(4, 0U) | /* Input */ \
    PIN_AFIO_AF(5, 0U) | /* Input */ \
    PIN_AFIO_AF(6, 0U) | /* Input */ \
    PIN_AFIO_AF(7, 0U)   /* Output */ \
)
#define VAL_GPIOE_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* Output */ \
    PIN_AFIO_AF(9, 0U)   | /* Output */ \
    PIN_AFIO_AF(10, 0U)  | /* Output */ \
    PIN_AFIO_AF(11, 0U)  | /* Output */ \
    PIN_AFIO_AF(12, 0U)  | /* Output */ \
    PIN_AFIO_AF(13, 0U)  | /* Output */ \
    PIN_AFIO_AF(14, 0U)  | /* Output */ \
    PIN_AFIO_AF(15, 0U)    /* Output */ \
)

// ----------------------------------------------------------------------
// GPIOF Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOF (PFx) pin:
// - Digital inputs for generic and trigger signals
// - Outputs for stepper 1 driver and low/high side controls
// - Pins not connected configured as safe inputs
// ----------------------------------------------------------------------

// GPIOF mode setup: set each pin's function
#define VAL_GPIOF_MODER ( \
    PIN_MODE_INPUT(0)     | /* PF0: Digital input - 6 */ \
    PIN_MODE_INPUT(1)     | /* PF1: Digital input - 5 */ \
    PIN_MODE_INPUT(2)     | /* PF2: Digital input - 4 */ \
    PIN_MODE_INPUT(3)     | /* PF3: Digital input - 3 */ \
    PIN_MODE_INPUT(4)     | /* PF4: Digital input - 2 */ \
    PIN_MODE_INPUT(5)     | /* PF5: Digital input - 1 */ \
    PIN_MODE_INPUT(6)     | /* PF6: not connected */ \
    PIN_MODE_OUTPUT(7)    | /* PF7: Output - stepper 1 dir */ \
    PIN_MODE_OUTPUT(8)    | /* PF8: Output - stepper 1 step */ \
    PIN_MODE_OUTPUT(9)    | /* PF9: Output - stepper 1 enable */ \
    PIN_MODE_INPUT(10)    | /* PF10: not connected */ \
    PIN_MODE_INPUT(11)    | /* PF11: not connected */ \
    PIN_MODE_INPUT(12)    | /* PF12: not connected */ \
    PIN_MODE_OUTPUT(13)   | /* PF13: Output - Low side 1 */ \
    PIN_MODE_OUTPUT(14)   | /* PF14: Output - Low side 2 */ \
    PIN_MODE_OUTPUT(15)     /* PF15: Output - High side 6 */ \
)

// GPIOF output type setup: push-pull for all (safe default)
#define VAL_GPIOF_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOF output speed: high speed for outputs
#define VAL_GPIOF_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12)| PIN_OSPEED_HIGH(13)| PIN_OSPEED_HIGH(14)| PIN_OSPEED_HIGH(15) \
)

// GPIOF pull-up/down setup: floating for inputs and outputs
#define VAL_GPIOF_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_FLOATING(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_PULLDOWN(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_PULLDOWN(10) | PIN_PUPDR_PULLDOWN(11) | \
    PIN_PUPDR_PULLDOWN(12)| PIN_PUPDR_FLOATING(13)| PIN_PUPDR_FLOATING(14)| PIN_PUPDR_FLOATING(15) \
)

// GPIOF default output value (ODR): all outputs start LOW
#define VAL_GPIOF_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOF alternate function registers:
// - AFRL: PF0–PF7 (all input/output, no alt func)
// - AFRH: PF8–PF15 (all output, no alt func)
#define VAL_GPIOF_AFRL ( \
    PIN_AFIO_AF(0, 0U) | PIN_AFIO_AF(1, 0U) | PIN_AFIO_AF(2, 0U) | PIN_AFIO_AF(3, 0U) | \
    PIN_AFIO_AF(4, 0U) | PIN_AFIO_AF(5, 0U) | PIN_AFIO_AF(6, 0U) | PIN_AFIO_AF(7, 0U) \
)
#define VAL_GPIOF_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* Output */ \
    PIN_AFIO_AF(9, 0U)   | /* Output */ \
    PIN_AFIO_AF(10, 0U)  | /* Input */ \
    PIN_AFIO_AF(11, 0U)  | /* Input */ \
    PIN_AFIO_AF(12, 0U)  | /* Input */ \
    PIN_AFIO_AF(13, 0U)  | /* Output */ \
    PIN_AFIO_AF(14, 0U)  | /* Output */ \
    PIN_AFIO_AF(15, 0U)    /* Output */ \
)

// ----------------------------------------------------------------------
// GPIOG Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOG (PGx) pin:
// - Outputs for high side and low side drivers, and status LEDs
// - Pins not connected configured as safe inputs
// ----------------------------------------------------------------------

// GPIOG mode setup: set each pin's function
#define VAL_GPIOG_MODER ( \
    PIN_MODE_OUTPUT(0)    | /* PG0: Output - High side 7 */ \
    PIN_MODE_OUTPUT(1)    | /* PG1: Output - High side 8 */ \
    PIN_MODE_OUTPUT(2)    | /* PG2: Output - Low side 11 */ \
    PIN_MODE_OUTPUT(3)    | /* PG3: Output - Low side 12 */ \
    PIN_MODE_OUTPUT(4)    | /* PG4: Output - Low side 13 */ \
    PIN_MODE_OUTPUT(5)    | /* PG5: Output - Low side 14 */ \
    PIN_MODE_OUTPUT(6)    | /* PG6: Output - Low side 15 */ \
    PIN_MODE_OUTPUT(7)    | /* PG7: Output - Low side 16 */ \
    PIN_MODE_OUTPUT(8)    | /* PG8: Output - Low side 17 */ \
    PIN_MODE_OUTPUT(9)    | /* PG9: Output - LED Running */ \
    PIN_MODE_OUTPUT(10)   | /* PG10: Output - LED Warning */ \
    PIN_MODE_OUTPUT(11)   | /* PG11: Output - LED Critical */ \
    PIN_MODE_OUTPUT(12)   | /* PG12: Output - LED Comms */ \
    PIN_MODE_INPUT(13)    | /* PG13: not connected */ \
    PIN_MODE_INPUT(14)    | /* PG14: not connected */ \
    PIN_MODE_INPUT(15)      /* PG15: not connected */ \
)

// GPIOG output type setup: push-pull for all (safe default)
#define VAL_GPIOG_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOG output speed: high speed for outputs
#define VAL_GPIOG_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12)| PIN_OSPEED_HIGH(13)| PIN_OSPEED_HIGH(14)| PIN_OSPEED_HIGH(15) \
)

// GPIOG pull-up/down setup: floating for outputs and inputs
#define VAL_GPIOG_PUPDR ( \
    PIN_PUPDR_FLOATING(0) | PIN_PUPDR_FLOATING(1) | PIN_PUPDR_FLOATING(2) | PIN_PUPDR_FLOATING(3) | \
    PIN_PUPDR_FLOATING(4) | PIN_PUPDR_FLOATING(5) | PIN_PUPDR_FLOATING(6) | PIN_PUPDR_FLOATING(7) | \
    PIN_PUPDR_FLOATING(8) | PIN_PUPDR_FLOATING(9) | PIN_PUPDR_FLOATING(10) | PIN_PUPDR_FLOATING(11) | \
    PIN_PUPDR_FLOATING(12)| PIN_PUPDR_PULLDOWN(13)| PIN_PUPDR_PULLDOWN(14)| PIN_PUPDR_PULLDOWN(15) \
)

// GPIOG default output value (ODR): all outputs start LOW
#define VAL_GPIOG_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOG alternate function registers:
// - AFRL: PG0–PG7 (all output/input, no alt func)
// - AFRH: PG8–PG15 (all output/input, no alt func)
#define VAL_GPIOG_AFRL ( \
    PIN_AFIO_AF(0, 0U) | PIN_AFIO_AF(1, 0U) | PIN_AFIO_AF(2, 0U) | PIN_AFIO_AF(3, 0U) | \
    PIN_AFIO_AF(4, 0U) | PIN_AFIO_AF(5, 0U) | PIN_AFIO_AF(6, 0U) | PIN_AFIO_AF(7, 0U) \
)
#define VAL_GPIOG_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | /* Output */ \
    PIN_AFIO_AF(9, 0U)   | /* Output */ \
    PIN_AFIO_AF(10, 0U)  | /* Output */ \
    PIN_AFIO_AF(11, 0U)  | /* Output */ \
    PIN_AFIO_AF(12, 0U)  | /* Output */ \
    PIN_AFIO_AF(13, 0U)  | /* Input */ \
    PIN_AFIO_AF(14, 0U)  | /* Input */ \
    PIN_AFIO_AF(15, 0U)    /* Input */ \
)

// ----------------------------------------------------------------------
// GPIOH Initial Configuration Table
// ----------------------------------------------------------------------
// This section sets the startup configuration for every GPIOH (PHx) pin:
// - PH0: OSC_IN and PH1: OSC_OUT configured for external oscillator/crystal (AF0)
// - Remaining pins not connected, configured as safe inputs
// ----------------------------------------------------------------------

// GPIOH mode setup: set each pin's function
#define VAL_GPIOH_MODER ( \
    PIN_MODE_ALTERNATE(0) | /* PH0: OSC_IN (AF0) */ \
    PIN_MODE_ALTERNATE(1) | /* PH1: OSC_OUT (AF0) */ \
    PIN_MODE_INPUT(2)     | /* PH2: not connected */ \
    PIN_MODE_INPUT(3)     | /* PH3: not connected */ \
    PIN_MODE_INPUT(4)     | /* PH4: not connected */ \
    PIN_MODE_INPUT(5)     | /* PH5: not connected */ \
    PIN_MODE_INPUT(6)     | /* PH6: not connected */ \
    PIN_MODE_INPUT(7)     | /* PH7: not connected */ \
    PIN_MODE_INPUT(8)     | /* PH8: not connected */ \
    PIN_MODE_INPUT(9)     | /* PH9: not connected */ \
    PIN_MODE_INPUT(10)    | /* PH10: not connected */ \
    PIN_MODE_INPUT(11)    | /* PH11: not connected */ \
    PIN_MODE_INPUT(12)    | /* PH12: not connected */ \
    PIN_MODE_INPUT(13)    | /* PH13: not connected */ \
    PIN_MODE_INPUT(14)    | /* PH14: not connected */ \
    PIN_MODE_INPUT(15)      /* PH15: not connected */ \
)

// GPIOH output type setup: push-pull for all (safe default)
#define VAL_GPIOH_OTYPER ( \
    PIN_OTYPE_PUSHPULL(0) | PIN_OTYPE_PUSHPULL(1) | PIN_OTYPE_PUSHPULL(2) | PIN_OTYPE_PUSHPULL(3) | \
    PIN_OTYPE_PUSHPULL(4) | PIN_OTYPE_PUSHPULL(5) | PIN_OTYPE_PUSHPULL(6) | PIN_OTYPE_PUSHPULL(7) | \
    PIN_OTYPE_PUSHPULL(8) | PIN_OTYPE_PUSHPULL(9) | PIN_OTYPE_PUSHPULL(10) | PIN_OTYPE_PUSHPULL(11) | \
    PIN_OTYPE_PUSHPULL(12) | PIN_OTYPE_PUSHPULL(13) | PIN_OTYPE_PUSHPULL(14) | PIN_OTYPE_PUSHPULL(15) \
)

// GPIOH output speed: high speed for oscillator pins
#define VAL_GPIOH_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12)| PIN_OSPEED_HIGH(13)| PIN_OSPEED_HIGH(14)| PIN_OSPEED_HIGH(15) \
)

// GPIOH pull-up/down setup: floating for all inputs/oscillator
#define VAL_GPIOH_PUPDR ( \
    PIN_PUPDR_PULLDOWN(0) | PIN_PUPDR_PULLDOWN(1) | PIN_PUPDR_PULLDOWN(2) | PIN_PUPDR_PULLDOWN(3) | \
    PIN_PUPDR_PULLDOWN(4) | PIN_PUPDR_PULLDOWN(5) | PIN_PUPDR_PULLDOWN(6) | PIN_PUPDR_PULLDOWN(7) | \
    PIN_PUPDR_PULLDOWN(8) | PIN_PUPDR_PULLDOWN(9) | PIN_PUPDR_PULLDOWN(10) | PIN_PUPDR_PULLDOWN(11) | \
    PIN_PUPDR_PULLDOWN(12)| PIN_PUPDR_PULLDOWN(13)| PIN_PUPDR_PULLDOWN(14)| PIN_PUPDR_PULLDOWN(15) \
)

// GPIOH default output value (ODR): all outputs start LOW (safe default for unused pins)
#define VAL_GPIOH_ODR ( \
    PIN_ODR_LOW(0) | PIN_ODR_LOW(1) | PIN_ODR_LOW(2) | PIN_ODR_LOW(3) | PIN_ODR_LOW(4) | PIN_ODR_LOW(5) | \
    PIN_ODR_LOW(6) | PIN_ODR_LOW(7) | PIN_ODR_LOW(8) | PIN_ODR_LOW(9) | PIN_ODR_LOW(10) | \
    PIN_ODR_LOW(11) | PIN_ODR_LOW(12) | PIN_ODR_LOW(13) | PIN_ODR_LOW(14) | PIN_ODR_LOW(15) \
)

// GPIOH alternate function registers:
// - AFRL: PH0–PH7
// - AFRH: PH8–PH15
#define VAL_GPIOH_AFRL ( \
    PIN_AFIO_AF(0, 0U) | /* PH0: OSC_IN (AF0) */ \
    PIN_AFIO_AF(1, 0U) | /* PH1: OSC_OUT (AF0) */ \
    PIN_AFIO_AF(2, 0U) | PIN_AFIO_AF(3, 0U) | PIN_AFIO_AF(4, 0U) | PIN_AFIO_AF(5, 0U) | \
    PIN_AFIO_AF(6, 0U) | PIN_AFIO_AF(7, 0U) \
)
#define VAL_GPIOH_AFRH ( \
    PIN_AFIO_AF(8, 0U)   | PIN_AFIO_AF(9, 0U)   | PIN_AFIO_AF(10, 0U)  | PIN_AFIO_AF(11, 0U) | \
    PIN_AFIO_AF(12, 0U)  | PIN_AFIO_AF(13, 0U)  | PIN_AFIO_AF(14, 0U)  | PIN_AFIO_AF(15, 0U) \
)

// ----------------------------------------------------------------------
// GPIOI Initial Configuration Table (SAFE DEFAULT for unused/not present pins)
// ----------------------------------------------------------------------
// Leave this block as-is if your MCU does not use these pins: all input, pulldown, push-pull, LOW, AF0
// ----------------------------------------------------------------------

#define VAL_GPIOI_MODER (PIN_MODE_INPUT(0) | \
						 PIN_MODE_INPUT(1) | \
						 PIN_MODE_INPUT(2) | \
						 PIN_MODE_INPUT(3) | \
						 PIN_MODE_INPUT(4) | \
						 PIN_MODE_INPUT(5) | \
						 PIN_MODE_INPUT(6) | \
						 PIN_MODE_INPUT(7) | \
						 PIN_MODE_INPUT(8) | \
						 PIN_MODE_INPUT(9) | \
						 PIN_MODE_INPUT(10) | \
						 PIN_MODE_INPUT(11) | \
						 PIN_MODE_INPUT(12) | \
						 PIN_MODE_INPUT(13) | \
						 PIN_MODE_INPUT(14) | \
						 PIN_MODE_INPUT(15))

#define VAL_GPIOI_OTYPER (PIN_OTYPE_PUSHPULL(0) | \
						 PIN_OTYPE_PUSHPULL(1) | \
						 PIN_OTYPE_PUSHPULL(2) | \
						 PIN_OTYPE_PUSHPULL(3) | \
						 PIN_OTYPE_PUSHPULL(4) | \
						 PIN_OTYPE_PUSHPULL(5) | \
						 PIN_OTYPE_PUSHPULL(6) | \
						 PIN_OTYPE_PUSHPULL(7) | \
						 PIN_OTYPE_PUSHPULL(8) | \
						 PIN_OTYPE_PUSHPULL(9) | \
						 PIN_OTYPE_PUSHPULL(10) | \
						 PIN_OTYPE_PUSHPULL(11) | \
						 PIN_OTYPE_PUSHPULL(12) | \
						 PIN_OTYPE_PUSHPULL(13) | \
						 PIN_OTYPE_PUSHPULL(14) | \
						 PIN_OTYPE_PUSHPULL(15))

#define VAL_GPIOI_OSPEEDR ( \
    PIN_OSPEED_HIGH(0) | PIN_OSPEED_HIGH(1) | PIN_OSPEED_HIGH(2) | PIN_OSPEED_HIGH(3) | \
    PIN_OSPEED_HIGH(4) | PIN_OSPEED_HIGH(5) | PIN_OSPEED_HIGH(6) | PIN_OSPEED_HIGH(7) | \
    PIN_OSPEED_HIGH(8) | PIN_OSPEED_HIGH(9) | PIN_OSPEED_HIGH(10) | PIN_OSPEED_HIGH(11) | \
    PIN_OSPEED_HIGH(12)| PIN_OSPEED_HIGH(13)| PIN_OSPEED_HIGH(14)| PIN_OSPEED_HIGH(15) \
)

#define VAL_GPIOI_PUPDR ( \
    PIN_PUPDR_PULLDOWN(0) | PIN_PUPDR_PULLDOWN(1) | PIN_PUPDR_PULLDOWN(2) | PIN_PUPDR_PULLDOWN(3) | \
    PIN_PUPDR_PULLDOWN(4) | PIN_PUPDR_PULLDOWN(5) | PIN_PUPDR_PULLDOWN(6) | PIN_PUPDR_PULLDOWN(7) | \
    PIN_PUPDR_PULLDOWN(8) | PIN_PUPDR_PULLDOWN(9) | PIN_PUPDR_PULLDOWN(10) | PIN_PUPDR_PULLDOWN(11) | \
    PIN_PUPDR_PULLDOWN(12)| PIN_PUPDR_PULLDOWN(13)| PIN_PUPDR_PULLDOWN(14)| PIN_PUPDR_PULLDOWN(15) \
)

#define VAL_GPIOI_ODR (PIN_ODR_LOW(0) | \
					   PIN_ODR_LOW(1) | \
					   PIN_ODR_LOW(2) | \
					   PIN_ODR_LOW(3) | \
					   PIN_ODR_LOW(4) | \
					   PIN_ODR_LOW(5) | \
					   PIN_ODR_LOW(6) | \
					   PIN_ODR_LOW(7) | \
					   PIN_ODR_LOW(8) | \
					   PIN_ODR_LOW(9) | \
					   PIN_ODR_LOW(10) | \
					   PIN_ODR_LOW(11) | \
					   PIN_ODR_LOW(12) | \
					   PIN_ODR_LOW(13) | \
					   PIN_ODR_LOW(14) | \
					   PIN_ODR_LOW(15))

#define VAL_GPIOI_AFRL (PIN_AFIO_AF(0, 0U) | \
						PIN_AFIO_AF(1, 0U) | \
						PIN_AFIO_AF(2, 0U) | \
						PIN_AFIO_AF(3, 0U) | \
						PIN_AFIO_AF(4, 0U) | \
						PIN_AFIO_AF(5, 0U) | \
						PIN_AFIO_AF(6, 0U) | \
						PIN_AFIO_AF(7, 0U))

#define VAL_GPIOI_AFRH (PIN_AFIO_AF(8, 0U) | \
						PIN_AFIO_AF(9, 0U) | \
						PIN_AFIO_AF(10, 0U) | \
						PIN_AFIO_AF(11, 0U) | \
						PIN_AFIO_AF(12, 0U) | \
						PIN_AFIO_AF(13, 0U) | \
						PIN_AFIO_AF(14, 0U) | \
						PIN_AFIO_AF(15, 0U))

#endif /* BOARD_IO_H */