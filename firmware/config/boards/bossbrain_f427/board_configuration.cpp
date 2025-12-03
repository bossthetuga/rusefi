#include "pch.h"
#include "board_overrides.h"
#include "board.h"  // So all your Gpio:: pins/macros are defined

// Board-specific LED pins
Gpio getCommsLedPin()    { return Gpio::G12; }
Gpio getRunningLedPin()  { return Gpio::G9;  }
Gpio getWarningLedPin()  { return Gpio::G10; }

static void customBoardDefaultConfiguration() {
    // Injectors (change pin numbers/labels to match your hardware)
    engineConfiguration->injectionPins[0] = Gpio::F13;
    engineConfiguration->injectionPins[1] = Gpio::F14;
    engineConfiguration->injectionPins[2] = Gpio::D8;
    engineConfiguration->injectionPins[3] = Gpio::D9;
    engineConfiguration->injectionPins[4] = Gpio::D10;
    engineConfiguration->injectionPins[5] = Gpio::D11;
    engineConfiguration->injectionPins[6] = Gpio::D12;
    engineConfiguration->injectionPins[7] = Gpio::D13;

    // Ignition
    engineConfiguration->ignitionPins[0] = Gpio::E15;
    engineConfiguration->ignitionPins[1] = Gpio::E14;
    engineConfiguration->ignitionPins[2] = Gpio::E13;
    engineConfiguration->ignitionPins[3] = Gpio::E12;
    engineConfiguration->ignitionPins[4] = Gpio::E11;
    engineConfiguration->ignitionPins[5] = Gpio::F15;
    engineConfiguration->ignitionPins[6] = Gpio::G0;
    engineConfiguration->ignitionPins[7] = Gpio::G1;

    // Throttle control (TLE9201 example)
    engineConfiguration->etbIo[0].controlPin      = Gpio::B8;
    engineConfiguration->etbIo[0].directionPin1   = Gpio::B9;
    engineConfiguration->etbIo[0].disablePin      = Gpio::B7;
    engineConfiguration->etbIo[1].controlPin      = Gpio::Unassigned;
    engineConfiguration->etbIo[1].directionPin1   = Gpio::Unassigned;
    engineConfiguration->etbIo[1].disablePin      = Gpio::Unassigned;
    engineConfiguration->etb_use_two_wires           = false;

    // Idle control stepper
    engineConfiguration->idle.stepperDirectionPin  = Gpio::F7;
    engineConfiguration->idle.stepperStepPin       = Gpio::F8;
    engineConfiguration->stepperEnablePin          = Gpio::F9;

    // engineConfiguration->vbattDividerCoeff        = (6.34 + 1) / 1; // Uncomment if custom divider
    engineConfiguration->analogInputDividerCoefficient= 1.56f;
    engineConfiguration->vbattAdcChannel    = EFI_ADC_0;
    engineConfiguration->adcVcc             = 3.3f;

    // Temperature sensor bias resistors
    engineConfiguration->clt.config.bias_resistor  = 2490;
    engineConfiguration->iat.config.bias_resistor  = 2490;

    // Enable software knock detection
    engineConfiguration->enableSoftwareKnock = true;

    // CAN bus pins (overwrite defaults)
    engineConfiguration->canRxPin     = Gpio::D0;
    engineConfiguration->canTxPin     = Gpio::D1;
    engineConfiguration->can2RxPin    = Gpio::B5;
    engineConfiguration->can2TxPin    = Gpio::B6;
}

// Register the above board-specific configuration
void setup_custom_board_overrides() {
    custom_board_DefaultConfiguration = customBoardDefaultConfiguration;
}