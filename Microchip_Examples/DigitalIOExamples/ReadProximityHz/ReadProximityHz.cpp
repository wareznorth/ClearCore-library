/*
 * Title: ReadProximityHz
 *
 * Objective:
 *    This example demonstrates how to measure an input signal frequency on a
 *    ClearCore digital input using a rising-edge interrupt.
 *
 * Description:
 *    This example attaches an interrupt on ConnectorA12. Each rising edge
 *    captures the elapsed microseconds since the previous rising edge. The
 *    period is converted to Hz and printed to the USB serial port.
 *
 * Requirements:
 * ** A proximity sensor or digital signal source connected to A-12.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 *
 *
 * Copyright (c) 2024 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

#include "ClearCore.h"

// Proximity sensor input pin.
#define ProximityInput ConnectorA12

// Select the baud rate to match the target serial device.
#define baudRate 9600

// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb

// Variables updated in interrupt context.
volatile uint32_t lastRiseUs = 0;
volatile uint32_t periodUs = 0;
volatile bool periodUpdated = false;

// Interrupt handler for rising edges.
void ProximityRiseCallback();

int main() {
    ProximityInput.Mode(Connector::INPUT_DIGITAL);

    // Set up rising-edge interrupt on A-12.
    ProximityInput.InterruptHandlerSet(ProximityRiseCallback,
                                       InputManager::RISING, false);
    ProximityInput.InterruptEnable(true);

    // Set up serial communication at a baud rate of 9600 bps then wait up to
    // 5 seconds for a port to open.
    // Serial communication is not required for this example to run, however the
    // example will appear to do nothing without serial output.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }

    while (true) {
        bool hasUpdate = periodUpdated;
        uint32_t localPeriod = periodUs;
        if (hasUpdate) {
            periodUpdated = false;
        }

        if (hasUpdate && localPeriod > 0) {
            float frequencyHz = 1000000.0f / static_cast<float>(localPeriod);
            SerialPort.Send("A12 frequency (Hz): ");
            SerialPort.SendLine(frequencyHz);
        }

        // Small delay to limit serial output rate.
        Delay_ms(100);
    }
}

// The function to be triggered on a rising-edge interrupt.
/*------------------------------------------------------------------------------
 * ProximityRiseCallback
 *
 *    Captures the time between rising edges.
 *
 * Parameters:
 *    None
 *
 * Returns: None
 */
void ProximityRiseCallback() {
    uint32_t nowUs = Microseconds();
    if (lastRiseUs != 0) {
        periodUs = nowUs - lastRiseUs;
        periodUpdated = true;
    }
    lastRiseUs = nowUs;
}
//------------------------------------------------------------------------------
