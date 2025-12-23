#include "ClearCore.h"

/**
 * Example: Enable/disable a motor based on a debounced switch on IO-2.
 *
 * - IO-2 is configured as a digital input.
 * - When the switch is ON (closed), the motor enable is requested.
 * - When the switch is OFF (open), the motor enable is removed.
 * - State changes are printed to USB serial if a terminal is connected.
 */

// Make it easy to change which motor connector is used.
#define motor ConnectorM0

// Select the baud rate to match the target serial device.
#define baudRate 9600

// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb

// Debounce time (ms) for the IO-2 switch.
#define debounceMs 50

int main(void) {
    // Configure IO-2 as a digital input for the switch.
    ConnectorIO2.Mode(Connector::INPUT_DIGITAL);

    // Ensure motor starts disabled.
    motor.EnableRequest(false);

    // Set up USB serial. The program will continue even if no terminal is open.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();

    // Wait up to 5 seconds for a terminal to connect (non-blocking after timeout).
    const uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    while (!SerialPort && (Milliseconds() - startTime < timeout)) {
        continue;
    }

    // Debounce tracking variables.
    bool rawState = ConnectorIO2.State();
    bool debouncedState = rawState;
    uint32_t lastChangeTime = Milliseconds();

    // Report initial state if serial is open.
    if (SerialPort) {
        SerialPort.Send("IO-2 Switch initial state: ");
        SerialPort.SendLine(debouncedState ? "ON" : "OFF");
    }

    while (true) {
        // Read the raw switch state.
        bool newRawState = ConnectorIO2.State();

        // If the raw state changes, restart the debounce timer.
        if (newRawState != rawState) {
            rawState = newRawState;
            lastChangeTime = Milliseconds();
        }

        // If the raw state has been stable long enough, accept it.
        if ((Milliseconds() - lastChangeTime) >= debounceMs &&
            debouncedState != rawState) {
            debouncedState = rawState;

            // Enable or disable the motor based on switch state.
            motor.EnableRequest(debouncedState);

            // Print state change if a serial terminal is connected.
            if (SerialPort) {
                SerialPort.Send("IO-2 Switch state: ");
                SerialPort.SendLine(debouncedState ? "ON" : "OFF");
            }
        }

        // Short delay to reduce CPU usage.
        Delay_ms(1);
    }
}
