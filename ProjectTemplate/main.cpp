#include "ClearCore.h"

/**
 * Example: Positive-direction homing using IO-0 as a Normally Closed limit switch.
 *
 * - Motor is configured for Step and Direction (ClearPath-SD).
 * - IO-0 is configured as a digital input (NC switch or NPN/NC sensor).
 * - Positive limit switch is associated with IO-0.
 * - Homing runs as a state machine with timeouts and debounce.
 */

// Make it easy to change which motor connector is used.
#define motor ConnectorM0

// Select the baud rate to match the target serial device.
#define baudRate 9600

// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb

// Homing parameters (steps/second and steps).
#define fastSeekVelocity 20000
#define slowLatchVelocity 2000
#define backoffVelocity 2000
#define backoffSteps 2000

// Timing parameters.
#define debounceMs 10
#define clearSwitchTimeoutMs 3000
#define fastSeekTimeoutMs 8000
#define backoffTimeoutMs 3000
#define slowLatchTimeoutMs 8000

// Helper to update a debounced input state.
static void DebounceInput(bool &rawState,
                          bool &debouncedState,
                          uint32_t &lastChangeTime) {
    bool newRawState = ConnectorIO0.State();
    if (newRawState != rawState) {
        rawState = newRawState;
        lastChangeTime = Milliseconds();
    }

    if ((Milliseconds() - lastChangeTime) >= debounceMs) {
        debouncedState = rawState;
    }
}

// Homing routine: home is in the positive direction using IO-0 as limit switch.
bool Home_Positive_IO0_LimitSwitchPos() {
    // Initialize debounce tracking.
    bool rawState = ConnectorIO0.State();
    bool debouncedState = rawState;
    uint32_t lastChangeTime = Milliseconds();

    // Ensure motor is enabled and ready.
    motor.ClearAlerts();
    if (motor.StatusReg().bit.AlertsPresent) {
        return false;
    }

    // If the NC switch is already de-asserted (open), move negative until it
    // re-asserts (closes). Timeout -> fail.
    if (!debouncedState) {
        motor.MoveVelocity(-backoffVelocity);
        uint32_t startTime = Milliseconds();
        while (true) {
            DebounceInput(rawState, debouncedState, lastChangeTime);
            if (debouncedState) {
                motor.MoveStopAbrupt();
                break;
            }
            if (motor.StatusReg().bit.AlertsPresent) {
                return false;
            }
            if (Milliseconds() - startTime >= clearSwitchTimeoutMs) {
                motor.MoveStopAbrupt();
                return false;
            }
        }

        // Wait for the motor to stop.
        uint32_t stopStart = Milliseconds();
        while (!motor.StepsComplete()) {
            if (motor.StatusReg().bit.AlertsPresent) {
                return false;
            }
            if (Milliseconds() - stopStart >= clearSwitchTimeoutMs) {
                return false;
            }
        }
    }

    // Fast seek toward home (positive direction) until the NC switch opens.
    motor.MoveVelocity(fastSeekVelocity);
    uint32_t fastSeekStart = Milliseconds();
    while (true) {
        DebounceInput(rawState, debouncedState, lastChangeTime);
        if (!debouncedState) {
            break;
        }
        if (motor.StatusReg().bit.AlertsPresent &&
            !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
            return false;
        }
        if (Milliseconds() - fastSeekStart >= fastSeekTimeoutMs) {
            motor.MoveStopAbrupt();
            return false;
        }
    }

    // Wait for motion to finish (limit switch should decelerate/stop).
    uint32_t fastStopStart = Milliseconds();
    while (!motor.StepsComplete()) {
        if (motor.StatusReg().bit.AlertsPresent &&
            !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
            return false;
        }
        if (Milliseconds() - fastStopStart >= fastSeekTimeoutMs) {
            return false;
        }
    }

    // Clear limit alert to allow motion away from the limit.
    motor.ClearAlerts();

    // Backoff: move negative until switch re-asserts or a short distance.
    motor.Move(-backoffSteps);
    uint32_t backoffStart = Milliseconds();
    while (!motor.StepsComplete()) {
        DebounceInput(rawState, debouncedState, lastChangeTime);
        if (debouncedState) {
            motor.MoveStopAbrupt();
            break;
        }
        if (motor.StatusReg().bit.AlertsPresent) {
            return false;
        }
        if (Milliseconds() - backoffStart >= backoffTimeoutMs) {
            motor.MoveStopAbrupt();
            return false;
        }
    }

    if (!debouncedState) {
        return false;
    }

    // Slow latch: creep toward home again for repeatable stop point.
    motor.MoveVelocity(slowLatchVelocity);
    uint32_t slowSeekStart = Milliseconds();
    while (true) {
        DebounceInput(rawState, debouncedState, lastChangeTime);
        if (!debouncedState) {
            break;
        }
        if (motor.StatusReg().bit.AlertsPresent &&
            !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
            return false;
        }
        if (Milliseconds() - slowSeekStart >= slowLatchTimeoutMs) {
            motor.MoveStopAbrupt();
            return false;
        }
    }

    // Wait for motion to finish at the limit.
    uint32_t slowStopStart = Milliseconds();
    while (!motor.StepsComplete()) {
        if (motor.StatusReg().bit.AlertsPresent &&
            !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
            return false;
        }
        if (Milliseconds() - slowStopStart >= slowLatchTimeoutMs) {
            return false;
        }
    }

    motor.ClearAlerts();

    // Zero the motor reference at home.
    motor.PositionRefSet(0);

    return true;
}

int main(void) {
    // Configure IO-0 as a digital input for the NC home/limit switch.
    ConnectorIO0.Mode(Connector::INPUT_DIGITAL);

    // Configure motor for Step and Direction mode.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);

    // Set velocity and acceleration limits.
    motor.VelMax(fastSeekVelocity);
    motor.AccelMax(100000);

    // Associate the positive limit switch with IO-0 (NC input required).
    motor.LimitSwitchPos(CLEARCORE_PIN_IO0);

    // Ensure motor starts disabled.
    motor.EnableRequest(false);

    // Set up USB serial. The program will continue even if no terminal is open.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();

    // Wait up to 5 seconds for a terminal to connect.
    const uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    while (!SerialPort && (Milliseconds() - startTime < timeout)) {
        continue;
    }

    // Enable the motor and run the homing routine.
    motor.EnableRequest(true);
    if (SerialPort) {
        SerialPort.SendLine("Starting positive-direction homing on IO-0...");
    }

    bool homed = Home_Positive_IO0_LimitSwitchPos();
    if (SerialPort) {
        SerialPort.SendLine(homed ? "Homing complete." : "Homing failed.");
    }

    while (true) {
        continue;
    }
}
