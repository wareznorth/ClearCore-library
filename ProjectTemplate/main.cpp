#include "ClearCore.h"
#include <math.h>

/**
 * Example: Positive-direction homing using IO-0 as a Normally Closed limit switch
 * with an enable switch on DI-6.
 *
 * Behavior summary:
 * - DI-6 (NC or NO, user-defined) enables/disables the motor with debounce.
 * - IO-0 is a Normally Closed (NC) home/limit switch for positive homing.
 * - Homing is a non-blocking state machine.
 * - If the home switch is already tripped when enabling, homing is skipped and
 *   position reference is zeroed immediately.
 */

// Make it easy to change which motor connector is used.
#define motor ConnectorM0

// Select the baud rate to match the target serial device.
#define baudRate 9600

// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb

// Motion parameters (RPM and steps).
// MSP Input Resolution is set to 800 pulses/rev.
#define pulsesPerRev 800
#define fastSeekRpm 3000
#define slowLatchRpm 150
#define backoffSteps 2000

// Motion limits (steps/second^2).
#define accelMax 1066676
#define stopDecel 1066676

// Timing parameters.
#define debounceMs 10
#define fastSeekTimeoutMs 15000
#define backoffTimeoutMs 3000
#define slowLatchTimeoutMs 8000
#define revPulseWidthMs 50
#define hlfbReportMs 100

// Buttonfullmov parameters.
#define buttonFullmovRpm 200
#define buttonFullmovCounts 256000


// Debounce helper for a digital input.
struct DebounceInput {
    bool rawState;
    bool debouncedState;
    uint32_t lastChangeTime;
};

static void DebounceUpdate(DebounceInput &input, bool newRawState) {
    if (newRawState != input.rawState) {
        input.rawState = newRawState;
        input.lastChangeTime = Milliseconds();
    }

    if ((Milliseconds() - input.lastChangeTime) >= debounceMs) {
        input.debouncedState = input.rawState;
    }
}

// Return true when the home/limit switch is tripped (NC switch opens).
static bool ReadHomeTrippedDebounced(DebounceInput &homeInput) {
    DebounceUpdate(homeInput, ConnectorIO0.State());
    return !homeInput.debouncedState;
}

// Read the enable switch with debounce.
static bool ReadEnableDebounced(DebounceInput &enableInput) {
    DebounceUpdate(enableInput, ConnectorDI6.State());
    return enableInput.debouncedState;
}

// Homing state machine.
enum HomingState {
    HOMING_IDLE,
    HOMING_FAST_SEEK,
    HOMING_FAST_WAIT_STOP,
    HOMING_BACKOFF,
    HOMING_BACKOFF_WAIT_STOP,
    HOMING_SLOW_SEEK,
    HOMING_SLOW_WAIT_STOP,
    HOMING_COMPLETE,
    HOMING_FAILED
};

enum ButtonfullmovState {
    BUTTONFULLMOV_IDLE,
    BUTTONFULLMOV_RUNNING
};

struct HomingContext {
    HomingState state;
    uint32_t stateStartMs;
    bool homed;
};

static void HomingStateEnter(HomingContext &ctx, HomingState nextState) {
    ctx.state = nextState;
    ctx.stateStartMs = Milliseconds();
}

static int32_t RpmToPulsesPerSec(int32_t rpm) {
    return (rpm * pulsesPerRev) / 60;
}

static void ReportHlfbTorque(uint32_t &lastReportMs) {
    if (!SerialPort) {
        return;
    }

    if (Milliseconds() - lastReportMs < hlfbReportMs) {
        return;
    }

    lastReportMs = Milliseconds();

    // Write the HLFB state to the serial port
    MotorDriver::HlfbStates hlfbState = motor.HlfbState();
    if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
        // Writes the torque measured, as a percent of motor peak torque rating
        SerialPort.Send(int8_t(round(motor.HlfbPercent())));
        SerialPort.SendLine("% torque");
    }
}

static void LogAlertsAndPosition(const char *label) {
    if (!SerialPort) {
        return;
    }

    SerialPort.SendLine(label);
    SerialPort.Send("AlertReg: ");
    SerialPort.SendLine(motor.AlertReg().reg);
    SerialPort.Send("PositionRefCommanded: ");
    SerialPort.SendLine(motor.PositionRefCommanded());
}

static void HomeLimitReached(const char *label) {
    LogAlertsAndPosition(label);
    // Per ClearCore limit switch recovery guidance:
    // 1) Do not command any further positive motion while the limit alert is set.
    // 2) Clear alerts before commanding motion in the opposite direction.
    motor.ClearAlerts();
    motor.PositionRefSet(0);
    if (SerialPort) {
        SerialPort.Send("PositionRefCommanded after zero: ");
        SerialPort.SendLine(motor.PositionRefCommanded());
    }
}

// Advance the homing state machine once per loop iteration.
static void HomingUpdate(HomingContext &ctx, bool homeTripped) {
    switch (ctx.state) {
        case HOMING_IDLE:
            break;

        case HOMING_FAST_SEEK:
            // Move toward home in the positive direction. The limit switch will
            // command an automatic decel/stop when it de-asserts.
            motor.MoveVelocity(RpmToPulsesPerSec(fastSeekRpm));
            HomingStateEnter(ctx, HOMING_FAST_WAIT_STOP);
            break;

        case HOMING_FAST_WAIT_STOP:
            // Wait for limit to trip and motion to stop.
            if (motor.AlertReg().bit.MotionCanceledPositiveLimit) {
                // Limit switch triggered; wait for stop completion.
                if (motor.StepsComplete()) {
                    HomeLimitReached("Home limit reached (fast seek).");
                    HomingStateEnter(ctx, HOMING_BACKOFF);
                }
            }
            if (motor.StatusReg().bit.AlertsPresent &&
                !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            if (Milliseconds() - ctx.stateStartMs >= fastSeekTimeoutMs) {
                motor.MoveStopDecel(stopDecel);
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            break;

        case HOMING_BACKOFF:
            // Move away from the switch in negative direction.
            motor.Move(-backoffSteps);
            HomingStateEnter(ctx, HOMING_BACKOFF_WAIT_STOP);
            break;

        case HOMING_BACKOFF_WAIT_STOP:
            // Stop once switch re-asserts (closed) or timeout.
            if (!homeTripped) {
                motor.MoveStopDecel(stopDecel);
                if (motor.StepsComplete()) {
                    HomingStateEnter(ctx, HOMING_SLOW_SEEK);
                }
            } else if (motor.StepsComplete()) {
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            if (motor.StatusReg().bit.AlertsPresent) {
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            if (Milliseconds() - ctx.stateStartMs >= backoffTimeoutMs) {
                motor.MoveStopDecel(stopDecel);
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            break;

        case HOMING_SLOW_SEEK:
            // Approach the switch slowly for a repeatable stop point.
            motor.MoveVelocity(RpmToPulsesPerSec(slowLatchRpm));
            HomingStateEnter(ctx, HOMING_SLOW_WAIT_STOP);
            break;

        case HOMING_SLOW_WAIT_STOP:
            if (motor.AlertReg().bit.MotionCanceledPositiveLimit) {
                if (motor.StepsComplete()) {
                    HomeLimitReached("Home limit reached (slow latch).");
                    ctx.homed = true;
                    HomingStateEnter(ctx, HOMING_COMPLETE);
                }
            }
            if (motor.StatusReg().bit.AlertsPresent &&
                !motor.AlertReg().bit.MotionCanceledPositiveLimit) {
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            if (Milliseconds() - ctx.stateStartMs >= slowLatchTimeoutMs) {
                motor.MoveStopDecel(stopDecel);
                HomingStateEnter(ctx, HOMING_FAILED);
            }
            break;

        case HOMING_COMPLETE:
        case HOMING_FAILED:
            // Stay in terminal state until externally reset.
            break;
    }
}

int main(void) {
    // Configure IO-0 as a digital input for the NC home/limit switch.
    ConnectorIO0.Mode(Connector::INPUT_DIGITAL);

    // Configure IO-4 as a digital input for the button.
    ConnectorIO4.Mode(Connector::INPUT_DIGITAL);

    // Configure IO-1 as a digital output (home limit indicator).
    ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);

    // Configure IO-3 as a digital output (once-per-rev pulse).
    ConnectorIO3.Mode(Connector::OUTPUT_DIGITAL);

    // Configure motor for Step and Direction mode.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);

    // Configure HLFB for ASG-Position w/ Measured Torque (bipolar PWM) at 482 Hz.
    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Set velocity and acceleration limits.
    motor.VelMax(RpmToPulsesPerSec(fastSeekRpm));
    motor.AccelMax(accelMax);

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

    DebounceInput homeInput = {ConnectorIO0.State(), ConnectorIO0.State(), Milliseconds()};
    DebounceInput buttonInput = {ConnectorIO4.State(), ConnectorIO4.State(), Milliseconds()};
    HomingContext homing = {HOMING_IDLE, Milliseconds(), false};
    bool motorEnabled = false;
    int32_t lastRevIndex = motor.PositionRefCommanded() / pulsesPerRev;
    uint32_t revPulseStartMs = 0;
    bool revPulseActive = false;
    ButtonfullmovState buttonFullmovState = BUTTONFULLMOV_IDLE;
    bool buttonPrevState = buttonInput.debouncedState;
    uint32_t lastHlfbReportMs = 0;
    // Power-up homing flow:
    // 1) If enable switch is OFF, auto-enable the motor once.
    // 2) If home switch is tripped, clear alerts and back off.
    // 3) Seek toward home (fast then slow latch) to re-trip the limit.
    // 4) Disable after homing completes or fails.

    // Power-up homing runs before DI-6 is configured.
    motor.EnableRequest(true);
    motor.ClearAlerts();
    motorEnabled = true;
    if (SerialPort) {
        SerialPort.SendLine("Power-up homing: motor enabled.");
    }

    bool homeTripped = ReadHomeTrippedDebounced(homeInput);
    if (homeTripped) {
        HomingStateEnter(homing, HOMING_BACKOFF);
        if (SerialPort) {
            SerialPort.SendLine("Power-up: home switch tripped, backing off.");
        }
    } else {
        HomingStateEnter(homing, HOMING_FAST_SEEK);
        if (SerialPort) {
            SerialPort.SendLine("Homing started at power-up.");
        }
    }

    while (homing.state != HOMING_COMPLETE && homing.state != HOMING_FAILED) {
        homeTripped = ReadHomeTrippedDebounced(homeInput);
        HomingUpdate(homing, homeTripped);
    }

    if (SerialPort) {
        SerialPort.SendLine(homing.state == HOMING_COMPLETE ?
                            "Power-up homing complete." :
                            "Power-up homing failed.");
    }

    motor.MoveStopDecel(stopDecel);
    motor.EnableRequest(false);
    motorEnabled = false;

    // Configure DI-6 as a digital input for the enable switch.
    ConnectorDI6.Mode(Connector::INPUT_DIGITAL);
    // Use DI-6 as the motor enable connector.
    motor.EnableConnector(CLEARCORE_PIN_DI6);
    DebounceInput enableInput = {ConnectorDI6.State(), ConnectorDI6.State(), Milliseconds()};

    while (true) {
        // Sample debounced input states (enable, home, button) and compute edges.
        bool enableRequested = ReadEnableDebounced(enableInput);
        homeTripped = ReadHomeTrippedDebounced(homeInput);
        DebounceUpdate(buttonInput, ConnectorIO4.State());
        bool buttonPressed = buttonInput.debouncedState;
        int32_t currentRevIndex = motor.PositionRefCommanded() / pulsesPerRev;
        bool buttonRisingEdge = buttonPressed && !buttonPrevState;
        buttonPrevState = buttonPressed;

        // Handle enable switch transitions: enable motor and start/skip homing,
        // or stop motion and disable immediately when enable is removed.
        if (enableRequested && !motorEnabled) {
            motor.EnableRequest(true);
            motor.ClearAlerts();
            motorEnabled = true;
            if (SerialPort) {
                SerialPort.SendLine("Enable ON: motor enabled.");
            }

            // If home switch is already tripped, skip motion and set home.
            if (homeTripped) {
                motor.PositionRefSet(0);
                homing.homed = true;
                HomingStateEnter(homing, HOMING_COMPLETE);
                if (SerialPort) {
                    SerialPort.SendLine("Home switch already tripped. Homing skipped.");
                }
            } else {
                homing.homed = false;
                HomingStateEnter(homing, HOMING_FAST_SEEK);
                if (SerialPort) {
                    SerialPort.SendLine("Homing started.");
                }
            }
        }

        if (!enableRequested && motorEnabled) {
            // Stop any motion before disabling.
            motor.MoveStopDecel(stopDecel);
            motor.EnableRequest(false);
            motorEnabled = false;
            if (SerialPort) {
                SerialPort.SendLine("Enable OFF: motor disabled.");
            }
            HomingStateEnter(homing, HOMING_IDLE);
            buttonFullmovState = BUTTONFULLMOV_IDLE;
            motor.VelMax(RpmToPulsesPerSec(fastSeekRpm));
        }

        // Update IO indicators and generate the once-per-rev pulse output.
        // Update home limit indicator on IO-1 (inverted).
        ConnectorIO1.State(!homeTripped);

        // Generate a pulse on IO-3 once per revolution.
        if (currentRevIndex != lastRevIndex) {
            lastRevIndex = currentRevIndex;
            revPulseActive = true;
            revPulseStartMs = Milliseconds();
            ConnectorIO3.State(true);
        }
        if (revPulseActive &&
            (Milliseconds() - revPulseStartMs >= revPulseWidthMs)) {
            revPulseActive = false;
            ConnectorIO3.State(false);
        }

        // Report HLFB torque measurements at a fixed interval over serial.
        ReportHlfbTorque(lastHlfbReportMs);

        if (motorEnabled) {
            // Start the Buttonfullmov on a rising edge when homing is idle.
            if (buttonRisingEdge && homing.state == HOMING_IDLE &&
                buttonFullmovState == BUTTONFULLMOV_IDLE) {
                motor.VelMax(RpmToPulsesPerSec(buttonFullmovRpm));
                motor.Move(-buttonFullmovCounts);
                buttonFullmovState = BUTTONFULLMOV_RUNNING;
                if (SerialPort) {
                    SerialPort.SendLine("Buttonfullmov started.");
                }
            }

            // Detect completion of the Buttonfullmov, log position, and start homing.
            if (buttonFullmovState == BUTTONFULLMOV_RUNNING &&
                motor.StepsComplete()) {
                buttonFullmovState = BUTTONFULLMOV_IDLE;
                motor.VelMax(RpmToPulsesPerSec(fastSeekRpm));
                if (SerialPort) {
                    SerialPort.Send("Buttonfullmov complete. PositionRefCommanded: ");
                    SerialPort.SendLine(motor.PositionRefCommanded());
                }
                if (homeTripped) {
                    motor.PositionRefSet(0);
                    homing.homed = true;
                    HomingStateEnter(homing, HOMING_COMPLETE);
                    if (SerialPort) {
                        SerialPort.SendLine("Home switch already tripped. Homing skipped.");
                    }
                } else {
                    homing.homed = false;
                    HomingStateEnter(homing, HOMING_FAST_SEEK);
                    if (SerialPort) {
                        SerialPort.SendLine("Homing started after Buttonfullmov.");
                    }
                }
            }

            // Advance the non-blocking homing state machine.
            HomingUpdate(homing, homeTripped);

            if (homing.state == HOMING_COMPLETE) {
                if (SerialPort) {
                    SerialPort.SendLine("Homing complete.");
                }
                HomingStateEnter(homing, HOMING_IDLE);
            } else if (homing.state == HOMING_FAILED) {
                if (SerialPort) {
                    SerialPort.SendLine("Homing failed (fault or timeout)." );
                }
                HomingStateEnter(homing, HOMING_IDLE);
            }
        }
    }
}
