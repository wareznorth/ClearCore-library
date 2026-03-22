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
#define faultFlashIntervalMs 250
#define proxWindowMs 100
#define proxMinHz 5
#define proxLogIntervalMs 500
#define stallReverseCounts 5000
#define proxZeroWindowsForStall 2
#define proxAvgEdges 5
#define jogLongPressMs 5000
#define jogRpm 400
#define pressureSensorMaxPsi 3000.0f
#define pressureSensorMaxVolts 10.0f
#define pressureLogIntervalMs 500

// Runtime switch: true uses A-11 Hz for speed adjustment; false uses HLFB.
bool useA11HzControl = true;

// Proximity measurement via rising-edge interrupt on A-11.
volatile uint32_t proxLastRiseUs = 0;
volatile uint32_t proxPeriodUs = 0;
volatile bool proxPeriodUpdated = false;
volatile uint32_t proxAvgPeriodUs = 0;
volatile bool proxAvgPeriodUpdated = false;
volatile uint32_t proxPeriodSamples[proxAvgEdges] = {0};
volatile uint32_t proxPeriodSumUs = 0;
volatile uint8_t proxPeriodIndex = 0;
volatile uint8_t proxPeriodCount = 0;

static void ProximityRiseCallback();

// Buttonfullmov parameters.
#define buttonFullmovRpm 200
#define buttonFullmovCounts 256000

// ButtonHalfmov parameters.
#define buttonHalfmovRpm 200
#define buttonHalfmovCounts 128000

// Torque regulation parameters (Button moves only).
#define torqueTargetPercent -10.0f
#define torqueRegMaxRpm 200.0f
#define torqueRegMinRpm 5.0f
#define torqueRegGainRpmPerPercent 0.80f
#define torqueRegIntervalMs 0
#define torqueRecoveryRampSeconds 3.0f
#define torqueRecoveryMaxIncreaseRpmPerSec ((torqueRegMaxRpm - torqueRegMinRpm) / torqueRecoveryRampSeconds)
#define hlfbAvgSamples 5


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
    BUTTONFULLMOV_RUNNING,
    BUTTONFULLMOV_STOPPING
};

enum ButtonHalfmovState {
    BUTTONHALFMOV_IDLE,
    BUTTONHALFMOV_RUNNING,
    BUTTONHALFMOV_STOPPING
};

enum StallState {
    STALL_IDLE,
    STALL_WAIT_STOP,
    STALL_REVERSING
};

enum StallResumeSource {
    STALL_RESUME_NONE,
    STALL_RESUME_FULL,
    STALL_RESUME_HALF
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

// The function to be triggered on a rising-edge interrupt.
static void ProximityRiseCallback() {
    uint32_t nowUs = Microseconds();
    if (proxLastRiseUs != 0) {
        uint32_t newPeriodUs = nowUs - proxLastRiseUs;
        proxPeriodUs = newPeriodUs;
        proxPeriodUpdated = true;

        if (proxPeriodCount < proxAvgEdges) {
            proxPeriodSamples[proxPeriodIndex] = newPeriodUs;
            proxPeriodSumUs += newPeriodUs;
            proxPeriodCount++;
        } else {
            proxPeriodSumUs -= proxPeriodSamples[proxPeriodIndex];
            proxPeriodSamples[proxPeriodIndex] = newPeriodUs;
            proxPeriodSumUs += newPeriodUs;
        }
        proxPeriodIndex = (proxPeriodIndex + 1) % proxAvgEdges;

        if (proxPeriodCount == proxAvgEdges) {
            proxAvgPeriodUs = proxPeriodSumUs / proxAvgEdges;
            proxAvgPeriodUpdated = true;
        }
    }
    proxLastRiseUs = nowUs;
}

static int32_t RpmToPulsesPerSec(int32_t rpm) {
    return (rpm * pulsesPerRev) / 60;
}

static float PressurePsiFromVolts(float volts) {
    if (volts <= 0.0f) {
        return 0.0f;
    }
    if (volts >= pressureSensorMaxVolts) {
        return pressureSensorMaxPsi;
    }
    return (volts / pressureSensorMaxVolts) * pressureSensorMaxPsi;
}

static float ClampRpm(float rpm) {
    if (rpm > torqueRegMaxRpm) {
        return torqueRegMaxRpm;
    }
    if (rpm < torqueRegMinRpm) {
        return torqueRegMinRpm;
    }
    return rpm;
}

static float ApplyRecoverySlew(float currentRpm, float requestedRpm, float dtSeconds) {
    if (requestedRpm <= currentRpm) {
        return requestedRpm;
    }
    float maxRise = torqueRecoveryMaxIncreaseRpmPerSec * dtSeconds;
    if (maxRise < 0.0f) {
        maxRise = 0.0f;
    }
    float rise = requestedRpm - currentRpm;
    if (rise > maxRise) {
        return currentRpm + maxRise;
    }
    return requestedRpm;
}

static void ReportHlfbTorque(uint32_t &lastReportMs) {
    (void)lastReportMs;
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

    // Configure IO-2 as a digital input for the half-move button.
    ConnectorIO2.Mode(Connector::INPUT_DIGITAL);

    // Configure IO-1 as a digital output (home limit indicator).
    ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);

    // Configure IO-3 as a digital output (once-per-rev pulse).
    ConnectorIO3.Mode(Connector::OUTPUT_DIGITAL);

    // Configure A-9 as an analog input for the 0-10V pressure transducer.
    ConnectorA9.Mode(Connector::INPUT_ANALOG);
    ConnectorA9.FilterTc(10, AdcManager::FILTER_UNIT_MS);

    // Configure A-11 as a digital input for the proximity sensor pulse output.
    ConnectorA11.Mode(Connector::INPUT_DIGITAL);
    // Set up rising-edge interrupt on A-11 for proximity frequency measurement.
    ConnectorA11.InterruptHandlerSet(ProximityRiseCallback,
                                     InputManager::RISING, false);
    ConnectorA11.InterruptEnable(true);

    // Configure A-12 as a digital input for the proximity sensor pulse output.
    ConnectorA12.Mode(Connector::INPUT_DIGITAL);

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
    DebounceInput buttonHalfInput = {ConnectorIO2.State(), ConnectorIO2.State(), Milliseconds()};
    HomingContext homing = {HOMING_IDLE, Milliseconds(), false};
    bool motorEnabled = false;
    int32_t lastRevIndex = motor.PositionRefCommanded() / pulsesPerRev;
    uint32_t revPulseStartMs = 0;
    bool revPulseActive = false;
    ButtonfullmovState buttonFullmovState = BUTTONFULLMOV_IDLE;
    ButtonHalfmovState buttonHalfmovState = BUTTONHALFMOV_IDLE;
    bool buttonPrevState = buttonInput.debouncedState;
    bool buttonHalfPrevState = buttonHalfInput.debouncedState;
    uint32_t buttonPressStartMs = 0;
    uint32_t buttonHalfPressStartMs = 0;
    bool manualJogMode = false;
    bool manualJogFlashState = false;
    uint32_t lastHlfbReportMs = 0;
    uint32_t lastTorqueRegMs = 0;
    float buttonFullmovRpmCommand = buttonFullmovRpm;
    float buttonHalfmovRpmCommand = buttonHalfmovRpm;
    int32_t buttonFullmovStartPos = 0;
    int32_t buttonHalfmovStartPos = 0;
    bool buttonFullmovCompleted = false;
    bool buttonHalfmovCompleted = false;
    bool stallHoldActive = false;
    bool stallHoldLogged = false;
    StallState stallState = STALL_IDLE;
    StallResumeSource stallResumeSource = STALL_RESUME_NONE;
    bool stallOccurred = false;
    bool stallResumed = false;
    uint32_t faultFlashStartMs = 0;
    bool faultFlashState = false;
    bool faultLogged = false;
    float proxHz = 0.0f;
    uint8_t proxZeroWindows = 0;
    uint32_t proxLastUpdateMs = Milliseconds();
    float proxHzAvg = 0.0f;
    bool proxAvgReady = false;
    float hlfbDutyAvg = 0.0f;
    float hlfbDutySum = 0.0f;
    uint8_t hlfbDutyCount = 0;
    bool hlfbAvgReady = false;
    uint32_t lastPressureLogMs = 0;
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

    if (homing.state == HOMING_COMPLETE) {
        buttonFullmovStartPos = motor.PositionRefCommanded();
        buttonHalfmovStartPos = motor.PositionRefCommanded();
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
        // Main loop order:
        // 1) Sample inputs and detect edges (enable, home, buttons).
        // 2) Update Proxout state/logs and compute proxOk.
        // 3) Handle enable transitions and motor enable/disable.
        // 4) Update indicators/rev pulse or fault flash.
        // 5) Start button moves (full/half) if eligible.
        // 6) Detect Proxout stall and stop motion if needed.
        // 7) Run torque regulation (HLFB) during button moves.
        // 8) Check move completion and start homing if allowed.
        // 9) Handle stall resume checks.
        // 10) Advance homing state machine and handle completion/failure.
        // Sample debounced input states (enable, home, button) and compute edges.
        bool enableRequested = ReadEnableDebounced(enableInput);
        homeTripped = ReadHomeTrippedDebounced(homeInput);
        DebounceUpdate(buttonInput, ConnectorIO4.State());
        DebounceUpdate(buttonHalfInput, ConnectorIO2.State());
        bool buttonPressed = buttonInput.debouncedState;
        bool buttonHalfPressed = buttonHalfInput.debouncedState;
        int32_t currentRevIndex = motor.PositionRefCommanded() / pulsesPerRev;
        bool buttonRisingEdge = buttonPressed && !buttonPrevState;
        bool buttonHalfRisingEdge = buttonHalfPressed && !buttonHalfPrevState;
        buttonPrevState = buttonPressed;
        buttonHalfPrevState = buttonHalfPressed;

        if (buttonPressed) {
            if (buttonPressStartMs == 0) {
                buttonPressStartMs = Milliseconds();
            }
        } else {
            buttonPressStartMs = 0;
        }

        if (buttonHalfPressed) {
            if (buttonHalfPressStartMs == 0) {
                buttonHalfPressStartMs = Milliseconds();
            }
        } else {
            buttonHalfPressStartMs = 0;
        }

        if (!manualJogMode &&
            ((buttonPressStartMs &&
              (Milliseconds() - buttonPressStartMs) >= jogLongPressMs) ||
             (buttonHalfPressStartMs &&
              (Milliseconds() - buttonHalfPressStartMs) >= jogLongPressMs))) {
            manualJogMode = true;
            manualJogFlashState = false;
            motor.MoveStopDecel(stopDecel);
            buttonFullmovState = BUTTONFULLMOV_IDLE;
            buttonHalfmovState = BUTTONHALFMOV_IDLE;
            if (SerialPort) {
                SerialPort.SendLine("Manual jog mode enabled.");
            }
        }

        // Update proximity sensor pulse frequency (Proxout on A-11).
        bool proxUpdated = false;
        bool periodReady = proxPeriodUpdated;
        uint32_t localPeriodUs = proxPeriodUs;
        bool avgPeriodReady = proxAvgPeriodUpdated;
        uint32_t localAvgPeriodUs = proxAvgPeriodUs;
        if (periodReady) {
            proxPeriodUpdated = false;
            if (localPeriodUs > 0) {
                proxHz = 1000000.0f / static_cast<float>(localPeriodUs);
                proxLastUpdateMs = Milliseconds();
                proxUpdated = true;
            }
        }
        if (avgPeriodReady) {
            proxAvgPeriodUpdated = false;
            if (localAvgPeriodUs > 0) {
                proxHzAvg = 1000000.0f / static_cast<float>(localAvgPeriodUs);
                proxAvgReady = true;
                proxUpdated = true;
            }
        }
        if ((Milliseconds() - proxLastUpdateMs) >= proxWindowMs) {
            proxHz = 0.0f;
            proxHzAvg = 0.0f;
            proxAvgReady = false;
            proxUpdated = true;
        }
        // A-12 override switch: assert to force Proxout above threshold.
        if (ConnectorA12.State()) {
            proxHz = proxMinHz + 1.0f;
            proxHzAvg = proxHz;
            proxAvgReady = true;
            proxUpdated = true;
        }
        if (proxUpdated) {
            if (proxHz == 0.0f) {
                if (proxZeroWindows < proxZeroWindowsForStall) {
                    proxZeroWindows++;
                }
            } else {
                proxZeroWindows = 0;
            }
        }
        bool proxOk = proxHz > proxMinHz;

        if (SerialPort && (Milliseconds() - lastPressureLogMs) >= pressureLogIntervalMs) {
            lastPressureLogMs = Milliseconds();
            float pressureVolts = ConnectorA9.AnalogVoltage();
            float pressurePsi = PressurePsiFromVolts(pressureVolts);
            SerialPort.Send("Pressure: ");
            SerialPort.Send(pressurePsi, 1);
            SerialPort.Send(" psi (");
            SerialPort.Send(pressureVolts, 2);
            SerialPort.SendLine(" V)");
        }

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
                buttonFullmovStartPos = 0;
                buttonHalfmovStartPos = 0;
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
            manualJogMode = false;
            manualJogFlashState = false;
            if (SerialPort) {
                SerialPort.SendLine("Enable OFF: motor disabled.");
            }
            HomingStateEnter(homing, HOMING_IDLE);
            buttonFullmovState = BUTTONFULLMOV_IDLE;
            buttonHalfmovState = BUTTONHALFMOV_IDLE;
            motor.VelMax(RpmToPulsesPerSec(fastSeekRpm));
        }

        // Update IO indicators and generate the once-per-rev pulse output.
        bool stepsActive = motor.StatusReg().bit.StepsActive;
        bool alertsPresent = motor.StatusReg().bit.AlertsPresent;
        bool motorFaultedWhileStopped = !stepsActive && alertsPresent;
            if (motorFaultedWhileStopped) {
                if (Milliseconds() - faultFlashStartMs >= faultFlashIntervalMs) {
                    faultFlashStartMs = Milliseconds();
                    faultFlashState = !faultFlashState;
                }
                ConnectorIO1.State(faultFlashState);
                ConnectorIO3.State(!faultFlashState);
                if (!faultLogged && SerialPort) {
                    SerialPort.SendLine("Motor faulted while stopped.");
                    SerialPort.Send("PositionRefCommanded: ");
                    SerialPort.SendLine(motor.PositionRefCommanded());
                    faultLogged = true;
                }
            } else if (manualJogMode) {
                if (Milliseconds() - faultFlashStartMs >= faultFlashIntervalMs) {
                    faultFlashStartMs = Milliseconds();
                    manualJogFlashState = !manualJogFlashState;
                }
                ConnectorIO1.State(manualJogFlashState);
                ConnectorIO3.State(manualJogFlashState);
            } else {
            faultLogged = false;
            faultFlashState = false;

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
        }

        // Report HLFB torque measurements at a fixed interval over serial.
        ReportHlfbTorque(lastHlfbReportMs);

        if (motorEnabled && manualJogMode) {
            int32_t delta =
                motor.PositionRefCommanded() - buttonFullmovStartPos;
            bool atNegativeLimit = delta <= -buttonFullmovCounts;

            if (buttonPressed && !atNegativeLimit) {
                motor.MoveVelocity(-RpmToPulsesPerSec(jogRpm));
            } else if (buttonHalfPressed && !homeTripped) {
                motor.MoveVelocity(RpmToPulsesPerSec(jogRpm));
            } else {
                motor.MoveStopDecel(stopDecel);
            }
        } else if (motorEnabled) {
            // Start the Buttonfullmov on a rising edge when homing is idle.
            if (buttonRisingEdge && homing.state == HOMING_IDLE &&
                buttonFullmovState == BUTTONFULLMOV_IDLE &&
                buttonHalfmovState == BUTTONHALFMOV_IDLE &&
                proxOk) {
                buttonFullmovRpmCommand = buttonFullmovRpm;
                buttonFullmovCompleted = false;
                motor.MoveVelocity(
                    -RpmToPulsesPerSec(
                        static_cast<int32_t>(buttonFullmovRpmCommand)));
                buttonFullmovState = BUTTONFULLMOV_RUNNING;
                stallResumeSource = STALL_RESUME_FULL;
                if (SerialPort) {
                    SerialPort.SendLine("Buttonfullmov started.");
                }
            }

            // Start the ButtonHalfmov on a rising edge when homing is idle.
            if (buttonHalfRisingEdge && homing.state == HOMING_IDLE &&
                buttonHalfmovState == BUTTONHALFMOV_IDLE &&
                buttonFullmovState == BUTTONFULLMOV_IDLE &&
                proxOk) {
                buttonHalfmovRpmCommand = buttonHalfmovRpm;
                buttonHalfmovCompleted = false;
                motor.MoveVelocity(
                    -RpmToPulsesPerSec(
                        static_cast<int32_t>(buttonHalfmovRpmCommand)));
                buttonHalfmovState = BUTTONHALFMOV_RUNNING;
                stallResumeSource = STALL_RESUME_HALF;
                if (SerialPort) {
                    SerialPort.SendLine("ButtonHalfmov started.");
                }
            }

            if (stallState == STALL_IDLE &&
                (buttonFullmovState == BUTTONFULLMOV_RUNNING ||
                 buttonHalfmovState == BUTTONHALFMOV_RUNNING) &&
                proxZeroWindows >= proxZeroWindowsForStall) {
                motor.MoveStopDecel(stopDecel);
                if (buttonFullmovState == BUTTONFULLMOV_RUNNING) {
                    buttonFullmovState = BUTTONFULLMOV_STOPPING;
                }
                if (buttonHalfmovState == BUTTONHALFMOV_RUNNING) {
                    buttonHalfmovState = BUTTONHALFMOV_STOPPING;
                }
                stallHoldActive = true;
                stallHoldLogged = false;
                stallState = STALL_WAIT_STOP;
                stallOccurred = true;
                stallResumed = false;
                if (SerialPort) {
                    SerialPort.SendLine("Proxout 0 Hz windowed. Motion stopped.");
                }
            }
            if (stallState == STALL_WAIT_STOP && motor.StepsComplete()) {
                motor.Move(stallReverseCounts);
                stallState = STALL_REVERSING;
                if (SerialPort) {
                    SerialPort.SendLine("Stall recovery: reversing 5000 counts.");
                }
            }
            if (stallState == STALL_REVERSING && motor.StepsComplete()) {
                stallState = STALL_IDLE;
                stallHoldActive = true;
                if (SerialPort) {
                    SerialPort.SendLine("Stall recovery complete. Waiting for button.");
                }
                if (homeTripped) {
                    stallHoldActive = false;
                    stallHoldLogged = false;
                    homing.homed = false;
                    HomingStateEnter(homing, HOMING_BACKOFF);
                    if (SerialPort) {
                        SerialPort.SendLine(
                            "Home switch tripped during stall recovery. Homing started.");
                    }
                }
            }
            // ================================
            // STALL HOLD RESUME CHECK
            // ================================
            if (stallHoldActive) {
                if (buttonRisingEdge || buttonHalfRisingEdge) {
                    int32_t fullDelta =
                        motor.PositionRefCommanded() - buttonFullmovStartPos;
                    int32_t halfDelta =
                        motor.PositionRefCommanded() - buttonHalfmovStartPos;
                    bool pastHalfCounts =
                        abs(fullDelta) >= buttonHalfmovCounts ||
                        abs(halfDelta) >= buttonHalfmovCounts;
                    bool resumeFullmov = pastHalfCounts || buttonRisingEdge;
                    stallHoldActive = false;
                    stallHoldLogged = false;
                    stallState = STALL_IDLE;
                    stallResumed = true;

                    if (resumeFullmov) {
                        buttonFullmovState = BUTTONFULLMOV_RUNNING;
                        buttonHalfmovState = BUTTONHALFMOV_IDLE;
                        stallResumeSource = STALL_RESUME_FULL;
                        buttonFullmovCompleted = false;
                        motor.MoveVelocity(
                            -RpmToPulsesPerSec(
                                static_cast<int32_t>(buttonFullmovRpmCommand)));
                        if (SerialPort) {
                            SerialPort.SendLine(
                                "Stall cleared. Resuming Buttonfullmov.");
                        }
                    } else {
                        buttonHalfmovState = BUTTONHALFMOV_RUNNING;
                        buttonFullmovState = BUTTONFULLMOV_IDLE;
                        stallResumeSource = STALL_RESUME_HALF;
                        buttonHalfmovCompleted = false;
                        motor.MoveVelocity(
                            -RpmToPulsesPerSec(
                                static_cast<int32_t>(buttonHalfmovRpmCommand)));
                        if (SerialPort) {
                            SerialPort.SendLine(
                                "Stall cleared. Resuming ButtonHalfmov.");
                        }
                    }
                }
            }

            // ================================
            // TORQUE REGULATION (BUTTON MOVES)
            // ================================
            // Adjust commanded velocity based on HLFB duty while a button move
            // is active. No torque regulation is applied during homing.
            if (!manualJogMode &&
                (buttonFullmovState == BUTTONFULLMOV_RUNNING ||
                 buttonHalfmovState == BUTTONHALFMOV_RUNNING)) {
                if (!useA11HzControl &&
                    SerialPort && (Milliseconds() - lastHlfbReportMs >= hlfbReportMs)) {
                    lastHlfbReportMs = Milliseconds();
                    SerialPort.Send("HLFB state: ");

                    // Check the current state of the ClearPath's HLFB.
                    MotorDriver::HlfbStates hlfbState = motor.HlfbState();

                    // Write the HLFB state to the serial port
                    if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
                        float hlfbDuty = motor.HlfbPercent();
                        SerialPort.Send("HLFB duty: ");
                        SerialPort.Send(hlfbDuty, 2);
                        if (hlfbAvgReady) {
                            SerialPort.Send(" avg: ");
                            SerialPort.Send(hlfbDutyAvg, 2);
                        }
                        SerialPort.SendLine("% torque");
                    } else if (hlfbState == MotorDriver::HLFB_ASSERTED) {
                        // Asserted indicates either "Move Done" for position modes, or
                        // "At Target Velocity" for velocity moves
                        SerialPort.SendLine("ASSERTED");
                    } else {
                        SerialPort.SendLine("DISABLED or SHUTDOWN");
                    }
}
                if (torqueRegIntervalMs == 0 ||
                    (Milliseconds() - lastTorqueRegMs) >= torqueRegIntervalMs) {
                    uint32_t nowMs = Milliseconds();
                    float dtSeconds = 0.0f;
                    if (lastTorqueRegMs != 0) {
                        dtSeconds = static_cast<float>(nowMs - lastTorqueRegMs) / 1000.0f;
                    }
                    lastTorqueRegMs = nowMs;
                    if (dtSeconds <= 0.0f) {
                        dtSeconds = 0.001f;
                    }
                    if (useA11HzControl) {
                        if (proxAvgReady) {
                            float error = proxHz - proxHzAvg;
                            if (buttonFullmovState == BUTTONFULLMOV_RUNNING) {
                                float requestedRpm = buttonFullmovRpmCommand +
                                    (error * torqueRegGainRpmPerPercent);
                                requestedRpm = ClampRpm(requestedRpm);
                                buttonFullmovRpmCommand = ApplyRecoverySlew(
                                    buttonFullmovRpmCommand, requestedRpm, dtSeconds);
                                motor.MoveVelocity(
                                    -RpmToPulsesPerSec(
                                        static_cast<int32_t>(buttonFullmovRpmCommand)));
                                if (SerialPort) {
                                    SerialPort.Send("Buttonfullmov RPM cmd: ");
                                    SerialPort.SendLine(
                                        static_cast<int32_t>(buttonFullmovRpmCommand));
                                }
                            } else {
                                float requestedRpm = buttonHalfmovRpmCommand +
                                    (error * torqueRegGainRpmPerPercent);
                                requestedRpm = ClampRpm(requestedRpm);
                                buttonHalfmovRpmCommand = ApplyRecoverySlew(
                                    buttonHalfmovRpmCommand, requestedRpm, dtSeconds);
                                motor.MoveVelocity(
                                    -RpmToPulsesPerSec(
                                        static_cast<int32_t>(buttonHalfmovRpmCommand)));
                                if (SerialPort) {
                                    SerialPort.Send("ButtonHalfmov RPM cmd: ");
                                    SerialPort.SendLine(
                                        static_cast<int32_t>(buttonHalfmovRpmCommand));
                                }
                            }
                        }
                    } else {
                        MotorDriver::HlfbStates hlfbState = motor.HlfbState();
                        if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
                            float measuredDuty = motor.HlfbPercent();
                            hlfbDutySum += measuredDuty;
                            hlfbDutyCount++;
                            if (hlfbDutyCount >= hlfbAvgSamples) {
                                hlfbDutyAvg = hlfbDutySum / static_cast<float>(hlfbDutyCount);
                                hlfbDutySum = 0.0f;
                                hlfbDutyCount = 0;
                                hlfbAvgReady = true;
                            }
                            float dutyForControl = hlfbAvgReady ? hlfbDutyAvg : measuredDuty;
                            float error = torqueTargetPercent - dutyForControl;
                            if (SerialPort) {
                                SerialPort.Send("Torque target: ");
                                SerialPort.Send(torqueTargetPercent, 2);
                                SerialPort.Send("%, HLFB avg used: ");
                                SerialPort.SendLine(dutyForControl, 2);
                            }
                            // RPM COMMAND UPDATE (ERROR SIGN REVERSED)
                            // Negative error -> increase RPM, positive error -> decrease RPM.
                            if (buttonFullmovState == BUTTONFULLMOV_RUNNING) {
                                float requestedRpm = buttonFullmovRpmCommand +
                                    ((-error) * torqueRegGainRpmPerPercent);
                                requestedRpm = ClampRpm(requestedRpm);
                                buttonFullmovRpmCommand = ApplyRecoverySlew(
                                    buttonFullmovRpmCommand, requestedRpm, dtSeconds);
                                motor.MoveVelocity(
                                    -RpmToPulsesPerSec(
                                        static_cast<int32_t>(buttonFullmovRpmCommand)));
                                if (SerialPort) {
                                    SerialPort.Send("Buttonfullmov RPM cmd: ");
                                    SerialPort.SendLine(
                                        static_cast<int32_t>(buttonFullmovRpmCommand));
                                }
                            } else {
                                float requestedRpm = buttonHalfmovRpmCommand +
                                    ((-error) * torqueRegGainRpmPerPercent);
                                requestedRpm = ClampRpm(requestedRpm);
                                buttonHalfmovRpmCommand = ApplyRecoverySlew(
                                    buttonHalfmovRpmCommand, requestedRpm, dtSeconds);
                                motor.MoveVelocity(
                                    -RpmToPulsesPerSec(
                                        static_cast<int32_t>(buttonHalfmovRpmCommand)));
                                if (SerialPort) {
                                    SerialPort.Send("ButtonHalfmov RPM cmd: ");
                                    SerialPort.SendLine(
                                        static_cast<int32_t>(buttonHalfmovRpmCommand));
                                }
                            }
                        }
                    }
                }
            }

            // Detect completion of the Buttonfullmov distance, then stop and home.
            if (buttonFullmovState == BUTTONFULLMOV_RUNNING) {
                int32_t delta =
                    motor.PositionRefCommanded() - buttonFullmovStartPos;
                if (abs(delta) >= buttonFullmovCounts) {
                    buttonFullmovCompleted = true;
                    motor.MoveStopDecel(stopDecel);
                    buttonFullmovState = BUTTONFULLMOV_STOPPING;
                }
            }
            if (buttonFullmovState == BUTTONFULLMOV_STOPPING &&
                motor.StepsComplete()) {
                buttonFullmovState = BUTTONFULLMOV_IDLE;
                stallResumeSource = STALL_RESUME_NONE;
                if (!stallOccurred || stallResumed) {
                    stallHoldActive = false;
                }
                if (!stallHoldActive && buttonFullmovCompleted) {
                    if (homeTripped) {
                        motor.PositionRefSet(0);
                        buttonFullmovStartPos = 0;
                        buttonHalfmovStartPos = 0;
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
                stallOccurred = false;
                stallResumed = false;
            }

            // Detect completion of the ButtonHalfmov distance, then stop and home.
            if (buttonHalfmovState == BUTTONHALFMOV_RUNNING) {
                int32_t delta =
                    motor.PositionRefCommanded() - buttonHalfmovStartPos;
                if (abs(delta) >= buttonHalfmovCounts) {
                    buttonHalfmovCompleted = true;
                    motor.MoveStopDecel(stopDecel);
                    buttonHalfmovState = BUTTONHALFMOV_STOPPING;
                }
            }
            if (buttonHalfmovState == BUTTONHALFMOV_STOPPING &&
                motor.StepsComplete()) {
                buttonHalfmovState = BUTTONHALFMOV_IDLE;
                stallResumeSource = STALL_RESUME_NONE;
                if (!stallOccurred || stallResumed) {
                    stallHoldActive = false;
                }
                if (!stallHoldActive && buttonHalfmovCompleted) {
                    if (homeTripped) {
                        motor.PositionRefSet(0);
                        buttonFullmovStartPos = 0;
                        buttonHalfmovStartPos = 0;
                        homing.homed = true;
                        HomingStateEnter(homing, HOMING_COMPLETE);
                        if (SerialPort) {
                            SerialPort.SendLine("Home switch already tripped. Homing skipped.");
                        }
                    } else {
                        homing.homed = false;
                        HomingStateEnter(homing, HOMING_FAST_SEEK);
                        if (SerialPort) {
                            SerialPort.SendLine("Homing started after ButtonHalfmov.");
                        }
                    }
                }
                stallOccurred = false;
                stallResumed = false;
            }

            if (SerialPort) {
                if (stallHoldActive && !stallHoldLogged) {
                    SerialPort.SendLine("stallHoldActive = true");
                    stallHoldLogged = true;
                } else if (!stallHoldActive && stallHoldLogged) {
                    SerialPort.SendLine("stallHoldActive = false");
                    stallHoldLogged = false;
                }
            }

            // Advance the non-blocking homing state machine.
            HomingUpdate(homing, homeTripped);

            if (homing.state == HOMING_COMPLETE) {
                if (SerialPort) {
                    SerialPort.SendLine("Homing complete.");
                }
                buttonFullmovStartPos = motor.PositionRefCommanded();
                buttonHalfmovStartPos = motor.PositionRefCommanded();
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
