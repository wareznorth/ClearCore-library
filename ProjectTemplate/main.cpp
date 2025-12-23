#include "ClearCore.h"

// Specify which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// Specify the home/limit switch connector.
#define HomeSwitch ConnectorIO0

// ClearCore pin ID for IO-0 used by LimitSwitchPos.
#define HOME_SWITCH_PIN CLEARCORE_PIN_IO0

// Homing tuning values (step pulses, pulses/sec, pulses/sec^2, milliseconds).
const int32_t kFastSeekDistance = 200000;
const int32_t kSlowSeekDistance = 20000;
const int32_t kBackoffVelocity = 2000;
const int32_t kFastVelocity = 10000;
const int32_t kSlowVelocity = 1000;
const int32_t kAccelMax = 100000;
const uint32_t kDebounceMs = 20;
const uint32_t kClearSwitchTimeoutMs = 5000;
const uint32_t kFastSeekTimeoutMs = 15000;
const uint32_t kBackoffTimeoutMs = 5000;
const uint32_t kSlowSeekTimeoutMs = 15000;

// Returns true when the NC switch is in its asserted (normal) state.
static bool HomeSwitchAsserted() {
    return HomeSwitch.State();
}

// Wait for the motor to finish motion.
static bool WaitForStepsComplete(uint32_t timeoutMs, bool ignoreAlerts) {
    uint32_t startTime = Milliseconds();
    while (!motor.StepsComplete()) {
        if (!ignoreAlerts && motor.StatusReg().bit.AlertsPresent) {
            return false;
        }
        if (Milliseconds() - startTime >= timeoutMs) {
            return false;
        }
    }
    return true;
}

// Debounce helper: returns true when the input remains in the desired state.
static bool DebouncedSwitchState(bool asserted, uint32_t debounceMs, uint32_t &debounceStart) {
    if (HomeSwitchAsserted() == asserted) {
        if (debounceStart == 0) {
            debounceStart = Milliseconds();
        }
        if (Milliseconds() - debounceStart >= debounceMs) {
            return true;
        }
    }
    else {
        debounceStart = 0;
    }
    return false;
}

// Homing routine where home is in the positive direction using IO-0 as the
// positive limit switch (NC input).
bool Home_Positive_IO0_LimitSwitchPos() {
    enum HomeState {
        HOME_INIT,
        HOME_CLEAR_SWITCH,
        HOME_FAST_SEEK,
        HOME_FAST_WAIT,
        HOME_BACKOFF,
        HOME_BACKOFF_WAIT,
        HOME_SLOW_SEEK,
        HOME_SLOW_WAIT,
        HOME_DONE,
        HOME_FAIL
    };

    HomeState state = HOME_INIT;
    uint32_t stateStart = Milliseconds();
    uint32_t debounceStart = 0;
    bool switchTriggered = false;

    while (state != HOME_DONE && state != HOME_FAIL) {
        switch (state) {
            case HOME_INIT:
                if (motor.StatusReg().bit.AlertsPresent) {
                    return false;
                }
                if (!HomeSwitchAsserted()) {
                    // Already on the switch (de-asserted). Move negative until
                    // the switch re-asserts.
                    motor.VelMax(kBackoffVelocity);
                    motor.AccelMax(kAccelMax);
                    motor.MoveVelocity(-kBackoffVelocity);
                    state = HOME_CLEAR_SWITCH;
                    stateStart = Milliseconds();
                    debounceStart = 0;
                }
                else {
                    state = HOME_FAST_SEEK;
                }
                break;

            case HOME_CLEAR_SWITCH:
                if (DebouncedSwitchState(true, kDebounceMs, debounceStart)) {
                    motor.MoveStopDecel();
                    if (!WaitForStepsComplete(kClearSwitchTimeoutMs, false)) {
                        state = HOME_FAIL;
                    }
                    else {
                        state = HOME_FAST_SEEK;
                    }
                }
                else if (Milliseconds() - stateStart >= kClearSwitchTimeoutMs) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                break;

            case HOME_FAST_SEEK:
                switchTriggered = false;
                motor.VelMax(kFastVelocity);
                motor.AccelMax(kAccelMax);
                motor.Move(kFastSeekDistance);
                state = HOME_FAST_WAIT;
                stateStart = Milliseconds();
                debounceStart = 0;
                break;

            case HOME_FAST_WAIT:
                if (!switchTriggered) {
                    if (DebouncedSwitchState(false, kDebounceMs, debounceStart)) {
                        switchTriggered = true;
                    }
                }

                if (switchTriggered && motor.StepsComplete()) {
                    motor.ClearAlerts();
                    state = HOME_BACKOFF;
                    stateStart = Milliseconds();
                }
                else if (!switchTriggered && motor.StepsComplete()) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                else if (Milliseconds() - stateStart >= kFastSeekTimeoutMs) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                break;

            case HOME_BACKOFF:
                motor.VelMax(kBackoffVelocity);
                motor.AccelMax(kAccelMax);
                motor.MoveVelocity(-kBackoffVelocity);
                stateStart = Milliseconds();
                debounceStart = 0;
                state = HOME_BACKOFF_WAIT;
                break;

            case HOME_BACKOFF_WAIT:
                if (DebouncedSwitchState(true, kDebounceMs, debounceStart)) {
                    motor.MoveStopDecel();
                    if (!WaitForStepsComplete(kBackoffTimeoutMs, false)) {
                        state = HOME_FAIL;
                    }
                    else {
                        state = HOME_SLOW_SEEK;
                    }
                }
                else if (Milliseconds() - stateStart >= kBackoffTimeoutMs) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                break;

            case HOME_SLOW_SEEK:
                switchTriggered = false;
                motor.VelMax(kSlowVelocity);
                motor.AccelMax(kAccelMax);
                motor.Move(kSlowSeekDistance);
                state = HOME_SLOW_WAIT;
                stateStart = Milliseconds();
                debounceStart = 0;
                break;

            case HOME_SLOW_WAIT:
                if (!switchTriggered) {
                    if (DebouncedSwitchState(false, kDebounceMs, debounceStart)) {
                        switchTriggered = true;
                    }
                }

                if (switchTriggered && motor.StepsComplete()) {
                    motor.ClearAlerts();
                    state = HOME_DONE;
                }
                else if (!switchTriggered && motor.StepsComplete()) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                else if (Milliseconds() - stateStart >= kSlowSeekTimeoutMs) {
                    motor.MoveStopDecel();
                    state = HOME_FAIL;
                }
                break;

            case HOME_DONE:
            case HOME_FAIL:
            default:
                break;
        }
    }

    if (state == HOME_DONE) {
        motor.PositionRefSet(0);
        return true;
    }

    return false;
}

int main(void) {
    // Configure IO-0 as a digital input for the NC home switch.
    HomeSwitch.Mode(Connector::INPUT_DIGITAL);

    // Sets the input clocking rate for step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);

    // Associate IO-0 with the positive limit switch input.
    motor.LimitSwitchPos(HOME_SWITCH_PIN);

    // Enable the motor.
    motor.EnableRequest(true);

    // Run the positive-direction homing routine.
    (void)Home_Positive_IO0_LimitSwitchPos();

    while (true) {
        continue;
    }
}
