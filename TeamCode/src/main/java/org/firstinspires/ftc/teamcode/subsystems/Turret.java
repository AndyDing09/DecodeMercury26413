package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    private final DcMotorEx turretMotor;
    private final Limelight3A limelight;

    // ==================== STATE MACHINE ====================
    private enum State { IDLE, MANUAL, SEARCHING, TRACKING }
    private State state = State.IDLE;

    // ==================== PID GAINS (tunable) ====================
    private double kP = 0.025;
    private double kI = 0.008;
    private double kD = 0.004;
    private double kF_velocity = 0.006;  // feedforward on target tx velocity (lead moving targets)

    // ==================== TUNING CONSTANTS ====================
    private double DEADZONE = 0.5;          // degrees — stop correcting within this
    private double INTEGRAL_ZONE = 8.0;     // degrees — only accumulate I when error < this
    private double MAX_INTEGRAL = 0.3;      // clamp integral contribution to this power
    private static final double NOMINAL_VOLTAGE = 12.0;

    // ==================== ENCODER LIMITS ====================
    private static final int LEFT_LIMIT  = -400;
    private static final int RIGHT_LIMIT =  400;
    private static final int LIMIT_SOFT_ZONE = 50;  // ticks from limit to start decelerating

    // ==================== SEARCH ====================
    private static final double SEARCH_POWER = 0.30;
    private boolean searchDirection = true;  // true = right

    // ==================== INIT ====================
    private static final double INIT_POWER    = 1.0;
    private static final int    INIT_TOLERANCE = 10;
    private boolean turretInitialized = false;

    // ==================== PID STATE ====================
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastTx = 0;
    private double integralSum = 0;
    private double lastDerivative = 0;
    private boolean pidInitialized = false;

    // Derivative low-pass filter
    private static final double D_FILTER = 0.6;

    // Target loss debounce
    private int framesWithoutTarget = 0;
    private static final int LOSS_DEBOUNCE = 5;  // frames to wait before declaring target lost

    // Last known tx for debounce hold
    private double lastKnownTx = 0;

    // ==================== LIVE TUNING ====================
    private int selectedParam = 0;
    private static final int NUM_PARAMS = 6;
    private static final double STEP_FINE   = 0.001;
    private static final double STEP_COARSE = 0.005;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    // ==================== TELEMETRY STATE ====================
    private String currentMode = "IDLE";
    private double lastOutputPower = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ==================== INIT ====================

    public void centerTurret(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        int error;
        do {
            int currentPos = turretMotor.getCurrentPosition();
            error = -currentPos;
            turretMotor.setPower(Math.signum(error) * INIT_POWER);
            opMode.sleep(20);
        } while (Math.abs(error) > INIT_TOLERANCE && opMode.opModeIsActive());

        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretInitialized = true;
    }

    public void resetEncoder(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        if (!turretInitialized) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretInitialized = true;
            opMode.sleep(200);
        }
    }

    // ==================== MAIN UPDATE ====================

    public void update(boolean toggleTrack, double manualStickX, VoltageSensor voltageSensor,
                       Telemetry telemetry, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        // Toggle auto-tracking
        if (toggleTrack) {
            if (state == State.IDLE) {
                state = State.SEARCHING;
                resetPID();
                searchDirection = true;
            } else {
                state = State.IDLE;
            }
            opMode.sleep(200);
        }

        // Manual override always takes priority
        if (Math.abs(manualStickX) > 0.15) {
            state = State.MANUAL;
        } else if (state == State.MANUAL) {
            // Released stick — go back to searching
            state = State.SEARCHING;
            resetPID();
        }

        int encoderPos = turretMotor.getCurrentPosition();
        double voltage = voltageSensor.getVoltage();
        double outputPower = 0;

        switch (state) {
            case IDLE:
                currentMode = "IDLE";
                outputPower = 0;
                break;

            case MANUAL:
                currentMode = "MANUAL";
                outputPower = manualStickX * 0.5;
                framesWithoutTarget = 0;
                resetPID();
                break;

            case SEARCHING:
                outputPower = doSearch(encoderPos);
                break;

            case TRACKING:
                outputPower = doTracking(voltage);
                break;
        }

        // Check for target acquisition/loss (only in SEARCHING or TRACKING)
        if (state == State.SEARCHING || state == State.TRACKING) {
            LLResult result = limelight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            if (hasTarget) {
                double tx = result.getTx();
                framesWithoutTarget = 0;
                lastKnownTx = tx;

                if (state == State.SEARCHING) {
                    // Target found! Switch to tracking
                    state = State.TRACKING;
                    resetPID();
                    lastTx = tx;
                    pidInitialized = true;
                    pidTimer.reset();
                }

                // Run PID on current frame
                outputPower = doTracking(tx, voltage);
            } else {
                framesWithoutTarget++;

                if (state == State.TRACKING) {
                    if (framesWithoutTarget <= LOSS_DEBOUNCE) {
                        // Hold position using last known tx (decaying)
                        currentMode = "HOLDING (" + framesWithoutTarget + ")";
                        outputPower = 0;
                    } else {
                        // Lost target — go back to searching
                        state = State.SEARCHING;
                        resetPID();
                        // Search in the direction we last saw the target
                        searchDirection = lastKnownTx > 0;
                    }
                }
            }
        }

        // Soft encoder limits — decelerate as we approach limits
        outputPower = applySoftLimits(outputPower, encoderPos);

        // Voltage compensation on output
        if (voltage > 0) {
            outputPower *= (NOMINAL_VOLTAGE / voltage);
        }

        // Final clamp
        outputPower = Math.max(-1.0, Math.min(1.0, outputPower));
        lastOutputPower = outputPower;
        turretMotor.setPower(outputPower);

        // Telemetry
        LLResult tlmResult = limelight.getLatestResult();
        telemetry.addData("Turret", currentMode);
        telemetry.addData("tx",     tlmResult != null && tlmResult.isValid()
                ? String.format("%.2f deg", tlmResult.getTx()) : "N/A");
        telemetry.addData("Power",  String.format("%.2f", lastOutputPower));
        telemetry.addData("Enc",    encoderPos);
    }

    // ==================== SEARCH LOGIC ====================

    private double doSearch(int encoderPos) {
        // Reverse at limits
        if (encoderPos >= RIGHT_LIMIT - LIMIT_SOFT_ZONE) searchDirection = false;
        else if (encoderPos <= LEFT_LIMIT + LIMIT_SOFT_ZONE) searchDirection = true;

        currentMode = "SEARCH " + (searchDirection ? "->" : "<-");
        return searchDirection ? SEARCH_POWER : -SEARCH_POWER;
    }

    // ==================== TRACKING PID ====================

    private double doTracking(double voltage) {
        // Called when we don't have a new frame — hold with 0 power (motor brake holds position)
        return 0;
    }

    private double doTracking(double tx, double voltage) {
        double dt = Math.max(0.001, pidTimer.seconds());
        pidTimer.reset();

        double error = tx;  // tx > 0 means target is to the right of center
        double absTx = Math.abs(tx);

        // Deadzone — close enough, stop correcting
        if (absTx <= DEADZONE) {
            currentMode = "LOCKED";
            // Don't reset integral — maintain hold against drift
            lastTx = tx;
            return 0;
        }

        // Integral — only within zone, prevents windup during large corrections
        if (absTx < INTEGRAL_ZONE) {
            integralSum += error * dt;
            double maxI = MAX_INTEGRAL / Math.max(kI, 1e-9);
            integralSum = Math.max(-maxI, Math.min(maxI, integralSum));
        } else {
            integralSum *= 0.9;  // decay when far away
        }

        // Time-based derivative with low-pass filter
        double rawDerivative = 0;
        if (pidInitialized) {
            rawDerivative = (tx - lastTx) / dt;
        }
        double derivative = D_FILTER * lastDerivative + (1.0 - D_FILTER) * rawDerivative;
        lastDerivative = derivative;
        lastTx = tx;
        pidInitialized = true;

        // PID output (negative because positive tx = target right = turret needs to go right = positive power...
        // but convention may be inverted — sign is tunable by flipping motor direction)
        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;
        double fTerm = kF_velocity * derivative;  // lead moving targets

        double output = -(pTerm + iTerm + dTerm + fTerm);

        // Back-calc anti-windup
        if (Math.abs(output) > 1.0) {
            integralSum -= (output - Math.signum(output)) / Math.max(kI, 1e-9) * dt;
        }

        currentMode = absTx > 5.0 ? "SLEWING" : "TRACKING";
        return output;
    }

    // ==================== SOFT LIMITS ====================

    private double applySoftLimits(double power, int encoderPos) {
        // Approaching right limit — scale down positive power
        if (encoderPos > RIGHT_LIMIT - LIMIT_SOFT_ZONE && power > 0) {
            double distToLimit = Math.max(0, RIGHT_LIMIT - encoderPos);
            double scale = distToLimit / (double) LIMIT_SOFT_ZONE;
            power *= scale;
        }
        // Approaching left limit — scale down negative power
        if (encoderPos < LEFT_LIMIT + LIMIT_SOFT_ZONE && power < 0) {
            double distToLimit = Math.max(0, encoderPos - LEFT_LIMIT);
            double scale = distToLimit / (double) LIMIT_SOFT_ZONE;
            power *= scale;
        }
        // Hard stop at limits
        if ((encoderPos >= RIGHT_LIMIT && power > 0) || (encoderPos <= LEFT_LIMIT && power < 0)) {
            power = 0;
        }
        return power;
    }

    // ==================== PID RESET ====================

    private void resetPID() {
        integralSum = 0;
        lastTx = 0;
        lastDerivative = 0;
        pidInitialized = false;
        pidTimer.reset();
        framesWithoutTarget = 0;
    }

    // ==================== LIVE TUNING ====================

    public void updateTuning(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight, boolean leftBumper) {
        if (dpadUp && !lastDpadUp)     selectedParam = (selectedParam + 1) % NUM_PARAMS;
        if (dpadDown && !lastDpadDown) selectedParam = (selectedParam + NUM_PARAMS - 1) % NUM_PARAMS;

        double step = leftBumper ? STEP_COARSE : STEP_FINE;
        if (dpadLeft && !lastDpadLeft)   adjustParam(selectedParam, -step);
        if (dpadRight && !lastDpadRight) adjustParam(selectedParam, +step);

        lastDpadUp    = dpadUp;
        lastDpadDown  = dpadDown;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;
    }

    private void adjustParam(int index, double delta) {
        switch (index) {
            case 0: kP           = Math.max(0, kP + delta);           break;
            case 1: kI           = Math.max(0, kI + delta);           break;
            case 2: kD           = Math.max(0, kD + delta);           break;
            case 3: kF_velocity  = Math.max(0, kF_velocity + delta);  break;
            case 4: DEADZONE     = Math.max(0.1, DEADZONE + delta);   break;
            case 5: INTEGRAL_ZONE = Math.max(1, INTEGRAL_ZONE + delta * 5); break;
        }
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        telemetry.addLine("--- TURRET TUNER (GP2 dpad) ---");
        String[] names = {"kP", "kI", "kD", "kF_vel", "Deadzone", "I_Zone"};
        double[] vals  = {kP, kI, kD, kF_velocity, DEADZONE, INTEGRAL_ZONE};
        for (int i = 0; i < NUM_PARAMS; i++) {
            String marker = (i == selectedParam) ? "> " : "  ";
            telemetry.addData(marker + names[i], String.format("%.4f", vals[i]));
        }

        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Auto-Track", state != State.IDLE ? "ON" : "OFF");
        telemetry.addData("Turret Enc", turretMotor.getCurrentPosition());
        telemetry.addData("Target",     result != null && result.isValid() ? "YES" : "NO");
    }

    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}
