package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    private final DcMotor turretMotor;
    private final Limelight3A limelight;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ==================== PID GAINS ====================
    private double kP       = 0.040;
    private double kD       = 0.006;
    private double kF       = 0.12;
    private double DEADZONE = 1.0;
    private double SCAN_PWR = 0.5;

    private static final double MAX_POWER = 0.90;

    // ==================== ENCODER LIMITS ====================
    private static final int LEFT_LIMIT  = -430;
    private static final int RIGHT_LIMIT =  430;

    // ==================== TARGET LOSS ====================
    private static final int LOSS_DEBOUNCE_FRAMES = 8;
    private int framesWithoutTarget = 0;

    // ==================== STATE ====================
    private boolean active = false;
    private double  lastTx        = 0;
    private boolean scanningRight = true;
    private boolean wasTracking   = false;
    private String  currentMode   = "IDLE";

    // ==================== INIT ====================
    private static final double INIT_POWER     = 1.0;
    private static final int    INIT_TOLERANCE = 10;

    // ==================== LIVE TUNER ====================
    private int selectedParam = 0;
    private static final int NUM_PARAMS = 5;
    private static final double STEP_FINE   = 0.001;
    private static final double STEP_COARSE = 0.010;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

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
    }

    public void resetEncoder(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.sleep(200);
    }

    // ==================== MAIN UPDATE ====================

    public void updateTracking(boolean toggleTrack, double manualStickX,
                               VoltageSensor voltageSensor, Telemetry telemetry,
                               com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        // Toggle active with button
        if (toggleTrack) {
            active = !active;
            if (active) {
                lastTx = 0;
                wasTracking = false;
                framesWithoutTarget = 0;
                scanningRight = true;
            }
            opMode.sleep(200);
        }

        if (!active) {
            turretMotor.setPower(0);
            currentMode = "IDLE";
            return;
        }

        double dt = Math.max(0.001, loopTimer.seconds());
        loopTimer.reset();

        LLResult result    = limelight.getLatestResult();
        boolean  hasTarget = result != null && result.isValid();
        int      currentPos = turretMotor.getCurrentPosition();
        double   outputPower = 0;

        // 1. MANUAL OVERRIDE
        if (Math.abs(manualStickX) > 0.1) {
            currentMode     = "MANUAL";
            outputPower     = manualStickX * 0.5;
            lastTx          = 0;
            wasTracking     = false;
            framesWithoutTarget = 0;
        }

        // 2. TRACKING (PD on tx)
        else if (hasTarget) {
            double tx         = result.getTx();
            double derivative = wasTracking ? (tx - lastTx) / dt : 0;

            framesWithoutTarget = 0;
            wasTracking         = true;
            lastTx              = tx;

            if (Math.abs(tx) > DEADZONE) {
                currentMode = Math.abs(tx) > 2.0 ? "SLEWING" : "TRACKING";
                outputPower = (kP * tx) + (kD * derivative);

                // Feedforward: minimum power to overcome herringbone static friction
                if (Math.abs(outputPower) > 0.001 && Math.abs(outputPower) < kF) {
                    outputPower = Math.signum(outputPower) * kF;
                }
            } else {
                currentMode = "LOCKED";
                outputPower = 0;
            }
        }

        // 3. DEBOUNCE HOLD
        else if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
            framesWithoutTarget++;
            currentMode = "HOLDING (" + framesWithoutTarget + "/" + LOSS_DEBOUNCE_FRAMES + ")";
            outputPower = 0;
        }

        // 4. SEARCH
        else {
            currentMode = "SEARCHING";
            wasTracking = false;

            if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                scanningRight = lastTx > 0;
            }
            framesWithoutTarget++;

            if (currentPos >= RIGHT_LIMIT) scanningRight = false;
            else if (currentPos <= LEFT_LIMIT) scanningRight = true;

            outputPower  = scanningRight ? SCAN_PWR : -SCAN_PWR;
            currentMode += scanningRight ? " ->" : " <-";
        }

        // 5. SAFETY — clamp and enforce encoder limits
        outputPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, outputPower));
        if (currentPos <= LEFT_LIMIT  && outputPower < 0) outputPower = 0;
        if (currentPos >= RIGHT_LIMIT && outputPower > 0) outputPower = 0;

        turretMotor.setPower(outputPower);
    }

    // ==================== LIVE TUNING ====================

    public void updateTuning(boolean dpadUp, boolean dpadDown,
                             boolean dpadLeft, boolean dpadRight, boolean leftBumper) {
        if (dpadUp && !lastDpadUp)     selectedParam = (selectedParam + 1) % NUM_PARAMS;
        if (dpadDown && !lastDpadDown) selectedParam = (selectedParam + NUM_PARAMS - 1) % NUM_PARAMS;

        double step = leftBumper ? STEP_COARSE : STEP_FINE;
        if (dpadRight && !lastDpadRight) adjustParam(+step);
        if (dpadLeft && !lastDpadLeft)   adjustParam(-step);

        lastDpadUp    = dpadUp;
        lastDpadDown  = dpadDown;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;
    }

    private void adjustParam(double delta) {
        switch (selectedParam) {
            case 0: kP       = Math.max(0.000, kP       + delta);       break;
            case 1: kD       = Math.max(0.000, kD       + delta);       break;
            case 2: kF       = Math.max(0.000, kF       + delta);       break;
            case 3: DEADZONE = Math.max(0.100, DEADZONE + delta * 5);   break;
            case 4: SCAN_PWR = Math.max(0.050, Math.min(MAX_POWER, SCAN_PWR + delta * 2)); break;
        }
    }

    // ==================== TELEMETRY ====================

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        int currentPos = turretMotor.getCurrentPosition();

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Mode",    currentMode);
        telemetry.addData("Active",  active ? "ON" : "OFF");
        telemetry.addData("Target",  result != null && result.isValid() ? "YES" : "no");
        telemetry.addData("TX (deg)", result != null && result.isValid()
                ? String.format("%.2f", result.getTx()) : "N/A");
        telemetry.addData("Encoder", String.format("%d  [%d to %d]", currentPos, LEFT_LIMIT, RIGHT_LIMIT));

        telemetry.addLine("--- TUNER (GP2 dpad) ---");
        String[] names = {"kP", "kD", "kF", "Deadzone", "ScanPower"};
        double[] vals  = {kP, kD, kF, DEADZONE, SCAN_PWR};
        for (int i = 0; i < NUM_PARAMS; i++) {
            String marker = (i == selectedParam) ? "> " : "  ";
            telemetry.addData(marker + names[i], String.format("%.4f", vals[i]));
        }
    }

    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}
