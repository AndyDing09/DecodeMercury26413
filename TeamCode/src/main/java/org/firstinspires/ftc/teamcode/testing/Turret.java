package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    private DcMotor turretMotor;
    private Limelight3A limelight;
    private ElapsedTime loopTimer = new ElapsedTime();

    // ---------- PID & Control Constants ----------
    // These are the finalized values from your live tuning sessions
    private final double kP             = 0.030000001;
    private final double kD             = 0.002;  // high damping — prevents lurch-oscillation from herringbone friction
    private final double kF             = 0.008;  // herringbone gears need ~0.10-0.15 to break static friction
    private final double DEADZONE       = 3.0;    // wider deadzone — don't fight friction on small errors, let brake hold
    private final double SCAN_PWR       = 0.5;
    private static final double MAX_POWER = 0.90;

    // ---------- Hardware Limits ----------
    private static final int LEFT_LIMIT  = -435;
    private static final int RIGHT_LIMIT = 415;

    // ---------- Tracking State ----------
    private static final int LOSS_DEBOUNCE_FRAMES = 8;
    private int framesWithoutTarget = 0;
    private double lastTx = 0;
    private boolean scanningRight = true;
    private boolean wasTracking = false;

    // ---------- Telemetry State ----------
    private String currentMode = "IDLE";
    private double currentTx = 0;
    private boolean currentlyHasTarget = false;
    private double currentOutputPower = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder on init, then run without it so we can command direct power
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        loopTimer.reset();
    }

    /**
     * Call this once per loop in your TeleOp or Autonomous.
     * It handles the automated tracking, debounce holding, and searching.
     */
    public void update() {
        double dt = Math.max(0.001, loopTimer.seconds());
        loopTimer.reset();

        LLResult result = limelight.getLatestResult();
        currentlyHasTarget = result != null && result.isValid();
        int currentPos = turretMotor.getCurrentPosition();

        currentOutputPower = 0;

        // 1. TRACKING
        if (currentlyHasTarget) {
            currentTx = result.getTx();
            double derivative = wasTracking ? (currentTx - lastTx) / dt : 0;

            framesWithoutTarget = 0;
            wasTracking = true;
            lastTx = currentTx;

            if (Math.abs(currentTx) > DEADZONE) {
                currentMode = Math.abs(currentTx) > 5.0 ? "SLEWING" : "TRACKING";
                currentOutputPower = (kP * currentTx) + (kD * derivative);

                // Apply Feedforward to break static friction
                if (Math.abs(currentOutputPower) > 0.001 && Math.abs(currentOutputPower) < kF) {
                    currentOutputPower = Math.signum(currentOutputPower) * kF;
                }
            } else {
                currentMode = "LOCKED";
                currentOutputPower = 0;
            }
        }

        // 2. DEBOUNCE HOLD (Target temporarily lost)
        else if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
            framesWithoutTarget++;
            currentMode = "HOLDING (" + framesWithoutTarget + "/" + LOSS_DEBOUNCE_FRAMES + ")";
            currentOutputPower = 0;
        }

        // 3. SEARCHING (Target fully lost, begin sweeping)
        else {
            currentMode = "SEARCHING";
            wasTracking = false;

            if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                // Start searching in the direction we last saw the target
                scanningRight = lastTx > 0;
            }
            framesWithoutTarget++;

            // Reverse direction if we hit the hard limits
            if (currentPos >= RIGHT_LIMIT) scanningRight = true;
            else if (currentPos <= LEFT_LIMIT) scanningRight = false;

            currentOutputPower = scanningRight ? SCAN_PWR : -SCAN_PWR;
            currentMode += scanningRight ? " ->" : " <-";
        }

        // 4. SAFETY LIMITS & EXECUTION
        currentOutputPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, currentOutputPower));

        if (currentPos <= LEFT_LIMIT && currentOutputPower > 0) currentOutputPower = 0;
        if (currentPos >= RIGHT_LIMIT && currentOutputPower < 0) currentOutputPower = 0;

        turretMotor.setPower(currentOutputPower);
    }

    /**
     * Call this in your main loop to push turret data to the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Target", currentlyHasTarget ? "YES" : "no");
        telemetry.addData("TX (deg)", currentlyHasTarget ? String.format("%.2f", currentTx) : "N/A");
        telemetry.addData("Encoder", String.format("%d  [limit: %d to %d]", turretMotor.getCurrentPosition(), LEFT_LIMIT, RIGHT_LIMIT));
        telemetry.addData("Power", String.format("%.3f", currentOutputPower));
    }

    /**
     * Optional utility if you need to manually re-zero the turret mid-match.
     */
    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Call this on OpMode stop() or inside the subsystem shutdown sequence.
     */
    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}