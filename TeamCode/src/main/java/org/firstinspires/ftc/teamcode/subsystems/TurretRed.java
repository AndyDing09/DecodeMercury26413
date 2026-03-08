package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretRed {
    private DcMotor turretMotor;
    private Limelight3A limelight;
    private ElapsedTime loopTimer = new ElapsedTime();

    // ---------- PID & Control Constants (Gain Scheduling) ----------

    // Gains for NEAR tracking (Aggressive, fast response)
    private final double NEAR_kP = 0.030000001;
    private final double NEAR_kD = 0.002;
    private final double NEAR_kF = 0.008;

    // Gains for FAR tracking (Smoother, heavily damped to prevent oscillation)
    // Note: Far kP is usually halved, and kD is increased to brake earlier.
    private final double FAR_kP  = 0.015;
    private final double FAR_kD  = 0.004;
    private final double FAR_kF  = 0.008;

    // Threshold for determining Near vs Far based on Limelight Target Area (ta)
    // You will need to tune this! Drive to your "boundary" distance and read the 'Target Area (ta)' in telemetry.
    private final double AREA_THRESHOLD = 1.5;

    private final double DEADZONE       = 3.0;
    private final double SCAN_PWR       = 0.5;
    private static final double MAX_POWER = 0.90;

    // ---------- Hardware Limits ----------
    private static final int LEFT_LIMIT  = -435;
    private static final int RIGHT_LIMIT = 380;

    // ---------- Tracking State ----------
    private static final int LOSS_DEBOUNCE_FRAMES = 8;
    private int framesWithoutTarget = 0;
    private double lastTx = 0;
    private boolean scanningRight = true;
    private boolean wasTracking = false;

    // ---------- Telemetry State ----------
    private String currentMode = "IDLE";
    private double currentTx = 0;
    private double currentTa = 0; // Added Target Area for telemetry
    private boolean currentlyHasTarget = false;
    private double currentOutputPower = 0;
    private boolean isFar = false;

    public TurretRed(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        loopTimer.reset();
    }

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
            currentTa = result.getTa(); // Read the target area (size of target on screen)

            // Determine if we are near or far based on the area threshold
            isFar = currentTa < AREA_THRESHOLD;

            // Select active gains based on distance
            double active_kP = isFar ? FAR_kP : NEAR_kP;
            double active_kD = isFar ? FAR_kD : NEAR_kD;
            double active_kF = isFar ? FAR_kF : NEAR_kF;

            double derivative = wasTracking ? (currentTx - lastTx) / dt : 0;

            framesWithoutTarget = 0;
            wasTracking = true;
            lastTx = currentTx;

            if (Math.abs(currentTx) > DEADZONE) {
                if (Math.abs(currentTx) > 5.0) {
                    currentMode = "SLEWING";
                } else {
                    currentMode = isFar ? "TRACKING (FAR)" : "TRACKING (NEAR)";
                }

                // Calculate power using the distance-scheduled gains
                currentOutputPower = (active_kP * currentTx) + (active_kD * derivative);

                if (Math.abs(currentOutputPower) > 0.001 && Math.abs(currentOutputPower) < active_kF) {
                    currentOutputPower = Math.signum(currentOutputPower) * active_kF;
                }
            } else {
                currentMode = "LOCKED";
                currentOutputPower = 0;
            }
        }

        // 2. DEBOUNCE HOLD
        else if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
            framesWithoutTarget++;
            currentMode = "HOLDING (" + framesWithoutTarget + "/" + LOSS_DEBOUNCE_FRAMES + ")";
            currentOutputPower = 0;
        }

        // 3. SEARCHING
        else {
            currentMode = "SEARCHING";
            wasTracking = false;

            if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                scanningRight = lastTx > 0;
            }
            framesWithoutTarget++;

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

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Target", currentlyHasTarget ? "YES" : "no");

        if (currentlyHasTarget) {
            telemetry.addData("TX (deg)", String.format("%.2f", currentTx));
            telemetry.addData("Target Area (ta)", String.format("%.2f%%  [%s]", currentTa, isFar ? "FAR" : "NEAR"));
        } else {
            telemetry.addData("TX / Area", "N/A");
        }

        telemetry.addData("Encoder", String.format("%d  [limit: %d to %d]", turretMotor.getCurrentPosition(), LEFT_LIMIT, RIGHT_LIMIT));
        telemetry.addData("Power", String.format("%.3f", currentOutputPower));
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}