package org.firstinspires.ftc.teamcode.Turret;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Turret Tracking + Search")
public class turretTracking extends OpMode {

    private DcMotor turretMotor;
    private Limelight3A limelight;

    // ---------- PID Tuning ----------
    private static final double kP = 0.022;
    private static final double kD = 0.0015;
    private static final double kF = 0.08;

    private static final double MAX_POWER = 0.6;
    private static final double SCAN_POWER = 0.15;
    private static final double DEADZONE_DEGREES = 1.0;

    // ---------- Hardware Constants ----------
    private static final int LEFT_LIMIT = -430;
    private static final int RIGHT_LIMIT = 430;
    private static final int LIMIT_BUFFER = 50; // Buffer before reversing scan

    // ---------- State Variables ----------
    private double lastError = 0;
    private boolean scanningRight = true;

    @Override
    public void init() {
        // Motor Setup
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // FIXED: Correct method name
        limelight.start();

        telemetry.addLine("Turret Initialized. Ensure robot is centered before Start.");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        int currentPos = turretMotor.getCurrentPosition();
        double outputPower = 0;
        String mode = "IDLE";

        // -------------------------------------------------------
        // 1. MANUAL OVERRIDE (High Priority)
        // -------------------------------------------------------
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            mode = "MANUAL";
            outputPower = gamepad1.right_stick_x * 0.5;
            lastError = 0;
        }

        // -------------------------------------------------------
        // 2. AUTOMATIC TRACKING (Medium Priority)
        // -------------------------------------------------------
        else if (result != null && result.isValid()) {
            mode = "TRACKING";
            double tx = result.getTx();

            if (Math.abs(tx) > DEADZONE_DEGREES) {
                // FIXED: Derivative calculation (current - previous)
                double derivative = (tx - lastError);
                outputPower = (kP * tx) + (kD * derivative);

                // Add Feedforward to overcome static friction
                if (Math.abs(outputPower) < kF && Math.abs(outputPower) > 0.001) {
                    outputPower = Math.signum(outputPower) * kF;
                }
            } else {
                outputPower = 0;
            }

            lastError = tx;
        }

        // -------------------------------------------------------
        // 3. SEARCH MODE (Low Priority)
        // -------------------------------------------------------
        else {
            mode = "SEARCHING";
            lastError = 0;

            // IMPROVED: Only reverse at actual limits, not buffer zone
            if (currentPos >= RIGHT_LIMIT) {
                scanningRight = false;
            } else if (currentPos <= LEFT_LIMIT) {
                scanningRight = true;
            }

            outputPower = scanningRight ? SCAN_POWER : -SCAN_POWER;
        }

        // -------------------------------------------------------
        // 4. SAFETY: SOFT LIMITS & CLAMPING
        // -------------------------------------------------------

        // Clamp maximum power
        outputPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, outputPower));

        // Hard stop at limits (this protects hardware)
        if (currentPos <= LEFT_LIMIT && outputPower < 0) {
            outputPower = 0;
        } else if (currentPos >= RIGHT_LIMIT && outputPower > 0) {
            outputPower = 0;
        }

        turretMotor.setPower(outputPower);

        // -------------------------------------------------------
        // TELEMETRY
        // -------------------------------------------------------
        telemetry.addData("Mode", mode);
        telemetry.addData("Target Found", (result != null && result.isValid()));
        telemetry.addData("TX Error (°)", result != null && result.isValid() ? String.format("%.2f", result.getTx()) : "N/A");
        telemetry.addData("Position", "%d / [%d to %d]", currentPos, LEFT_LIMIT, RIGHT_LIMIT);
        telemetry.addData("Power", "%.2f", outputPower);
        telemetry.addData("Scan Direction", scanningRight ? "→ RIGHT" : "← LEFT");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}