package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="hoodtest", group="Testing")
public class hoodtest extends LinearOpMode {
    // hood constants
    private static final double MIN_HOOD_ANGLE = 26.0;
    private static final double MAX_HOOD_ANGLE = 45.0;

    // hood servos
    private Servo hoodServo1;
    private Servo hoodServo2;
    private double currentHoodAngle = MIN_HOOD_ANGLE;

    // gear constants
    private static final double SMALL_GEAR_DIAMETER = 104;
    private static final double LARGE_GEAR_DIAMETER = 375.0;
    private static final double GEAR_RATIO = LARGE_GEAR_DIAMETER / SMALL_GEAR_DIAMETER;
    private static final double SERVO_START_POS = 0.5;
    private static final double SERVO_UNITS_PER_HOOD_DEGREE = GEAR_RATIO / 180.0;
    private static final double MAX_REACHABLE_HOOD_ANGLE = MIN_HOOD_ANGLE + (1.0 - SERVO_START_POS) * 180.0 / GEAR_RATIO;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        // ================= HARDWARE MAP =================
        hoodServo1 = hardwareMap.servo.get("angleChange1");
        hoodServo2 = hardwareMap.servo.get("angleChange2");

        // Initialize hood to lowest position
        currentHoodAngle = MIN_HOOD_ANGLE;
        updateHoodServoPosition(currentHoodAngle);

        telemetry.addLine("Hood Test Initialized — Waiting for Start");
        telemetry.addData("Min Angle", MIN_HOOD_ANGLE + " deg");
        telemetry.addData("Max Angle", MAX_HOOD_ANGLE + " deg");
        telemetry.addData("Max Reachable", String.format("%.2f deg", MAX_REACHABLE_HOOD_ANGLE));
        telemetry.addLine("D-Pad Up: +1 deg | D-Pad Down: -1 deg");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= HOOD ANGLE CONTROL =================
            boolean currentDpadUp   = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // Rising-edge detection: one degree per button tap
            if (currentDpadUp && !lastDpadUp) {
                currentHoodAngle += 1.0;
            } else if (currentDpadDown && !lastDpadDown) {
                currentHoodAngle -= 1.0;
            }

            // Clamp to valid range
            double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
            if (currentHoodAngle < MIN_HOOD_ANGLE) currentHoodAngle = MIN_HOOD_ANGLE;
            if (currentHoodAngle > effectiveMax)   currentHoodAngle = effectiveMax;

            updateHoodServoPosition(currentHoodAngle);

            lastDpadUp   = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // ================= TELEMETRY =================
            double servoPos = angleToServoPosition(currentHoodAngle);
            telemetry.addLine("=== Hood Control ===");
            telemetry.addData("Hood Angle (deg)",    String.format("%.1f", currentHoodAngle));
            telemetry.addData("Servo1 Position (flipped)", String.format("%.4f", 1.0 - servoPos));
            telemetry.addData("Servo2 Position",           String.format("%.4f", servoPos));
            telemetry.addData("Min / Max Angle",     MIN_HOOD_ANGLE + " / " + String.format("%.1f", effectiveMax));
            telemetry.addLine("D-Pad Up: +1 deg | D-Pad Down: -1 deg");
            telemetry.update();

            sleep(20);
        }
    }

    /**
     * Convert a hood angle (degrees) to servo position.
     * Servo 0.5 = MIN_HOOD_ANGLE (lowest). Increases to raise hood.
     */
    private double angleToServoPosition(double angle) {
        double servoPos = SERVO_START_POS + (angle - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    private void updateHoodServoPosition(double angle) {
        double servoPos = angleToServoPosition(angle);
        hoodServo2.setPosition(1.0 - servoPos);  // flipped — same axle, opposite direction
        hoodServo1.setPosition(servoPos);
    }
}