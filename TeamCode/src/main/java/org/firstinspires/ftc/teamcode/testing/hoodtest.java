package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LauncherMathTest", group="Testing")
public class hoodtest extends LinearOpMode {
    // hood constants
    private static final double MIN_HOOD_ANGLE = 26.0;
    private static final double MAX_HOOD_ANGLE = 41;

    // hood servos
    private Servo hoodServo1;
    private Servo hoodServo2;
    private double currentHoodAngle = MIN_HOOD_ANGLE;

    // gear constants
    // Small gear on servo axle, large gear on hood pivot
    private static final double SMALL_GEAR_DIAMETER = 57.25;  // mm (servo-driven gear)
    private static final double LARGE_GEAR_DIAMETER = 375.0;   // mm (hood output gear)
    private static final double GEAR_RATIO = LARGE_GEAR_DIAMETER / SMALL_GEAR_DIAMETER; // ~6.55:1
    // Servo position 0.5 = MIN_HOOD_ANGLE (lowest point)
    // Standard servo: position 0.0-1.0 = 180 degrees of rotation
    // 1 degree of hood change requires GEAR_RATIO degrees of servo rotation
    // In servo units: GEAR_RATIO / 180.0 per degree of hood angle
    private static final double SERVO_START_POS = 0.5;
    private static final double SERVO_UNITS_PER_HOOD_DEGREE = GEAR_RATIO / 180.0;
    // Servo INCREASES from 0.5 to raise the hood (0.5 = lowest, 1.0 = highest)
    // Max reachable hood angle given servo range [0.5, 1.0] = 0.5 * 180 / GEAR_RATIO degrees
    private static final double MAX_REACHABLE_HOOD_ANGLE = MIN_HOOD_ANGLE + (1.0 - SERVO_START_POS) * 180.0 / GEAR_RATIO;
    private boolean lastG1Y = false, lastG1X = false;

    // ================= SHOOTER PIDF (uses PIDFMotorController, same as TeleOp2) =================

    @Override
    public void runOpMode() {
        // ================= HARDWARE MAP =================
        hoodServo1 = hardwareMap.servo.get("angleChange1");
        hoodServo2 = hardwareMap.servo.get("angleChange2");


        // Initialize hood to lowest position (servo 0.5)
        currentHoodAngle = MIN_HOOD_ANGLE;
        updateHoodServoPosition(currentHoodAngle);

        telemetry.addLine("Physics Shooter Initialized");
        telemetry.addLine("--- GAMEPAD 1 ---");
        telemetry.addLine("D-Pad Up/Down: +/- 0.1m | Bumpers: +/- 0.5m");
        telemetry.addLine("Y/X: +/- 1 deg hood | A: Off | B: Reset 2.0m");
        telemetry.addData("Max Reachable Hood", "%.1f deg", MAX_REACHABLE_HOOD_ANGLE);
        telemetry.addLine("--- GAMEPAD 2 (TUNING) ---");
        telemetry.addLine("D-Pad L/R: Select P,I,D,F | U/D: Adjust");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: HOOD ANGLE CONTROL =================
            boolean currentG1Y = gamepad1.y;
            boolean currentG1X = gamepad1.x;

            // Clamp to valid range (physical limit is MAX_REACHABLE_HOOD_ANGLE)
            double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
            if (currentHoodAngle < MIN_HOOD_ANGLE) currentHoodAngle = MIN_HOOD_ANGLE;
            if (currentHoodAngle > effectiveMax) currentHoodAngle = effectiveMax;

            updateHoodServoPosition(currentHoodAngle);

            lastG1Y = currentG1Y;
            lastG1X = currentG1X;

            // ================= TELEMETRY =================

            telemetry.addData("Hood Angle", "%.1f deg %s", currentHoodAngle);
            telemetry.addData("Hood Servo Pos", "%.3f", angleToServoPosition(currentHoodAngle));
            telemetry.addLine();

            telemetry.update();
        }
    }
    /**
     * Convert a hood angle (degrees) to servo position
     * Servo 0.5 = MIN_HOOD_ANGLE (lowest point)
     * Servo INCREASES to raise the hood (add for higher angle)
     * 1 deg hood = GEAR_RATIO deg servo = GEAR_RATIO/180 servo units
     */
    private double angleToServoPosition(double angle) {
        double servoPos = SERVO_START_POS + (angle - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
        return Math.max(0.0, Math.min(1.0, servoPos));
    }
    private void updateHoodServoPosition(double angle) {
        double servoPos = angleToServoPosition(angle);
        hoodServo1.setPosition(servoPos);
        hoodServo2.setPosition(servoPos);
    }
}
