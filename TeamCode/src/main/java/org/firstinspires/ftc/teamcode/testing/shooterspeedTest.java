package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Dual Shooter with Custom PIDF", group="Testing")
public class shooterspeedTest extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // ================= SPEED VARIABLES =================
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;

    // ================= CUSTOM PIDF VARIABLES =================
    // IMPORTANT: Custom PIDF outputs motor power (0.0 to 1.0).
    // These values will be much smaller than the built-in REV hub values!
    private double kP = 0.002;
    private double kI = 0.000;
    private double kD = 0.0001;
    private double kF = 0.0003; // Feedforward is critical for flywheels

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Tuning selection state
    private enum TuneState { P, I, D, F }
    private TuneState currentSelected = TuneState.P;

    // Gamepad 2 Edge Detection
    private boolean lastG2DpadUp = false, lastG2DpadDown = false;
    private boolean lastG2DpadLeft = false, lastG2DpadRight = false;

    @Override
    public void runOpMode() {
        // HARDWARE MAP
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // We use RUN_WITHOUT_ENCODER so we can feed it raw power from our custom PIDF math,
        // but the encoders will STILL track velocity for us to read.
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("✅ Custom PIDF Shooter Initialized");
        telemetry.addLine("--- GAMEPAD 1 (SPEED) ---");
        telemetry.addLine("D-Pad Up/Down: +/- 100 RPM");
        telemetry.addLine("A: 0 | X: 1000 | Y: 2000 | B: 2500");
        telemetry.addLine("--- GAMEPAD 2 (TUNING) ---");
        telemetry.addLine("D-Pad L/R: Select P, I, D, or F");
        telemetry.addLine("D-Pad U/D: Adjust value");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: SPEED CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;

            if (currentG1DpadUp && !lastG1DpadUp) targetRPM += 100;
            if (currentG1DpadDown && !lastG1DpadDown) targetRPM -= 100;

            if (gamepad1.a) targetRPM = 0;
            if (gamepad1.x) targetRPM = 1000;
            if (gamepad1.y) targetRPM = 2000;
            if (gamepad1.b) targetRPM = 2500;

            if (targetRPM < 0) targetRPM = 0;

            // Reset integral sum if we shut off the motors to prevent wind-up
            if (targetRPM == 0) integralSum = 0;

            lastG1DpadUp = currentG1DpadUp;
            lastG1DpadDown = currentG1DpadDown;

            // ================= GAMEPAD 2: PIDF TUNING =================
            boolean currentG2DpadUp = gamepad2.dpad_up;
            boolean currentG2DpadDown = gamepad2.dpad_down;
            boolean currentG2DpadLeft = gamepad2.dpad_left;
            boolean currentG2DpadRight = gamepad2.dpad_right;

            // Cycle through P, I, D, F
            if (currentG2DpadRight && !lastG2DpadRight) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.F;
                else if (currentSelected == TuneState.F) currentSelected = TuneState.P;
            } else if (currentG2DpadLeft && !lastG2DpadLeft) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.F;
                else if (currentSelected == TuneState.F) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.P;
            }

            // Custom PIDF values need tiny adjustments (0.0001 or 0.001)
            double increment = gamepad2.left_bumper ? 0.0001 : 0.001;

            if (currentG2DpadUp && !lastG2DpadUp) {
                switch(currentSelected) {
                    case P: kP += increment; break;
                    case I: kI += increment; break;
                    case D: kD += increment; break;
                    case F: kF += increment; break;
                }
            } else if (currentG2DpadDown && !lastG2DpadDown) {
                switch(currentSelected) {
                    case P: kP -= increment; break;
                    case I: kI -= increment; break;
                    case D: kD -= increment; break;
                    case F: kF -= increment; break;
                }
            }

            lastG2DpadUp = currentG2DpadUp;
            lastG2DpadDown = currentG2DpadDown;
            lastG2DpadLeft = currentG2DpadLeft;
            lastG2DpadRight = currentG2DpadRight;

            // ================= CUSTOM PIDF MATH =================
            // 1. Calculate our target velocity in Ticks Per Second
            double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.0;

            // 2. Get current velocity (averaging the two motors for stability)
            double currentVelocityLeft = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();
            double avgCurrentVelocity = (currentVelocityLeft + currentVelocityRight) / 2.0;

            // 3. Calculate Error and Time
            double error = targetVelocity - avgCurrentVelocity;
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // 4. Calculate P, I, D, F components
            integralSum += (error * dt);
            double derivative = (error - lastError) / dt;
            lastError = error;

            // Feedforward (kF) is base power based on target speed. P, I, and D correct the errors.
            double motorPower = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetVelocity);

            // 5. Clamp power to normal motor ranges (0.0 to 1.0)
            if (targetRPM == 0) {
                motorPower = 0; // Absolute shutoff
            } else {
                motorPower = Math.max(0.0, Math.min(1.0, motorPower)); // Prevents negative power or going over 1.0
            }

            // 6. Apply Raw Power
            shooterLeft.setPower(motorPower);
            shooterRight.setPower(motorPower);

            // ================= TELEMETRY =================
            telemetry.addData("TARGET RPM", targetRPM);
            telemetry.addData("Calculated Motor Power", "%.2f", motorPower);
            telemetry.addLine();

            telemetry.addLine("--- PIDF TUNING ---");
            telemetry.addData("Selected", "-> " + currentSelected.name() + " <-");
            telemetry.addData("kP", "%.4f %s", kP, currentSelected == TuneState.P ? "<--" : "");
            telemetry.addData("kI", "%.4f %s", kI, currentSelected == TuneState.I ? "<--" : "");
            telemetry.addData("kD", "%.4f %s", kD, currentSelected == TuneState.D ? "<--" : "");
            telemetry.addData("kF", "%.4f %s", kF, currentSelected == TuneState.F ? "<--" : "");
            telemetry.addLine("(Hold G2 Left Bumper for 0.0001 increments)");
            telemetry.addLine();

            telemetry.addLine("--- MOTOR PERFORMANCE ---");
            telemetry.addData("Target Ticks/Sec", targetVelocity);
            telemetry.addData("Left Actual RPM", (currentVelocityLeft * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", (currentVelocityRight * 60.0) / TICKS_PER_REV);

            telemetry.update();
        }
    }
}