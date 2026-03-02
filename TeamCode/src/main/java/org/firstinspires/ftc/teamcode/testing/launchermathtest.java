package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LauncherMathTest", group="Testing")
public class launchermathtest extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // ================= PHYSICS & DISTANCE VARIABLES =================
    private double targetDistance = 2.0; // Start at 2 meters

    // IMPORTANT: Make sure these match your robot's physical dimensions!
    private static final double GRAVITY = 9.81;
    private static final double TARGET_HEIGHT = 1.22; // Height of the goal in meters (adjust to reality)
    private static final double LAUNCHER_HEIGHT = 0.2; // Height of your shooter in meters
    private static final double MIN_HOOD_ANGLE = 16.0;
    private static final double MAX_HOOD_ANGLE = 50.0;
    private static final double LAUNCHER_MAX_BALL_VELOCITY = 15.42; // Max achievable physical speed
    private static final double MAX_DRIVE_VELOCITY = 15.0;

    // Ticks per revolution (used for RPM telemetry calculation)
    private static final double TICKS_PER_REV = 28.0;

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;

    // ================= CUSTOM PIDF VARIABLES =================
    private double kP = 0.002;
    private double kI = 0.000;
    private double kD = 0.0001;
    private double kF = 0.0003;

    // Split error tracking for independent motor control
    private double integralSumLeft = 0;
    private double lastErrorLeft = 0;
    private double integralSumRight = 0;
    private double lastErrorRight = 0;

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

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("✅ Physics Shooter Initialized");
        telemetry.addLine("--- GAMEPAD 1 (SIMULATE DISTANCE) ---");
        telemetry.addLine("D-Pad Up/Down: +/- 0.1 Meters");
        telemetry.addLine("Bumpers: +/- 0.5 Meters");
        telemetry.addLine("A: Shut Off | B: Reset to 2.0m");
        telemetry.addLine("--- GAMEPAD 2 (TUNING) ---");
        telemetry.addLine("D-Pad L/R: Select P, I, D, or F");
        telemetry.addLine("D-Pad U/D: Adjust value");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: DISTANCE CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;
            boolean currentG1RightBumper = gamepad1.right_bumper;
            boolean currentG1LeftBumper = gamepad1.left_bumper;

            if (currentG1DpadUp && !lastG1DpadUp) targetDistance += 0.1;
            if (currentG1DpadDown && !lastG1DpadDown) targetDistance -= 0.1;
            if (currentG1RightBumper && !lastG1RightBumper) targetDistance += 0.5;
            if (currentG1LeftBumper && !lastG1LeftBumper) targetDistance -= 0.5;

            if (gamepad1.a) targetDistance = 0;
            if (gamepad1.b) targetDistance = 2.0;

            if (targetDistance < 0) targetDistance = 0;

            if (targetDistance == 0) {
                integralSumLeft = 0;
                integralSumRight = 0;
            }

            lastG1DpadUp = currentG1DpadUp;
            lastG1DpadDown = currentG1DpadDown;
            lastG1RightBumper = currentG1RightBumper;
            lastG1LeftBumper = currentG1LeftBumper;

            // ================= GAMEPAD 2: PIDF TUNING =================
            boolean currentG2DpadUp = gamepad2.dpad_up;
            boolean currentG2DpadDown = gamepad2.dpad_down;
            boolean currentG2DpadLeft = gamepad2.dpad_left;
            boolean currentG2DpadRight = gamepad2.dpad_right;

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

            // ================= PHYSICS & INTERPOLATION =================
            double[] calculatedValues = distanceToLauncherValues(targetDistance);
            double requiredVelocityMS = calculatedValues[0]; // Velocity in m/s
            double requiredHoodAngle = calculatedValues[1];  // Angle in degrees

            double targetVelocityTicks = 0;

            // Only apply velocity if physics returned a valid/possible number
            if (!Double.isNaN(requiredVelocityMS) && targetDistance > 0) {
                targetVelocityTicks = interpolateToTicks(requiredVelocityMS);
            }

            // ================= CUSTOM PIDF MATH =================
            double currentVelocityLeft = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();

            double dt = pidTimer.seconds();
            pidTimer.reset();

            // --- LEFT MOTOR PIDF ---
            double errorLeft = targetVelocityTicks - currentVelocityLeft;
            integralSumLeft += (errorLeft * dt);
            double derivativeLeft = dt > 0 ? (errorLeft - lastErrorLeft) / dt : 0;
            lastErrorLeft = errorLeft;

            double powerLeft = (kP * errorLeft) + (kI * integralSumLeft) + (kD * derivativeLeft) + (kF * targetVelocityTicks);
            if (targetVelocityTicks == 0) powerLeft = 0;
            else powerLeft = Math.max(0.0, Math.min(1.0, powerLeft));

            // --- RIGHT MOTOR PIDF ---
            double errorRight = targetVelocityTicks - currentVelocityRight;
            integralSumRight += (errorRight * dt);
            double derivativeRight = dt > 0 ? (errorRight - lastErrorRight) / dt : 0;
            lastErrorRight = errorRight;

            double powerRight = (kP * errorRight) + (kI * integralSumRight) + (kD * derivativeRight) + (kF * targetVelocityTicks);
            if (targetVelocityTicks == 0) powerRight = 0;
            else powerRight = Math.max(0.0, Math.min(1.0, powerRight));

            // Apply the independent powers
            shooterLeft.setPower(powerLeft);
            shooterRight.setPower(powerRight);

            // ================= TELEMETRY =================
            telemetry.addData("1. Target Distance (m)", "%.2f", targetDistance);

            if (Double.isNaN(requiredVelocityMS)) {
                telemetry.addData("2. Physics Result", "IMPOSSIBLE SHOT!");
            } else {
                telemetry.addData("2. Req. Velocity (m/s)", "%.2f", requiredVelocityMS);
                telemetry.addData("3. Req. Hood Angle", "%.1f°", requiredHoodAngle);
            }

            telemetry.addData("4. Target Ticks/Sec", "%.0f", targetVelocityTicks);
            telemetry.addData("Motor Power (L/R)", "%.2f / %.2f", powerLeft, powerRight);
            telemetry.addLine();

            telemetry.addLine("--- PIDF TUNING ---");
            telemetry.addData("Selected", "-> " + currentSelected.name() + " <-");
            telemetry.addData("kP", "%.4f %s", kP, currentSelected == TuneState.P ? "<--" : "");
            telemetry.addData("kI", "%.4f %s", kI, currentSelected == TuneState.I ? "<--" : "");
            telemetry.addData("kD", "%.4f %s", kD, currentSelected == TuneState.D ? "<--" : "");
            telemetry.addData("kF", "%.4f %s", kF, currentSelected == TuneState.F ? "<--" : "");
            telemetry.addLine();

            telemetry.addLine("--- MOTOR PERFORMANCE ---");
            telemetry.addData("Left Actual RPM", "%.0f", (currentVelocityLeft * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", "%.0f", (currentVelocityRight * 60.0) / TICKS_PER_REV);

            telemetry.update();
        }
    }

    // =================================================================================
    // HELPER METHOD: PHYSICS CALCULATION
    // =================================================================================
    public static double[] distanceToLauncherValues(double distance) {
        if (distance <= 0) return new double[]{Double.NaN, Double.NaN};

        double x = distance;
        double deltaY = TARGET_HEIGHT - LAUNCHER_HEIGHT;

        double minVelocitySquared = GRAVITY * (deltaY + Math.sqrt(Math.pow(deltaY, 2) + Math.pow(x, 2)));
        double minVelocity = Math.sqrt(minVelocitySquared);

        double tanThetaMin = minVelocitySquared / (GRAVITY * x);
        double optimalAngleHoriz = Math.toDegrees(Math.atan(tanThetaMin));
        double optimalAngleVert = 90.0 - optimalAngleHoriz;

        double finalAngleVert = 0;
        double finalAngleHoriz = 0;
        boolean forceOverride = false;

        if (distance <= 1) {
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE;
            forceOverride = true;
        } else if (distance >= 4) {
            finalAngleVert = MAX_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
            forceOverride = true;
        }

        if (!forceOverride) {
            if (optimalAngleVert >= MIN_HOOD_ANGLE && optimalAngleVert <= MAX_HOOD_ANGLE) {
                finalAngleVert = optimalAngleVert;
                finalAngleHoriz = optimalAngleHoriz;

                if (minVelocity > LAUNCHER_MAX_BALL_VELOCITY) {
                    return new double[]{Double.NaN, Double.NaN}; // Impossible
                }
                return new double[]{minVelocity, finalAngleVert};
            } else if (optimalAngleVert < MIN_HOOD_ANGLE) {
                finalAngleVert = MIN_HOOD_ANGLE;
                finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE;
            } else {
                finalAngleVert = MAX_HOOD_ANGLE;
                finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
            }
        }

        double angleToUseRad = Math.toRadians(finalAngleHoriz);
        double tanTheta = Math.tan(angleToUseRad);
        double cosTheta = Math.cos(angleToUseRad);

        double denominator = 2 * (cosTheta * cosTheta) * (x * tanTheta - deltaY);
        if (denominator <= 0) {
            return new double[]{Double.NaN, Double.NaN}; // Denominator <= 0 makes it impossible
        }

        double requiredVelocity = Math.sqrt((GRAVITY * x * x) / denominator);
        if (requiredVelocity > MAX_DRIVE_VELOCITY) {
            return new double[]{Double.NaN, Double.NaN};
        }

        return new double[]{requiredVelocity, finalAngleVert};
    }

    // =================================================================================
    // HELPER METHOD: LINEAR INTERPOLATION (m/s -> Ticks/sec)
    // =================================================================================
    public static double interpolateToTicks(double velocityMs) {
        // Values pulled from your LauncherMathTest.java arrays
        double[] inputMs = {-0.01, 0.0, 7.376, 8.9408, 10.2819, 12.07, 13.4112, 14.5288, 15.42288};
        double[] outputTicks = {-0.01, 0.0, 933.33, 1166.67, 1400.0, 1633.33, 1866.667, 2100, 2240.0};

        // If outside lower bounds
        if (velocityMs <= inputMs[0]) return outputTicks[0];
        // If outside upper bounds
        if (velocityMs >= inputMs[inputMs.length - 1]) return outputTicks[outputTicks.length - 1];

        // Linear interpolation logic
        for (int i = 0; i < inputMs.length - 1; i++) {
            if (velocityMs >= inputMs[i] && velocityMs <= inputMs[i + 1]) {
                double fraction = (velocityMs - inputMs[i]) / (inputMs[i + 1] - inputMs[i]);
                return outputTicks[i] + fraction * (outputTicks[i + 1] - outputTicks[i]);
            }
        }
        return 0.0;
    }
}