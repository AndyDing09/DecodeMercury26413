package org.firstinspires.ftc.teamcode.testing;

// import static org.firstinspires.ftc.teamcode.testing.MathLib.interpolateToShootingDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

@Config
@TeleOp(name="LauncherMathTest", group="Testing")
public class launchermathtest extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotor middleTransfer;
    private VoltageSensor voltageSensor;
    private Servo gate;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // ================= INTAKE / GATE =================
    private boolean intakeOn = false;
    private boolean gateOpen = false;
    private boolean lastG2Circle = false;
    private boolean lastG2Triangle = false;

    // ================= PHYSICS & DISTANCE VARIABLES =================
    private double targetDistance = 2.0; // Start at 2 meters
    private Follower follower;
    private boolean useOdometry = true;

    // IMPORTANT: Make sure these match your robot's physical dimensions!
    private static final double GRAVITY = 9.81;
    private static final double TARGET_HEIGHT = 1.22;          // Height of the goal in meters
    private static final double LAUNCHER_HEIGHT = 0.32385;     // Height of your shooter in meters
    private static final double MIN_HOOD_ANGLE = 26.0;
    private static final double MAX_HOOD_ANGLE = 45.0;

    // ================= SERVOS - HOOD ANGLE CONTROL =================
    private Servo hoodServo1;
    private Servo hoodServo2;
    private double currentHoodAngle = MIN_HOOD_ANGLE;
    private boolean manualHoodOverride = false;

    // ================= GEAR RATIO CONSTANTS =================
    // Small gear on servo axle, large gear on hood pivot
    private static final double SMALL_GEAR_DIAMETER = 104.0;   // mm (servo-driven gear)
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
    private static final double LAUNCHER_MAX_BALL_VELOCITY = 15.0;

    private static final double MAX_DRIVE_VELOCITY = 15.0;

    // Goal geometry for lip clearance and backboard targeting
    private static final double GOAL_LIP = 0.45;
    private static final double BACKBOARD_Y_OFFSET = 0.1;
    private static final double LIP_BUFFER = 8 * 0.0254;
    private static final double DISTANCE_OFFSET = 0.0;  // no distance sensor — input is true distance

    // Ticks per revolution (used for RPM telemetry calculation)
    private static final double TICKS_PER_REV = 28.0;

    private final Pose startPose         = new Pose(72, 72, Math.toRadians(0));

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;
    private boolean lastG1Y = false, lastG1X = false;

    // ================= SHOOTER PIDF (uses PIDFMotorController, same as TeleOp2) =================
    // Aggressive gains for fast recovery after ball launch
    public static double kP = 0.00002;
    public static double kI = 0.00002;
    public static double kD = 0.0000;
    public static double kF = 0.0004;

    private PIDFMotorController leftController  = null;
    private PIDFMotorController rightController = null;

    // Tuning selection state
    private enum TuneState { P, I, D, F }
    private TuneState currentSelected = TuneState.P;

    // Gamepad 2 Edge Detection
    private boolean lastG2DpadUp = false, lastG2DpadDown = false;
    private boolean lastG2DpadLeft = false, lastG2DpadRight = false;

    @Override
    public void runOpMode() {
        // ================= HARDWARE MAP =================
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServo1 = hardwareMap.servo.get("angleChange1");
        hoodServo2 = hardwareMap.servo.get("angleChange2");

//        follower = new Follower(hardwareMap);
//        follower.start();

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        middleTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hardwareMap.servo.get("Gate");
        gate.setPosition(0.09);

        // Initialize hood to lowest position (servo 0.5)
        currentHoodAngle = MIN_HOOD_ANGLE;
        updateHoodServoPosition(currentHoodAngle);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Physics Shooter Initialized");
        telemetry.addLine("--- GAMEPAD 1 ---");
        telemetry.addLine("D-Pad Up/Down: +/- 0.1m | Bumpers: +/- 0.5m");
        telemetry.addLine("Y/X: +/- 1 deg hood | A: Odometry | B: Reset 2.0m");
        telemetry.addData("Max Reachable Hood", "%.1f deg", MAX_REACHABLE_HOOD_ANGLE);
        telemetry.addLine("--- GAMEPAD 2 (TUNING) ---");
        telemetry.addLine("D-Pad L/R: Select P,I,D,F | U/D: Adjust");
        telemetry.update();

        waitForStart();

        leftController  = new PIDFMotorController(kP, kI, kD, kF, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP, kI, kD, kF, TICKS_PER_REV);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        while (opModeIsActive()) {

            follower.update();

            // ================= GAMEPAD 2: INTAKE (circle) + GATE (triangle) =================
            boolean curG2Circle = gamepad2.circle;
            boolean curG2Triangle = gamepad2.triangle;

            if (curG2Circle && !lastG2Circle) {
                intakeOn = !intakeOn;
                middleTransfer.setPower(intakeOn ? Math.min(1.0, NOMINAL_VOLTAGE / voltageSensor.getVoltage()) : 0);
            }
            if (curG2Triangle && !lastG2Triangle) {
                gateOpen = !gateOpen;
                gate.setPosition(gateOpen ? 0.27 : 0.09);
            }
            lastG2Circle = curG2Circle;
            lastG2Triangle = curG2Triangle;

            // ================= GAMEPAD 1: DISTANCE CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;
            boolean currentG1RightBumper = gamepad1.right_bumper;
            boolean currentG1LeftBumper = gamepad1.left_bumper;

            if (currentG1DpadUp && !lastG1DpadUp) { targetDistance += 0.1; manualHoodOverride = false; useOdometry = false; }
            if (currentG1DpadDown && !lastG1DpadDown) { targetDistance -= 0.1; manualHoodOverride = false; useOdometry = false; }
            if (currentG1RightBumper && !lastG1RightBumper) { targetDistance += 0.5; manualHoodOverride = false; useOdometry = false; }
            if (currentG1LeftBumper && !lastG1LeftBumper) { targetDistance -= 0.5; manualHoodOverride = false; useOdometry = false; }

            if (gamepad1.a) { useOdometry = true; manualHoodOverride = false; }
            if (gamepad1.b) { targetDistance = 2.0; manualHoodOverride = false; useOdometry = false; }

            if (targetDistance < 0) targetDistance = 0;

            if (targetDistance == 0) {
                if (leftController != null) leftController.reset();
                if (rightController != null) rightController.reset();
            }

            if (useOdometry) {
                Pose pose = follower.getPose();
                // Calculate distance from robot position to goal center
                targetDistance = Math.hypot(pose.getX() - MathLib.GOAL_CENTER_X, pose.getY() - MathLib.GOAL_CENTER_Y) * MathLib.INCHES_TO_METERS;
            }

            // ================= GAMEPAD 1: HOOD ANGLE CONTROL =================
            boolean currentG1Y = gamepad1.y;
            boolean currentG1X = gamepad1.x;

            if (currentG1Y && !lastG1Y) { currentHoodAngle += 1.0; manualHoodOverride = true; }
            if (currentG1X && !lastG1X) { currentHoodAngle -= 1.0; manualHoodOverride = true; }

            // Clamp to valid range (physical limit is MAX_REACHABLE_HOOD_ANGLE)
            double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
            if (currentHoodAngle < MIN_HOOD_ANGLE) currentHoodAngle = MIN_HOOD_ANGLE;
            if (currentHoodAngle > effectiveMax) currentHoodAngle = effectiveMax;

            updateHoodServoPosition(currentHoodAngle);

            lastG1Y = currentG1Y;
            lastG1X = currentG1X;
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
            double requiredVelocityMS = calculatedValues[0];
            double requiredHoodAngle = calculatedValues[1];

            double targetVelocityTicks = 0;

            if (!Double.isNaN(requiredVelocityMS) && targetDistance > 0) {
                targetVelocityTicks = interpolateToTicks(requiredVelocityMS);

                // Only auto-set hood angle if user hasn't manually adjusted
                if (!manualHoodOverride && !Double.isNaN(requiredHoodAngle)) {
                    currentHoodAngle = requiredHoodAngle;
                    double effectiveMaxAuto = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
                    if (currentHoodAngle < MIN_HOOD_ANGLE) currentHoodAngle = MIN_HOOD_ANGLE;
                    if (currentHoodAngle > effectiveMaxAuto) currentHoodAngle = effectiveMaxAuto;
                    updateHoodServoPosition(currentHoodAngle);
                }
            }

            // ================= SHOOTER PIDF (PIDFMotorController + voltage comp) =================
            // Update tunings live (for gamepad2 adjustment)
            leftController .setTunings(kP, kI, kD, kF);
            rightController.setTunings(kP, kI, kD, kF);

            double currentVelocityLeft  = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();
            double currentVoltage = voltageSensor.getVoltage();

            // Convert target ticks/sec to RPM for PIDFMotorController
            double targetRPMCalc = targetVelocityTicks * 60.0 / TICKS_PER_REV;

            double powerLeft, powerRight;
            if (targetVelocityTicks == 0) {
                powerLeft = 0; powerRight = 0;
                leftController.reset(); rightController.reset();
            } else {
                powerLeft  = leftController .computePowerForTargetRPMWithVoltageCompensation(targetRPMCalc, currentVelocityLeft,  currentVoltage, NOMINAL_VOLTAGE);
                powerRight = rightController.computePowerForTargetRPMWithVoltageCompensation(targetRPMCalc, currentVelocityRight, currentVoltage, NOMINAL_VOLTAGE);
            }

            shooterLeft.setPower(powerLeft);
            shooterRight.setPower(powerRight);

            // ================= TELEMETRY =================
            double targetRPM = targetVelocityTicks * 60.0 / TICKS_PER_REV;
            telemetry.addData("1. Target Distance (m)", "%.2f %s", targetDistance, useOdometry ? "(ODO)" : "(MANUAL)");

            if (Double.isNaN(requiredVelocityMS)) {
                telemetry.addData("2. Physics Result", "IMPOSSIBLE SHOT!");
            } else {
                telemetry.addData("2. Req. Velocity (m/s)", "%.2f", requiredVelocityMS);
                telemetry.addData("3. Req. Hood Angle", "%.1f deg", requiredHoodAngle);
            }

            telemetry.addData("4. Target Ticks/Sec", "%.0f", targetVelocityTicks);
            telemetry.addData("5. Target RPM", "%.0f", targetRPMCalc);
            telemetry.addData("Motor Power (L/R)", "%.2f / %.2f", powerLeft, powerRight);
            double baseServo = angleToServoPosition(currentHoodAngle);
            telemetry.addData("Hood Angle", "%.1f deg %s", currentHoodAngle, manualHoodOverride ? "(MANUAL)" : "(AUTO)");
            telemetry.addData("Servo1 (normal)",  "%.4f", baseServo);
            telemetry.addData("Servo2 (flipped)", "%.4f", 1.0 - baseServo);
            telemetry.addLine();

            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");
            telemetry.addLine();
            telemetry.addLine("--- PIDF TUNING ---");
            telemetry.addData("Selected", "-> " + currentSelected.name() + " <-");
            telemetry.addData("kP", "%.5f %s", kP, currentSelected == TuneState.P ? "<--" : "");
            telemetry.addData("kI", "%.5f %s", kI, currentSelected == TuneState.I ? "<--" : "");
            telemetry.addData("kD", "%.5f %s", kD, currentSelected == TuneState.D ? "<--" : "");
            telemetry.addData("kF", "%.5f %s", kF, currentSelected == TuneState.F ? "<--" : "");
            telemetry.addLine();

            telemetry.addLine("--- MOTOR PERFORMANCE ---");
            telemetry.addData("Left Actual RPM", "%.0f", (currentVelocityLeft * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", "%.0f", (currentVelocityRight * 60.0) / TICKS_PER_REV);
            telemetry.addData("Battery", "%.1f V", currentVoltage);

            telemetry.update();
        }
    }

    // =================================================================================
    // HOOD SERVO METHODS
    // =================================================================================

    /**
     * Convert a hood angle (degrees) to servo position.
     * Servo 0.5 = MIN_HOOD_ANGLE (lowest point).
     * Servo INCREASES to raise the hood (add for higher angle).
     * 1 deg hood = GEAR_RATIO deg servo = GEAR_RATIO/180 servo units.
     */
    private double angleToServoPosition(double angle) {
        double servoPos = SERVO_START_POS + (angle - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    /** Update both hood servos — matching hoodtest: servo1 normal, servo2 flipped. */
    private void updateHoodServoPosition(double angle) {
        double servoPos = angleToServoPosition(angle);
        hoodServo1.setPosition(servoPos);
        hoodServo2.setPosition(1.0 - servoPos);
    }

    // =================================================================================
    // PHYSICS CALCULATION (full ballistics solver with lip clearance)
    // =================================================================================
    public static double[] distanceToLauncherValues(double distance) {
        // distance = interpolateToShootingDistance(distance);

        distance += DISTANCE_OFFSET;
        if (distance <= 0) return new double[]{Double.NaN, Double.NaN};

        double g = GRAVITY;
        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // Attempt 1: Backboard shot (priority - faster, flatter)
        double[] result = calculateBestShot(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, g);

        // Attempt 2: Goal center fallback
        if (Double.isNaN(result[0])) {
            result = calculateBestShot(x, TARGET_HEIGHT, xLip, deltaYLip, g);
        }

        return result;
    }

    private static double[] calculateBestShot(double x, double targetY, double xLip, double deltaYLip, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double minPhysAngleH = 90.0 - MAX_HOOD_ANGLE;
        double maxPhysAngleH = 90.0 - MIN_HOOD_ANGLE;

        double minGeomAngle = Math.toDegrees(Math.atan(deltaY / x)) + 0.1;

        double minLipH = 0.0;
        if (xLip > 0) {
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            if (Math.abs(den) > 1e-5) {
                minLipH = Math.toDegrees(Math.atan(num / den));
            } else {
                minLipH = 89.9;
            }
        }

        double targetAngleH = Math.max(minPhysAngleH, Math.max(minLipH, minGeomAngle));

        if (targetAngleH > maxPhysAngleH) {
            return new double[]{Double.NaN, Double.NaN};
        }

        double vReq = calculateVelocity(x, deltaY, targetAngleH, g);

        if (!Double.isNaN(vReq) && vReq <= LAUNCHER_MAX_BALL_VELOCITY) {
            return new double[]{vReq, 90.0 - targetAngleH};
        }

        double v = LAUNCHER_MAX_BALL_VELOCITY;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;
        double disc = B * B - 4 * A * C;

        if (disc < 0) return new double[]{Double.NaN, Double.NaN};

        double sqrtD = Math.sqrt(disc);
        double tan1 = (-B - sqrtD) / (2 * A);
        double tan2 = (-B + sqrtD) / (2 * A);

        double a1 = Math.toDegrees(Math.atan(tan1));
        double a2 = Math.toDegrees(Math.atan(tan2));

        if (a1 >= targetAngleH && a1 <= maxPhysAngleH) return new double[]{v, 90.0 - a1};
        if (a2 >= targetAngleH && a2 <= maxPhysAngleH) return new double[]{v, 90.0 - a2};

        return new double[]{Double.NaN, Double.NaN};
    }

    private static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 1e-9) return Double.NaN;

        return Math.sqrt((g * x * x) / denom);
    }

    // =================================================================================
    // LINEAR INTERPOLATION (m/s -> Ticks/sec)
    // =================================================================================
    public static double interpolateToTicks(double velocityMs) {
        double[] inputMs = {-0.01, 0.0, 4.29,   4.49,   4.76,   5.22,   5.65,   6.06,   6.44,   6.86,   7.2,    10.0 };
        double[] outputTicks = {-0.01, 0.0, 1220.0, 1280.0, 1360.0, 1500.0, 1660.0, 1800.0, 1960.0, 2120.0, 2160.0, 2280.0};

        if (velocityMs <= inputMs[0]) return outputTicks[0];
        if (velocityMs >= inputMs[inputMs.length - 1]) return outputTicks[outputTicks.length - 1];

        for (int i = 0; i < inputMs.length - 1; i++) {
            if (velocityMs >= inputMs[i] && velocityMs <= inputMs[i + 1]) {
                double fraction = (velocityMs - inputMs[i]) / (inputMs[i + 1] - inputMs[i]);
                return outputTicks[i] + fraction * (outputTicks[i + 1] - outputTicks[i]);
            }
        }
        return 0.0;
    }
}
