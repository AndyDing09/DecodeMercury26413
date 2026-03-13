package org.firstinspires.ftc.teamcode.testing;

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
@TeleOp(name="LauncherMathTestv2", group="Testing")
public class launchermathtestv2 extends LinearOpMode {

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

    // ================= HOOD SERVO (LOCKED) =================
    private Servo hoodServo1;
    private Servo hoodServo2;
    private static final double HOOD_LOCKED_POSITION = 0.9; // Fixed servo position

    // Ticks per revolution (used for RPM telemetry calculation)
    private static final double TICKS_PER_REV = 28.0;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;

    // ================= SHOOTER PIDF =================
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

        // Lock hood to fixed position immediately
        hoodServo1.setPosition(HOOD_LOCKED_POSITION);
        hoodServo2.setPosition(1.0 - HOOD_LOCKED_POSITION);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Physics Shooter Initialized");
        telemetry.addData("Hood", "LOCKED at servo pos %.2f", HOOD_LOCKED_POSITION);
        telemetry.addLine("--- GAMEPAD 1 ---");
        telemetry.addLine("D-Pad Up/Down: +/- 0.1m | Bumpers: +/- 0.5m");
        telemetry.addLine("A: Odometry | B: Reset 2.0m");
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

            // Keep hood locked every loop (safety)
            hoodServo1.setPosition(HOOD_LOCKED_POSITION);
            hoodServo2.setPosition(1.0 - HOOD_LOCKED_POSITION);

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

            if (currentG1DpadUp && !lastG1DpadUp) { targetDistance += 0.1; useOdometry = false; }
            if (currentG1DpadDown && !lastG1DpadDown) { targetDistance -= 0.1; useOdometry = false; }
            if (currentG1RightBumper && !lastG1RightBumper) { targetDistance += 0.5; useOdometry = false; }
            if (currentG1LeftBumper && !lastG1LeftBumper) { targetDistance -= 0.5; useOdometry = false; }

            if (gamepad1.x) { useOdometry = true; }
            if (gamepad1.circle) { targetDistance = 2.0; useOdometry = false; }

            if (targetDistance < 0) targetDistance = 0;

            if (useOdometry) {
                Pose pose = follower.getPose();
                targetDistance = Math.hypot(pose.getX() - MathLib.GOAL_CENTER_X, pose.getY() - MathLib.GOAL_CENTER_Y) * MathLib.INCHES_TO_METERS;
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

            // ================= DISTANCE -> TICKS LOOKUP =================
            double targetVelocityTicks = interpolateDistanceToTicks(targetDistance);

            // ================= SHOOTER PIDF =================
            leftController .setTunings(kP, kI, kD, kF);
            rightController.setTunings(kP, kI, kD, kF);

            double currentVelocityLeft  = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();
            double currentVoltage = voltageSensor.getVoltage();

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
            telemetry.addData("1. Target Distance (m)", "%.2f %s", targetDistance, useOdometry ? "(ODO)" : "(MANUAL)");
            telemetry.addData("2. Target Ticks/Sec", "%.0f", targetVelocityTicks);
            telemetry.addData("3. Target RPM", "%.0f", targetRPMCalc);
            telemetry.addData("Motor Power (L/R)", "%.2f / %.2f", powerLeft, powerRight);
            telemetry.addData("Hood", "LOCKED at %.2f", HOOD_LOCKED_POSITION);
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
            telemetry.addData("Left Actual RPM",  "%.0f", (currentVelocityLeft  * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", "%.0f", (currentVelocityRight * 60.0) / TICKS_PER_REV);
            telemetry.addData("Battery", "%.1f V", currentVoltage);

            telemetry.update();
        }
    }

    // =================================================================================
    // DISTANCE -> TICKS LOOKUP TABLE
    // Replace placeholder values below with your measured data:
    //   inputDistances  = measured distances from goal (meters)
    //   outputTicks     = motor ticks/sec needed at that distance
    // =================================================================================
    public static double interpolateDistanceToTicks(double distanceM) {
        double[] inputDistances = { 1.1,  1.5,  2.08,  2.4, 2.75, 3.08, 3.34 }; // meters  — REPLACE WITH YOUR DATA
        double[] outputTicks =    { 1283, 1540, 1633, 1680, 1715, 1820, 1855 }; // ticks/s — REPLACE WITH YOUR DATA

        if (distanceM <= inputDistances[0]) return outputTicks[0];
        if (distanceM >= inputDistances[inputDistances.length - 1]) return outputTicks[outputTicks.length - 1];

        for (int i = 0; i < inputDistances.length - 1; i++) {
            if (distanceM >= inputDistances[i] && distanceM <= inputDistances[i + 1]) {
                double fraction = (distanceM - inputDistances[i]) / (inputDistances[i + 1] - inputDistances[i]);
                return outputTicks[i] + fraction * (outputTicks[i + 1] - outputTicks[i]);
            }
        }
        return 0.0;
    }
}