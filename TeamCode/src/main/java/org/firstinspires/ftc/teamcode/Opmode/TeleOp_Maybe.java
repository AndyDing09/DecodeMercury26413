package org.firstinspires.ftc.teamcode.Opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
import org.firstinspires.ftc.teamcode.Storedvalues.RobotPose;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TeleOpMaybe", group = "TeleOp")
public class TeleOp_Maybe extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // ================= MECHANISMS =================
    private DcMotor middleTransfer;
    private DcMotor turretMotor;
    private VoltageSensor voltageSensor;
    private Servo transferBlocker;
    private Servo Gate;

    // ================= SUBSYSTEMS =================
    private Follower follower;
    private Shooter shooter;

    // ================= HOOP POSITION (field coordinates, inches) =================
    private static final double HOOP_X = 136.0;
    private static final double HOOP_Y = 136.0;

    // ================= TURRET TRACKING (PD controller) =================
    // kP: how hard to push per degree of error
    // kD: how hard to brake as error is closing
    // kF: minimum power floor to overcome stiction
    private static final double TURRET_kP        = 0.012;
    private static final double TURRET_kD        = 0.004;
    private static final double TURRET_kF        = 0.06;
    private static final double TURRET_DEADZONE  = 2.0;   // degrees — don't move if within this
    private static final double TURRET_MAX_POWER = 0.90;

    // ================= TURRET ENCODER LIMITS =================
    // Turret stops commanding power at these limits but tracking math keeps running
    private static final int TURRET_LEFT_LIMIT  = -200;
    private static final int TURRET_RIGHT_LIMIT = 200;

    // ================= TURRET CONVERSION =================
    // How many encoder ticks = 1 degree of turret rotation. TUNE THIS.
    private static final double TICKS_PER_DEGREE = 5.0;

    // ================= TRACKING STATE =================
    private double lastAngleError = 0;

    // ================= GATE POSITIONS =================
    private static final double GATE_CLOSED = 0.5;
    private static final double GATE_OPEN   = 0.27;

    // ================= SERVO POSITIONS =================
    private static final double SERVO_HOME = 0.5;

    // ================= BUTTON STATE =================
    private boolean lastGamepad2X = false;

    @Override
    public void runOpMode() {

        // ── Drive Motors ──────────────────────────────────────────────────
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Other Hardware ─────────────────────────────────────────────────
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        transferBlocker = hardwareMap.servo.get("transferBlocker");
        transferBlocker.setPosition(SERVO_HOME);

        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

        // ── Shooter ────────────────────────────────────────────────────────
        shooter = new Shooter(hardwareMap);
        shooter.initControllers();
        shooter.setShooterOn(false);

        // ── Pedro Pathing — set starting pose from auto ────────────────────
        follower = Constants.createFollower(hardwareMap);

        Pose startPose;
        if (RobotPose.hasData) {
            // Auto was run — use the exact pose the robot ended at
            startPose = new Pose(RobotPose.endX, RobotPose.endY, RobotPose.endHeading);
            telemetry.addLine("✅ Pose loaded from Auto");
        } else {
            // No auto data — fall back to default (useful for standalone teleop testing)
            startPose = new Pose(RobotPose.DEFAULT_X, RobotPose.DEFAULT_Y, RobotPose.DEFAULT_HEADING);
            telemetry.addLine("⚠️ No auto data — using default pose");
        }

        follower.setStartingPose(startPose);

        telemetry.addData("Start X",       String.format("%.1f", startPose.getX()));
        telemetry.addData("Start Y",       String.format("%.1f", startPose.getY()));
        telemetry.addData("Start Heading", String.format("%.1f°", Math.toDegrees(startPose.getHeading())));
        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── 1. UPDATE ODOMETRY ────────────────────────────────────────
            // follower.update() keeps the pose estimate fresh every loop
            // We are NOT following a path — just using it as a localizer
            follower.update();

            // ── 2. DRIVE ──────────────────────────────────────────────────
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            // ── 3. TURRET TRACKING ────────────────────────────────────────
            updateTurretTracking();

            // ── 4. INTAKE ─────────────────────────────────────────────────
            if (gamepad2.left_bumper)  middleTransfer.setPower(1.0);
            if (gamepad2.right_bumper) middleTransfer.setPower(0.0);

            // ── 5. GATE ───────────────────────────────────────────────────
            boolean currentGamepad2X = gamepad2.x;
            if (currentGamepad2X && !lastGamepad2X) {
                Gate.setPosition(Gate.getPosition() == GATE_CLOSED ? GATE_OPEN : GATE_CLOSED);
            }
            lastGamepad2X = currentGamepad2X;

            // ── 6. SHOOTER ────────────────────────────────────────────────
            if (gamepad2.dpad_up) {
                shooter.setShooterOn(true);
                shooter.setTargetRPM(4800);
            }
            if (gamepad2.dpad_down) {
                shooter.setShooterOn(true);
                shooter.setTargetRPM(3000);
            }
            shooter.updatePIDF(voltageSensor, telemetry);

            // ── 7. TELEMETRY ──────────────────────────────────────────────
            Pose currentPose = follower.getPose();
            telemetry.addLine("===== ODOMETRY =====");
            telemetry.addData("X",       String.format("%.2f", currentPose.getX()));
            telemetry.addData("Y",       String.format("%.2f", currentPose.getY()));
            telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(currentPose.getHeading())));
            telemetry.addLine("===== TURRET =====");
            telemetry.addData("Encoder",      turretMotor.getCurrentPosition());
            telemetry.addData("Intake Power", middleTransfer.getPower());
            telemetry.addData("Gate",         Gate.getPosition() == GATE_OPEN ? "OPEN" : "CLOSED");
            telemetry.addData("Battery",      String.format("%.1f V", voltageSensor.getVoltage()));
            telemetry.update();
        }

        // ── Cleanup ────────────────────────────────────────────────────────
        shooter.setShooterOn(false);
        shooter.updatePIDF(voltageSensor, telemetry);
        middleTransfer.setPower(0);
        turretMotor.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    /**
     * Odometry-based turret tracking.
     *
     * Every loop:
     *  1. Get current robot pose from Pedro localizer
     *  2. Compute absolute field angle from robot to hoop using atan2
     *  3. Subtract robot heading to get the turret angle error in robot-relative frame
     *  4. Normalize error to [-180, 180] degrees
     *  5. Run PD controller — but clamp output at encoder limits
     *     (tracking math keeps running even when at a limit)
     */
    private void updateTurretTracking() {
        Pose pose = follower.getPose();

        double robotX       = pose.getX();
        double robotY       = pose.getY();
        double robotHeading = pose.getHeading(); // radians

        // Angle from robot to hoop in field frame (radians)
        double angleToHoopField = Math.atan2(HOOP_Y - robotY, HOOP_X - robotX);

        // Convert to robot-relative: how far does the turret need to rotate?
        double angleErrorRad = angleToHoopField - robotHeading;

        // Normalize to [-π, π] so we always take the shortest path
        while (angleErrorRad >  Math.PI) angleErrorRad -= 2 * Math.PI;
        while (angleErrorRad < -Math.PI) angleErrorRad += 2 * Math.PI;

        // Convert to degrees for the PD controller
        double angleErrorDeg = Math.toDegrees(angleErrorRad);

        // PD controller
        double derivative = angleErrorDeg - lastAngleError;
        lastAngleError = angleErrorDeg;

        double outputPower = 0;

        if (Math.abs(angleErrorDeg) > TURRET_DEADZONE) {
            outputPower = (TURRET_kP * angleErrorDeg) + (TURRET_kD * derivative);

            // Feed-forward floor to overcome stiction
            if (Math.abs(outputPower) > 0.001 && Math.abs(outputPower) < TURRET_kF) {
                outputPower = Math.signum(outputPower) * TURRET_kF;
            }
        }

        // Clamp to max power
        outputPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, outputPower));

        // ── SAFETY LIMITS ────────────────────────────────────────────────
        // If at a hardware limit, zero out power in that direction ONLY.
        // The pose tracking and angle math above keeps running unaffected.
        int currentPos = turretMotor.getCurrentPosition();
        if (currentPos <= TURRET_LEFT_LIMIT  && outputPower < 0) outputPower = 0;
        if (currentPos >= TURRET_RIGHT_LIMIT && outputPower > 0) outputPower = 0;

        turretMotor.setPower(outputPower);

        telemetry.addData("Angle to Hoop", String.format("%.2f°", angleErrorDeg));
        telemetry.addData("Turret Power",  String.format("%.3f", outputPower));
        telemetry.addData("At Limit",      (currentPos <= TURRET_LEFT_LIMIT || currentPos >= TURRET_RIGHT_LIMIT) ? "YES ⚠️" : "no");
    }
}