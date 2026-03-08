package org.firstinspires.ftc.teamcode.Opmode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
import org.firstinspires.ftc.teamcode.testing.PIDFMotorController;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "RC2", group = "Auto")
public class Red_Close_15_2 extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor voltageSensor;
    private Servo Gate;
    private Servo transferBlocker;
    private Servo hoodServo1, hoodServo2;

    // Hood angle — starts at 0.7, drops 0.02 per 200 ticks of velocity drop (ball launch)
    private static final double HOOD_POS_START = 0.5;
    private static final double HOOD_DROP_PER_LAUNCH = 0.02;
    private static final double LAUNCH_DROP_THRESHOLD = 300.0;  // ticks/sec velocity drop = ball fired
    private double currentHoodPos = HOOD_POS_START;
    private double lastAvgVelocity = 0;

    // =======================
    // Shooter Constants
    // =======================
    private static final double TICKS_PER_REV      = 28.0;
    private static final double NOMINAL_VOLTAGE    = 12.0;
    private static final double TARGET_RPM_INITIAL = 3200;
    private static final double TARGET_RPM_FUTURE  = 3200;

    // Dual gain sets — match Shooter.java / ShooterConstants
    private static final double kP_low  = 0.0012, kI_low  = 0.0003, kD_low  = 0.00008, kF_low  = 0.00045;
    private static final double kP_high = 0.0015, kI_high = 0.0004, kD_high = 0.00010, kF_high = 0.00045;
    private static final double RPM_GAIN_THRESHOLD = 3600.0;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    // Active RPM target — starts at initial, swapped to future after first shot
    private double activeTargetRPM = TARGET_RPM_INITIAL;

    // =======================
    // Gate Positions
    // =======================
    private static final double GATE_CLOSED = 0.02;
    private static final double GATE_OPEN   = 0.27;

    // =======================
    // Intake Constants
    // =======================
    private static final double SERVO_HOME           = 0.5;
    private static final double SERVO_EXTENDED       = 0.0;
    private static final double TRANSFER_RESET_DELAY = 0.35;
    private static final double INTAKE_SPEED = 0.75;

    // =======================
    // Intake wait at pick pose
    // =======================
    private static final double PICK_FROM_CLEAR_SECONDS = 1.5;
    private static final double WAIT_AT_GATE = 0.25;

    // =======================
    // Poses
    // =======================
    private final Pose startPose         = new Pose(124, 124, Math.toRadians(45));
    private final Pose shootPose         = new Pose(96,  96,  Math.toRadians(45));
    private final Pose pickupPose2       = new Pose(102, 60,  Math.toRadians(0));
    private final Pose Intake2End        = new Pose(134, 60,  Math.toRadians(0));
    private final Pose clearPose         = new Pose(129, 68,  Math.toRadians(0));
    private final Pose pickFromClearPose = new Pose(132, 66,  Math.toRadians(45));
    private final Pose pickupPose1       = new Pose(102, 84,  Math.toRadians(0));
    private final Pose Intake1End        = new Pose(128, 84,  Math.toRadians(0));
    private final Pose intermediatePose1 = new Pose(108, 60,  Math.toRadians(22.5));
    private final Pose intermediatePose2 = new Pose(102, 66,  Math.toRadians(22.5));
    private final Pose parkPose          = new Pose(118, 68,  Math.toRadians(0));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShootFromStart;
    private PathChain toPickup2Start;
    private PathChain toPickup2End;
    private PathChain toShootFromPickup2;
    private PathChain toClear;
    private PathChain toPickFromClear;
    private PathChain toShootFromPickFromClear;
    private PathChain toPickup1Start;
    private PathChain toPickup1End;
    private PathChain toShootFromPickup1;
    private PathChain toPark;

    // =======================
    // State Machine
    // =======================
    private final Timer actionTimer = new Timer();
    private int state = 0;

    private static final double SPINUP_TIME_1 = 0.275;
    private static final double SPINUP_TIME_2 = 0.475;
    private static final double SHOOT_TIME_1  = 0.45;
    private static final double SHOOT_TIME_2  = 0.65;

    @Override
    public void runOpMode() {

        // ── Hardware Init ──────────────────────────────────────────────────
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

        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodServo1 = hardwareMap.servo.get("angleChange1");
        hoodServo2 = hardwareMap.servo.get("angleChange2");
        hoodServo1.setPosition(HOOD_POS_START);
        hoodServo2.setPosition(HOOD_POS_START);

        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

        transferBlocker = hardwareMap.servo.get("transferBlocker");
        transferBlocker.setPosition(SERVO_HOME);

        // ── PIDF Controllers (init with high gains since target is 3700 RPM) ──
        leftController  = new PIDFMotorController(kP_high, kI_high, kD_high, kF_high, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_high, kI_high, kD_high, kF_high, TICKS_PER_REV);

        // ── Pedro Init ─────────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        toShootFromStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPickup2Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose2.getHeading())
                .build();

        // Intake drive: Pedro follows from pickupPose2 to afterIntake1 while intake runs
        toPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, Intake2End))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), Intake2End.getHeading())
                .build();

        toShootFromPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(Intake2End, intermediatePose1, shootPose))
                .setLinearHeadingInterpolation(Intake2End.getHeading(), shootPose.getHeading())
                .build();

        // ── Paths for third/fourth ball pickup ────────────────────────────
        toClear = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intermediatePose2, clearPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), clearPose.getHeading())
                .build();

        toPickFromClear = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, pickFromClearPose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), pickFromClearPose.getHeading())
                .build();

        toShootFromPickFromClear = follower.pathBuilder()
                .addPath(new BezierCurve(pickFromClearPose, intermediatePose1, shootPose))
                .setLinearHeadingInterpolation(pickFromClearPose.getHeading(), shootPose.getHeading())
                .build();

        // ── Fifth pickup cycle paths (sweep along y=84) ───────────────────
        toPickup1Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose1.getHeading())
                .build();

        toPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1, Intake1End))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), Intake1End.getHeading())
                .build();

        toShootFromPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Intake1End, shootPose))
                .setLinearHeadingInterpolation(Intake1End.getHeading(), shootPose.getHeading())
                .build();

        // ── Park path (shootPose → parkPose) ──────────────────────────────
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();

        telemetry.addLine("✅ Initialized");
        telemetry.update();

        waitForStart();

        // Reset PIDF timers so first frame doesn't have 30+ second stale dt
        leftController.reset();
        rightController.reset();

        setState(0);

        while (opModeIsActive()) {
            follower.update();
            updateShooter();
            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Path Busy", follower.isBusy());
            telemetry.addData("Timer (s)", String.format("%.2f", actionTimer.getElapsedTimeSeconds()));
            double rpmL = shooterLeft.getVelocity()  * 60.0 / TICKS_PER_REV;
            double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
            double avgRPM = (rpmL + rpmR) / 2.0;
            telemetry.addData("RPM", String.format("L:%d R:%d Avg:%d", (int) rpmL, (int) rpmR, (int) avgRPM));
            telemetry.addData("Target RPM", (int) activeTargetRPM);
            telemetry.addData("RPM Error", String.format("%+d", (int)(activeTargetRPM - avgRPM)));
            telemetry.addData("Power L/R", String.format("%.2f / %.2f", shooterLeft.getPower(), shooterRight.getPower()));
            telemetry.addData("Hood Pos", String.format("%.3f", currentHoodPos));
            telemetry.addData("Battery", String.format("%.1f V", voltageSensor.getVoltage()));
            telemetry.update();
        }

        // Cleanup on stop
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        middleTransfer.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    // ── Shooter PIDF update — runs entire auto, never turns off ────────────
    private void updateShooter() {
        // Switch gains based on RPM (same as Shooter.java in TeleOp)
        if (activeTargetRPM >= RPM_GAIN_THRESHOLD) {
            leftController.setTunings(kP_high, kI_high, kD_high, kF_high);
            rightController.setTunings(kP_high, kI_high, kD_high, kF_high);
        } else {
            leftController.setTunings(kP_low, kI_low, kD_low, kF_low);
            rightController.setTunings(kP_low, kI_low, kD_low, kF_low);
        }

        double voltage = voltageSensor.getVoltage();
        double powerL  = leftController.computePowerForTargetRPMWithVoltageCompensation(
                activeTargetRPM, shooterLeft.getVelocity(), voltage, NOMINAL_VOLTAGE);
        double powerR  = rightController.computePowerForTargetRPMWithVoltageCompensation(
                activeTargetRPM, shooterRight.getVelocity(), voltage, NOMINAL_VOLTAGE);
        shooterLeft.setPower(powerL);
        shooterRight.setPower(powerR);

        // Detect ball launch: velocity drop > 200 ticks/sec means a ball just fired
        double avgVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        double velocityDrop = lastAvgVelocity - avgVelocity;
        if (velocityDrop >= LAUNCH_DROP_THRESHOLD && lastAvgVelocity > 0) {
            currentHoodPos = Math.max(0.0, currentHoodPos - HOOD_DROP_PER_LAUNCH);
            hoodServo1.setPosition(currentHoodPos);
            hoodServo2.setPosition(currentHoodPos);
        }
        lastAvgVelocity = avgVelocity;
    }

    // ── State Machine ──────────────────────────────────────────────────────
    // PATH: shoot → pickup2 → shoot → clear→pickFromClear → shoot → clear→pickFromClear → shoot → pickup1 → shoot → park
    private void updateStateMachine() {
        switch (state) {

            // ── FIRST SHOOT CYCLE (TARGET_RPM_INITIAL) ────────────────────

            case 0:
                activeTargetRPM = TARGET_RPM_INITIAL;
                middleTransfer.setPower(1.0);
                follower.followPath(toShootFromStart, true);
                middleTransfer.setPower(0.0);
                setState(1);
                break;

            case 1: {
                double driveElapsed = actionTimer.getElapsedTimeSeconds();
                double rampDuration = 1.0;
                double t = Math.min(1.0, driveElapsed / rampDuration);
                currentHoodPos = HOOD_POS_START - (t * HOOD_DROP_PER_LAUNCH);
                hoodServo1.setPosition(currentHoodPos);
                hoodServo2.setPosition(currentHoodPos);
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;
            }

            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_1) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(3);
                }
                break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_1) {
                    Gate.setPosition(GATE_CLOSED);
                    resetHood();
                    follower.followPath(toPickup2Start, true);
                    middleTransfer.setPower(0);
                    setState(4);
                }
                break;

            // ── RAPID INTAKE ──────────────────────────────────────────────

            case 4:
                if (!follower.isBusy()) {
                    transferBlocker.setPosition(SERVO_HOME);
                    setState(5);
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= TRANSFER_RESET_DELAY) {
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    middleTransfer.setPower(1.0);
                    follower.setMaxPower(INTAKE_SPEED);
                    follower.followPath(toPickup2End, true);
                    follower.setMaxPower(1.0);
                    setState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromPickup2, true);
                    setState(7);
                }
                break;

            // ── SECOND SHOOT CYCLE ────────────────────────────────────────

            case 7:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(8);
                }
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(9);
                }
                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    resetHood();
                    follower.followPath(toClear, true);
                    middleTransfer.setPower(0);
                    setState(10);
                }
                break;

            // ── THIRD PICKUP CYCLE ────────────────────────────────────────

            // STATE 10: Wait to arrive at clearPose
            case 10:
                if (!follower.isBusy()) {
                    setState(11);
                }
                break;

            // STATE 11: Wait at gate, then start intake and drive to pickFromClearPose
            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= WAIT_AT_GATE) {
                    middleTransfer.setPower(1.0);
                    follower.followPath(toPickFromClear, true);
                    setState(12);
                }
                break;

            // STATE 12: Wait to arrive at pickFromClearPose (intake running)
            case 12:
                if (!follower.isBusy()) {
                    setState(13);
                }
                break;

            // STATE 13: Wait at pickFromClearPose, then stop intake and drive to shoot pose
            case 13:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS) {
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    follower.followPath(toShootFromPickFromClear, true);
                    setState(14);
                }
                break;

            // STATE 14: Wait to arrive at shoot pose
            case 14:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(15);
                }
                break;

            // STATE 15: Spinup hold, then open gate and start transfer
            case 15:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(16);
                }
                break;

            // STATE 16: Shoot, then close gate, reset hood, drive to fourth pickup
            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    resetHood();
                    follower.followPath(toClear, true);
                    middleTransfer.setPower(0);
                    setState(17);
                }
                break;

            // ── FOURTH PICKUP CYCLE ───────────────────────────────────────

            // STATE 17: Wait to arrive at clearPose
            case 17:
                if (!follower.isBusy()) {
                    setState(18);
                }
                break;

            // STATE 18: Wait at gate, then start intake and drive to pickFromClearPose
            case 18:
                if (actionTimer.getElapsedTimeSeconds() >= WAIT_AT_GATE) {
                    middleTransfer.setPower(1.0);
                    follower.followPath(toPickFromClear, true);
                    setState(19);
                }
                break;

            // STATE 19: Wait to arrive at pickFromClearPose (intake running)
            case 19:
                if (!follower.isBusy()) {
                    setState(20);
                }
                break;

            // STATE 20: Wait at pickFromClearPose, then stop intake and drive to shoot pose
            case 20:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS) {
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    follower.followPath(toShootFromPickFromClear, true);
                    setState(21);
                }
                break;

            // STATE 21: Wait to arrive at shoot pose
            case 21:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(22);
                }
                break;

            // STATE 22: Spinup hold, then open gate and start transfer
            case 22:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(23);
                }
                break;

            // STATE 23: Shoot, then close gate, reset hood, drive to pre-sweep pose
            case 23:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    resetHood();
                    follower.followPath(toPickup1Start, true);
                    middleTransfer.setPower(0);
                    setState(24);
                }
                break;

            // ── FIFTH PICKUP CYCLE (sweep along y=84) ─────────────────────

            // STATE 24: Wait to arrive at preSweepPose
            case 24:
                if (!follower.isBusy()) {
                    setState(25);
                }
                break;

            // STATE 25: Turn on intake, drive sweep to (122, 84)
            case 25:
                middleTransfer.setPower(1.0);
                follower.setMaxPower(INTAKE_SPEED);
                follower.followPath(toPickup1End, true);
                follower.setMaxPower(1.0);
                setState(26);
                break;

            // STATE 26: Wait to arrive at sweepEndPose, then drive to shoot pose (intake stays on)
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromPickup1, true);
                    setState(27);
                }
                break;

            // STATE 27: Wait to arrive at shoot pose, then stop intake
            case 27:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(28);
                }
                break;

            // STATE 28: Spinup hold, then open gate and start transfer
            case 28:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(29);
                }
                break;

            // STATE 29: Shoot, then close gate, reset hood, drive to park
            case 29:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0);
                    resetHood();
                    follower.followPath(toPark, true);
                    setState(30);
                }
                break;

            // ── PARK ─────────────────────────────────────────────────────

            // STATE 30: Wait to arrive at parkPose, then stop
            case 30:
                if (!follower.isBusy()) {
                    setState(-1);
                }
                break;

            default:
                // Done — shooter keeps spinning, updateShooter() handles PIDF
                break;
        }
    }

    private void resetHood() {
        currentHoodPos = HOOD_POS_START;
        hoodServo1.setPosition(currentHoodPos);
        hoodServo2.setPosition(currentHoodPos);
    }

    private void setState(int newState) {
        state = newState;
        actionTimer.resetTimer();
    }
}