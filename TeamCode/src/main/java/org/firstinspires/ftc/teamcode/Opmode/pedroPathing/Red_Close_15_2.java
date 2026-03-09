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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
import org.firstinspires.ftc.teamcode.Storedvalues.RobotPose;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "RC2", group = "Auto")
public class Red_Close_15_2 extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    private VoltageSensor voltageSensor;
    private Servo transferBlocker;
    private Servo Gate; // Controlled locally for precise Auto timing


    private Shooter shooter;

    private com.qualcomm.robotcore.util.ElapsedTime matchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // =======================
    // Shooter Constants
    // =======================
    private static final double TARGET_RPM_INITIAL = 3400;
    private static final double TARGET_RPM_FUTURE  = 3360;
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
    private static final double TRANSFER_RESET_DELAY = 0.35;
    private static final double INITIAL_SHOOT_SPEED = 0.25;
    private static final double INTAKE_SPEED = 0.6;

    // =======================
    // Intake wait at pick pose
    // =======================
    private static final double PICK_FROM_CLEAR_SECONDS = 1.5;
    private static final double WAIT_AT_GATE = 0.25;

    // =======================
    // Poses (Unchanged)
    // =======================
    private final Pose startPose         = new Pose(124, 124, Math.toRadians(45));
    private final Pose shootPose         = new Pose(96,  96,  Math.toRadians(45));
    private final Pose pickupPose2       = new Pose(102, 60,  Math.toRadians(0));
    private final Pose Intake2End        = new Pose(134, 60,  Math.toRadians(0));
    private final Pose clearPose         = new Pose(122, 63,  Math.toRadians(0));
    private final Pose pickFromClearPose = new Pose(135, 60,  Math.toRadians(37.5));
    private final Pose pickupPose1       = new Pose(102, 84,  Math.toRadians(0));
    private final Pose Intake1End        = new Pose(128, 84,  Math.toRadians(0));
    private final Pose intermediatePose1 = new Pose(108, 60,  Math.toRadians(22.5));
    private final Pose intermediatePose2 = new Pose(102, 66,  Math.toRadians(22.5));
    private final Pose parkPose          = new Pose(118, 68,  Math.toRadians(0));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShootFromStart, toPickup2Start, toPickup2End, toShootFromPickup2;
    private PathChain toClear, toPickFromClear, toShootFromPickFromClear;
    private PathChain toPickup1Start, toPickup1End, toShootFromPickup1, toPark;

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

        transferBlocker = hardwareMap.servo.get("transferBlocker");
        transferBlocker.setPosition(SERVO_HOME);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // ── Subsystem Init ─────────────────────────────────────────────────


        shooter = new Shooter(hardwareMap);
        shooter.initControllers();

        // Grab the gate directly from the hardware map or via the new getGate() method
        // so we can execute precise open/close timings in the autonomous state machine
        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

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

        toPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, Intake2End))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), Intake2End.getHeading())
                .build();

        toShootFromPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(Intake2End, intermediatePose1, shootPose))
                .setLinearHeadingInterpolation(Intake2End.getHeading(), shootPose.getHeading())
                .build();

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

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();

        telemetry.addLine("✅ Initialized");
        telemetry.update();

        waitForStart();

        matchTimer.reset();
        setState(0);

        // Turn on the shooter for the duration of autonomous
        shooter.setShooterOn(true);

        while (opModeIsActive()) {
            follower.update();
            updateShooter(); // Delegate logic entirely to the Shooter class

            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Path Busy", follower.isBusy());
            telemetry.addData("Timer (s)", String.format("%.2f", actionTimer.getElapsedTimeSeconds()));
            telemetry.update(); // Note: Shooter class also handles RPM telemetry internally
        }

        // Positions for end of auto for passing on
        RobotPose.endX       = follower.getPose().getX();
        RobotPose.endY       = follower.getPose().getY();
        RobotPose.endHeading = follower.getPose().getHeading();
        RobotPose.hasData    = true;

        // Cleanup on stop
        shooter.setShooterOn(false);
        shooter.updatePIDF(voltageSensor, telemetry); // Forces motors to 0
        middleTransfer.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    // ── Shooter Loop (Delegated to Shooter Class) ──────────────────────────
    private void updateShooter() {
        shooter.setTargetRPM(activeTargetRPM);
        shooter.updatePIDF(voltageSensor, telemetry);
    }

    // ── State Machine ──────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (state) {

            // ── FIRST SHOOT CYCLE ────────────────────
            case 0:
                activeTargetRPM = TARGET_RPM_INITIAL;
                follower.setMaxPower(INITIAL_SHOOT_SPEED);
                middleTransfer.setPower(1.0);
                follower.followPath(toShootFromStart, true);
                middleTransfer.setPower(0.0);
                follower.setMaxPower(1.0);
                setState(1);
                break;

            case 1:
                double driveElapsed = actionTimer.getElapsedTimeSeconds();
                double rampDuration = 1.0;
                double t = Math.min(1.0, driveElapsed / rampDuration);

                // Starts at 0.5 (HOOD_SERVO_INIT) and drops down based on time
                shooter.setHoodAnglePos(0.5 - (t * 0.02));

                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

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
                    shooter.setHoodAnglePos(0.5); // Reset Hood
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
                    shooter.setHoodAnglePos(0.5);
                    follower.followPath(toClear, true);
                    middleTransfer.setPower(0);
                    setState(10);
                }
                break;

            // ── THIRD PICKUP CYCLE ────────────────────────────────────────
            case 10:
                if (!follower.isBusy()) {
                    setState(11);
                }
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= WAIT_AT_GATE) {
                    middleTransfer.setPower(1.0);
                    follower.followPath(toPickFromClear, true);
                    setState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    setState(13);
                }
                break;

            case 13:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS) {
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    follower.followPath(toShootFromPickFromClear, true);
                    setState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    setState(15);
                }
                break;

            case 15:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    setState(16);
                }
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    follower.followPath(toClear, true);
                    middleTransfer.setPower(0);
                    setState(17);
                }
                break;

            // ── FOURTH PICKUP CYCLE ───────────────────────────────────────
            case 17:
                if (!follower.isBusy()) {
                    setState(18);
                }
                break;

            case 18:
                if (actionTimer.getElapsedTimeSeconds() >= WAIT_AT_GATE) {
                    middleTransfer.setPower(1.0);
                    follower.followPath(toPickFromClear, true);
                    setState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    setState(20);
                }
                break;

            case 20:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS) {
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    follower.followPath(toShootFromPickFromClear, true);
                    setState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    setState(22);
                }
                break;

            case 22:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    setState(23);
                }
                break;

            case 23:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    follower.followPath(toPickup1Start, true);
                    middleTransfer.setPower(0);
                    setState(24);
                }
                break;

            // ── FIFTH PICKUP CYCLE ─────────────────────
            case 24:
                if (!follower.isBusy()) {
                    setState(25);
                }
                break;

            case 25:
                middleTransfer.setPower(1.0);
                follower.setMaxPower(INTAKE_SPEED);
                follower.followPath(toPickup1End, true);
                follower.setMaxPower(1.0);
                setState(26);
                break;

            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromPickup1, true);
                    setState(27);
                }
                break;

            case 27:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(28);
                }
                break;

            case 28:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(29);
                }
                break;

            case 29:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0);
                    shooter.setHoodAnglePos(0.5);
                    follower.followPath(toPark, true);
                    setState(30);
                }
                break;

            // ── PARK ─────────────────────────────────────────────────────
            case 30:
                if (!follower.isBusy()) {
                    setState(-1);
                }
                break;

            default:
                // Done — Shooter class keeps spinning via updatePIDF
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
        actionTimer.resetTimer();
    }
}