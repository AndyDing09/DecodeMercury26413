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

@Autonomous(name = "CycleClearTest", group = "Auto")
public class CycleClearTest extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    private VoltageSensor voltageSensor;
    private Servo transferBlocker;
    private Servo Gate;
    private Servo axon;

    private Shooter shooter;

    private com.qualcomm.robotcore.util.ElapsedTime matchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // =======================
    // Shooter Constants
    // =======================
    private static final double TARGET_RPM_INITIAL = 3535;
    private static final double TARGET_RPM_NORMAL  = 3600;
    private static final double TARGET_RPM_FINAL   = 3375;
    private double activeTargetRPM = TARGET_RPM_NORMAL;

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
    private static final double INITIAL_SHOOT_SPEED  = 0.01;
    private static final double INTAKE_SPEED         = 0.6;

    // =======================
    // Timing
    // =======================
    private static final double PICK_FROM_CLEAR_SECONDS = 1.5;
    private static final double WAIT_AT_GATE            = 0.25;
    private static final double SPINUP_TIME             = 0.2;
    private static final double SHOOT_TIME              = 0.375;

    // =======================
    // Poses (all preserved from original)
    // =======================
    private final Pose startPose         = new Pose(124, 124, Math.toRadians(45));
    private final Pose InitialShootPose  = new Pose(96,  96,  Math.toRadians(45));
    private final Pose NormalShootPose   = new Pose(100, 84, Math.toRadians(53.75));
    private final Pose FinalShootPose    = new Pose(108, 120, Math.toRadians(26.5));
    private final Pose pickupPose2       = new Pose(102, 60,  Math.toRadians(0));
    private final Pose Intake2End        = new Pose(128, 60,  Math.toRadians(0));
    private final Pose clearPose         = new Pose(122, 65,  Math.toRadians(0));
    private final Pose pickFromClearPose = new Pose(133.5, 61.75,  Math.toRadians(38.5));
    private final Pose pickupPose1       = new Pose(102, 84,  Math.toRadians(0));
    private final Pose Intake1End        = new Pose(127, 84,  Math.toRadians(0));
    private final Pose intermediatePose1 = new Pose(108, 60,  Math.toRadians(22.5));
    private final Pose intermediatePose2 = new Pose(102, 66,  Math.toRadians(22.5));
    private final Pose intermediatePosePickup2 = new Pose(104, 58, Math.toRadians(0));
    private final Pose parkPose          = new Pose(118, 68,  Math.toRadians(0));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShootFromStart, toClear, toPickFromClear, toShootFromPickFromClear;

    // =======================
    // State Machine
    // =======================
    private final Timer actionTimer = new Timer();
    private int state = 0;

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
        axon = hardwareMap.get(Servo.class, "TurnTable");
        axon.setPosition(0.5);
        transferBlocker = hardwareMap.servo.get("Gate");
        transferBlocker.setPosition(SERVO_HOME);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // ── Subsystem Init ─────────────────────────────────────────────────
        shooter = new Shooter(hardwareMap);
        shooter.initControllers();

        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

        // ── Pedro Init ─────────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(NormalShootPose); // Start directly at shoot pose

        toShootFromStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, InitialShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), InitialShootPose.getHeading())
                .build();

        toClear = follower.pathBuilder()
                .addPath(new BezierCurve(NormalShootPose, intermediatePose2, clearPose))
                .setLinearHeadingInterpolation(NormalShootPose.getHeading(), clearPose.getHeading())
                .build();

        toPickFromClear = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, pickFromClearPose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), pickFromClearPose.getHeading())
                .build();

        toShootFromPickFromClear = follower.pathBuilder()
                .addPath(new BezierCurve(pickFromClearPose, intermediatePose1, NormalShootPose))
                .setLinearHeadingInterpolation(pickFromClearPose.getHeading(), NormalShootPose.getHeading())
                .build();

        telemetry.addLine("✅ Initialized — ClearCycleTest");
        telemetry.update();

        waitForStart();

        matchTimer.reset();
        setState(0);

        shooter.setShooterOn(true);

        while (opModeIsActive()) {
            follower.update();
            updateShooter();
            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Path Busy", follower.isBusy());
            telemetry.addData("Timer (s)", String.format("%.2f", actionTimer.getElapsedTimeSeconds()));
            telemetry.update();
        }

        RobotPose.endX       = follower.getPose().getX();
        RobotPose.endY       = follower.getPose().getY();
        RobotPose.endHeading = follower.getPose().getHeading();
        RobotPose.hasData    = true;

        shooter.setShooterOn(false);
        shooter.updatePIDF(voltageSensor, telemetry);
        middleTransfer.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    // ── Shooter Loop ──────────────────────────────────────────────────────
    private void updateShooter() {
        shooter.setTargetRPM(activeTargetRPM);
        shooter.updatePIDF(voltageSensor, telemetry);
    }

    // ── State Machine ──────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (state) {

            // ── DRIVE TO CLEAR POSE ───────────────────────────────────────
            case 0:
                follower.followPath(toClear, true);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

            // ── WAIT AT GATE ──────────────────────────────────────────────
            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= WAIT_AT_GATE) {
                    middleTransfer.setPower(1.0);
                    follower.followPath(toPickFromClear, true);
                    setState(3);
                }
                break;

            // ── DRIVE TO PICK FROM CLEAR ──────────────────────────────────
            case 3:
                if (!follower.isBusy()) {
                    setState(4);
                }
                break;

            // ── WAIT AT PICK FROM CLEAR ───────────────────────────────────
            case 4:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS) {
                    follower.followPath(toShootFromPickFromClear, true);
                    setState(5);
                }
                break;

            // ── DRIVE BACK TO SHOOT POSE ──────────────────────────────────
            case 5:
                if (!follower.isBusy()) {
                    setState(6);
                }
                break;

            // ── SPINUP ────────────────────────────────────────────────────
            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(7);
                }
                break;

            // ── SHOOT ─────────────────────────────────────────────────────
            case 7:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    middleTransfer.setPower(0);
                    setState(-1); // Done
                }
                break;

            default:
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
        actionTimer.resetTimer();
    }
}