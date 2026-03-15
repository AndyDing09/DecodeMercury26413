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

@Autonomous(name = "BCPartner", group = "Auto")
public class Blue_Close_18_Partner extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    // private DcMotor turret; // Turret motor for position locking during auto
    private VoltageSensor voltageSensor;
    private Servo transferBlocker;
    private Servo Gate; // Controlled locally for precise Auto timing
    private Servo axon;

    private Shooter shooter;

    private com.qualcomm.robotcore.util.ElapsedTime matchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // =======================
    // Shooter Constants
    // =======================
    private static final double TARGET_RPM_INITIAL = 3400;
    private static final double TARGET_RPM_NORMAL = 2950;
    private static final double TARGET_RPM_FINAL  = 3030;
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

    // =======================
    // Intake wait at pick pose
    // =======================
    private static final double PICK_FROM_CLEAR_SECONDS_INITIAL = 1.4;
    private static final double PICK_FROM_CLEAR_SECONDS_FUTURE = 1.15;
    // private static final double WAIT_AT_GATE = 0.25;

    // =======================
    // Poses
    // Mirrored from Red: x -> (144 - x), y unchanged, heading -> (90deg + heading)
    // =======================
    //  Red: (123.5, 123, 43deg)  -> Blue: (20.5, 123, 133deg)
    private final Pose startPose         = new Pose(20.5,  123, Math.toRadians(133));
    //  Red: (97, 97, 46deg)      -> Blue: (47, 97, 136deg)
    private final Pose InitialShootPose  = new Pose(47,    97,  Math.toRadians(136));
    //  Red: (97, 84, 53.4deg)    -> Blue: (47, 84, 143.4deg)
    private final Pose NormalShootPose   = new Pose(47,    84,  Math.toRadians(143.4));
    //  Red: (97, 106, 39.3deg)   -> Blue: (47, 106, 129.3deg)
    private final Pose FinalShootPose    = new Pose(47,   106,  Math.toRadians(129.3));
    //  Red: (139.5, 60, 0deg)    -> Blue: (4.5, 60, 90deg)
    private final Pose Intake2End        = new Pose(4.5,   60,  Math.toRadians(180));
    //  Red: (122, 65, 0deg)      -> Blue: (22, 65, 90deg)
    private final Pose clearPose         = new Pose(22,    65,  Math.toRadians(180));
    //  Red: (138.4, 58, 25.5deg) -> Blue: (5.6, 58, 115.5deg)
    private final Pose PickFromClearPose_INITIAL = new Pose(5.6,  58,  Math.toRadians(115.5));
    //  Red: (139, 58.35, 32.25deg) -> Blue: (5, 58.35, 122.25deg)
    private final Pose PickFromClearPose_FUTURE  = new Pose(5,    58.35, Math.toRadians(122.25));
    //  Red: (102, 84, 0deg)      -> Blue: (42, 84, 90deg)
    private final Pose pickupPose1       = new Pose(42,    84,  Math.toRadians(180));
    //  Red: (96, 86, 15deg)      -> Blue: (48, 86, 105deg)
    private final Pose IntermediatePosePickup1 = new Pose(48,   86,  Math.toRadians(105));
    //  Red: (130.5, 84, 0deg)    -> Blue: (13.5, 84, 90deg)
    private final Pose Intake1End        = new Pose(13.5,  84,  Math.toRadians(180));
    //  Red: (108, 63, 20deg)     -> Blue: (36, 63, 110deg)
    private final Pose intermediatePose1 = new Pose(36,    63,  Math.toRadians(110));
    //  Red: (102, 66, 22.5deg)   -> Blue: (42, 66, 112.5deg)
    private final Pose intermediatePose2 = new Pose(42,    66,  Math.toRadians(112.5));
    //  Red: (94, 50, 15deg)      -> Blue: (50, 50, 105deg)
    private final Pose intermediatePosePickup2 = new Pose(50,   50,  Math.toRadians(105));
    // private final Pose parkPose          = new Pose(...);

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShootFromStart, toPickup2, toShootFromPickup2;
    private PathChain toClear, toPickFromClear, toShootFromPickFromClear;
    private PathChain toPickup1, toShootFromPickup1, toPark;
    private PathChain toPickFromClearDirect;
    private PathChain toPickFromClearDirect_INITIAL, toShootFromPickFromClear_INITIAL, toPickFromClearDirect_FUTURE, toShootFromPickFromClear_FUTURE;

    // =======================
    // State Machine
    // =======================
    private final Timer actionTimer = new Timer();
    private int state = 0;

    private static final double SPINUP_TIME = 0.01;
    private static final double SHOOT_TIME  = 0.475;

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
        follower.setStartingPose(startPose);

        toShootFromStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, InitialShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), InitialShootPose.getHeading())
                .build();

        toPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(InitialShootPose, intermediatePosePickup2, Intake2End))
                .setLinearHeadingInterpolation(InitialShootPose.getHeading(), Intake2End.getHeading())
                .build();

        toShootFromPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(Intake2End, intermediatePose1, NormalShootPose))
                .setLinearHeadingInterpolation(Intake2End.getHeading(), NormalShootPose.getHeading())
                .build();

        toClear = follower.pathBuilder()
                .addPath(new BezierCurve(NormalShootPose, intermediatePose2, clearPose))
                .setLinearHeadingInterpolation(NormalShootPose.getHeading(), clearPose.getHeading())
                .build();

        toPickFromClearDirect_INITIAL = follower.pathBuilder()
                .addPath(new BezierCurve(NormalShootPose, intermediatePose2, PickFromClearPose_INITIAL))
                .setLinearHeadingInterpolation(NormalShootPose.getHeading(), PickFromClearPose_INITIAL.getHeading())
                .build();

        toShootFromPickFromClear_INITIAL = follower.pathBuilder()
                .addPath(new BezierCurve(PickFromClearPose_INITIAL, intermediatePose1, NormalShootPose))
                .setLinearHeadingInterpolation(PickFromClearPose_INITIAL.getHeading(), NormalShootPose.getHeading())
                .build();

        toPickFromClearDirect_FUTURE = follower.pathBuilder()
                .addPath(new BezierCurve(NormalShootPose, intermediatePose2, PickFromClearPose_FUTURE))
                .setLinearHeadingInterpolation(NormalShootPose.getHeading(), PickFromClearPose_FUTURE.getHeading())
                .build();

        toShootFromPickFromClear_FUTURE = follower.pathBuilder()
                .addPath(new BezierCurve(PickFromClearPose_FUTURE, intermediatePose1, NormalShootPose))
                .setLinearHeadingInterpolation(PickFromClearPose_FUTURE.getHeading(), NormalShootPose.getHeading())
                .build();

        toPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(NormalShootPose, IntermediatePosePickup1, Intake1End))
                .setLinearHeadingInterpolation(NormalShootPose.getHeading(), Intake1End.getHeading())
                .build();

        toShootFromPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Intake1End, FinalShootPose))
                .setLinearHeadingInterpolation(Intake1End.getHeading(), FinalShootPose.getHeading())
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
            updateShooter();

            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Path Busy", follower.isBusy());
            telemetry.addData("Timer (s)", String.format("%.2f", actionTimer.getElapsedTimeSeconds()));
            telemetry.update();
        }

        // Positions for end of auto for passing on
        Pose finalPose = follower.getPose();
        RobotPose.endX       = follower.getPose().getX();
        RobotPose.endY       = follower.getPose().getY();
        RobotPose.endHeading = follower.getPose().getHeading();
        RobotPose.hasData    = true;

        // Cleanup on stop
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

            // ── FIRST SHOOT CYCLE ────────────────────
            case 0:
                activeTargetRPM = TARGET_RPM_INITIAL;
                middleTransfer.setPower(1);
                follower.followPath(toShootFromStart, true);
                middleTransfer.setPower(0.0);
                Gate.setPosition(GATE_OPEN);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    middleTransfer.setPower(0.9);
                    setState(3);
                }
                break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    activeTargetRPM = TARGET_RPM_NORMAL;
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0.85);
                    follower.followPath(toPickup2, true);
                    setState(4);
                }
                break;

            // ── DRIVE TO INTAKE2 END via BezierCurve (intake on whole time) ──
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromPickup2, true);
                    setState(5);
                }
                break;

            // ── RETURN TO SHOOT POSE FROM PICKUP2 ────────────────────────
            case 5:
                if (!follower.isBusy()) {
                    setState(6);
                }
                break;

            // ── SECOND SHOOT CYCLE ────────────────────────────────────────
            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(0.85);
                    setState(7);
                }
                break;

            case 7:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    follower.followPath(toPickFromClearDirect_INITIAL, true);
                    setState(8);
                }
                break;

            // ── THIRD PICKUP CYCLE (1st direct shoot→pickFromClear→shoot) ─
            case 8:
                if (!follower.isBusy()) {
                    setState(9);
                }
                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS_INITIAL) {
                    follower.followPath(toShootFromPickFromClear_INITIAL, true);
                    setState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    setState(11);
                }
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    setState(12);
                }
                break;

            case 12:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0.85);
                    follower.followPath(toPickFromClearDirect_FUTURE, true);
                    setState(13);
                }
                break;

            // ── FOURTH PICKUP CYCLE (2nd direct shoot→pickFromClear→shoot) ─
            case 13:
                if (!follower.isBusy()) {
                    setState(14);
                }
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS_FUTURE) {
                    follower.followPath(toShootFromPickFromClear_FUTURE, true);
                    setState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    setState(16);
                }
                break;

            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    setState(17);
                }
                break;

            case 17:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0.85);
                    follower.followPath(toPickFromClearDirect_FUTURE, true);
                    setState(18);
                }
                break;

            // ── FIFTH PICKUP CYCLE (3rd direct shoot→pickFromClear→shoot) ─
            case 18:
                if (!follower.isBusy()) {
                    setState(19);
                }
                break;

            case 19:
                if (actionTimer.getElapsedTimeSeconds() >= PICK_FROM_CLEAR_SECONDS_FUTURE) {
                    follower.followPath(toShootFromPickFromClear_FUTURE, true);
                    setState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    setState(21);
                }
                break;

            case 21:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    setState(22);
                }
                break;

            case 22:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0.85);
                    follower.followPath(toPickup1, true);
                    setState(23);
                }
                break;

            // ── SIXTH PICKUP CYCLE (pickup1 sweep) ────────────────────────
            case 23:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromPickup1, true);
                    activeTargetRPM = TARGET_RPM_FINAL;
                    setState(24);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    setState(25);
                }
                break;

            case 25:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(0.85);
                    setState(26);
                }
                break;

            case 26:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    setState(27);
                }
                break;

            // ── PARK ─────────────────────────────────────────────────────
            case 27:
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