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

@Autonomous(name = "RF", group = "Auto")
public class Red_Far extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    private VoltageSensor voltageSensor;
    private Servo transferBlocker;
    private Servo Gate;

    private Shooter shooter;

    private com.qualcomm.robotcore.util.ElapsedTime matchTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // =======================
    // Shooter Constants
    // =======================
    private static final double TARGET_RPM = 4800;
    private double activeTargetRPM = TARGET_RPM;

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
    private static final double SLOW_INTAKE_SPEED    = 0.5;

    // =======================
    // Timing Constants
    // =======================
    private static final double SPINUP_TIME_1 = 0.1;
    private static final double SPINUP_TIME_2 = 0.1;
    private static final double SHOOT_TIME_1  = 0.45;
    private static final double SHOOT_TIME_2  = 0.45;

    // =======================
    // Poses
    // =======================
    private final Pose startPose      = new Pose(84,  8.3, Math.toRadians(90));
    private final Pose shootPose      = new Pose(84,  12,  Math.toRadians(65.6));

    // Intake 1 (row 1 — far side)
    private final Pose preIntake1Pose = new Pose(102, 36,  Math.toRadians(0));
    private final Pose intake1End     = new Pose(132, 36,  Math.toRadians(0));

    // Intake 2 (row 2 — closer to wall)
    private final Pose preIntake2Pose = new Pose(126, 9,   Math.toRadians(0));
    private final Pose intake2End     = new Pose(134, 9,   Math.toRadians(0));

    // Park
    private final Pose parkPose       = new Pose(120, 18,  Math.toRadians(45));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;

    // Start → Shoot
    private PathChain toShootFromStart;
    // Shoot → Pre-Intake 1
    private PathChain toPreIntake1;
    // Pre-Intake 1 → Intake 1 End
    private PathChain toIntake1End;
    // Intake 1 End → Shoot
    private PathChain toShootFromIntake1;
    // Shoot → Pre-Intake 2
    private PathChain toPreIntake2;
    // Pre-Intake 2 → Intake 2 End (slow)
    private PathChain toIntake2End;
    // Intake 2 End → Pre-Intake 2 (slow, back out)
    private PathChain toPreIntake2Back;
    // Pre-Intake 2 → Shoot
    private PathChain toShootFromIntake2;
    // Shoot → Park
    private PathChain toPark;

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

        // Build paths
        toShootFromStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPreIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preIntake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntake1Pose.getHeading())
                .build();

        toIntake1End = follower.pathBuilder()
                .addPath(new BezierLine(preIntake1Pose, intake1End))
                .setLinearHeadingInterpolation(preIntake1Pose.getHeading(), intake1End.getHeading())
                .build();

        toShootFromIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1End, shootPose))
                .setLinearHeadingInterpolation(intake1End.getHeading(), shootPose.getHeading())
                .build();

        toPreIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preIntake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preIntake2Pose.getHeading())
                .build();

        toIntake2End = follower.pathBuilder()
                .addPath(new BezierLine(preIntake2Pose, intake2End))
                .setLinearHeadingInterpolation(preIntake2Pose.getHeading(), intake2End.getHeading())
                .build();

        toPreIntake2Back = follower.pathBuilder()
                .addPath(new BezierLine(intake2End, preIntake2Pose))
                .setLinearHeadingInterpolation(intake2End.getHeading(), preIntake2Pose.getHeading())
                .build();

        toShootFromIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(preIntake2Pose, shootPose))
                .setLinearHeadingInterpolation(preIntake2Pose.getHeading(), shootPose.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();

        telemetry.addLine("✅ Initialized — Far Auto");
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

        // Save final pose for teleop handoff
        RobotPose.endX       = follower.getPose().getX();
        RobotPose.endY       = follower.getPose().getY();
        RobotPose.endHeading = follower.getPose().getHeading();
        RobotPose.hasData    = true;

        // Cleanup
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

            // ══════════════════════════════════════════
            // DRIVE TO FIRST SHOOT POSE
            // ══════════════════════════════════════════

            // State 0: Begin driving to shoot pose (slow initial crawl + transfer on)
            case 0:
                activeTargetRPM = TARGET_RPM;
                follower.setMaxPower(INITIAL_SHOOT_SPEED);
                middleTransfer.setPower(1.0);
                follower.followPath(toShootFromStart, true);
                middleTransfer.setPower(0.0);
                Gate.setPosition(GATE_OPEN);
                follower.setMaxPower(1.0);
                setState(1);
                break;

            // State 1: Ramp hood angle while driving; wait for arrival
            case 1: {
                double elapsed = actionTimer.getElapsedTimeSeconds();
                double t = Math.min(1.0, elapsed / 1.0);
                shooter.setHoodAnglePos(0.5 - (t * 0.02));
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;
            }

            // State 2: Spinup wait
            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_1) {
                    middleTransfer.setPower(1.0);
                    setState(3);
                }
                break;

            // State 3: Shoot duration, then close gate and head to intake 1
            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_1) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    middleTransfer.setPower(0);
                    follower.followPath(toPreIntake1, true);
                    setState(4);
                }
                break;

            // ══════════════════════════════════════════
            // INTAKE 1 CYCLE (row at y=36)
            // ══════════════════════════════════════════

            // State 4: Wait to arrive at pre-intake 1
            case 4:
                if (!follower.isBusy()) {
            //         transferBlocker.setPosition(SERVO_HOME);
                    setState(5);
                }
                break;

            // State 5: Short blocker reset delay, then start intake and drive to intake end
            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= TRANSFER_RESET_DELAY) {
                    activeTargetRPM = TARGET_RPM;
                    middleTransfer.setPower(1.0);
                    follower.setMaxPower(INTAKE_SPEED);
                    follower.followPath(toIntake1End, true);
                    follower.setMaxPower(1.0);
                    setState(6);
                }
                break;

            // State 6: Wait for intake sweep to finish, then head to shoot
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(toShootFromIntake1, true);
                    setState(7);
                }
                break;

            // ══════════════════════════════════════════
            // SECOND SHOOT CYCLE
            // ══════════════════════════════════════════

            // State 7: Arrive at shoot pose — stop transfer
            case 7:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(8);
                }
                break;

            // State 8: Spinup wait
            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(9);
                }
                break;

            // State 9: Shoot duration, close gate, head to pre-intake 2
            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    middleTransfer.setPower(0);
                    follower.followPath(toPreIntake2, true);
                    setState(10);
                }
                break;

            // ══════════════════════════════════════════
            // INTAKE 2 CYCLE (wall row at y=7, slow speed)
            // ══════════════════════════════════════════

            // State 10: Wait to arrive at pre-intake 2
            case 10:
                if (!follower.isBusy()) {
                    transferBlocker.setPosition(SERVO_HOME);
                    setState(11);
                }
                break;

            // State 11: Blocker reset delay, then slow-drive forward into intake 2 end
            case 11:
                if (actionTimer.getElapsedTimeSeconds() >= TRANSFER_RESET_DELAY) {
                    activeTargetRPM = TARGET_RPM;
                    middleTransfer.setPower(1.0);
                    follower.setMaxPower(SLOW_INTAKE_SPEED);
                    follower.followPath(toIntake2End, true);
                    // NOTE: keep speed at SLOW_INTAKE_SPEED for the return trip too
                    setState(12);
                }
                break;

            // State 12: Wait to reach intake 2 end, then slow-drive back to pre-intake 2
            case 12:
                if (!follower.isBusy()) {
                    // Stay at slow speed for the back-out
                    follower.followPath(toPreIntake2Back, true);
                    setState(13);
                }
                break;

            // State 13: Wait to return to pre-intake 2, then restore full speed and head to shoot
            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toShootFromIntake2, true);
                    activeTargetRPM = TARGET_RPM;
                    setState(14);
                }
                break;

            // ══════════════════════════════════════════
            // THIRD SHOOT CYCLE
            // ══════════════════════════════════════════

            // State 14: Arrive at shoot pose — stop transfer
            case 14:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    setState(15);
                }
                break;

            // State 15: Spinup wait
            case 15:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(16);
                }
                break;

            // State 16: Shoot duration, close gate, then park
            case 16:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    shooter.setHoodAnglePos(0.5);
                    middleTransfer.setPower(0);
                    follower.followPath(toPark, true);
                    setState(17);
                }
                break;

            // ══════════════════════════════════════════
            // PARK
            // ══════════════════════════════════════════

            // State 17: Wait to park, then done
            case 17:
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