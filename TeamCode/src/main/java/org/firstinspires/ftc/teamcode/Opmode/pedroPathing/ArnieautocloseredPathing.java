package org.firstinspires.ftc.teamcode.Opmode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

import static java.lang.Math.min;

// IMPORTANT: this must match your project’s Pedro constants class.
// In many Pedro setups, you have something like: org.firstinspires.ftc.teamcode.pedroPathing.Constants
// which provides Constants.createFollower(hardwareMap) (common pattern in examples) :contentReference[oaicite:2]{index=2}


@Autonomous(name = "ArnieautocloseredPathing", group = "Auto")
public class ArnieautocloseredPathing extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // =======================
    // Servo Positions
    // =======================
    private final double servoHome = 0.075;
    private final double servoExtendedPos = 0.0;
    private final double ServoStart = 0.5;     // blocker "open"
    private final double blockerClosed = 0.65; // blocker "closed"
    private final double turntableStart = 0.5;

    // Gate positions (NEW - from TeleOp)
    private final double GATE_CLOSED = 0.23;   // Gate closed (safe position)
    private final double GATE_OPEN = 0.09;     // Gate open (shooting position)

    // =======================
    // Shooter Constants (NEW RAMP SYSTEM)
    // =======================
    private static final double TICKS_PER_REV = 28.0;

    // PIDF Coefficients (from TeleOp)
    private double P = 200;
    private double I = 0;
    private double D = 3;
    private double F = 15;

    // Shooting parameters
    private double targetRPM = 0;
    private final double fastMaxRPM = 2500;
    private double shootStartTime = -1;
    private boolean shooterActive = false;

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private Pose currentPose;

    // ====== EDIT THESE POSES FOR YOUR START/ALLIANCE ======
    // FTC field coords: origin (0,0) is start pos. Units are inches in most FTC path libs :contentReference[oaicite:3]{index=3}

    private final Pose startPose         = new Pose(120, 120, Math.toRadians(45));
    private final Pose shootPose         = new Pose(84, 84, Math.toRadians(45));
    private final Pose pickupPose1       = new Pose(102, 84, Math.toRadians(90));
    private final Pose pickupPose2       = new Pose(102, 60, Math.toRadians(90));
    private final Pose pickupPose3       = new Pose(102, 36, Math.toRadians(90));
    private final Pose PreclearPose      = new Pose(120, 72, Math.toRadians(90));
    private final Pose clearPose         = new Pose(132, 72, Math.toRadians(90));
    private final Pose PickfromclearPose = new Pose(134, 56, Math.toRadians(30));
    private final Pose parkPose          = new Pose(120, 72, Math.toRadians(0));


    // Paths
    private Path     toShoot;
    private PathChain toPickup2, backToShoot_from2;
    private PathChain toClear, toPickFromClear, backToShoot_fromClear;
    private PathChain toPickup1, backToShoot_from1;
    private PathChain toPickup3, backToShoot_from3;
    private PathChain toPark;

    // Timers + state machine
    private final Timer pathTimer = new Timer();
    private final Timer actionTimer = new Timer();
    private int state = 0;

    @Override
    public void runOpMode() {

        // =======================
        // Hardware init
        // =======================

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set PIDF coefficients
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);


        // =======================
        // Pedro init
        // =======================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addLine("✅ Pathing Initialized");
        telemetry.update();

        waitForStart();

        setState(0);

        while (opModeIsActive()) {
            follower.update();
            currentPose = follower.getPose();

            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H(deg)", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Target RPM", (int)targetRPM);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.update();
        }

    }

    private void buildPaths() {
        // Start -> shoot
        toShoot = new Path(new BezierLine(startPose, shootPose));
        toShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // shoot -> pickup2
        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose2.getHeading())
                .build();

        // pickup2 -> shoot
        backToShoot_from2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, shootPose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), shootPose.getHeading())
                .build();

        // shoot -> preclear -> clear
        toClear = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PreclearPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PreclearPose.getHeading())
                .addPath(new BezierLine(PreclearPose, clearPose))
                .setLinearHeadingInterpolation(PreclearPose.getHeading(), clearPose.getHeading())
                .build();

        // clear -> pickfromclear
        toPickFromClear = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, PickfromclearPose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), PickfromclearPose.getHeading())
                .build();

        // pickfromclear -> shoot
        backToShoot_fromClear = follower.pathBuilder()
                .addPath(new BezierLine(PickfromclearPose, shootPose))
                .setLinearHeadingInterpolation(PickfromclearPose.getHeading(), shootPose.getHeading())
                .build();

        // shoot -> preclear -> clear -> pickfromclear (for 2nd and 3rd clear cycles)
        // Reuse toClear and toPickFromClear — same poses, just called again in state machine

        // shoot -> pickup1
        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose1.getHeading())
                .build();

        // pickup1 -> shoot
        backToShoot_from1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1, shootPose))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), shootPose.getHeading())
                .build();

        // shoot -> pickup3
        toPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose3.getHeading())
                .build();

        // pickup3 -> shoot
        backToShoot_from3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose3, shootPose))
                .setLinearHeadingInterpolation(pickupPose3.getHeading(), shootPose.getHeading())
                .build();

        // shoot -> park
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private void updateStateMachine() {
        switch (state) {

            // ── Initial drive to shoot ──────────────────────────────────────
            case 0:
                follower.followPath(toShoot);
                setState(1);
                break;
            case 1:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(2); }
                break;
            case 2: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(3);
                break;

            // ── pickup2 → shoot ─────────────────────────────────────────────
            case 3:
                follower.followPath(toPickup2, true); setState(4);
                break;
            case 4:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(5); }
                break;
            case 5: // intake wait
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_from2, true); setState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(7); }
                break;
            case 7: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(8);
                break;

            // ── clear cycle 1: preclear→clear→pickfromclear→shoot ──────────
            case 8:
                follower.followPath(toClear, true); setState(9);
                break;
            case 9:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(10); }
                break;
            case 10: // brief pause at clear
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toPickFromClear, true); setState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(12); }
                break;
            case 12: // intake wait
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_fromClear, true); setState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(14); }
                break;
            case 14: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(15);
                break;

            // ── clear cycle 2: preclear→clear→pickfromclear→shoot ──────────
            case 15:
                follower.followPath(toClear, true); setState(16);
                break;
            case 16:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(17); }
                break;
            case 17:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toPickFromClear, true); setState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(19); }
                break;
            case 19:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_fromClear, true); setState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(21); }
                break;
            case 21: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(22);
                break;

            // ── clear cycle 3: preclear→clear→pickfromclear→shoot ──────────
            case 22:
                follower.followPath(toClear, true); setState(23);
                break;
            case 23:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(24); }
                break;
            case 24:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toPickFromClear, true); setState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(26); }
                break;
            case 26:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_fromClear, true); setState(27);
                }
                break;
            case 27:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(28); }
                break;
            case 28: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(29);
                break;

            // ── pickup1 → shoot ─────────────────────────────────────────────
            case 29:
                follower.followPath(toPickup1, true); setState(30);
                break;
            case 30:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(31); }
                break;
            case 31:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_from1, true); setState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(33); }
                break;
            case 33: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(34);
                break;

            // ── pickup3 → shoot ─────────────────────────────────────────────
            case 34:
                follower.followPath(toPickup3, true); setState(35);
                break;
            case 35:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(36); }
                break;
            case 36:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(backToShoot_from3, true); setState(37);
                }
                break;
            case 37:
                if (!follower.isBusy()) { actionTimer.resetTimer(); setState(38); }
                break;
            case 38: // shoot
                if (actionTimer.getElapsedTimeSeconds() > 6.5) setState(39);
                break;

            // ── park ────────────────────────────────────────────────────────
            case 39:
                follower.followPath(toPark, true); setState(40);
                break;
            case 40:
                if (!follower.isBusy()) setState(-1);
                break;

            default:
                break; // idle / done
        }
    }




    private void setState(int newState) {
        state = newState;
        pathTimer.resetTimer();
    }
}