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


@Autonomous(name = "autocloseredShooter - Ramp", group = "Auto")
public class ArnieautocloseredShooter extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor Intake1, Intake2;
    private DcMotorEx shooterMotor, shooterMotor1;
    private Servo Transfer, TurnTable2, transferBlocker, Gate;

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
    // FTC field coords: origin (0,0) is bottom left. Units are inches in most FTC path libs :contentReference[oaicite:3]{index=3}

    private final Pose startPose   = new Pose(120, 120, Math.toRadians(45));
    private final Pose shootPose   = new Pose(96, 96, Math.toRadians(45));
    private final Pose pickupPose1 = new Pose(96, 84, Math.toRadians(90));
    private final Pose pickupPose2 = new Pose(24, -35, Math.toRadians(180));
    private final Pose parkPose    = new Pose(55, -58, Math.toRadians(90));

    // Paths
    private Path toShoot;
    private PathChain toPickup1;
    private PathChain backToShoot1;
    private PathChain toPickup2;
    private PathChain backToShoot2;
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
        Intake1 = hardwareMap.dcMotor.get("Intake1");
        Intake2 = hardwareMap.dcMotor.get("Intake2");
        Transfer = hardwareMap.servo.get("Transfer");
        transferBlocker = hardwareMap.servo.get("transferBlocker");
        Gate = hardwareMap.servo.get("Gate");  // NEW

        shooterMotor  = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        TurnTable2 = hardwareMap.servo.get("TurnTable");

        Intake2.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set PIDF coefficients
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Initial servo positions
        Transfer.setPosition(servoHome);
        TurnTable2.setPosition(turntableStart);
        transferBlocker.setPosition(ServoStart);
        Gate.setPosition(GATE_CLOSED);  // Start with gate closed

        // =======================
        // Pedro init
        // =======================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addLine("✅ Ramp Shooter Auto Initialized");
        telemetry.addLine("CHECK your Pose coordinates before running!");
        telemetry.update();

        waitForStart();

        setState(0);

        while (opModeIsActive()) {
            follower.update();
            currentPose = follower.getPose();

            updateStateMachine();
            updateShooter();  // Continuously update shooter based on ramp logic

            telemetry.addData("State", state);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H(deg)", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Target RPM", (int)targetRPM);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.update();
        }

        // Safety stop
        stopShooter();
        stopIntake();
    }

    private void buildPaths() {
        // Start -> shoot
        toShoot = new Path(new BezierLine(startPose, shootPose));
        toShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // Shoot -> pickup 1 -> back to shoot
        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose1.getHeading())
                .build();

        backToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1, shootPose))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), shootPose.getHeading())
                .build();

        // Shoot -> pickup 2 -> back to shoot
        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose2.getHeading())
                .build();

        backToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, shootPose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), shootPose.getHeading())
                .build();

        // Shoot -> park
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private void updateStateMachine() {
        switch (state) {

            case 0:
                // Drive to shooting pose and start spinning up
                startShooter();
                follower.followPath(toShoot);
                setState(1);
                break;

            case 1:
                // Wait until arrive at shooting position
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setState(2);
                }
                break;

            case 2:
                // Wait for shooter to reach speed (ramp will auto-shoot after 2s)
                // After 6 seconds, ramp cycle completes automatically
                if (actionTimer.getElapsedTimeSeconds() > 6.5) {
                    stopShooter();
                    setState(3);
                }
                break;

            case 3:
                // Go pick up first batch while intaking
                startIntake();
                follower.followPath(toPickup1, true);
                setState(4);
                break;

            case 4:
                // Wait until arrive
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setState(5);
                }
                break;

            case 5:
                // Give intake time to collect
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    stopIntake();
                    follower.followPath(backToShoot1, true);
                    setState(6);
                }
                break;

            case 6:
                // wait to shoot
                if (!follower.isBusy()) {
                    startShooter();
                    actionTimer.resetTimer();
                    setState(7);
                }
                break;

            case 7:
                // Wait for second shooting cycle to complete
                if (actionTimer.getElapsedTimeSeconds() > 6.5) {
                    stopShooter();
                    setState(8);
                }
                break;

            case 8:
                // Pickup second batch
                startIntake();
                follower.followPath(toPickup2, true);
                setState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setState(10);
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    stopIntake();
                    follower.followPath(backToShoot2, true);
                    setState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startShooter();
                    actionTimer.resetTimer();
                    setState(12);
                }
                break;

            case 12:
                // Wait for third shooting cycle to complete
                if (actionTimer.getElapsedTimeSeconds() > 6.5) {
                    stopShooter();
                    setState(13);
                }
                break;

            case 13:
                // Park
                follower.followPath(toPark, true);
                setState(14);
                break;

            case 14:
                if (!follower.isBusy()) {
                    stopShooter();
                    stopIntake();
                    setState(-1); // Done
                }
                break;

            default:
                // Idle
                break;
        }
    }

    // =======================
    // NEW RAMP SHOOTING SYSTEM
    // =======================

    private void startShooter() {
        shooterActive = true;
        shootStartTime = getRuntime();
        Gate.setPosition(GATE_CLOSED);  // Start CLOSED - prevent premature feeding
    }

    private void updateShooter() {
        if (!shooterActive) {
            return;
        }

        // RAMP LOGIC (from TeleOp fast mode)
        double t = getRuntime() - shootStartTime;

        if (t < 2) {
            // Initial spin-up phase - GATE CLOSED
            targetRPM = 1400;
            Gate.setPosition(GATE_CLOSED);  // Keep closed during spin-up
        } else if (t < 6) {
            // Ramp up phase - GATE OPENS to allow feeding
            targetRPM = min(1100 + t * 100, 1600);
            Gate.setPosition(GATE_OPEN);  // NOW open - shooter is up to speed
        } else {
            // Shooting complete
            targetRPM = 0;
            Gate.setPosition(GATE_CLOSED);  // Close when done
        }

        // Set shooter velocity
        double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
        shooterMotor1.setVelocity(ticksPerSec);
    }

    private void stopShooter() {
        shooterActive = false;
        targetRPM = 0;
        shooterMotor.setVelocity(0);
        shooterMotor1.setVelocity(0);
        Gate.setPosition(GATE_CLOSED);  // Always close gate when stopping
    }

    private void startIntake() {
        Intake1.setPower(1.0);
        Intake2.setPower(1.0);
    }

    private void stopIntake() {
        Intake1.setPower(0.0);
        Intake2.setPower(0.0);
    }

    private void setState(int newState) {
        state = newState;
        pathTimer.resetTimer();
    }
}