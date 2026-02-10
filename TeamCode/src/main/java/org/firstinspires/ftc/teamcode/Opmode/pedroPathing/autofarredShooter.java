package org.firstinspires.ftc.teamcode.Opmode.pedroPathing;

import static java.lang.Math.min;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Opmode.TeleOp2;
import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

// IMPORTANT: this must match your project’s Pedro constants class.
// In many Pedro setups, you have something like: org.firstinspires.ftc.teamcode.pedroPathing.Constants
// which provides Constants.createFollower(hardwareMap) (common pattern in examples) :contentReference[oaicite:2]{index=2}


@Autonomous(name = "autofarredShooter", group = "Auto")
public class autofarredShooter extends LinearOpMode {

    // =======================
    // Hardware (same names as your TeleOp)
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor frontIntake, middleTransfer;
    private DcMotorEx shooter, shooter1;
    private Servo TurnTable2, turntable3, Gate;
    private boolean shooterOn = false;

    // =======================
    // Servos (same as your TeleOp)
    // =======================
    private final double servoHome = 0.075;
    private final double servoExtendedPos = 0.0;
    private final double ServoStart = 0.5;     // blocker "open" (your start)
    private final double blockerClosed = 0.65; // blocker "closed" (your code uses 0.65)

    private final double turntableStart = 0.575;

    // =======================
    // Shooter constants (same as your TeleOp)
    // =======================
    private static final double TICKS_PER_REV = 28.0; // goBILDA 6000rpm YJ encoder
    private final double fastRPM = 325;
    private final double slowRPM = 265;
    private double targetRPM = 0;

    private double fastStartTime = -1;

    enum ShooterMode {
        FAST,
        OFF
    }
    private ShooterMode shooterMode = ShooterMode.OFF;
    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private Pose currentPose;

    // ====== EDIT THESE POSES FOR YOUR START/ALLIANCE ======
    // FTC field coords: origin (0,0) is field center. Units are inches in most FTC path libs :contentReference[oaicite:3]{index=3}
    // You said: "start from starting zone right next to the far shooting area"
    // Put the robot center where it starts, heading pointing where the robot faces.
    private final Pose startPose = new Pose(64, 8, Math.toRadians(90)); // <-- CHANGE THIS
    private final Pose shootPose = new Pose(64, 8, Math.toRadians(90)); // <-- CHANGE THIS (aimed at far target)
    private final Pose pickupPose1 = new Pose(80, 8, Math.toRadians(90)); // <-- CHANGE THIS (first pickup)
    private final Pose pickupPose2 = new Pose(-24, -35, Math.toRadians(-180)); // <-- CHANGE THIS (second pickup)
    private final Pose parkPose = new Pose(-55, -58, Math.toRadians(90)); // <-- CHANGE THIS (park)

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
        // Hardware init (copied from your TeleOp)
        // =======================
        frontIntake = hardwareMap.dcMotor.get("frontIntake");
        middleTransfer = hardwareMap.dcMotor.get("middleTransfer");
        Gate = hardwareMap.servo.get("Gate");

        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        TurnTable2 = hardwareMap.servo.get("TurnTable");
        turntable3 = hardwareMap.servo.get("turntable3");



        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD); // set if needed

        TurnTable2.setPosition(turntableStart);

        Gate.setPosition(0.2);

        // =======================
        // Pedro init (common pattern) :contentReference[oaicite:4]{index=4}
        // =======================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addLine("Initialized. CHECK your Pose coordinates before running!");
        telemetry.update();

        waitForStart();

        // Start: go to shoot pose (or shoot immediately if already lined up)
        setState(0);

        while (opModeIsActive()) {
            follower.update();
            currentPose = follower.getPose();

            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H(deg)", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }

        // safety stop
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
                // Drive to shooting pose
                //follower.followPath(toShoot);
                setState(1);
                break;

            case 1:
                // Wait until arrive, then shoot preload
                if (!follower.isBusy()) {
                    // Spin up
                    //setShooterRPM(fastRPM);
                    //Gate.setPosition(0.09); // allow feeding
                    setState(2);
                }
                break;

            case 2:
                // Wait for spin-up, then fire preload (adjust count/timing!)
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    setState(3);
                }
                break;

            case 3:
                // Go pick up first batch while intaking
                //startIntake();
                follower.followPath(toPickup1, true);
                setState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    // Give intake a moment to finish pulling in (tune)
                    actionTimer.resetTimer();
                    setState(5);
                }
                break;

            /*case 5:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopIntake();
                    follower.followPath(backToShoot1, true);
                    setState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setShooterRPM(fastRPM);
                    actionTimer.resetTimer();
                    setState(7);
                }
                break;

            case 7:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    Intake1.setPower(1);
                    Intake2.setPower(1);
                    shootBurst(2); // <-- CHANGE to how many you actually have preloaded
                    sleep(100);
                    shootBurst(1); // <-- CHANGE to how many you actually have preloaded
                    setState(3);
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
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    stopIntake();
                    follower.followPath(backToShoot2, true);
                    setState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setShooterRPM(fastRPM);
                    actionTimer.resetTimer();
                    setState(12);
                }
                break;

            case 12:
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    shootBurst(3); // <-- tune
                    setState(13);
                }
                break;

            case 13:
                // Park
                stopShooter();
                follower.followPath(toPark, true);
                setState(14);
                break;

            case 14:
                if (!follower.isBusy()) {
                    stopShooter();
                    stopIntake();
                    setState(-1);
                }
                break;*/

            default:
                // do nothing
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
        pathTimer.resetTimer();
    }

    // =======================
    // “Shoot” helpers (based on your TeleOp macro)
    // =======================
//    if (shooterOn) {
//        if (shooterMode == ShooterMode.FAST) {
//            double t = getRuntime() - fastStartTime;
//
//            if (t < 2) {
//                targetRPM = 1400;
//            } else if (t < 8) {
//                targetRPM = min(900 + t * 200, 1800);
//            } else {
//                targetRPM = 0;
//                Gate.setPosition(0.23);
//            }
//        }
//    }
    private void setShooterRPM(double rpm) {
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        shooter.setVelocity(ticksPerSec);
        shooter1.setVelocity(ticksPerSec);
    }

    private void stopShooter() {
        shooter.setVelocity(0);
        shooter1.setVelocity(0);
    }

    private void startIntake() {
        frontIntake.setPower(1.0);
    }

    private void stopIntake() {
        frontIntake.setPower(0.0);
    }


        }



