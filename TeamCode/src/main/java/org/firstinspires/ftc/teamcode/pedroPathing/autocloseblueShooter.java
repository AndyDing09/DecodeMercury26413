package org.firstinspires.ftc.teamcode.pedroPathing;

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

// IMPORTANT: this must match your project’s Pedro constants class.
// In many Pedro setups, you have something like: org.firstinspires.ftc.teamcode.pedroPathing.Constants
// which provides Constants.createFollower(hardwareMap) (common pattern in examples) :contentReference[oaicite:2]{index=2}


@Autonomous(name = "autocloseblueShooterr", group = "Auto")
public class autocloseblueShooter extends LinearOpMode {

    // =======================
    // Hardware (same names as your TeleOp)
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor Intake1, Intake2;
    private DcMotorEx shooterMotor, shooterMotor1;
    private Servo Transfer, TurnTable2, transferBlocker;

    // =======================
    // Servos (same as your TeleOp)
    // =======================
    private final double servoHome = 0.075;
    private final double servoExtendedPos = 0.0;
    private final double ServoStart = 0.5;     // blocker "open" (your start)
    private final double blockerClosed = 0.65; // blocker "closed" (your code uses 0.65)

    private final double turntableStart = 0.5;

    // =======================
    // Shooter constants (same as your TeleOp)
    // =======================
    private static final double TICKS_PER_REV = 28.0; // goBILDA 6000rpm YJ encoder
    private final double fastRPM = 2850;
    private final double slowRPM = 3000;

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private Pose currentPose;

    // ====== EDIT THESE POSES FOR YOUR START/ALLIANCE ======
    // FTC field coords: origin (0,0) is field center. Units are inches in most FTC path libs :contentReference[oaicite:3]{index=3}
    // You said: "start from starting zone right next to the far shooting area"
    // Put the robot center where it starts, heading pointing where the robot faces.
    private final Pose startPose   = new Pose(  24, 120, Math.toRadians( 135)); // <-- CHANGE THIS
    private final Pose shootPose   = new Pose(  49, 95, Math.toRadians(135)); // <-- CHANGE THIS (aimed at far target)
    private final Pose pickupPose1 = new Pose(  49, 72, Math.toRadians(90)); // <-- CHANGE THIS (first pickup)
    //private final Pose pickupPose1_5 = newPose
    private final Pose pickupPose2 = new Pose(  24, -35, Math.toRadians(180)); // <-- CHANGE THIS (second pickup)
    private final Pose parkPose    = new Pose(  55, -58, Math.toRadians( 90)); // <-- CHANGE THIS (park)

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
        Intake1 = hardwareMap.dcMotor.get("Intake1");
        Intake2 = hardwareMap.dcMotor.get("Intake2");
        Transfer = hardwareMap.servo.get("Transfer");
        transferBlocker = hardwareMap.servo.get("transferBlocker");

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
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD); // set if needed

        Transfer.setPosition(servoHome);
        TurnTable2.setPosition(turntableStart);
        transferBlocker.setPosition(ServoStart);

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
                setShooterRPM(fastRPM);
                follower.followPath(toShoot);
                setState(1);
                break;

            case 1:
                // Wait until arrive, then shoot preload
                if (!follower.isBusy()) {
                    // Spin up
                    transferBlocker.setPosition(ServoStart); // allow feeding
                    //sleep(500);
                    actionTimer.resetTimer();
                    setState(2);
                }
                break;

            case 2:
                // Wait for spin-up, then fire preload (adjust count/timing!)
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    transferBlocker.setPosition(0.65);
                    Transfer.setPosition(servoExtendedPos);
                    sleep(500);
                    Transfer.setPosition(servoHome);
                    transferBlocker.setPosition(ServoStart);
                    sleep(200);
                    Intake1.setPower(1);
                    Intake2.setPower(1);
                    sleep(1500);
                    shootBurst(3);
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
                if (!follower.isBusy()) {
                    // Give intake a moment to finish pulling in (tune)
                    actionTimer.resetTimer();
                    shooterMotor.setPower(0);
                    shooterMotor1.setPower(0);
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
                    shootBurst(3); // <-- tune to how many you collected
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
    private void setShooterRPM(double rpm) {
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
        shooterMotor1.setVelocity(ticksPerSec);
    }

    private void stopShooter() {
        shooterMotor.setVelocity(0);
        shooterMotor1.setVelocity(0);
    }

    private void startIntake() {
        Intake1.setPower(1.0);
        Intake2.setPower(1.0);
    }

    private void stopIntake() {
        Intake1.setPower(0.0);
        Intake2.setPower(0.0);
    }

    // Fires N balls by pulsing the transfer servo like your Square macro.
    // Tune delays to your mechanism.
    private void shootBurst(int count) {
        for (int i = 0; i < count && opModeIsActive(); i++) {
            // briefly “anti-jam” like your code
            Intake1.setPower(-0.5);
            Intake2.setPower(-0.5);
            sleep(200);
            Intake1.setPower(0);
            Intake2.setPower(0);
            transferBlocker.setPosition(0.65);
            Transfer.setPosition(servoExtendedPos);
            sleep(500);
            Transfer.setPosition(servoHome);
            transferBlocker.setPosition(ServoStart);
            sleep(200);
            Intake1.setPower(1);
            Intake2.setPower(1);
            sleep(2000);
        }
    }
}
