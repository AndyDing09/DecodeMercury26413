package org.firstinspires.ftc.teamcode.Opmode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "Autonomous", group = "Auto")
public class Auto_nomous extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer; // intake/transfer motor — also feeds balls to shooter
    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor voltageSensor;
    private Servo Gate;

    // =======================
    // Shooter Constants (from TeleOp - LOW range settings)
    // =======================
    private static final double TICKS_PER_REV    = 28.0;
    private static final double NOMINAL_VOLTAGE  = 12.0;
    private static final double TARGET_RPM       = 2500; // "low" speed from TeleOp right bumper

    // PIDF coefficients - LOW range (from TeleOp)
    private static final double kP_shooter = 0.006;
    private static final double kI_shooter = 0.0005;
    private static final double kD_shooter = 0.0002;
    private static final double kF_shooter = 0.00620;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    // =======================
    // Gate Positions (from TeleOp)
    // =======================
    private static final double GATE_CLOSED = 0.5;   // Gate closed (safe/idle position)
    private static final double GATE_OPEN   = 0.27;  // Gate open (shooting position)

    // =======================
    // Poses (from existing auto)
    // =======================
    private final Pose startPose    = new Pose(120, 120, Math.toRadians(45));
    private final Pose shootPose    = new Pose(84, 84,  Math.toRadians(45));
    private final Pose pickupPose2  = new Pose(60, 102, Math.toRadians(90));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShoot;
    private PathChain toPickup2;

    // =======================
    // State Machine
    // =======================
    private final Timer actionTimer = new Timer();
    private int state = 0;

    // How long to spin up shooter after arriving before opening gate (seconds)
    private static final double SPINUP_TIME = 1.0;
    // How long to hold gate open for shooting (seconds)
    private static final double SHOOT_TIME  = 3.0;

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

        // middleTransfer doubles as intake and transfer feed to shooter
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

        // ── PIDF Controllers ───────────────────────────────────────────────
        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);

        // ── Pedro Init ─────────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build path: start → shoot (PathChain as requested)
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose2.getHeading())
                .build();

        telemetry.addLine("✅ Initialized — waiting for start");
        telemetry.update();

        waitForStart();

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
            telemetry.addData("Left RPM",  (int) rpmL);
            telemetry.addData("Right RPM", (int) rpmR);
            telemetry.addData("Target RPM", (int) TARGET_RPM);
            telemetry.addData("Battery", String.format("%.1f V", voltageSensor.getVoltage()));
            telemetry.update();
        }

        // Cleanup on stop
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        middleTransfer.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    // ── Shooter PIDF update (called every loop) ────────────────────────────
    private void updateShooter() {
        double voltage  = voltageSensor.getVoltage();
        double powerL   = leftController.computePowerForTargetRPMWithVoltageCompensation(
                TARGET_RPM, shooterLeft.getVelocity(), voltage, NOMINAL_VOLTAGE);
        double powerR   = rightController.computePowerForTargetRPMWithVoltageCompensation(
                TARGET_RPM, shooterRight.getVelocity(), voltage, NOMINAL_VOLTAGE);
        shooterLeft.setPower(powerL);
        shooterRight.setPower(powerR);
    }

    // ── State Machine ──────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (state) {

            // STATE 0: Turn on shooter, immediately start driving to shoot pose
            case 0:
                // Shooter is already being driven by updateShooter() — nothing extra needed.
                // Begin path to shoot pose
                follower.followPath(toShoot, true);
                setState(1);
                break;

            // STATE 1: Wait for robot to reach shoot pose
            case 1:
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

            // STATE 2: 1s spinup hold — then open gate AND start transfer to feed balls
            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0); // run transfer to feed balls into shooter
                    setState(3);
                }
                break;

            // STATE 3: Gate open + transfer running — shoot until time elapses
            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0); // stop transfer when done shooting
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    leftController.reset();
                    rightController.reset();
                    follower.followPath(toPickup2, true);
                    setState(4);
                }
                break;

            // STATE 4: Wait for robot to reach pickupPose2
            case 4:
                if (!follower.isBusy()) {
                    setState(-1); // Done
                }
                break;

            default:
                // Idle / done — cut shooter and transfer power
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                middleTransfer.setPower(0);
                leftController.reset();
                rightController.reset();
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
        actionTimer.resetTimer();
    }
}