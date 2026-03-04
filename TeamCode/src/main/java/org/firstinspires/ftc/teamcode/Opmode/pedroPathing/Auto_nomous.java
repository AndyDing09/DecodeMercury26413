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
    private DcMotor middleTransfer;
    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor voltageSensor;
    private Servo Gate;
    private Servo transferBlocker;

    // =======================
    // Shooter Constants
    // =======================
    private static final double TICKS_PER_REV      = 28.0;
    private static final double NOMINAL_VOLTAGE    = 12.0;
    private static final double TARGET_RPM_INITIAL = 1550;
    private static final double TARGET_RPM_FUTURE  = 2025;

    private static final double kP_shooter = 0.006;
    private static final double kI_shooter = 0.0005;
    private static final double kD_shooter = 0.0002;
    private static final double kF_shooter = 0.00620;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    // Active RPM target — starts at initial, swapped to future after first shot
    private double activeTargetRPM = TARGET_RPM_INITIAL;

    // =======================
    // Gate Positions
    // =======================
    private static final double GATE_CLOSED = 0.5;
    private static final double GATE_OPEN   = 0.27;

    // =======================
    // Intake Constants
    // =======================
    private static final double SERVO_HOME           = 0.5;
    private static final double SERVO_EXTENDED       = 0.0;
    private static final double TRANSFER_RESET_DELAY = 0.35;

    // =======================
    // Poses
    // =======================
    private final Pose startPose    = new Pose(124, 124, Math.toRadians(45));
    private final Pose shootPose    = new Pose(96,  96,  Math.toRadians(45));
    private final Pose pickupPose2  = new Pose(100, 61.5,  Math.toRadians(0));
    private final Pose afterIntake1 = new Pose(132, 61.5,  Math.toRadians(0));

    // =======================
    // PedroPathing
    // =======================
    private Follower follower;
    private PathChain toShoot;
    private PathChain toPickup2;
    private PathChain toAfterIntake1;
    private PathChain toShoot2;

    // =======================
    // State Machine
    // =======================
    private final Timer actionTimer = new Timer();
    private int state = 0;

    private static final double SPINUP_TIME_1 = 0.375;
    private static final double SPINUP_TIME_2 = 0.85;
    private static final double SHOOT_TIME_1  = 0.5;
    private static final double SHOOT_TIME_2  = 1.0;

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

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gate = hardwareMap.servo.get("Gate");
        Gate.setPosition(GATE_CLOSED);

        transferBlocker = hardwareMap.servo.get("transferBlocker");
        transferBlocker.setPosition(SERVO_HOME);

        // ── PIDF Controllers ───────────────────────────────────────────────
        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);

        // ── Pedro Init ─────────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose2.getHeading())
                .build();

        // Intake drive: Pedro follows from pickupPose2 to afterIntake1 while intake runs
        toAfterIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, afterIntake1))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), afterIntake1.getHeading())
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(afterIntake1, shootPose))
                .setLinearHeadingInterpolation(afterIntake1.getHeading(), shootPose.getHeading())
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
            telemetry.addData("Target RPM", (int) activeTargetRPM);
            telemetry.addData("Battery", String.format("%.1f V", voltageSensor.getVoltage()));
            telemetry.update();
        }

        // Cleanup on stop
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        middleTransfer.setPower(0);
        Gate.setPosition(GATE_CLOSED);
    }

    // ── Shooter PIDF update — uses activeTargetRPM ─────────────────────────
    private void updateShooter() {
        double voltage = voltageSensor.getVoltage();
        double powerL  = leftController.computePowerForTargetRPMWithVoltageCompensation(
                activeTargetRPM, shooterLeft.getVelocity(), voltage, NOMINAL_VOLTAGE);
        double powerR  = rightController.computePowerForTargetRPMWithVoltageCompensation(
                activeTargetRPM, shooterRight.getVelocity(), voltage, NOMINAL_VOLTAGE);
        shooterLeft.setPower(powerL);
        shooterRight.setPower(powerR);
    }

    // ── State Machine ──────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (state) {

            // ── FIRST SHOOT CYCLE (TARGET_RPM_INITIAL) ────────────────────

            // STATE 0: Start shooter at initial RPM, begin driving to shoot pose
            case 0:
                activeTargetRPM = TARGET_RPM_INITIAL;
                follower.followPath(toShoot, true);
                setState(1);
                break;

            // STATE 1: Wait to arrive at shoot pose
            case 1:
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

            // STATE 2: Spinup hold, then open gate and start transfer
            case 2:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_1) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(3);
                }
                break;

            // STATE 3: Shoot, then close gate and drive to pickup
            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_1) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0);
                    leftController.reset();
                    rightController.reset();
                    follower.followPath(toPickup2, true);
                    setState(4);
                }
                break;

            // ── RAPID INTAKE ──────────────────────────────────────────────

            // STATE 4: Wait to arrive at pickupPose2
            case 4:
                if (!follower.isBusy()) {
                    transferBlocker.setPosition(SERVO_HOME);
                    setState(5);
                }
                break;

            // STATE 5: Brief pause after resetting transfer servo
            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= TRANSFER_RESET_DELAY) {
                    // Start intake, swap to future RPM, then let Pedro drive to afterIntake1
                    middleTransfer.setPower(1.0);
                    activeTargetRPM = TARGET_RPM_FUTURE;
                    follower.followPath(toAfterIntake1, true);
                    setState(6);
                }
                break;

            // STATE 6: Wait to arrive at afterIntake1 (intake runs the whole time)
            case 6:
                if (!follower.isBusy()) {
                    middleTransfer.setPower(0);
                    follower.followPath(toShoot2, true);
                    setState(7);
                }
                break;

            // ── SECOND SHOOT CYCLE (TARGET_RPM_FUTURE) ────────────────────

            // STATE 7: Wait to arrive back at shoot pose
            case 7:
                if (!follower.isBusy()) {
                    setState(8);
                }
                break;

            // STATE 8: Spinup hold, then open gate and start transfer
            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= SPINUP_TIME_2) {
                    Gate.setPosition(GATE_OPEN);
                    middleTransfer.setPower(1.0);
                    setState(9);
                }
                break;

            // STATE 9: Shoot, then shut everything down
            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= SHOOT_TIME_2) {
                    Gate.setPosition(GATE_CLOSED);
                    middleTransfer.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    leftController.reset();
                    rightController.reset();
                    setState(-1);
                }
                break;

            default:
                // Idle / done
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
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