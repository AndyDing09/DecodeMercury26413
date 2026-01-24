package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="TeleOp2", group="TeleOp")
public class TeleOp2 extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // ================= MECHANISMS =================
    private DcMotor middleTransfer;
    private DcMotorEx shooterMotor, shooterMotor2, frontIntake;
    private VoltageSensor voltageSensor;

    private Servo Intakelift;
    private Servo TurnTable2, turntable3, transferBlocker, Gate;

    // ================= INTAKE =================
    private boolean intakeOn = false;
    private boolean lastCircle = false;

    // ================= SHOOTER =================
    private boolean shooterOn = false;
    private boolean shooterKilled = false;

    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private final double fastMaxRPM = 2500;
    private final double slowMaxRPM = 700;

    private double fastStartTime = -1;
    private double slowStartTime = -1;

    private static final double TICKS_PER_REV = 28.0;

    enum ShooterMode {
        OFF,
        FAST,
        SLOW
    }

    private ShooterMode shooterMode = ShooterMode.OFF;

    // ================= PIDF =================
    private double P = 200;
    private double I = 0;
    private double D = 3;
    private double F = 15;

    // ================= SERVO POSITIONS =================
    private final double ServoStart = 0.5;
    private final double ServoDown = 0.0;

    // ================= TURRET =================
    private double turretPos = 0.5;

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterMotor  = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        frontIntake   = hardwareMap.get(DcMotorEx.class, "frontIntake");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Intakelift = hardwareMap.servo.get("Intakelift");
        TurnTable2 = hardwareMap.servo.get("TurnTable");
        turntable3 = hardwareMap.servo.get("turntable3");
        transferBlocker = hardwareMap.servo.get("transferBlocker");
        Gate = hardwareMap.servo.get("Gate");

        // ================= DIRECTIONS =================
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ================= INIT =================
        Intakelift.setPosition(ServoDown);
        transferBlocker.setPosition(ServoStart);
        Gate.setPosition(0.23);

        TurnTable2.setPosition(1 - turretPos);
        turntable3.setPosition(turretPos);

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            // ================= INTAKE =================
            if (gamepad1.square) {
                frontIntake.setPower(-1);
                middleTransfer.setPower(1);
                Intakelift.setPosition(ServoStart);
            } else {
                if (gamepad1.circle && !lastCircle) intakeOn = !intakeOn;
                lastCircle = gamepad1.circle;

                if (intakeOn) {
                    frontIntake.setPower(-1);
                    middleTransfer.setPower(-1);
                    Intakelift.setPosition(ServoDown);
                } else {
                    frontIntake.setPower(0);
                    middleTransfer.setPower(0);
                }
            }

            // ================= SHOOTER INPUTS =================
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean tri = gamepad1.triangle;

            // === HARD KILL ===
            if (tri && !lastTriangle) {
                shooterOn = false;
                shooterKilled = true;
                shooterMode = ShooterMode.OFF;
                targetRPM = 0;
                fastStartTime = -1;
                slowStartTime = -1;
                Gate.setPosition(0.23);
            }

            // FAST SHOOTER
            if (lb && !lastLeftBumper) {
                shooterOn = true;
                shooterKilled = false;
                shooterMode = ShooterMode.FAST;
                fastStartTime = getRuntime();
                Gate.setPosition(0.09);
            }

            // SLOW SHOOTER
            if (rb && !lastRightBumper) {
                shooterOn = true;
                shooterKilled = false;
                shooterMode = ShooterMode.SLOW;
                slowStartTime = getRuntime();
                Gate.setPosition(0.09);
            }

            lastLeftBumper = lb;
            lastRightBumper = rb;
            lastTriangle = tri;

            // ================= RAMP LOGIC =================
            if (shooterOn && !shooterKilled) {

                if (shooterMode == ShooterMode.FAST) {
                    double t = getRuntime() - fastStartTime;

                    if (t < 2) {
                        targetRPM = 1500;
                    } else if (t < 8) {
                        targetRPM = 900 + t * 300;
                    } else {
                        targetRPM = 0;
                        Gate.setPosition(0.23);
                    }
                }

                if (shooterMode == ShooterMode.SLOW) {
                    double t = getRuntime() - slowStartTime;

                    if (t < 2) {
                        targetRPM = slowMaxRPM;
                    } else if (t < 8) {
                        targetRPM = 600 + t * 50; // linear ramp 2->8 sec
                    } else {
                        targetRPM = 0;
                        Gate.setPosition(0.23);
                    }
                }
            }

            // ================= PIDF =================
            PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            // ================= VELOCITY =================
            double ticksPerSec = 0;
            if (shooterOn && !shooterKilled) {
                ticksPerSec = targetRPM * TICKS_PER_REV / 60.0;
            }

            shooterMotor.setVelocity(ticksPerSec);
            shooterMotor2.setVelocity(ticksPerSec);
        }
    }
}
