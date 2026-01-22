package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name="TeleOp2", group="TeleOp")
public class TeleOp2 extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // ================= MECHANISMS =================
    private DcMotor middleTransfer;
    private DcMotorEx shooterMotor, shooterMotor2, frontIntake;

    private Servo Intakelift;
    private Servo TurnTable2;
    private Servo transferBlocker;

    // ================= LIMELIGHT =================
    private Limelight3A limelight;

    // ================= TURRET CONTROL =================
    private static final double SERVO_STOP = 0.5;
    private double kP = 0.006;
    private double kD = 0.001;
    private double DEADZONE_DEG = 0.5;

    private double lastTx = 0;
    private double lastTime = 0;

    private final double[] stepSizes = {0.0001, 0.0005, 0.001, 0.005, 0.01};
    private int stepIndex = 2;

    private boolean lastDpadUp, lastDpadDown, lastDpadRight, lastDpadLeft, lastSquare;
    private int servoDirection = 1;

    // ================= SHOOTER =================
    private boolean shooterOn = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private final double fastRPM = 3350;
    private final double slowRPM = 2900;

    private static final double TICKS_PER_REV = 28.0;

    // ================= SERVO POSITIONS =================
    private final double ServoStart = 0.5;
    private final double ServoUp = 0.4;
    private final double ServoDown = 0.0;

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        middleTransfer = hardwareMap.dcMotor.get("middleTransfer");

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterMotor  = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter1");

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");

        Intakelift = hardwareMap.servo.get("Intakelift");
        TurnTable2 = hardwareMap.servo.get("TurnTable");
        transferBlocker = hardwareMap.servo.get("transferBlocker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ================= DIRECTIONS =================
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter setup
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================= LIMELIGHT =================
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ================= INIT POSITIONS =================
        Intakelift.setPosition(ServoDown);
        transferBlocker.setPosition(ServoStart);
        TurnTable2.setPosition(SERVO_STOP);

        telemetry.addLine("TeleOp2 READY – Dual Shooter Enabled");
        telemetry.update();

        waitForStart();
        lastTime = getRuntime();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            double now = getRuntime();
            double dt = Math.max(now - lastTime, 0.01);
            lastTime = now;

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
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            // ================= LIMELIGHT PD =================
            double servoCmd = SERVO_STOP;
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double error = servoDirection * -result.getTx();
                double dError = (error - lastTx) / dt;
                lastTx = error;

                if (Math.abs(error) > DEADZONE_DEG) {
                    servoCmd = clamp(SERVO_STOP + kP * error + kD * dError, 0, 1);
                }
            } else {
                lastTx = 0;
            }

            TurnTable2.setPosition(servoCmd);

            // ================= INTAKE =================
            if (gamepad1.circle) {
                frontIntake.setPower(-1);
                middleTransfer.setPower(-1);
            } else {
                frontIntake.setPower(0);
                middleTransfer.setPower(0);
            }


            // ================= INTAKE LIFT =================
            if (gamepad1.dpad_up) Intakelift.setPosition(ServoUp);
            if (gamepad1.dpad_down) Intakelift.setPosition(ServoDown);

            // ================= SHOOTER =================
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean tri = gamepad1.triangle;

            if (lb && !lastLeftBumper) { shooterOn = true; targetRPM = fastRPM; }
            if (rb && !lastRightBumper) { shooterOn = true; targetRPM = slowRPM; }
            if (tri && !lastTriangle)   { shooterOn = false; targetRPM = 0; }

            lastLeftBumper = lb;
            lastRightBumper = rb;
            lastTriangle = tri;

            double ticksPerSec = shooterOn
                    ? (targetRPM * TICKS_PER_REV) / 60.0
                    : 0;

            shooterMotor.setVelocity(ticksPerSec);
            shooterMotor2.setVelocity(ticksPerSec);

            // ================= TELEMETRY =================
            telemetry.addData("Shooter1 RPM", shooterMotor.getVelocity() * 60 / TICKS_PER_REV);
            telemetry.addData("Shooter2 RPM", shooterMotor2.getVelocity() * 60 / TICKS_PER_REV);
            telemetry.update();
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
