package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp1")
public class TeleOp1 extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor Intake1;
    private DcMotor Intake2;

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor1;

    private Servo Transfer;
    private Servo TurnTable2;
    private Servo transferBlocker;
    // ---------- Intake toggle ----------
    private boolean intakeOn = false;
    private boolean lastCircle = false;

    // ---------- Shooter toggle + preset ----------
    private boolean shooterOn = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastTriangle = false; // optional off button

    // ---------- Servo positions ----------
    private final double servoHome = 0.075;
    private final double servoExtendedPos = 0.0;
    private final double ServoStart = 0.5;

    private final double turntableStart = 0.5;

    // ---------- Turntable increment ----------
    private double turntablePos = turntableStart;
    private static final double TURN_STEP = 0.02;
    private static final double TURN_MIN = 0.0;
    private static final double TURN_MAX = 1.0;
    private boolean lastLeftBumper2 = false;
    private boolean lastRightBumper2 = false;

    // ---------- Shooter speed ----------
    private double targetRPM = 0;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;
    private final double fastRPM = 3350;
    private final double slowRPM = 2900;

    // goBILDA 6000RPM Yellow Jacket (1:1) encoder: 28 ticks per output revolution
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {

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

        // Directions
        Intake2.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If your 2 shooter motors are mirrored mechanically, you may need to reverse one
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD); // change if needed

        // Init servo positions
        Transfer.setPosition(servoHome);
        TurnTable2.setPosition(turntableStart);
        transferBlocker.setPosition(ServoStart);
        waitForStart();

        while (opModeIsActive()) {

            // =======================
            // DRIVING (unchanged logic)
            // =======================
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 0.7) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // =======================
            // TURNTABLE (gamepad2 bumpers)
            // =======================
            boolean lBump2 = gamepad2.left_bumper;
            boolean rBump2 = gamepad2.right_bumper;

            if (lBump2 && !lastLeftBumper2) {
                turntablePos = Math.min(turntablePos + TURN_STEP, TURN_MAX);
                TurnTable2.setPosition(turntablePos);
            }
            if (rBump2 && !lastRightBumper2) {
                turntablePos = Math.max(turntablePos - TURN_STEP, TURN_MIN);
                TurnTable2.setPosition(turntablePos);
            }

            lastLeftBumper2 = lBump2;
            lastRightBumper2 = rBump2;

            // =======================
            // INTAKE TOGGLE (gamepad1 circle)
            // =======================
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle) {
                intakeOn = !intakeOn;
            }
            lastCircle = circle;

            // Reverse intake while holding X (cross). Otherwise follow intakeOn.
            if (gamepad1.cross) {
                Intake1.setPower(-1.0);
                Intake2.setPower(-1.0);
            } else if (intakeOn) {
                Intake1.setPower(1.0);
                Intake2.setPower(1.0);
            } else {
                Intake1.setPower(0.0);
                Intake2.setPower(0.0);
            }

            // =======================
            // TRANSFER SERVO MANUAL (dpad up/down)
            // =======================
            if (gamepad1.dpad_up) {
                Transfer.setPosition(servoHome);
            } else if (gamepad1.dpad_down) {
                Transfer.setPosition(servoExtendedPos);
            }

            // =======================
            // TRANSFER MACRO (square)
            // =======================
            if (gamepad1.square) {
                boolean wasIntakeOn = intakeOn;

                // Stop intake immediately (regardless of toggle)
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
                // Restore intake power only if it was on AND user isn't holding cross to reverse
                if (wasIntakeOn && !gamepad1.cross) {
                    Intake1.setPower(1);
                    Intake2.setPower(1);
                }

                sleep(250); // keep this macro debounce (only runs when square is pressed)
            }
            // Servo that blocks balls from coming in using button
            //Use the dpad up and down from the second gamepad
            if (gamepad2.dpad_up) {
                transferBlocker.setPosition(ServoStart);
            } else if (gamepad2.dpad_down) {
                transferBlocker.setPosition(0.65);
            }
            // =======================
            // SHOOTER CONTROL (gamepad1 bumpers)
            // - tap LB: turn ON + set FAST
            // - tap RB: turn ON + set SLOW
            // - tap TRIANGLE: turn OFF (optional)
            // =======================
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean triangle = gamepad1.triangle;

            if (leftBumper && !lastLeftBumper) {
                shooterOn = true;
                targetRPM = fastRPM;
            }
            if (rightBumper && !lastRightBumper) {
                shooterOn = true;
                targetRPM = slowRPM;
            }
            if (triangle && !lastTriangle) {
                shooterOn = false;
                targetRPM = 0;
            }

            lastLeftBumper = leftBumper;
            lastRightBumper = rightBumper;
            lastTriangle = triangle;

            // Clamp RPM
            targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));

            // RPM -> ticks/sec for setVelocity()
            double cmdTicksPerSec = shooterOn ? (targetRPM * TICKS_PER_REV) / 60.0 : 0.0;

            shooterMotor.setVelocity(cmdTicksPerSec);
            shooterMotor1.setVelocity(cmdTicksPerSec);

            // =======================
            // TELEMETRY (correct RPM)
            // =======================
            double actualRPM0 = shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
            double actualRPM1 = shooterMotor1.getVelocity() * 60.0 / TICKS_PER_REV;

            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Transfer Pos", Transfer.getPosition());
            telemetry.addData("Turntable Pos", turntablePos);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Cmd Vel (ticks/s)", cmdTicksPerSec);

            telemetry.addData("Shooter RPM", actualRPM0);
            telemetry.addData("Shooter1 RPM", actualRPM1);

            telemetry.update();
        }
    }
}