package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Velocity Control Test", group="Testing")
public class VelocityControlTest extends LinearOpMode {

    // GoBilda motor
    private DcMotorEx shooterMotor;

    // RPM control
    private double targetRPM = 1000;      // start speed
    private final double RPM_INCREMENT = 50; // change step
    private final double MAX_RPM = 6000;  // depends on motor type
    private final double MIN_RPM = 0;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Velocity Control Initialized");
        telemetry.addLine("Use ▲/Δ (triangle) to increase RPM, X (square) to decrease RPM");
        telemetry.addLine("Press ▶ (circle) to stop motor");
        telemetry.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            // Adjust RPM using PS5 buttons
            if (gamepad1.triangle) {
                targetRPM += RPM_INCREMENT;
                sleep(200); // debounce
            } else if (gamepad1.square) {
                targetRPM -= RPM_INCREMENT;
                sleep(200);
            }

            // Clamp RPM
            targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));

            // Convert RPM to ticks per second (GoBilda 312RPM motor = 537.7 ticks/rev)
            double ticksPerSec = (targetRPM * 537.7) / 60.0;

            // Set velocity
            shooterMotor.setVelocity(ticksPerSec);

            // Stop motor on Circle
            if (gamepad1.circle) {
                shooterMotor.setPower(0);
                targetRPM = 0;
            }

            // Telemetry output
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual Velocity (ticks/s)", shooterMotor.getVelocity());
            telemetry.addData("Actual RPM", (shooterMotor.getVelocity() * 60) / 537.7);
            telemetry.addData("Elapsed Time", "%.2f s", timer.seconds());
            telemetry.update();
        }
    }

    @TeleOp(name="TeleOp1", group="TeleOp")
    public static class TeleOp1 extends LinearOpMode {

        // ================= DRIVE =================
        private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

        // ================= MECHANISMS =================
        private DcMotor backwheelTransfer, middleTransfer;
        private DcMotorEx shooterMotor, frontIntake;

        private Servo Transfer;
        private Servo TurnTable2;
        private Servo transferBlocker;

        // ================= LIMELIGHT =================
        private Limelight3A limelight;

        // ================= TURRET CONTROL (CR pseudo-PD) =================
        private static final double SERVO_STOP = 0.5;  // CR servo neutral
        private double kP = 0.006;  // proportional gain
        private double kD = 0.001;  // derivative gain
        private double DEADZONE_DEG = 0.5;

        private double lastTx = 0;
        private double lastTime = 0;

        // Step sizes for tuning
        private final double[] stepSizes = {0.0001, 0.0005, 0.001, 0.005, 0.01};
        private int stepIndex = 2;  // default starting step size

        // Edge detection for tuning
        private boolean lastDpadUp = false;
        private boolean lastDpadDown = false;
        private boolean lastDpadRight = false;
        private boolean lastDpadLeft = false;
        private boolean lastSquare = false;

        // Servo direction fix
        private int servoDirection = 1; // 1 = normal, -1 = reversed

        // ================= SHOOTER =================
        private boolean shooterOn = false;
        private boolean lastLeftBumper = false;
        private boolean lastRightBumper = false;
        private boolean lastTriangle = false;

        private double targetRPM = 0;
        private final double fastRPM = 3350;
        private final double slowRPM = 2900;
        private static final double MAX_RPM = 6000;
        private static final double MIN_RPM = 0;
        private static final double TICKS_PER_REV = 28.0;

        // ================= SERVO POSITIONS =================
        private final double servoHome = 0.075;
        private final double servoExtendedPos = 0;
        private final double ServoStart = 0.5;

        @Override
        public void runOpMode() {

            // ================= HARDWARE MAP =================
            backwheelTransfer = hardwareMap.dcMotor.get("backwheelTransfer");
            middleTransfer = hardwareMap.dcMotor.get("middleTransfer");
            Transfer = hardwareMap.servo.get("Transfer");
            transferBlocker = hardwareMap.servo.get("transferBlocker");
            TurnTable2 = hardwareMap.servo.get("TurnTable");

            shooterMotor  = hardwareMap.get(DcMotorEx.class, "Shooter");
            frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");

            frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
            backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            // ================= DIRECTIONS =================
            middleTransfer.setDirection(DcMotor.Direction.FORWARD);
            frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            backwheelTransfer.setDirection(DcMotor.Direction.REVERSE);

            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);

            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // ================= LIMELIGHT SETUP =================
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();

            // ================= INIT POSITIONS =================
            Transfer.setPosition(servoHome);
            transferBlocker.setPosition(ServoStart);
            TurnTable2.setPosition(SERVO_STOP);

            telemetry.addLine("TeleOp + Limelight READY");
            telemetry.addLine("Dpad U/D: P +/- | Dpad L/R: D +/- | Square: step size");
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

                // ================= LIMELIGHT + TURNTABLE (PD) =================
                double servoCmd = SERVO_STOP;
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double error = servoDirection * -tx;

                    double dError = (error - lastTx) / dt;
                    lastTx = error;

                    if (Math.abs(error) > DEADZONE_DEG) {
                        servoCmd = SERVO_STOP + kP * error + kD * dError;
                        servoCmd = clamp(servoCmd, 0.0, 1.0);
                    } else {
                        servoCmd = SERVO_STOP;
                    }

                    telemetry.addData("Target", "YES");
                    telemetry.addData("tx", "%.2f", tx);

                } else {
                    servoCmd = SERVO_STOP;
                    lastTx = 0;
                    telemetry.addData("Target", "NO");
                }

                TurnTable2.setPosition(servoCmd);

                // ================= CONTROLLER TUNING =================
                if (gamepad1.dpad_up && !lastDpadUp)    kP += stepSizes[stepIndex];
                if (gamepad1.dpad_down && !lastDpadDown)  kP = Math.max(0, kP - stepSizes[stepIndex]);
                if (gamepad1.dpad_right && !lastDpadRight) kD += stepSizes[stepIndex];
                if (gamepad1.dpad_left && !lastDpadLeft)  kD = Math.max(0, kD - stepSizes[stepIndex]);

                if (gamepad1.square && !lastSquare) {
                    stepIndex = (stepIndex + 1) % stepSizes.length;
                    sleep(200); // debounce
                }

                lastDpadUp = gamepad1.dpad_up;
                lastDpadDown = gamepad1.dpad_down;
                lastDpadRight = gamepad1.dpad_right;
                lastDpadLeft = gamepad1.dpad_left;
                lastSquare = gamepad1.square;

                // ================= INTAKE =================
                if (gamepad1.circle) {
                    frontIntake.setPower(-1);
                    middleTransfer.setPower(-1);
                } else {
                    frontIntake.setPower(0);
                    middleTransfer.setPower(0);
                }

                if (gamepad1.cross) {
                    backwheelTransfer.setPower(-1);
                } else {
                    backwheelTransfer.setPower(0);
                }

                // ================= TRANSFER =================
                if (gamepad1.dpad_up) {
                    Transfer.setPosition(servoHome);
                } else if (gamepad1.dpad_down) {
                    Transfer.setPosition(servoExtendedPos);
                }

                // ================= SHOOTER =================
                boolean lb = gamepad1.left_bumper;
                boolean rb = gamepad1.right_bumper;
                boolean tri = gamepad1.triangle;

                if (lb && !lastLeftBumper) {
                    shooterOn = true;
                    targetRPM = fastRPM;
                }
                if (rb && !lastRightBumper) {
                    shooterOn = true;
                    targetRPM = slowRPM;
                }
                if (tri && !lastTriangle) {
                    shooterOn = false;
                    targetRPM = 0;
                }

                lastLeftBumper = lb;
                lastRightBumper = rb;
                lastTriangle = tri;

                targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));
                double ticksPerSec = shooterOn ? (targetRPM * TICKS_PER_REV) / 60.0 : 0;
                shooterMotor.setVelocity(ticksPerSec);

                // ================= TELEMETRY =================
                telemetry.addData("Servo Cmd", servoCmd);
                telemetry.addData("kP (Dpad U/D)", kP);
                telemetry.addData("kD (Dpad L/R)", kD);
                telemetry.addData("Step (Square)", stepSizes[stepIndex]);
                telemetry.addData("Shooter RPM", shooterMotor.getVelocity() * 60 / TICKS_PER_REV);
                telemetry.update();
            }
        }

        // ================= UTILITY =================
        private static double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }
    }
}
