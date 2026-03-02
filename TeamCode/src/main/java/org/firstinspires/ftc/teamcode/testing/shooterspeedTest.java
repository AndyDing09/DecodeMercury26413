package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// ElapsedTime no longer required; PIDFMotorController has its own timer

@TeleOp(name="Dual Shooter with Custom PIDF", group="Testing")
public class shooterspeedTest extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // ================= SPEED VARIABLES =================
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;

    // ================= CUSTOM PIDF VARIABLES =================
    // IMPORTANT: Custom PIDF outputs motor power (0.0 to 1.0).
    // These values will be much smaller than the built-in REV hub values!
    private double kP = 0.0064;  // Gain has been tuned up with hardware.
    private double kI = 0.00001;
    private double kD = 0.0000;
    
    // Split Feedforward for asymmetric tuning
    private double kF_Left = 0.0007; // Gain has been tuned up with hardware.
    private double kF_Right = 0.0002; 

    // Controllers (one per motor)
    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    // Tuning selection state - Now includes independent F values
    private enum TuneState { P, I, D, F_LEFT, F_RIGHT }
    private TuneState currentSelected = TuneState.P;

    // Gamepad 2 Edge Detection
    private boolean lastG2DpadUp = false, lastG2DpadDown = false;
    private boolean lastG2DpadLeft = false, lastG2DpadRight = false;

    @Override
    public void runOpMode() {
        // HARDWARE MAP
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // We use RUN_WITHOUT_ENCODER so we can feed it raw power from our custom PIDF math,
        // but the encoders will STILL track velocity for us to read.
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("✅ Custom PIDF Shooter Initialized");
        telemetry.addLine("--- GAMEPAD 1 (SPEED) ---");
        telemetry.addLine("D-Pad Up/Down: +/- 100 RPM");
        telemetry.addLine("A: 0 | X: 1000 | Y: 2000 | B: 2500");
        telemetry.addLine("--- GAMEPAD 2 (TUNING) ---");
        telemetry.addLine("D-Pad L/R: Select P, I, D, F_Left, or F_Right");
        telemetry.addLine("D-Pad U/D: Adjust value");
        telemetry.update();

    waitForStart();

    // instantiate controllers using tuned gains and independent feed-forwards
    leftController = new PIDFMotorController(kP, kI, kD, kF_Left, TICKS_PER_REV);
    rightController = new PIDFMotorController(kP, kI, kD, kF_Right, TICKS_PER_REV);

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: SPEED CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;

            if (currentG1DpadUp && !lastG1DpadUp) targetRPM += 100;
            if (currentG1DpadDown && !lastG1DpadDown) targetRPM -= 100;

            if (gamepad1.a) targetRPM = 0;
            if (gamepad1.x) targetRPM = 1000;
            if (gamepad1.y) targetRPM = 2000;
            if (gamepad1.b) targetRPM = 2500;

            if (targetRPM < 0) targetRPM = 0;

            // Reset controllers if we shut off the motors to prevent wind-up
            if (targetRPM == 0) {
                if (leftController != null) leftController.reset();
                if (rightController != null) rightController.reset();
            }

            lastG1DpadUp = currentG1DpadUp;
            lastG1DpadDown = currentG1DpadDown;

            // ================= GAMEPAD 2: PIDF TUNING =================
            boolean currentG2DpadUp = gamepad2.dpad_up;
            boolean currentG2DpadDown = gamepad2.dpad_down;
            boolean currentG2DpadLeft = gamepad2.dpad_left;
            boolean currentG2DpadRight = gamepad2.dpad_right;

            // Cycle through P, I, D, F_LEFT, F_RIGHT
            if (currentG2DpadRight && !lastG2DpadRight) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.F_LEFT;
                else if (currentSelected == TuneState.F_LEFT) currentSelected = TuneState.F_RIGHT;
                else if (currentSelected == TuneState.F_RIGHT) currentSelected = TuneState.P;
            } else if (currentG2DpadLeft && !lastG2DpadLeft) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.F_RIGHT;
                else if (currentSelected == TuneState.F_RIGHT) currentSelected = TuneState.F_LEFT;
                else if (currentSelected == TuneState.F_LEFT) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.P;
            }

            // Custom increment sizes
            double increment;
            if (currentSelected == TuneState.I) {
                // Integral term needs to be 10x smaller to prevent instant saturation
                increment = gamepad2.left_bumper ? 0.00001 : 0.0001;
            } else {
                // Standard tuning steps for P, D, and F
                increment = gamepad2.left_bumper ? 0.0001 : 0.001;
            }

            if (currentG2DpadUp && !lastG2DpadUp) {
                switch(currentSelected) {
                    case P: kP += increment; break;
                    case I: kI += increment; break;
                    case D: kD += increment; break;
                    case F_LEFT: kF_Left += increment; break;
                    case F_RIGHT: kF_Right += increment; break;
                }
            } else if (currentG2DpadDown && !lastG2DpadDown) {
                switch(currentSelected) {
                    case P: kP -= increment; break;
                    case I: kI -= increment; break;
                    case D: kD -= increment; break;
                    case F_LEFT: kF_Left -= increment; break;
                    case F_RIGHT: kF_Right -= increment; break;
                }
            }

            lastG2DpadUp = currentG2DpadUp;
            lastG2DpadDown = currentG2DpadDown;
            lastG2DpadLeft = currentG2DpadLeft;
            lastG2DpadRight = currentG2DpadRight;

            // ================= CUSTOM PIDF (via PIDFMotorController) =================
            double powerLeft = 0.0;
            double powerRight = 0.0;

            if (targetRPM == 0) {
                shooterLeft.setPower(0.0);
                shooterRight.setPower(0.0);
            } else {
                // Apply live tunings (keeps controllers in sync with gamepad adjustments)
                if (leftController != null) leftController.setTunings(kP, kI, kD, kF_Left);
                if (rightController != null) rightController.setTunings(kP, kI, kD, kF_Right);

                powerLeft = leftController.applyToMotor(shooterLeft, targetRPM);
                powerRight = rightController.applyToMotor(shooterRight, targetRPM);
            }

            // ================= TELEMETRY =================
            double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.0;
            double currentVelocityLeft = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();

            telemetry.addData("TARGET RPM", targetRPM);
            telemetry.addData("Power (L/R)", "%.2f / %.2f", powerLeft, powerRight);
            telemetry.addLine();

            telemetry.addLine("--- PIDF TUNING ---");
            telemetry.addData("Selected", "-> " + currentSelected.name() + " <-");
            telemetry.addData("kP", "%.5f %s", kP, currentSelected == TuneState.P ? "<--" : "");
            telemetry.addData("kI", "%.5f %s", kI, currentSelected == TuneState.I ? "<--" : "");
            telemetry.addData("kD", "%.5f %s", kD, currentSelected == TuneState.D ? "<--" : "");
            telemetry.addData("kF_Left", "%.5f %s", kF_Left, currentSelected == TuneState.F_LEFT ? "<--" : "");
            telemetry.addData("kF_Right", "%.5f %s", kF_Right, currentSelected == TuneState.F_RIGHT ? "<--" : "");
            telemetry.addLine("(Hold G2 Left Bumper for finer increments. 'I' is 10x finer)");
            telemetry.addLine();

            telemetry.addLine("--- MOTOR PERFORMANCE ---");
            telemetry.addData("Target Ticks/Sec", targetVelocity);
            telemetry.addData("Left Actual RPM", (currentVelocityLeft * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", (currentVelocityRight * 60.0) / TICKS_PER_REV);

            telemetry.update();
        }
    }
}