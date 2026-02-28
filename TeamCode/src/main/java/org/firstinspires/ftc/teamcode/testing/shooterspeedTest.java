package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="Dual Shooter with PIDF Tuner", group="Testing")
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

    // ================= PIDF VARIABLES =================
    // Starting with the values from your original code
    private double p = 200.0;
    private double i = 0.0;
    private double d = 3.0;
    private double f = 15.0;

    // Tuning selection state
    private enum TuneState { P, I, D, F }
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

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("✅ Dual Shooter & Tuner Initialized");
        telemetry.addLine("--- GAMEPAD 1 (SPEED) ---");
        telemetry.addLine("D-Pad Up/Down: +/- 100 RPM");
        telemetry.addLine("Bumpers: +/- 500 RPM");
        telemetry.addLine("A: 0 | X: 1000 | Y: 2000 | B: 2500");
        telemetry.addLine("--- GAMEPAD 2 (PIDF) ---");
        telemetry.addLine("D-Pad Left/Right: Select P, I, D, or F");
        telemetry.addLine("D-Pad Up/Down: Increase/Decrease selected value");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: SPEED CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;
            boolean currentG1RightBumper = gamepad1.right_bumper;
            boolean currentG1LeftBumper = gamepad1.left_bumper;

            if (currentG1DpadUp && !lastG1DpadUp) targetRPM += 100;
            if (currentG1DpadDown && !lastG1DpadDown) targetRPM -= 100;
            if (currentG1RightBumper && !lastG1RightBumper) targetRPM += 500;
            if (currentG1LeftBumper && !lastG1LeftBumper) targetRPM -= 500;

            if (gamepad1.a) targetRPM = 0;
            if (gamepad1.x) targetRPM = 1000;
            if (gamepad1.y) targetRPM = 2000;
            if (gamepad1.b) targetRPM = 2500;

            if (targetRPM < 0) targetRPM = 0;

            lastG1DpadUp = currentG1DpadUp;
            lastG1DpadDown = currentG1DpadDown;
            lastG1RightBumper = currentG1RightBumper;
            lastG1LeftBumper = currentG1LeftBumper;

            // ================= GAMEPAD 2: PIDF TUNING =================
            boolean currentG2DpadUp = gamepad2.dpad_up;
            boolean currentG2DpadDown = gamepad2.dpad_down;
            boolean currentG2DpadLeft = gamepad2.dpad_left;
            boolean currentG2DpadRight = gamepad2.dpad_right;

            // Cycle through P, I, D, F
            if (currentG2DpadRight && !lastG2DpadRight) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.F;
                else if (currentSelected == TuneState.F) currentSelected = TuneState.P;
            } else if (currentG2DpadLeft && !lastG2DpadLeft) {
                if (currentSelected == TuneState.P) currentSelected = TuneState.F;
                else if (currentSelected == TuneState.F) currentSelected = TuneState.D;
                else if (currentSelected == TuneState.D) currentSelected = TuneState.I;
                else if (currentSelected == TuneState.I) currentSelected = TuneState.P;
            }

            // Adjust the selected value (Holding Left Bumper makes it change by smaller increments)
            double increment = gamepad2.left_bumper ? 0.1 : 1.0;

            if (currentG2DpadUp && !lastG2DpadUp) {
                switch(currentSelected) {
                    case P: p += increment; break;
                    case I: i += increment; break;
                    case D: d += increment; break;
                    case F: f += increment; break;
                }
            } else if (currentG2DpadDown && !lastG2DpadDown) {
                switch(currentSelected) {
                    case P: p -= increment; break;
                    case I: i -= increment; break;
                    case D: d -= increment; break;
                    case F: f -= increment; break;
                }
            }

            lastG2DpadUp = currentG2DpadUp;
            lastG2DpadDown = currentG2DpadDown;
            lastG2DpadLeft = currentG2DpadLeft;
            lastG2DpadRight = currentG2DpadRight;

            // Prevent negative PIDF values
            if (p < 0) p = 0;
            if (i < 0) i = 0;
            if (d < 0) d = 0;
            if (f < 0) f = 0;

            // ================= APPLY PIDF & VELOCITY =================
            PIDFCoefficients currentPIDF = new PIDFCoefficients(p, i, d, f);

            shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentPIDF);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentPIDF);

            double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
            shooterLeft.setVelocity(ticksPerSec);
            shooterRight.setVelocity(ticksPerSec);

            // ================= TELEMETRY =================
            telemetry.addData("TARGET RPM", targetRPM);
            telemetry.addLine();

            telemetry.addLine("--- PIDF TUNING ---");
            telemetry.addData("Selected", "-> " + currentSelected.name() + " <-");
            telemetry.addData("P", "%.1f %s", p, currentSelected == TuneState.P ? "<--" : "");
            telemetry.addData("I", "%.1f %s", i, currentSelected == TuneState.I ? "<--" : "");
            telemetry.addData("D", "%.1f %s", d, currentSelected == TuneState.D ? "<--" : "");
            telemetry.addData("F", "%.1f %s", f, currentSelected == TuneState.F ? "<--" : "");
            telemetry.addLine("(Hold G2 Left Bumper for 0.1 increments)");
            telemetry.addLine();

            telemetry.addLine("--- MOTOR PERFORMANCE ---");
            double actualTicksLeft = shooterLeft.getVelocity();
            double actualTicksRight = shooterRight.getVelocity();

            telemetry.addData("Left Actual RPM", (actualTicksLeft * 60.0) / TICKS_PER_REV);
            telemetry.addData("Right Actual RPM", (actualTicksRight * 60.0) / TICKS_PER_REV);

            telemetry.update();
        }
    }
}