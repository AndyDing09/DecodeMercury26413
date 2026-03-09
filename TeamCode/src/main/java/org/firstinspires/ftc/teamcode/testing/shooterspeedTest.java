package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config // <--- TELLS THE DASHBOARD TO LOOK AT THIS CLASS
@TeleOp(name="Dual Shooter with Custom PIDF", group="Testing")
public class shooterspeedTest extends LinearOpMode {

    // ================= MOTORS =================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotor middleTransfer;
    private VoltageSensor voltageSensor;

    // ================= SPEED VARIABLES =================
    // Changed to public static so it can be edited live on the dashboard
    public static double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // Gamepad 1 Edge Detection
    private boolean lastG1DpadUp = false, lastG1DpadDown = false;
    private boolean lastG1RightBumper = false, lastG1LeftBumper = false;
    private boolean lastCircle = false;

    // ================= INTAKE =================
    private boolean intakeOn = false;

    // ================= CUSTOM PIDF VARIABLES =================
    // IMPORTANT: Changed to public static. Now you can type these values
    // directly into the FTC Dashboard on your laptop!
    public static double kP_shooter = 0.0064;
    public static double kI_shooter = 0.00001;
    public static double kD_shooter = 0.0;
    public static double kF_shooter = 0.0007;

    // Controllers (one per motor)
    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    // Tuning selection state (for Gamepad 2)
    private enum TuneState { P, I, D, F }
    private TuneState currentSelected = TuneState.P;

    // Gamepad 2 Edge Detection
    private boolean lastG2DpadUp = false, lastG2DpadDown = false;
    private boolean lastG2DpadLeft = false, lastG2DpadRight = false;

    @Override
    public void runOpMode() {

        // --- FTC DASHBOARD SETUP ---
        // Route all telemetry to both the Driver Station AND the web browser
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // HARDWARE MAP
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("✅ Custom PIDF Shooter w/ Dashboard Initialized");
        telemetry.addLine("Connect to: 192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        // instantiate controllers using tuned gains
        leftController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);

        while (opModeIsActive()) {

            // ================= GAMEPAD 1: SPEED CONTROL =================
            boolean currentG1DpadUp = gamepad1.dpad_up;
            boolean currentG1DpadDown = gamepad1.dpad_down;

            // Still works! Will update the dashboard number automatically
            if (currentG1DpadUp && !lastG1DpadUp) targetRPM += 100;
            if (currentG1DpadDown && !lastG1DpadDown) targetRPM -= 100;

            if (gamepad1.a) targetRPM = 0;
            if (gamepad1.x) targetRPM = 1000;
            if (gamepad1.y) targetRPM = 2000;
            if (gamepad1.b) targetRPM = 2500;

            if (targetRPM < 0) targetRPM = 0;

            // Reset controllers if we shut off the motors
            if (targetRPM == 0) {
                if (leftController != null) leftController.reset();
                if (rightController != null) rightController.reset();
            }

            lastG1DpadUp = currentG1DpadUp;
            lastG1DpadDown = currentG1DpadDown;

            // ================= INTAKE =================
            if (gamepad1.circle && !lastCircle) intakeOn = !intakeOn;
            lastCircle = gamepad1.circle;

            if (intakeOn) {
                middleTransfer.setPower(1);
            } else {
                middleTransfer.setPower(0);
            }

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

            double increment = (currentSelected == TuneState.I) ?
                    (gamepad2.left_bumper ? 0.00001 : 0.0001) :
                    (gamepad2.left_bumper ? 0.0001 : 0.001);

            if (currentG2DpadUp && !lastG2DpadUp) {
                switch(currentSelected) {
                    case P: kP_shooter += increment; break;
                    case I: kI_shooter += increment; break;
                    case D: kD_shooter += increment; break;
                    case F: kF_shooter += increment; break;
                }
            } else if (currentG2DpadDown && !lastG2DpadDown) {
                switch(currentSelected) {
                    case P: kP_shooter -= increment; break;
                    case I: kI_shooter -= increment; break;
                    case D: kD_shooter -= increment; break;
                    case F: kF_shooter -= increment; break;
                }
            }

            lastG2DpadUp = currentG2DpadUp;
            lastG2DpadDown = currentG2DpadDown;
            lastG2DpadLeft = currentG2DpadLeft;
            lastG2DpadRight = currentG2DpadRight;

            // ================= CUSTOM PIDF =================
            double powerLeft = 0.0;
            double powerRight = 0.0;
            double currentVoltage = voltageSensor.getVoltage();

            if (targetRPM == 0) {
                shooterLeft.setPower(0.0);
                shooterRight.setPower(0.0);
            } else {
                // Apply live tunings (Pulls directly from Dashboard or Gamepad variables)
                if (leftController != null) leftController.setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);
                if (rightController != null) rightController.setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);

                powerLeft = leftController.computePowerForTargetRPMWithVoltageCompensation(
                        targetRPM, shooterLeft.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);
                powerRight = rightController.computePowerForTargetRPMWithVoltageCompensation(
                        targetRPM, shooterRight.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);

                shooterLeft.setPower(powerLeft);
                shooterRight.setPower(powerRight);
            }

            // ================= TELEMETRY / GRAPHING =================
            double currentVelocityLeft = shooterLeft.getVelocity();
            double currentVelocityRight = shooterRight.getVelocity();
            double rpmL = (currentVelocityLeft * 60.0) / TICKS_PER_REV;
            double rpmR = (currentVelocityRight * 60.0) / TICKS_PER_REV;

            // Data added here is automatically graphed on the Dashboard
            telemetry.addData("TARGET RPM", targetRPM);
            telemetry.addData("Left Actual RPM", rpmL);
            telemetry.addData("Right Actual RPM", rpmR);
            telemetry.addData("Power L", powerLeft);
            telemetry.addData("Power R", powerRight);
            telemetry.addData("Battery Voltage", currentVoltage);

            telemetry.addLine("\n--- PIDF TUNING ---");
            telemetry.addData("kP", kP_shooter);
            telemetry.addData("kI", kI_shooter);
            telemetry.addData("kD", kD_shooter);
            telemetry.addData("kF", kF_shooter);

            // Pushes to Driver Station and Browser
            telemetry.update();
        }
    }
}