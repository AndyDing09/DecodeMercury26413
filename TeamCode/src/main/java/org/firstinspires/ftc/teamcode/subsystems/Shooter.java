package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.PIDFMotorController;

public class Shooter {
    private final DcMotorEx shooterLeft, shooterRight;
    private final Servo gate;
    private final Servo hoodServo1, hoodServo2;
    private final Servo transferBlocker;

    public enum OuttakeState { IDLE, RAMPING, GATE_OPEN, TRANSFERRING }
    private OuttakeState outtakeState = OuttakeState.IDLE;
    private long outtakeStateStartTime = 0;
    private static final long RAMP_DELAY_MS = 2000;

    private static final double TICKS_PER_REV   = 28.0;
    private static final double NOMINAL_VOLTAGE  = 12.0;
    private static final double MAX_SHOOTER_RPM  = 4800.0;

    private static final double HOOD_SERVO_INIT = 0.5;
    private static final double MIN_HOOD_SERVO  = 0.0;
    private static final double MAX_HOOD_SERVO  = 1.0;
    private static final double HOOD_ANGLE_STEP = 0.05;

    private double currentHoodAnglePos = HOOD_SERVO_INIT;
    private boolean lastG1Y = false;
    private boolean lastG1X = false;

    private boolean shooterOn = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;
    private final double kP_shooter_low  = 0.0012, kI_shooter_low  = 0.0003, kD_shooter_low  = 0.00008, kF_shooter_low  = 0.00045;
    private final double kP_shooter_high = 0.0015, kI_shooter_high = 0.0004, kD_shooter_high = 0.00010, kF_shooter_high = 0.00045;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    private double lastShooterVelocity = 0;
    private static final double LAUNCH_DROP_THRESHOLD = 200.0;
    private static final double HOOD_DIP_AMOUNT = 0.05;

    public Shooter(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        gate         = hardwareMap.servo.get("Gate");
        transferBlocker = hardwareMap.servo.get("transferBlocker");
        hoodServo1   = hardwareMap.servo.get("angleChange1");
        hoodServo2   = hardwareMap.servo.get("angleChange2");

        shooterLeft .setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferBlocker.setPosition(0.5);
        gate.setPosition(0.5);
        updateHoodServos(currentHoodAnglePos);
    }

    public void initControllers() {
        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
    }

    public void handleHoodInput(boolean yPressed, boolean xPressed) {
        if (yPressed && !lastG1Y) currentHoodAnglePos = Math.min(MAX_HOOD_SERVO, currentHoodAnglePos + HOOD_ANGLE_STEP);
        if (xPressed && !lastG1X) currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_ANGLE_STEP);
        lastG1Y = yPressed;
        lastG1X = xPressed;
        updateHoodServos(currentHoodAnglePos);
    }

    public void handleShooterInput(boolean leftBumper, boolean rightBumper, boolean triangle, Intake intake) {
        if (triangle && !lastTriangle) {
            shooterOn     = false;
            shooterKilled = true;
            targetRPM     = 0;
            outtakeState  = OuttakeState.IDLE;
            gate.setPosition(0.5);
            intake.stopIfNotIntaking();
        }
        if (leftBumper && !lastLeftBumper) {
            shooterOn     = true;
            shooterKilled = false;
            targetRPM     = MAX_SHOOTER_RPM;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }
        if (rightBumper && !lastRightBumper) {
            shooterOn     = true;
            shooterKilled = false;
            targetRPM     = 3000;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastTriangle = triangle;
    }

    public void updateOuttakeSequence(Intake intake, VoltageSensor voltageSensor) {
        long elapsed = System.currentTimeMillis() - outtakeStateStartTime;
        switch (outtakeState) {
            case RAMPING:
                if (elapsed >= RAMP_DELAY_MS) {
                    gate.setPosition(0.27);
                    intake.runTransfer(voltageSensor);
                    outtakeState = OuttakeState.GATE_OPEN;
                    outtakeStateStartTime = System.currentTimeMillis();
                }
                break;
            case GATE_OPEN:
            case TRANSFERRING:
            case IDLE:
            default:
                break;
        }
    }

    public void updatePIDF(VoltageSensor voltageSensor, Telemetry telemetry) {
        if (!shooterOn || shooterKilled || targetRPM <= 0) {
            shooterLeft .setPower(0);
            shooterRight.setPower(0);
            if (leftController  != null) leftController .reset();
            if (rightController != null) rightController.reset();
            lastShooterVelocity = 0;
            return;
        }

        if (targetRPM >= 3600) {
            kP_shooter = kP_shooter_high; kI_shooter = kI_shooter_high;
            kD_shooter = kD_shooter_high; kF_shooter = kF_shooter_high;
        } else {
            kP_shooter = kP_shooter_low;  kI_shooter = kI_shooter_low;
            kD_shooter = kD_shooter_low;  kF_shooter = kF_shooter_low;
        }
        if (leftController  != null) leftController .setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);
        if (rightController != null) rightController.setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);

        double currentVoltage = voltageSensor.getVoltage();
        double avgVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        double powerLeft  = leftController .computePowerForTargetRPMWithVoltageCompensation(targetRPM, shooterLeft .getVelocity(), currentVoltage, NOMINAL_VOLTAGE);
        double powerRight = rightController.computePowerForTargetRPMWithVoltageCompensation(targetRPM, shooterRight.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);

        shooterLeft .setPower(powerLeft);
        shooterRight.setPower(powerRight);

        double velocityDrop = lastShooterVelocity - avgVelocity;
        if (velocityDrop > LAUNCH_DROP_THRESHOLD && lastShooterVelocity > 0) {
            double dippedGate = Math.max(0.0, Math.min(1.0, 0.27 - HOOD_DIP_AMOUNT));
            gate.setPosition(dippedGate);
            double dippedHood = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_DIP_AMOUNT);
            updateHoodServos(dippedHood);
        } else {
            gate.setPosition(0.27);
            updateHoodServos(currentHoodAnglePos);
        }
        lastShooterVelocity = avgVelocity;

        double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
        double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
        double avgRPM = (rpmL + rpmR) / 2.0;
        double rpmError = targetRPM - avgRPM;
        telemetry.addData("Target RPM", (int) targetRPM);
        telemetry.addData("Actual RPM", String.format("L:%d  R:%d  Avg:%d", (int) rpmL, (int) rpmR, (int) avgRPM));
        telemetry.addData("RPM Error", String.format("%+d (%.1f%%)", (int) rpmError, (rpmError / Math.max(targetRPM, 1)) * 100));
        telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
        telemetry.addData("Battery",  String.format("%.1f V", currentVoltage));
    }

    private void updateHoodServos(double position) {
        position = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, position));
        hoodServo1.setPosition(position);
        hoodServo2.setPosition(position);
    }

    public OuttakeState getOuttakeState() { return outtakeState; }
    public double getTargetRPM() { return targetRPM; }
    public double getHoodAnglePos() { return currentHoodAnglePos; }
}
