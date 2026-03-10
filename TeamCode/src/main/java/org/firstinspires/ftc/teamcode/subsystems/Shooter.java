package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.LauncherSolution;
import org.firstinspires.ftc.teamcode.testing.MathLib;
import org.firstinspires.ftc.teamcode.testing.PIDFMotorController;

@Config
public class Shooter {
    private final DcMotorEx shooterLeft, shooterRight;
    private final Servo gate;
    private final Servo hoodServo1, hoodServo2;

    public enum OuttakeState { IDLE, RAMPING, GATE_OPEN, TRANSFERRING }
    private OuttakeState outtakeState = OuttakeState.IDLE;
    private long outtakeStateStartTime = 0;
    private static final long INTAKE_DELAY_MS = 500;
    private static final long RPM_DROP_DELAY_MS = 5000;
    private static final double CRUISE_RPM = 2000;

    private static final double TICKS_PER_REV   = 28.0;
    private static final double NOMINAL_VOLTAGE  = 12.0;
    private static final double LEFT_BUMPER_RPM  = 4500;
    private static final double RIGHT_BUMPER_RPM = 3400;

    private static final double HOOD_SERVO_INIT = 0.7;
    private static final double MIN_HOOD_SERVO  = 0.0;
    private static final double MAX_HOOD_SERVO  = 1.0;
    private static final double HOOD_ANGLE_STEP = 0.05;
    private static final double HOOD_DEADZONE   = 0.03;

    private static final double GATE_OPEN   = 0.26;
    private static final double GATE_CLOSED = 0.1;

    private double currentHoodAnglePos = HOOD_SERVO_INIT;
    private boolean lastG1Y = false;
    private boolean lastG1X = false;

    private boolean shooterOn = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;
    public static double kP_shooter_low  = 0.00001;
    public static double kI_shooter_low  = 0.00002;
    public static double kD_shooter_low  = 0.0000;
    public static double kF_shooter_low  = 0.00043;
    public static double kP_shooter_high = 0.00012;
    public static double kI_shooter_high = 0.00003;
    public static double kD_shooter_high = 0.00008;
    public static double kF_shooter_high = 0.00045;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    private double lastShooterVelocity = 0;
    private static final double LAUNCH_DROP_THRESHOLD = 50.0; // ~107 RPM drop between loops
    private static final double HOOD_DIP_AMOUNT = 0.1;
    private double cumulativeHoodDip = 0;
    private double lastHoodServoPos = -1;
    private double initialRPM = 0; // the high RPM before dropping to CRUISE_RPM
    private boolean intakeStarted = false;

    // Odometry-based auto-calc
    private boolean autoCalcEnabled = false;
    private double autoCalcRPM = 0;
    private double autoCalcServoPos = HOOD_SERVO_INIT;
    public void setShooterOn(boolean on) { this.shooterOn = on; }
    public Servo getGate() { return gate; }
    public Shooter(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        gate         = hardwareMap.servo.get("Gate");
        hoodServo1   = hardwareMap.servo.get("angleChange1");
        hoodServo2   = hardwareMap.servo.get("angleChange2");

        shooterLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate.setPosition(GATE_CLOSED);
        updateHoodServos(currentHoodAnglePos);
    }

    public void initControllers() {
        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
    }

    public void handleHoodInput(boolean yPressed, boolean xPressed) {
        boolean changed = false;
        if (yPressed && !lastG1Y) { currentHoodAnglePos = Math.min(MAX_HOOD_SERVO, currentHoodAnglePos + HOOD_ANGLE_STEP); changed = true; }
        if (xPressed && !lastG1X) { currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_ANGLE_STEP); changed = true; }
        lastG1Y = yPressed;
        lastG1X = xPressed;
        if (changed) {
            cumulativeHoodDip = 0;
            updateHoodServos(currentHoodAnglePos);
        }
    }

    public void handleShooterInput(boolean leftBumper, boolean rightBumper, boolean triangle, Intake intake) {
        if (triangle && !lastTriangle) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.IDLE;
            cumulativeHoodDip = 0;
            currentHoodAnglePos = HOOD_SERVO_INIT;
            updateHoodServos(currentHoodAnglePos);
        }
        if (leftBumper && !lastLeftBumper) {
            shooterOn     = true;
            shooterKilled = false;
            initialRPM    = LEFT_BUMPER_RPM;
            targetRPM     = LEFT_BUMPER_RPM;
            cumulativeHoodDip = 0;
            currentHoodAnglePos = HOOD_SERVO_INIT;
            updateHoodServos(currentHoodAnglePos);
            gate.setPosition(GATE_OPEN);
            intakeStarted = false;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }
        if (rightBumper && !lastRightBumper) {
            shooterOn     = true;
            shooterKilled = false;
            initialRPM    = RIGHT_BUMPER_RPM;
            targetRPM     = RIGHT_BUMPER_RPM;
            cumulativeHoodDip = 0;
            currentHoodAnglePos = HOOD_SERVO_INIT;
            updateHoodServos(currentHoodAnglePos);
            gate.setPosition(GATE_OPEN);
            intakeStarted = false;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastTriangle = triangle;
    }

    public void updateOuttakeSequence(Intake intake, VoltageSensor voltageSensor) {
        if (outtakeState == OuttakeState.IDLE) return;

        long elapsed = System.currentTimeMillis() - outtakeStateStartTime;

        // 0.5s after bumper: turn on intake
        if (!intakeStarted && elapsed >= INTAKE_DELAY_MS) {
            intake.runTransfer(voltageSensor);
            intakeStarted = true;
            outtakeState = OuttakeState.GATE_OPEN;
        }

        // 5s after bumper: drop RPM to cruise
        if (elapsed >= RPM_DROP_DELAY_MS && targetRPM > CRUISE_RPM) {
            targetRPM = CRUISE_RPM;
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
            cumulativeHoodDip += HOOD_DIP_AMOUNT;
        }
        lastShooterVelocity = avgVelocity;

        // Apply hood with cumulative dip (gate stays at GATE_OPEN)
        double effectiveHood = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - cumulativeHoodDip);
        updateHoodServos(effectiveHood);

        double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
        double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
        double avgRPM = (rpmL + rpmR) / 2.0;
        double rpmError = targetRPM - avgRPM;
        telemetry.addData("Target RPM", (int) targetRPM);
        telemetry.addData("Actual RPM", String.format("L:%d  R:%d  Avg:%d", (int) rpmL, (int) rpmR, (int) avgRPM));
        telemetry.addData("RPM Error", String.format("%+d (%.1f%%)", (int) rpmError, (rpmError / Math.max(targetRPM, 1)) * 100));
        telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
        telemetry.addData("Hood", String.format("base=%.2f  dip=%.2f  effective=%.2f", currentHoodAnglePos, cumulativeHoodDip, effectiveHood));
        telemetry.addData("Vel Drop", String.format("%.1f ticks/s (threshold: %.1f)", velocityDrop, LAUNCH_DROP_THRESHOLD));
        telemetry.addData("Battery",  String.format("%.1f V", currentVoltage));
    }

    private void updateHoodServos(double position) {
        position = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, position));
        // Only update servos if position changed more than deadzone
        if (lastHoodServoPos >= 0 && Math.abs(position - lastHoodServoPos) < HOOD_DEADZONE) {
            return;
        }
        lastHoodServoPos = position;
        hoodServo1.setPosition(position);
        hoodServo2.setPosition(position);
    }


    public void updateFromOdometry(Follower follower, Telemetry telemetry) {
        Pose pose = follower.getPose();
        // Calculate distance from robot position to goal center
        double distance = Math.hypot(pose.getX() - 132.0, pose.getY() - 132.0) * 0.0254;
        LauncherSolution solution = MathLib.distanceToLauncherValues(distance);

        if (solution.isValid()) {
            autoCalcEnabled = true;
            autoCalcRPM = MathLib.solutionToRPM(solution);
            autoCalcServoPos = MathLib.solutionToServoPos(solution);
        } else {
            autoCalcEnabled = false;
        }

        if (shooterOn && !shooterKilled && autoCalcEnabled) {
            targetRPM = autoCalcRPM;
            currentHoodAnglePos = autoCalcServoPos;
            updateHoodServos(currentHoodAnglePos);
        }

        double dx = MathLib.GOAL_CENTER_X - pose.getX();
        double dy = MathLib.GOAL_CENTER_Y - pose.getY();
        double distIn = Math.sqrt(dx * dx + dy * dy);
        telemetry.addData("Robot Pos", String.format("(%.1f, %.1f) hdg=%.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        telemetry.addData("Dist to Goal", String.format("%.1f in (%.2f m)", distIn, distIn * 0.0254));
        telemetry.addData("Auto-Calc", autoCalcEnabled ? "VALID" : "NO SOLUTION");
        if (autoCalcEnabled) {
            telemetry.addData("Calc RPM", (int) autoCalcRPM);
            telemetry.addData("Calc Hood Servo", String.format("%.3f", autoCalcServoPos));
        }
    }


    public OuttakeState getOuttakeState() { return outtakeState; }
    public double getTargetRPM() { return targetRPM; }
    public double getHoodAnglePos() { return currentHoodAnglePos; }
    public boolean isShooterOn() { return shooterOn && !shooterKilled; }
    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }
    public void setHoodAnglePos(double pos) {
        this.currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, pos));
        updateHoodServos(this.currentHoodAnglePos);
    }
}
