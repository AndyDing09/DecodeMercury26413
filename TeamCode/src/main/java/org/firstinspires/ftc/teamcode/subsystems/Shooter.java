/*

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private final Limelight3A limelight;

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

    // Hood dip: drop hood 0.05 each time RPM suddenly drops 200+ between loops
    private static final double HOOD_DIP_RPM_THRESHOLD = 200.0;
    private static final double HOOD_DIP_AMOUNT        = 0.05;

    // For tele
    private static final long GATE_CLOSE_DELAY_MS = 1500;

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
    public static double kP_shooter_low  = 0.000026;
    public static double kI_shooter_low  = 0.00002;
    public static double kD_shooter_low  = 0.0000;
    public static double kF_shooter_low  = 0.0004;
    public static double kP_shooter_high = 0.00012;
    public static double kI_shooter_high = 0.00003;
    public static double kD_shooter_high = 0.00008;
    public static double kF_shooter_high = 0.00045;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    private double lastShooterVelocity = 0;
    private double lastHoodServoPos = -1;
    private double initialRPM = 0; // the high RPM before dropping to CRUISE_RPM
    private boolean intakeStarted = false;

    // Odometry-based auto-calc
    private boolean autoCalcEnabled = false;
    private double autoCalcRPM = 0;
    private double autoCalcServoPos = HOOD_SERVO_INIT;

    public static double CAMERA_HEIGHT_METERS = 0.3; // TODO: Tune this to your robot's camera height
    public static double CAMERA_MOUNT_ANGLE_DEGREES = 20.0; // TODO: Tune this to your camera's mounting angle

    public void setShooterOn(boolean on) { this.shooterOn = on; }
    public Servo getGate() { return gate; }
    public Shooter(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        gate         = hardwareMap.servo.get("Gate");
        hoodServo1   = hardwareMap.servo.get("angleChange1");
        hoodServo2   = hardwareMap.servo.get("angleChange2");
        limelight    = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();

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

            updateHoodServos(currentHoodAnglePos);
        }
    }

    public void handleShooterInput(boolean leftBumper, boolean rightBumper, boolean triangle, Intake intake) {
        if (triangle && !lastTriangle) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.IDLE;

            currentHoodAnglePos = HOOD_SERVO_INIT;
            updateHoodServos(currentHoodAnglePos);
        }
        if (leftBumper && !lastLeftBumper) {
            shooterOn     = true;
            shooterKilled = false;
            initialRPM    = LEFT_BUMPER_RPM;
            targetRPM     = LEFT_BUMPER_RPM;

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

        // 2s after bumper: close the gate
        if (elapsed >= GATE_CLOSE_DELAY_MS && outtakeState == OuttakeState.GATE_OPEN) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.TRANSFERRING;
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

        if (targetRPM >= 4000) {
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

        // Detect sudden RPM drop (200+) between loops → dip hood down 0.05, cumulative
        double avgRPMNow = avgVelocity * 60.0 / TICKS_PER_REV;
        double lastRPM = lastShooterVelocity * 60.0 / TICKS_PER_REV;
        if (lastRPM > 0 && (lastRPM - avgRPMNow) >= HOOD_DIP_RPM_THRESHOLD) {
            currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_DIP_AMOUNT);
        }
        lastShooterVelocity = avgVelocity;

        updateHoodServos(currentHoodAnglePos);

        double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
        double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
        double avgRPM = (rpmL + rpmR) / 2.0;
        double rpmError = targetRPM - avgRPM;
        telemetry.addData("Target RPM", (int) targetRPM);
        telemetry.addData("Actual RPM", String.format("L:%d  R:%d  Avg:%d", (int) rpmL, (int) rpmR, (int) avgRPM));
        telemetry.addData("RPM Error", String.format("%+d (%.1f%%)", (int) rpmError, (rpmError / Math.max(targetRPM, 1)) * 100));
        telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
        telemetry.addData("Hood Pos", String.format("%.2f", currentHoodAnglePos));
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
        hoodServo2.setPosition(1.0 - position);
    }

    public void updateFromLimelight(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalDegrees = CAMERA_MOUNT_ANGLE_DEGREES + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            // Calculate distance: (Target Height - Camera Height) / tan(Mount Angle + ty)
            double distanceMeters = (MathLib.TARGET_HEIGHT - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);

            LauncherSolution solution = MathLib.distanceToLauncherValues(distanceMeters);

            if (solution.isValid()) {
                autoCalcEnabled = true;
                autoCalcRPM = MathLib.solutionToRPM(solution);
                autoCalcServoPos = MathLib.solutionToServoPos(solution);

                if (shooterOn && !shooterKilled) {
                    targetRPM = autoCalcRPM;
                    currentHoodAnglePos = autoCalcServoPos;
                    updateHoodServos(currentHoodAnglePos);
                }
            } else {
                autoCalcEnabled = false;
            }
            telemetry.addData("LL Dist", "%.3f m", distanceMeters);
        }
    }


    public void updateFromOdometry(Follower follower, Telemetry telemetry) {
        Pose pose = follower.getPose();
        // Calculate distance from robot position to goal center
        double distance = Math.hypot(pose.getX() - MathLib.GOAL_CENTER_X, pose.getY() - MathLib.GOAL_CENTER_Y) * 0.0254;
        LauncherSolution solution = MathLib.distanceToLauncherValues(distance);

        if (solution.isValid()) {
            autoCalcEnabled = true;
            autoCalcRPM = MathLib.solutionToRPM(solution);
            autoCalcServoPos = MathLib.solutionToServoPos(solution);
        } else {
            autoCalcEnabled = false;
        }

        if (shooterOn && !shooterKilled && autoCalcEnabled) {
            // Only update RPM if it changed by more than 50 to prevent PIDF from chasing noise
            if (Math.abs(autoCalcRPM - targetRPM) > 50) {
                targetRPM = autoCalcRPM;
            }
            // Only update hood if it changed meaningfully
            if (Math.abs(autoCalcServoPos - currentHoodAnglePos) > 0.02) {
                currentHoodAnglePos = autoCalcServoPos;
                updateHoodServos(currentHoodAnglePos);
            }
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


 */





























































package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private final Limelight3A limelight;

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

    // ================= HOOD LOCKED =================
    private static final double HOOD_LOCKED_POSITION = 0.9; // Fixed servo position — update after testing
    // private static final double HOOD_SERVO_INIT = 0.7;   // OLD: was used for dynamic hood
    // private static final double MIN_HOOD_SERVO  = 0.0;   // OLD
    // private static final double MAX_HOOD_SERVO  = 1.0;   // OLD
    // private static final double HOOD_ANGLE_STEP = 0.05;  // OLD
    // private static final double HOOD_DEADZONE   = 0.03;  // OLD

    // Hood dip: drop hood 0.05 each time RPM suddenly drops 200+ between loops
    // private static final double HOOD_DIP_RPM_THRESHOLD = 200.0; // OLD — disabled with locked hood
    // private static final double HOOD_DIP_AMOUNT        = 0.05;  // OLD — disabled with locked hood

    // For tele
    private static final long GATE_CLOSE_DELAY_MS = 1500;

    private static final double GATE_OPEN   = 0.26;
    private static final double GATE_CLOSED = 0.1;

    private double currentHoodAnglePos = HOOD_LOCKED_POSITION; // Always locked
    // private boolean lastG1Y = false; // OLD — hood manual control removed
    // private boolean lastG1X = false; // OLD — hood manual control removed

    private boolean shooterOn = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;
    public static double kP_shooter_low  = 0.000026;
    public static double kI_shooter_low  = 0.00002;
    public static double kD_shooter_low  = 0.0000;
    public static double kF_shooter_low  = 0.0004;
    public static double kP_shooter_high = 0.00012;
    public static double kI_shooter_high = 0.00003;
    public static double kD_shooter_high = 0.00008;
    public static double kF_shooter_high = 0.00045;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    private double lastShooterVelocity = 0;
    private double lastHoodServoPos = -1;
    private double initialRPM = 0;
    private boolean intakeStarted = false;

    // Odometry-based auto-calc
    private boolean autoCalcEnabled = false;
    private double autoCalcRPM = 0;
    private double autoCalcServoPos = HOOD_LOCKED_POSITION; // Always locked

    public static double CAMERA_HEIGHT_METERS = 0.3;
    public static double CAMERA_MOUNT_ANGLE_DEGREES = 20.0;

    public void setShooterOn(boolean on) { this.shooterOn = on; }
    public Servo getGate() { return gate; }

    public Shooter(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        gate         = hardwareMap.servo.get("Gate");
        hoodServo1   = hardwareMap.servo.get("angleChange1");
        hoodServo2   = hardwareMap.servo.get("angleChange2");
        limelight    = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();

        shooterLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate.setPosition(GATE_CLOSED);
        // Lock hood immediately on init
        updateHoodServos(HOOD_LOCKED_POSITION);
    }

    public void initControllers() {
        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
    }

    // OLD: manual hood control — disabled, hood is now locked
    // public void handleHoodInput(boolean yPressed, boolean xPressed) {
    //     boolean changed = false;
    //     if (yPressed && !lastG1Y) { currentHoodAnglePos = Math.min(MAX_HOOD_SERVO, currentHoodAnglePos + HOOD_ANGLE_STEP); changed = true; }
    //     if (xPressed && !lastG1X) { currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_ANGLE_STEP); changed = true; }
    //     lastG1Y = yPressed;
    //     lastG1X = xPressed;
    //     if (changed) updateHoodServos(currentHoodAnglePos);
    // }

    public void handleShooterInput(boolean leftBumper, boolean rightBumper, boolean triangle, Intake intake) {
        if (triangle && !lastTriangle) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.IDLE;
            // Hood stays locked — no reset needed
            // currentHoodAnglePos = HOOD_SERVO_INIT; // OLD
            updateHoodServos(HOOD_LOCKED_POSITION);
        }
        if (leftBumper && !lastLeftBumper) {
            shooterOn     = true;
            shooterKilled = false;
            initialRPM    = LEFT_BUMPER_RPM;
            targetRPM     = LEFT_BUMPER_RPM;
            // Hood stays locked
            updateHoodServos(HOOD_LOCKED_POSITION);
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
            // Hood stays locked
            updateHoodServos(HOOD_LOCKED_POSITION);
            gate.setPosition(GATE_OPEN);
            intakeStarted = false;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
        lastTriangle    = triangle;
    }

    public void updateOuttakeSequence(Intake intake, VoltageSensor voltageSensor) {
        if (outtakeState == OuttakeState.IDLE) return;

        long elapsed = System.currentTimeMillis() - outtakeStateStartTime;

        if (!intakeStarted && elapsed >= INTAKE_DELAY_MS) {
            intake.runTransfer(voltageSensor);
            intakeStarted = true;
            outtakeState = OuttakeState.GATE_OPEN;
        }

        if (elapsed >= GATE_CLOSE_DELAY_MS && outtakeState == OuttakeState.GATE_OPEN) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.TRANSFERRING;
        }

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

        if (targetRPM >= 4000) {
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

        // OLD: hood dip on RPM drop — disabled, hood is locked
        // double avgRPMNow = avgVelocity * 60.0 / TICKS_PER_REV;
        // double lastRPM = lastShooterVelocity * 60.0 / TICKS_PER_REV;
        // if (lastRPM > 0 && (lastRPM - avgRPMNow) >= HOOD_DIP_RPM_THRESHOLD) {
        //     currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_DIP_AMOUNT);
        // }

        lastShooterVelocity = avgVelocity;

        // Keep hood locked every loop
        updateHoodServos(HOOD_LOCKED_POSITION);

        double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
        double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
        double avgRPM = (rpmL + rpmR) / 2.0;
        double rpmError = targetRPM - avgRPM;
        telemetry.addData("Target RPM", (int) targetRPM);
        telemetry.addData("Actual RPM", String.format("L:%d  R:%d  Avg:%d", (int) rpmL, (int) rpmR, (int) avgRPM));
        telemetry.addData("RPM Error", String.format("%+d (%.1f%%)", (int) rpmError, (rpmError / Math.max(targetRPM, 1)) * 100));
        telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
        telemetry.addData("Hood Pos", String.format("LOCKED at %.2f", HOOD_LOCKED_POSITION));
        telemetry.addData("Battery",  String.format("%.1f V", currentVoltage));
    }

    private void updateHoodServos(double position) {
        // Hood is locked — ignore deadzone and always enforce locked position
        hoodServo1.setPosition(HOOD_LOCKED_POSITION);
        hoodServo2.setPosition(1.0 - HOOD_LOCKED_POSITION);
        lastHoodServoPos = HOOD_LOCKED_POSITION;

        // OLD: dynamic hood servo update
        // position = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, position));
        // if (lastHoodServoPos >= 0 && Math.abs(position - lastHoodServoPos) < HOOD_DEADZONE) return;
        // lastHoodServoPos = position;
        // hoodServo1.setPosition(position);
        // hoodServo2.setPosition(1.0 - position);
    }

    public void updateFromLimelight(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalDegrees = CAMERA_MOUNT_ANGLE_DEGREES + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            double distanceMeters = (MathLib.TARGET_HEIGHT - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);

            // OLD: physics-based solution — replaced by distance table
            // LauncherSolution solution = MathLib.distanceToLauncherValues(distanceMeters);
            // if (solution.isValid()) {
            //     autoCalcEnabled  = true;
            //     autoCalcRPM      = MathLib.solutionToRPM(solution);
            //     autoCalcServoPos = MathLib.solutionToServoPos(solution);
            //     if (shooterOn && !shooterKilled) {
            //         targetRPM = autoCalcRPM;
            //         currentHoodAnglePos = autoCalcServoPos;
            //         updateHoodServos(currentHoodAnglePos);
            //     }
            // } else {
            //     autoCalcEnabled = false;
            // }

            // NEW: distance table lookup
            double calcRPM = interpolateDistanceToRPM(distanceMeters);
            if (calcRPM > 0) {
                autoCalcEnabled = true;
                autoCalcRPM     = calcRPM;
                if (shooterOn && !shooterKilled) {
                    targetRPM = autoCalcRPM;
                }
            } else {
                autoCalcEnabled = false;
            }

            telemetry.addData("LL Dist", "%.3f m", distanceMeters);
        }
    }

    public void updateFromOdometry(Follower follower, Telemetry telemetry) {
        Pose pose = follower.getPose();
        double distance = Math.hypot(pose.getX() - MathLib.GOAL_CENTER_X, pose.getY() - MathLib.GOAL_CENTER_Y) * 0.0254;

        // OLD: physics-based solution — replaced by distance table
        // LauncherSolution solution = MathLib.distanceToLauncherValues(distance);
        // if (solution.isValid()) {
        //     autoCalcEnabled  = true;
        //     autoCalcRPM      = MathLib.solutionToRPM(solution);
        //     autoCalcServoPos = MathLib.solutionToServoPos(solution);
        // } else {
        //     autoCalcEnabled = false;
        // }

        // NEW: distance table lookup, hood stays locked
        double calcRPM = interpolateDistanceToRPM(distance);
        if (calcRPM > 0) {
            autoCalcEnabled = true;
            autoCalcRPM     = calcRPM;
        } else {
            autoCalcEnabled = false;
        }

        if (shooterOn && !shooterKilled && autoCalcEnabled) {
            if (Math.abs(autoCalcRPM - targetRPM) > 50) {
                targetRPM = autoCalcRPM;
            }
            // Hood stays locked — no servo update needed
            // OLD: if (Math.abs(autoCalcServoPos - currentHoodAnglePos) > 0.02) {
            //     currentHoodAnglePos = autoCalcServoPos;
            //     updateHoodServos(currentHoodAnglePos);
            // }
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
            telemetry.addData("Hood", String.format("LOCKED at %.2f", HOOD_LOCKED_POSITION));
        }
    }

    // =================================================================================
    // DISTANCE -> RPM LOOKUP TABLE
    // Replace placeholder values with your measured data:
    //   inputDistances = distances from goal in meters
    //   outputRPM      = shooter RPM needed at that distance
    // =================================================================================
    public static double interpolateDistanceToRPM(double distanceM) {
        double[] inputDistances = { 1.1,    1.5,    2.08,    2.4,   2.75,    3.08,  3.34    }; // meters — REPLACE WITH YOUR DATA
        double[] outputRPM      = { 2800.0, 3250.0, 3500.0, 3600.0, 3675.0, 3900.0, 3975.0 }; // RPM    — REPLACE WITH YOUR DATA

        if (distanceM <= inputDistances[0]) return outputRPM[0];
        if (distanceM >= inputDistances[inputDistances.length - 1]) return outputRPM[outputRPM.length - 1];

        for (int i = 0; i < inputDistances.length - 1; i++) {
            if (distanceM >= inputDistances[i] && distanceM <= inputDistances[i + 1]) {
                double fraction = (distanceM - inputDistances[i]) / (inputDistances[i + 1] - inputDistances[i]);
                return outputRPM[i] + fraction * (outputRPM[i + 1] - outputRPM[i]);
            }
        }
        return 0.0;
    }

    public OuttakeState getOuttakeState() { return outtakeState; }
    public double getTargetRPM() { return targetRPM; }
    public double getHoodAnglePos() { return currentHoodAnglePos; }
    public boolean isShooterOn() { return shooterOn && !shooterKilled; }
    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }
    public void setHoodAnglePos(double pos) {
        // OLD: dynamic hood — now locked, ignore input
        // this.currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, pos));
        // updateHoodServos(this.currentHoodAnglePos);
        updateHoodServos(HOOD_LOCKED_POSITION);
    }
}