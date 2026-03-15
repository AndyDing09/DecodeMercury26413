package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    private static final long INTAKE_DELAY_MS = 50;
    private static final long RPM_DROP_DELAY_MS = 5000;
    private static final double CRUISE_RPM = 2000;
    private static final double ShooterScale = .925;
    private static final double TICKS_PER_REV   = 28.0;
    private static final double NOMINAL_VOLTAGE  = 12.0;

    // ================= HOOD LOCKED =================
    private static final double HOOD_LOCKED_POSITION = 0.9; // Fixed servo position — update after testing

    // For tele
    private static final long GATE_CLOSE_DELAY_MS = 2500;

    private static final double GATE_OPEN   = 0.27;
    private static final double GATE_CLOSED = 0.02;

    private double currentHoodAnglePos = HOOD_LOCKED_POSITION; // Always locked

    private boolean shooterOn = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;
    public static double kP_shooter_low  = 0.000026;
    public static double kI_shooter_low  = 0.0000;
    public static double kD_shooter_low  = 0.0000;
    public static double kF_shooter_low  = 0.0004;
    public static double kP_shooter_high = 0.00012;
    public static double kI_shooter_high = 0.000;
    public static double kD_shooter_high = 0.00008;
    public static double kF_shooter_high = 0.00045;

    private PIDFMotorController leftController;
    private PIDFMotorController rightController;

    private double lastShooterVelocity = 0;
    private double lastHoodServoPos = -1;
    private boolean intakeStarted = false;

    // Odometry-based auto-calc
    private boolean autoCalcEnabled = false;
    private double autoCalcRPM = 0;
    private double lastDistanceMeters = 0;
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

    public void startShooterOnly(boolean leftBumper, boolean lastLeft) {
        if (leftBumper && !lastLeft) {
            shooterOn     = true;
            shooterKilled = false;
            // RPM comes from odometry auto-calc
            updateHoodServos(HOOD_LOCKED_POSITION);
        }
    }


    public void handleShooterInput(boolean leftBumper, boolean rightBumper, boolean triangle, Intake intake) {
        if (triangle && !lastTriangle) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.IDLE;
            updateHoodServos(HOOD_LOCKED_POSITION);
        }
        if (leftBumper && !lastLeftBumper) {
            if (!shooterOn) {
                shooterOn  = true;
                shooterKilled = false;
            }
            intakeStarted = false;
            outtakeState        = OuttakeState.RAMPING;
            outtakeStateStartTime = System.currentTimeMillis();
        }

        if (rightBumper && !lastRightBumper) {
            shooterOn     = true;
            shooterKilled = false;
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
            double scale = (lastDistanceMeters >= 3.0) ? 0.35 : 0.55;
            intake.runTransfer(voltageSensor, scale);
            intakeStarted = true;
            outtakeState = OuttakeState.GATE_OPEN;
        }

        if (elapsed >= GATE_CLOSE_DELAY_MS && outtakeState == OuttakeState.GATE_OPEN) {
            gate.setPosition(GATE_CLOSED);
            outtakeState = OuttakeState.TRANSFERRING;
        }

        // Gate closed → intake at full power
        if (outtakeState == OuttakeState.TRANSFERRING) {
            intake.runTransfer(voltageSensor, 1.0);
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

        double avgVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        double powerLeft  = leftController .computePowerForTargetRPM(targetRPM, shooterLeft .getVelocity());
        double powerRight = rightController.computePowerForTargetRPM(targetRPM, shooterRight.getVelocity());

        shooterLeft .setPower(powerLeft);
        shooterRight.setPower(powerRight);

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
    }

    private void updateHoodServos(double position) {
        // Hood is locked — ignore deadzone and always enforce locked position
        hoodServo1.setPosition(HOOD_LOCKED_POSITION);
        hoodServo2.setPosition(1.0 - HOOD_LOCKED_POSITION);
        lastHoodServoPos = HOOD_LOCKED_POSITION;
    }

    public void updateFromLimelight(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalDegrees = CAMERA_MOUNT_ANGLE_DEGREES + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            double distanceMeters = (MathLib.TARGET_HEIGHT - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);

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
        lastDistanceMeters = distance;

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
    // =================================================================================
    public static double interpolateDistanceToRPM(double distanceM) {
        double[] inputDistances = { 1.1,    1.8,    2.38,    2.7,   3.05,    3.38,  3.64    }; // meters
        double[] outputRPM      = { 2725.0 * ShooterScale, 3165.0 * ShooterScale, 3425.0 * ShooterScale, 3605.0 * ShooterScale, 3740.0 * ShooterScale, 3950.0 * ShooterScale, 4000.0 * ShooterScale}; // RPM

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
        updateHoodServos(HOOD_LOCKED_POSITION);
    }
}
