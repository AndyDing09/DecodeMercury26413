package org.firstinspires.ftc.teamcode.Opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.testing.PIDFMotorController;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;


@TeleOp(name = "TeleOp with Turret Tracking - Fixed", group = "TeleOp")
public class TeleOp2 extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // ================= MECHANISMS =================
    private DcMotor middleTransfer;
    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor voltageSensor;
    private Servo transferBlocker, Gate;

    // ================= HOOD ANGLE SERVOS =================
    private Servo hoodServo1, hoodServo2;
    private double currentHoodAnglePos = 0.5; // Servo position (0.0–1.0), init at 0.5
    private static final double HOOD_SERVO_INIT = 0.5;
    private static final double MIN_HOOD_SERVO   = 0.0;
    private static final double MAX_HOOD_SERVO   = 1.0;
    private static final double HOOD_ANGLE_STEP  = 0.05; // Per button press adjustment

    // Hood angle gamepad edge detection
    private boolean lastG1Y = false;
    private boolean lastG1X = false;

    // ================= TURRET =================
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // -------------------------------------------------------
    //  LARGE mode: aggressive — used when |tx| > TURRET_THRESHOLD
    // -------------------------------------------------------
    private double kP_large = 0.018;
    private double kD_large = 0.025;
    private double kF_large = 0.0;
    private double MAX_OUTPUT_large = 1.0;

    // -------------------------------------------------------
    //  SMALL mode: gentle — used when |tx| <= TURRET_THRESHOLD
    // -------------------------------------------------------
    private double kP_small = 0.012;
    private double kD_small = 0.020;
    private double kF_small = 0.0;
    private double MAX_OUTPUT_small = 0.6;

    // -------------------------------------------------------
    //  Shared tuning constants
    // -------------------------------------------------------
    private double TURRET_THRESHOLD   = 6.0;
    private double DEADZONE_DEGREES   = 0.3;
    private double TURRET_OPEN_F      = 0.015;
    private double TURRET_VEL_FF      = 0.003;
    private static final double DEFAULT_VOLTAGE = 12.0;

    private static final double MAX_TURRET_POWER  = 1.0;
    private static final double FAST_SEARCH_POWER = 0.35;

    private int TARGET_LOSS_THRESHOLD = 1;

    // ================= TURRET LIMITS =================
    private static final int LEFT_LIMIT      = -400;
    private static final int RIGHT_LIMIT     =  400;
    private static final int CENTER_POSITION =    0;

    private static final double BRAKE_VEL_THRESHOLD = 80.0;
    private static final double BRAKE_POWER         = 0.25;
    private double kEncoderDamp = 0.0002;

    private static final boolean AUTO_CENTER_ON_INIT = true;
    private static final double  INIT_POWER          = 1.0;
    private static final int     INIT_TOLERANCE       = 10;

    // ================= TURRET STATE =================
    private double  lastTx              = 0;
    private boolean lastTxValid         = false;
    private double  txVelocity          = 0;
    private boolean scanningRight       = true;
    private boolean autoTrackEnabled    = false;
    private boolean turretInitialized   = false;
    private boolean wasSearching        = false;
    private boolean braking             = false;
    private int     framesOnTarget      = 0;
    private int     framesWithoutTarget = 0;

    // ================= TUNING UI STATE =================
    private int    selectedParam  = 0;
    private static final int    NUM_PARAMS   = 10;
    private static final double STEP_FINE    = 0.001;
    private static final double STEP_COARSE  = 0.01;
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;

    // ================= INTAKE =================
    private boolean intakeOn   = false;
    private boolean lastCircle = false;

    // ================= SHOOTER =================
    private boolean shooterOn     = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private static final double TICKS_PER_REV  = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MAX_SHOOTER_RPM = 4800.0;

    private double kP_shooter_low  = 0.0012, kI_shooter_low  = 0.0003, kD_shooter_low  = 0.00008, kF_shooter_low  = 0.00045;
    private double kP_shooter_high = 0.0015, kI_shooter_high = 0.0004, kD_shooter_high = 0.00010, kF_shooter_high = 0.00045;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;

    private PIDFMotorController leftController  = null;
    private PIDFMotorController rightController = null;

    // Ball launch detection
    private double lastShooterVelocity = 0;
    private static final double LAUNCH_DROP_THRESHOLD = 200.0;
    private static final double HOOD_DIP_AMOUNT = 0.05; // servo position units to dip on launch

    private final double ServoStart = 0.5;

    // ===================================================================
    @Override
    public void runOpMode() {

        middleTransfer  = hardwareMap.get(DcMotor.class,   "middleTransfer");
        frontLeftDrive  = hardwareMap.get(DcMotor.class,   "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class,   "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class,   "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class,   "back_right_drive");
        shooterLeft     = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight    = hardwareMap.get(DcMotorEx.class, "shooterRight");
        voltageSensor   = hardwareMap.voltageSensor.iterator().next();
        transferBlocker = hardwareMap.servo.get("transferBlocker");
        Gate            = hardwareMap.servo.get("Gate");
        turretMotor     = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight       = hardwareMap.get(Limelight3A.class, "limelight");

        // ================= HOOD ANGLE SERVOS =================
        hoodServo1 = hardwareMap.servo.get("angleChange1");
        hoodServo2 = hardwareMap.servo.get("angleChange2");
        currentHoodAnglePos = HOOD_SERVO_INIT;
        updateHoodServos(currentHoodAnglePos);

        frontLeftDrive .setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive  .setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive .setDirection(DcMotor.Direction.FORWARD);
        middleTransfer .setDirection(DcMotor.Direction.FORWARD);
        shooterRight   .setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft    .setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();

        transferBlocker.setPosition(ServoStart);
        Gate.setPosition(0.5);

        telemetry.addLine("Ready. GP1: Y/X=Hood Up/Down | GP2: UP/DOWN=cycle param, LEFT/RIGHT=adjust, LB=coarse, A=reset enc, X=track");
        telemetry.update();

        if (AUTO_CENTER_ON_INIT) centerTurret();

        waitForStart();

        leftController  = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);

        while (opModeIsActive()) {

            if (gamepad2.a && !turretInitialized) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretInitialized = true;
                sleep(200);
            }

            // ---- DRIVE ----
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

            frontLeftDrive .setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive  .setPower(bl);
            backRightDrive .setPower(br);

            // ---- INTAKE ----
            if (gamepad1.circle && !lastCircle) intakeOn = !intakeOn;
            lastCircle = gamepad1.circle;
            middleTransfer.setPower(intakeOn ? 1.0 : 0.0);

            // ---- HOOD ANGLE CONTROL (Gamepad1 Y / X) ----
            boolean curY = gamepad1.y;
            boolean curX = gamepad1.x;
            if (curY && !lastG1Y) currentHoodAnglePos = Math.min(MAX_HOOD_SERVO, currentHoodAnglePos + HOOD_ANGLE_STEP);
            if (curX && !lastG1X) currentHoodAnglePos = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_ANGLE_STEP);
            lastG1Y = curY;
            lastG1X = curX;
            updateHoodServos(currentHoodAnglePos);

            // ---- SHOOTER ----
            boolean lb  = gamepad1.left_bumper;
            boolean rb  = gamepad1.right_bumper;
            boolean tri = gamepad1.triangle;

            if (tri && !lastTriangle) {
                shooterOn = false; shooterKilled = true; targetRPM = 0;
                Gate.setPosition(0.5);
            }
            if (lb && !lastLeftBumper) {
                shooterOn = true; shooterKilled = false;
                targetRPM = MAX_SHOOTER_RPM;
                Gate.setPosition(0.27);
            }
            if (rb && !lastRightBumper) {
                shooterOn = true; shooterKilled = false;
                targetRPM = 3000;
                Gate.setPosition(0.27);
            }
            lastLeftBumper = lb; lastRightBumper = rb; lastTriangle = tri;

            // ---- SHOOTER PIDF ----
            if (!shooterOn || shooterKilled || targetRPM <= 0) {
                shooterLeft .setPower(0);
                shooterRight.setPower(0);
                if (leftController  != null) leftController .reset();
                if (rightController != null) rightController.reset();
                lastShooterVelocity = 0;
            } else {
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

                // ---- BALL LAUNCH DETECTION ----
                // A sudden velocity drop of 200+ ticks/sec means a ball just fired.
                // Dip BOTH the Gate AND the hood angle servos down by 0.05 to help feed next ball.
                double velocityDrop = lastShooterVelocity - avgVelocity;
                if (velocityDrop > LAUNCH_DROP_THRESHOLD && lastShooterVelocity > 0) {
                    // Ball fired! Dip Gate servo
                    double dippedGate = Math.max(0.0, Math.min(1.0, 0.27 - HOOD_DIP_AMOUNT));
                    Gate.setPosition(dippedGate);

                    // Dip hood angle servos down by 0.05
                    double dippedHood = Math.max(MIN_HOOD_SERVO, currentHoodAnglePos - HOOD_DIP_AMOUNT);
                    updateHoodServos(dippedHood);
                } else {
                    // Gradually restore Gate to 0.27
                    Gate.setPosition(0.27);
                    // Restore hood servos to driver-set position
                    updateHoodServos(currentHoodAnglePos);
                }
                lastShooterVelocity = avgVelocity;

                double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
                double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
                telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
                telemetry.addData("LeftRPM",  (int) rpmL);
                telemetry.addData("RightRPM", (int) rpmR);
                telemetry.addData("Battery",  String.format("%.1f V", currentVoltage));
                telemetry.addData("Hood Servo Pos", String.format("%.3f", currentHoodAnglePos));
            }

            updateTurretTuning();
            updateTurretTracking();
            updateTelemetry();
        }

        limelight.stop();
        turretMotor.setPower(0);
    }

    // ===================================================================
    //  HOOD SERVO HELPERS
    // ===================================================================

    /** Sets both hood servos to the same position (0.0–1.0). */
    private void updateHoodServos(double position) {
        position = Math.max(MIN_HOOD_SERVO, Math.min(MAX_HOOD_SERVO, position));
        hoodServo1.setPosition(position);
        hoodServo2.setPosition(position);
    }

    // ===================================================================
    //  LIVE TUNING — gamepad2 dpad
    // ===================================================================
    private void updateTurretTuning() {
        boolean dU  = gamepad2.dpad_up;
        boolean dD  = gamepad2.dpad_down;
        boolean dL  = gamepad2.dpad_left;
        boolean dR  = gamepad2.dpad_right;
        boolean lb2 = gamepad2.left_bumper;

        if (dU && !lastDpadUp)   selectedParam = (selectedParam + 1) % NUM_PARAMS;
        if (dD && !lastDpadDown) selectedParam = (selectedParam + NUM_PARAMS - 1) % NUM_PARAMS;

        double step = lb2 ? STEP_COARSE : STEP_FINE;
        if (dL && !lastDpadLeft)  adjustParam(selectedParam, -step);
        if (dR && !lastDpadRight) adjustParam(selectedParam, +step);

        lastDpadUp    = dU;
        lastDpadDown  = dD;
        lastDpadLeft  = dL;
        lastDpadRight = dR;
    }

    private void adjustParam(int index, double delta) {
        switch (index) {
            case 0: kP_large         = Math.max(0,   kP_large         + delta);      break;
            case 1: kD_large         = Math.max(0,   kD_large         + delta);      break;
            case 2: kP_small         = Math.max(0,   kP_small         + delta);      break;
            case 3: kD_small         = Math.max(0,   kD_small         + delta);      break;
            case 4: TURRET_THRESHOLD = Math.max(1,   TURRET_THRESHOLD + delta * 10); break;
            case 5: DEADZONE_DEGREES = Math.max(0.1, DEADZONE_DEGREES + delta * 5);  break;
            case 6: TURRET_OPEN_F    = Math.max(0,   TURRET_OPEN_F    + delta);      break;
            case 7: TURRET_VEL_FF    = Math.max(0,   TURRET_VEL_FF    + delta);      break;
            case 8: MAX_OUTPUT_small = Math.max(0.1, Math.min(1.0, MAX_OUTPUT_small + delta * 5)); break;
            case 9: kEncoderDamp     = Math.max(0,   kEncoderDamp     + delta);      break;
        }
    }

    private String paramName(int i) {
        switch (i) {
            case 0: return "kP_large";
            case 1: return "kD_large";
            case 2: return "kP_small";
            case 3: return "kD_small";
            case 4: return "THRESHOLD (deg)";
            case 5: return "DEADZONE (deg)";
            case 6: return "OPEN_F (stiction)";
            case 7: return "VEL_FF";
            case 8: return "MAX_OUT_small";
            case 9: return "ENC_DAMP";
            default: return "?";
        }
    }

    private double paramValue(int i) {
        switch (i) {
            case 0: return kP_large;
            case 1: return kD_large;
            case 2: return kP_small;
            case 3: return kD_small;
            case 4: return TURRET_THRESHOLD;
            case 5: return DEADZONE_DEGREES;
            case 6: return TURRET_OPEN_F;
            case 7: return TURRET_VEL_FF;
            case 8: return MAX_OUTPUT_small;
            case 9: return kEncoderDamp;
            default: return 0;
        }
    }

    // ===================================================================
    //  TURRET TRACKING
    // ===================================================================
    private void updateTurretTracking() {
        if (gamepad2.cross) {
            autoTrackEnabled    = !autoTrackEnabled;
            lastTx              = 0;
            lastTxValid         = false;
            txVelocity          = 0;
            framesWithoutTarget = 0;
            framesOnTarget      = 0;
            wasSearching        = false;
            braking             = false;
            sleep(200);
        }

        LLResult result      = limelight.getLatestResult();
        int      currentPos  = turretMotor.getCurrentPosition();
        double   encoderVel  = turretMotor.getVelocity();
        double   outputPower = 0;
        String   mode        = "IDLE";
        boolean  atLeftLimit  = currentPos <= LEFT_LIMIT;
        boolean  atRightLimit = currentPos >= RIGHT_LIMIT;

        // 1. MANUAL OVERRIDE
        if (Math.abs(gamepad2.right_stick_x) > 0.2) {
            mode                = "MANUAL";
            outputPower         = gamepad2.right_stick_x * 0.5;
            autoTrackEnabled    = false;
            framesWithoutTarget = 0;
            framesOnTarget      = 0;
            wasSearching        = false;
            braking             = false;
            lastTxValid         = false;
            txVelocity          = 0;
        }
        // 2. AUTO TRACKING — valid target
        else if (autoTrackEnabled && result != null && result.isValid()) {
            double tx    = result.getTx();
            double absTx = Math.abs(tx);
            framesWithoutTarget = 0;

            if (wasSearching && !braking) {
                braking     = true;
                lastTx      = tx;
                lastTxValid = true;
                txVelocity  = 0;
            }

            if (braking) {
                if (Math.abs(encoderVel) > BRAKE_VEL_THRESHOLD) {
                    outputPower = -Math.signum(encoderVel) * BRAKE_POWER;
                    mode        = "BRAKING";
                    lastTx      = tx;
                } else {
                    braking        = false;
                    wasSearching   = false;
                    lastTx         = tx;
                    txVelocity     = 0;
                    framesOnTarget = 0;
                }
            }

            if (!braking) {
                if (!lastTxValid) {
                    lastTx         = tx;
                    txVelocity     = 0;
                    lastTxValid    = true;
                    wasSearching   = false;
                    framesOnTarget = 0;
                } else {
                    txVelocity = tx - lastTx;
                }

                double kP, kD, maxOut;
                if (absTx > TURRET_THRESHOLD) {
                    kP     = kP_large;
                    kD     = kD_large;
                    maxOut = MAX_OUTPUT_large;
                    mode   = "TRACKING (LARGE)";
                    framesOnTarget = 0;
                } else {
                    kP     = kP_small;
                    kD     = kD_small;
                    maxOut = MAX_OUTPUT_small;
                    mode   = "LOCKED";
                    framesOnTarget++;
                }

                boolean atSetPoint = absTx <= DEADZONE_DEGREES;

                if (!atSetPoint) {
                    outputPower = -((kP * tx) + (kD * (tx - lastTx)));
                    outputPower -= encoderVel * kEncoderDamp;

                    double voltage      = voltageSensor.getVoltage();
                    double voltageScale = DEFAULT_VOLTAGE / voltage;
                    outputPower += TURRET_OPEN_F * voltageScale * Math.signum(outputPower);
                    outputPower += txVelocity * TURRET_VEL_FF * voltageScale;
                    outputPower = Math.max(-maxOut, Math.min(maxOut, outputPower));
                } else {
                    outputPower = 0;
                    mode        = "LOCKED OK";
                }

                if ((atRightLimit && outputPower > 0) ||
                        (atLeftLimit && outputPower < 0)) {
                    outputPower    = 0;
                    autoTrackEnabled = true;
                    wasSearching     = true;
                    scanningRight    = !atRightLimit;
                    lastTxValid      = false;
                    mode             = "LIMIT->SEARCH";
                }

                lastTx = tx;
            }
        }
        // 3. SEARCH SWEEP
        else if (autoTrackEnabled) {
            framesWithoutTarget++;
            lastTxValid = false;
            braking     = false;
            txVelocity  = 0;

            if (framesWithoutTarget < TARGET_LOSS_THRESHOLD) {
                mode        = "DEBOUNCE";
                outputPower = 0;
            } else {
                mode           = "SEARCHING";
                lastTx         = 0;
                wasSearching   = true;
                framesOnTarget = 0;

                if (atRightLimit)      scanningRight = false;
                else if (atLeftLimit)  scanningRight = true;

                outputPower = scanningRight ? FAST_SEARCH_POWER : -FAST_SEARCH_POWER;
                mode += scanningRight ? " ->" : " <-";
            }
        }

        outputPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, outputPower));
        turretMotor.setPower(outputPower);

        telemetry.addData("Turret Mode",    mode);
        telemetry.addData("tx Error",       result != null && result.isValid()
                ? String.format("%.2f deg", result.getTx()) : "N/A");
        telemetry.addData("tx Velocity",    String.format("%.3f deg/frame", txVelocity));
        telemetry.addData("Enc Velocity",   String.format("%.0f ticks/s", encoderVel));
        telemetry.addData("Frames Stable",  framesOnTarget);
    }

    // ===================================================================
    //  TELEMETRY
    // ===================================================================
    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        LLResult result = limelight.getLatestResult();

        telemetry.addLine("--- TURRET PID TUNER (gamepad2 dpad) ---");
        for (int i = 0; i < NUM_PARAMS; i++) {
            String marker = (i == selectedParam) ? "> " : "  ";
            telemetry.addData(marker + paramName(i), String.format("%.4f", paramValue(i)));
        }

        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Auto-Track",     autoTrackEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Enc",     turretMotor.getCurrentPosition());
        telemetry.addData("Target Found",   result != null && result.isValid());
        telemetry.addData("Shooter RPM",    (int) targetRPM);
        telemetry.addData("Hood Pos",       String.format("%.3f", currentHoodAnglePos));
        telemetry.update();
    }

    // ===================================================================
    //  INIT — CENTER TURRET
    // ===================================================================
    private void centerTurret() {
        int currentPos;
        int error;
        do {
            currentPos = turretMotor.getCurrentPosition();
            error      = CENTER_POSITION - currentPos;
            turretMotor.setPower(Math.signum(error) * INIT_POWER);
            sleep(20);
        } while (Math.abs(error) > INIT_TOLERANCE && opModeIsActive());

        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretInitialized = true;
    }
}