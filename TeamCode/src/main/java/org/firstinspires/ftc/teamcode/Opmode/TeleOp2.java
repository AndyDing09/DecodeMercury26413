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
    private static final int LEFT_LIMIT      = -430;
    private static final int RIGHT_LIMIT     =  450;
    private static final int CENTER_POSITION =    0;

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
    private int     framesOnTarget      = 0;
    private int     framesWithoutTarget = 0;

    // ================= TUNING UI STATE =================
    private int    selectedParam  = 0;
    private static final int    NUM_PARAMS   = 9;
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

    // ================= OUTTAKE SEQUENCE =================
    private enum OuttakeState { IDLE, RAMPING, GATE_OPEN, TRANSFERRING }
    private OuttakeState outtakeState = OuttakeState.IDLE;
    private long outtakeStateStartTime = 0;
    private static final long RAMP_DELAY_MS = 2000;

    private double targetRPM = 0;
    private static final double TICKS_PER_REV  = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MAX_SHOOTER_RPM = 4800.0;

    private double kP_shooter_low  = 0.006, kI_shooter_low  = 0.0005, kD_shooter_low  = 0.0002, kF_shooter_low  = 0.00620;
    private double kP_shooter_high = 0.007, kI_shooter_high = 0.0008, kD_shooter_high = 0.0003, kF_shooter_high = 0.00520;
    private double kP_shooter, kI_shooter, kD_shooter, kF_shooter;

    private PIDFMotorController leftController  = null;
    private PIDFMotorController rightController = null;

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

        telemetry.addLine("Ready. Gamepad2: UP/DOWN=cycle param, LEFT/RIGHT=adjust, LB=coarse, A=reset enc, X=track");
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

            // ---- SHOOTER BUTTON HANDLING ----
            boolean lb  = gamepad1.left_bumper;
            boolean rb  = gamepad1.right_bumper;
            boolean tri = gamepad1.triangle;

            if (tri && !lastTriangle) {
                // Kill everything and reset outtake sequence
                shooterOn     = false;
                shooterKilled = true;
                targetRPM     = 0;
                outtakeState  = OuttakeState.IDLE;
                Gate.setPosition(0.5);
                // Only stop transfer if not actively intaking
                if (!intakeOn) middleTransfer.setPower(0.0);
            }
            if (lb && !lastLeftBumper) {
                shooterOn     = true;
                shooterKilled = false;
                targetRPM     = MAX_SHOOTER_RPM;
                // Begin outtake sequence: ramp up first, gate and transfer follow after delay
                outtakeState        = OuttakeState.RAMPING;
                outtakeStateStartTime = System.currentTimeMillis();
            }
            if (rb && !lastRightBumper) {
                shooterOn     = true;
                shooterKilled = false;
                targetRPM     = 3000;
                // Begin outtake sequence: ramp up first, gate and transfer follow after delay
                outtakeState        = OuttakeState.RAMPING;
                outtakeStateStartTime = System.currentTimeMillis();
            }
            lastLeftBumper = lb; lastRightBumper = rb; lastTriangle = tri;

            // ---- OUTTAKE SEQUENCE STATE MACHINE ----
            updateOuttakeSequence();

            // ---- SHOOTER PIDF ----
            if (!shooterOn || shooterKilled || targetRPM <= 0) {
                shooterLeft .setPower(0);
                shooterRight.setPower(0);
                if (leftController  != null) leftController .reset();
                if (rightController != null) rightController.reset();
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
                double powerLeft  = leftController .computePowerForTargetRPMWithVoltageCompensation(targetRPM, shooterLeft .getVelocity(), currentVoltage, NOMINAL_VOLTAGE);
                double powerRight = rightController.computePowerForTargetRPMWithVoltageCompensation(targetRPM, shooterRight.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);

                shooterLeft .setPower(powerLeft);
                shooterRight.setPower(powerRight);

                double rpmL = shooterLeft .getVelocity() * 60.0 / TICKS_PER_REV;
                double rpmR = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
                telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
                telemetry.addData("LeftRPM",  (int) rpmL);
                telemetry.addData("RightRPM", (int) rpmR);
                telemetry.addData("Battery",  String.format("%.1f V", currentVoltage));
            }

            updateTurretTuning();
            updateTurretTracking();
            updateTelemetry();
        }

        limelight.stop();
        turretMotor.setPower(0);
    }

    // ===================================================================
    //  OUTTAKE SEQUENCE STATE MACHINE
    //  RAMPING  → wait RAMP_DELAY_MS → GATE_OPEN (open gate) → TRANSFERRING (transfer on)
    // ===================================================================
    private void updateOuttakeSequence() {
        if (outtakeState == OuttakeState.IDLE) return;

        long now     = System.currentTimeMillis();
        long elapsed = now - outtakeStateStartTime;

        switch (outtakeState) {
            case RAMPING:
                // Shooter is spinning up — do nothing to gate or transfer yet.
                // After 2 seconds, open the gate.
                if (elapsed >= RAMP_DELAY_MS) {
                    Gate.setPosition(0.27);
                    outtakeState        = OuttakeState.GATE_OPEN;
                    outtakeStateStartTime = now;
                }
                break;

            case GATE_OPEN:
                // Gate just opened — turn on the transfer immediately so balls feed through.
                middleTransfer.setPower(1.0);
                outtakeState = OuttakeState.TRANSFERRING;
                break;

            case TRANSFERRING:
                // Steady state: shooter running, gate open, transfer on.
                // Everything stays on until the driver kills it with triangle.
                break;

            default:
                break;
        }
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
            sleep(200);
        }

        LLResult result      = limelight.getLatestResult();
        int      currentPos  = turretMotor.getCurrentPosition();
        double   outputPower = 0;
        String   mode        = "IDLE";

        // 1. MANUAL OVERRIDE
        if (Math.abs(gamepad2.right_stick_x) > 0.2) {
            mode                = "MANUAL";
            outputPower         = gamepad2.right_stick_x * 0.5;
            autoTrackEnabled    = false;
            framesWithoutTarget = 0;
            framesOnTarget      = 0;
            wasSearching        = false;
            lastTxValid         = false;
            txVelocity          = 0;
        }
        // 2. AUTO TRACKING — valid target
        else if (autoTrackEnabled && result != null && result.isValid()) {
            double tx    = result.getTx();
            double absTx = Math.abs(tx);
            framesWithoutTarget = 0;

            if (wasSearching || !lastTxValid) {
                lastTx      = tx;
                txVelocity  = 0;
                lastTxValid = true;
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

                double voltage       = voltageSensor.getVoltage();
                double voltageScale  = DEFAULT_VOLTAGE / voltage;
                outputPower += TURRET_OPEN_F * voltageScale * Math.signum(outputPower);
                outputPower += txVelocity * TURRET_VEL_FF * voltageScale;

                outputPower = Math.max(-maxOut, Math.min(maxOut, outputPower));
            } else {
                outputPower = 0;
                mode        = "LOCKED OK";
            }

            if ((currentPos >= RIGHT_LIMIT && outputPower > 0) ||
                    (currentPos <= LEFT_LIMIT  && outputPower < 0)) {
                outputPower = 0;
                mode        = "AT LIMIT";
            }

            lastTx = tx;
        }
        // 3. SEARCH SWEEP
        else if (autoTrackEnabled) {
            framesWithoutTarget++;
            lastTxValid = false;
            txVelocity  = 0;

            if (framesWithoutTarget < TARGET_LOSS_THRESHOLD) {
                mode        = "DEBOUNCE";
                outputPower = 0;
            } else {
                mode           = "SEARCHING";
                lastTx         = 0;
                wasSearching   = true;
                framesOnTarget = 0;

                if      (currentPos >= RIGHT_LIMIT) scanningRight = false;
                else if (currentPos <= LEFT_LIMIT)  scanningRight = true;

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
        telemetry.addData("Auto-Track",    autoTrackEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Enc",    turretMotor.getCurrentPosition());
        telemetry.addData("Target Found",  result != null && result.isValid());
        telemetry.addData("Shooter RPM",   (int) targetRPM);
        telemetry.addData("Outtake State", outtakeState.toString());
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