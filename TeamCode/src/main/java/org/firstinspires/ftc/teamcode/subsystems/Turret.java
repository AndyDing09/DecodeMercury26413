package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    private final DcMotorEx turretMotor;
    private final Limelight3A limelight;

    // PD gains — large mode (aggressive)
    private double kP_large = 0.018;
    private double kD_large = 0.025;
    private double MAX_OUTPUT_large = 1.0;

    // PD gains — small mode (gentle)
    private double kP_small = 0.012;
    private double kD_small = 0.020;
    private double MAX_OUTPUT_small = 0.6;

    // Shared tuning
    private double TURRET_THRESHOLD = 6.0;
    private double DEADZONE_DEGREES = 0.3;
    private double TURRET_OPEN_F    = 0.015;
    private double TURRET_VEL_FF    = 0.003;
    private double kEncoderDamp     = 0.0002;
    private static final double DEFAULT_VOLTAGE = 12.0;

    private static final double MAX_TURRET_POWER  = 1.0;
    private static final double FAST_SEARCH_POWER = 0.35;
    private int TARGET_LOSS_THRESHOLD = 1;

    // Limits
    private static final int LEFT_LIMIT      = -400;
    private static final int RIGHT_LIMIT     =  400;
    private static final int CENTER_POSITION =    0;

    private static final double BRAKE_VEL_THRESHOLD = 80.0;
    private static final double BRAKE_POWER         = 0.25;

    private static final double INIT_POWER     = 1.0;
    private static final int    INIT_TOLERANCE = 10;

    // State
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

    // Tuning UI
    private int selectedParam = 0;
    private static final int    NUM_PARAMS  = 10;
    private static final double STEP_FINE   = 0.001;
    private static final double STEP_COARSE = 0.01;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void centerTurret(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        int currentPos;
        int error;
        do {
            currentPos = turretMotor.getCurrentPosition();
            error      = CENTER_POSITION - currentPos;
            turretMotor.setPower(Math.signum(error) * INIT_POWER);
            opMode.sleep(20);
        } while (Math.abs(error) > INIT_TOLERANCE && opMode.opModeIsActive());

        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretInitialized = true;
    }

    public void resetEncoder(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        if (!turretInitialized) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretInitialized = true;
            opMode.sleep(200);
        }
    }

    public void updateTuning(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight, boolean leftBumper) {
        if (dpadUp && !lastDpadUp)     selectedParam = (selectedParam + 1) % NUM_PARAMS;
        if (dpadDown && !lastDpadDown) selectedParam = (selectedParam + NUM_PARAMS - 1) % NUM_PARAMS;

        double step = leftBumper ? STEP_COARSE : STEP_FINE;
        if (dpadLeft && !lastDpadLeft)   adjustParam(selectedParam, -step);
        if (dpadRight && !lastDpadRight) adjustParam(selectedParam, +step);

        lastDpadUp    = dpadUp;
        lastDpadDown  = dpadDown;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;
    }

    public void updateTracking(boolean crossPressed, double manualStickX, VoltageSensor voltageSensor, Telemetry telemetry, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        if (crossPressed) {
            autoTrackEnabled    = !autoTrackEnabled;
            lastTx              = 0;
            lastTxValid         = false;
            txVelocity          = 0;
            framesWithoutTarget = 0;
            framesOnTarget      = 0;
            wasSearching        = false;
            braking             = false;
            opMode.sleep(200);
        }

        LLResult result      = limelight.getLatestResult();
        int      currentPos  = turretMotor.getCurrentPosition();
        double   encoderVel  = turretMotor.getVelocity();
        double   outputPower = 0;
        String   mode        = "IDLE";
        boolean  atLeftLimit  = currentPos <= LEFT_LIMIT;
        boolean  atRightLimit = currentPos >= RIGHT_LIMIT;

        if (Math.abs(manualStickX) > 0.2) {
            mode                = "MANUAL";
            outputPower         = manualStickX * 0.5;
            autoTrackEnabled    = false;
            framesWithoutTarget = 0;
            framesOnTarget      = 0;
            wasSearching        = false;
            braking             = false;
            lastTxValid         = false;
            txVelocity          = 0;
        }
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
                    outputPower      = 0;
                    autoTrackEnabled = true;
                    wasSearching     = true;
                    scanningRight    = !atRightLimit;
                    lastTxValid      = false;
                    mode             = "LIMIT->SEARCH";
                }

                lastTx = tx;
            }
        }
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

        telemetry.addData("Turret Mode",   mode);
        telemetry.addData("tx Error",      result != null && result.isValid()
                ? String.format("%.2f deg", result.getTx()) : "N/A");
        telemetry.addData("tx Velocity",   String.format("%.3f deg/frame", txVelocity));
        telemetry.addData("Enc Velocity",  String.format("%.0f ticks/s", encoderVel));
        telemetry.addData("Frames Stable", framesOnTarget);
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetry(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        telemetry.addLine("--- TURRET PID TUNER (gamepad2 dpad) ---");
        for (int i = 0; i < NUM_PARAMS; i++) {
            String marker = (i == selectedParam) ? "> " : "  ";
            telemetry.addData(marker + paramName(i), String.format("%.4f", paramValue(i)));
        }

        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Auto-Track",   autoTrackEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Enc",   turretMotor.getCurrentPosition());
        telemetry.addData("Target Found", result != null && result.isValid());
    }

    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
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
}
