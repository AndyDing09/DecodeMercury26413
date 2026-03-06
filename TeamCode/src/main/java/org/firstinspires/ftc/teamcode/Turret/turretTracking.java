package org.firstinspires.ftc.teamcode.Turret;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Turret Tracking + Search")
public class turretTracking extends OpMode {

    private DcMotor     turretMotor;
    private Limelight3A limelight;
    private ElapsedTime loopTimer = new ElapsedTime();

    // ---------- PID Gains (mutable for live tuner) ----------
    private double kP       = 0.022;
    private double kD       = 0.0015;
    private double kF       = 0.0;
    private double DEADZONE = 1.0;   // degrees
    private double SCAN_PWR = 0.15;

    // MAX_POWER lowered from 0.6 → 0.45.
    // Your 108RPM turret doesn't need more than this, and higher speeds
    // cause motion blur that drops AprilTag decodes.
    private static final double MAX_POWER = 0.45;

    // Speed scaling: when error is smaller than this, cap output power to
    // NEAR_POWER to keep the turret moving slowly enough for the Limelight
    // to maintain a clean tag decode. This is the main fix for losing the
    // tag during tracking — the turret was moving too fast near center.
    private static final double NEAR_THRESHOLD_DEG = 8.0;  // degrees
    private static final double NEAR_POWER         = 0.25; // max power when close to target

    // ---------- Hardware Constants ----------
    private static final int LEFT_LIMIT  = -430;
    private static final int RIGHT_LIMIT =  430;

    // ---------- Target loss debounce ----------
    // Raised from 8 → 20 frames (~400ms).
    // AprilTag decoding drops frames during motion — 8 frames was expiring
    // before the Limelight had a chance to reacquire, sending the turret
    // into a search sweep that moved it away from the tag entirely.
    // If it still loses too easily, raise toward 30.
    private static final int LOSS_DEBOUNCE_FRAMES = 20;
    private int framesWithoutTarget = 0;

    // ---------- State Variables ----------
    private double  lastTx        = 0;
    private boolean scanningRight = true;
    private boolean wasTracking   = false;

    // =====================================================================
    //  LIVE TUNER
    //  GP1 START         — toggle tuner ON/OFF
    //  GP1 dpad UP/DOWN  — select parameter (shown with >>> on telemetry)
    //  GP1 dpad LEFT/RIGHT — decrease/increase value
    //  GP1 LEFT BUMPER   — hold for coarse steps (10x)
    // =====================================================================
    private boolean tunerActive = false;
    private int     tunerParam  = 0;
    private static final int    NUM_PARAMS   = 5;
    private static final double STEP_FINE    = 0.001;
    private static final double STEP_COARSE  = 0.010;

    private boolean lastStart, lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Turret Initialized. Center turret before Start.");
        telemetry.addLine("GP1 START = toggle live tuner");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        double dt = Math.max(0.001, loopTimer.seconds());
        loopTimer.reset();

        updateTuner();

        LLResult result     = limelight.getLatestResult();
        boolean  hasTarget  = result != null && result.isValid();
        int      currentPos = turretMotor.getCurrentPosition();
        double   outputPower = 0;
        String   mode        = "IDLE";

        // -------------------------------------------------------
        // 1. MANUAL OVERRIDE
        // -------------------------------------------------------
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            mode                = "MANUAL";
            outputPower         = gamepad1.right_stick_x * 0.5;
            lastTx              = 0;
            wasTracking         = false;
            framesWithoutTarget = 0;
        }

        // -------------------------------------------------------
        // 2. TRACKING
        // -------------------------------------------------------
        else if (hasTarget) {
            double tx         = result.getTx();
            double derivative = wasTracking ? (tx - lastTx) / dt : 0;

            framesWithoutTarget = 0;
            wasTracking         = true;
            lastTx              = tx;

            if (Math.abs(tx) > DEADZONE) {
                mode        = Math.abs(tx) > 5.0 ? "SLEWING" : "TRACKING";
                outputPower = (kP * tx) + (kD * derivative);

                // Feedforward — overcome static friction at small errors
                if (Math.abs(outputPower) > 0.001 && Math.abs(outputPower) < kF) {
                    outputPower = Math.signum(outputPower) * kF;
                }

                // Speed limiting near target — keeps turret slow enough that
                // the Limelight can maintain a clean AprilTag decode.
                // Without this, the turret swings through center fast enough
                // to blur the tag and drop the detection every pass.
                if (Math.abs(tx) < NEAR_THRESHOLD_DEG) {
                    double nearScale = Math.abs(tx) / NEAR_THRESHOLD_DEG;
                    double cappedPower = NEAR_POWER + (MAX_POWER - NEAR_POWER) * nearScale;
                    if (Math.abs(outputPower) > cappedPower) {
                        outputPower = Math.signum(outputPower) * cappedPower;
                    }
                    mode = "TRACKING (slow)";
                }

            } else {
                mode        = "LOCKED";
                outputPower = 0;
            }
        }

        // -------------------------------------------------------
        // 3. DEBOUNCE HOLD
        //    Hold position while waiting for tag to reappear.
        //    Motor brake keeps turret still — no power needed.
        //    Raised to 20 frames so the Limelight has time to
        //    reacquire after a motion-blur dropout.
        // -------------------------------------------------------
        else if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
            framesWithoutTarget++;
            mode        = "HOLDING (" + framesWithoutTarget + "/" + LOSS_DEBOUNCE_FRAMES + ")";
            outputPower = 0;
        }

        // -------------------------------------------------------
        // 4. SEARCH — only after debounce expires
        // -------------------------------------------------------
        else {
            mode        = "SEARCHING";
            wasTracking = false;

            if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                // Search toward where we last saw the tag
                scanningRight = lastTx > 0;
            }
            framesWithoutTarget++;

            if (currentPos >= RIGHT_LIMIT) scanningRight = false;
            else if (currentPos <= LEFT_LIMIT) scanningRight = true;

            outputPower  = scanningRight ? SCAN_PWR : -SCAN_PWR;
            mode        += scanningRight ? " ->" : " <-";
        }

        // -------------------------------------------------------
        // 5. SAFETY
        // -------------------------------------------------------
        outputPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, outputPower));
        if (currentPos <= LEFT_LIMIT  && outputPower < 0) outputPower = 0;
        if (currentPos >= RIGHT_LIMIT && outputPower > 0) outputPower = 0;

        turretMotor.setPower(outputPower);

        updateTelemetry(mode, hasTarget, result, currentPos, outputPower, dt);
    }

    // =====================================================================
    //  LIVE TUNER
    // =====================================================================
    private void updateTuner() {
        boolean start = gamepad1.start;
        if (start && !lastStart) tunerActive = !tunerActive;
        lastStart = start;

        if (!tunerActive) return;

        boolean dU     = gamepad1.dpad_up;
        boolean dD     = gamepad1.dpad_down;
        boolean dL     = gamepad1.dpad_left;
        boolean dR     = gamepad1.dpad_right;
        boolean coarse = gamepad1.left_bumper;

        if (dU && !lastDpadUp)   tunerParam = (tunerParam + 1) % NUM_PARAMS;
        if (dD && !lastDpadDown) tunerParam = (tunerParam + NUM_PARAMS - 1) % NUM_PARAMS;

        double step = coarse ? STEP_COARSE : STEP_FINE;
        if (dR && !lastDpadRight) adjustParam(+step);
        if (dL && !lastDpadLeft)  adjustParam(-step);

        lastDpadUp    = dU;
        lastDpadDown  = dD;
        lastDpadLeft  = dL;
        lastDpadRight = dR;
    }

    private void adjustParam(double delta) {
        switch (tunerParam) {
            case 0: kP       = Math.max(0.000, kP       + delta);             break;
            case 1: kD       = Math.max(0.000, kD       + delta);             break;
            case 2: kF       = Math.max(0.000, kF       + delta);             break;
            case 3: DEADZONE = Math.max(0.100, DEADZONE + delta * 5);         break;
            case 4: SCAN_PWR = Math.max(0.050, Math.min(0.40, SCAN_PWR + delta * 2)); break;
        }
    }

    // =====================================================================
    //  TELEMETRY
    // =====================================================================
    @SuppressLint("DefaultLocale")
    private void updateTelemetry(String mode, boolean hasTarget, LLResult result,
                                 int currentPos, double outputPower, double dt) {
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Mode",     mode);
        telemetry.addData("Target",   hasTarget ? "YES" : "no");
        telemetry.addData("TX (deg)", hasTarget
                ? String.format("%.2f", result.getTx()) : "N/A");
        telemetry.addData("Encoder",  String.format("%d  [%d to %d]",
                currentPos, LEFT_LIMIT, RIGHT_LIMIT));
        telemetry.addData("Power",    String.format("%.3f", outputPower));
        telemetry.addData("Scan Dir", scanningRight ? "RIGHT ->" : "<- LEFT");
        telemetry.addData("dt (ms)",  String.format("%.1f", dt * 1000));

        telemetry.addLine("===== LIVE TUNER =====");
        telemetry.addData("GP1 START", "toggle tuner ON/OFF");
        telemetry.addData("Tuner", tunerActive
                ? "ON  — dpad UP/DN:select  L/R:adjust  LB:coarse"
                : "OFF");

        String[] names = { "kP",   "kD",   "kF",  "Deadzone(deg)", "ScanPower" };
        double[] vals  = {  kP,     kD,     kF,    DEADZONE,         SCAN_PWR   };
        for (int i = 0; i < NUM_PARAMS; i++) {
            telemetry.addData(
                    (tunerActive && i == tunerParam ? ">>> " : "    ") + names[i],
                    String.format("%.4f", vals[i]));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}