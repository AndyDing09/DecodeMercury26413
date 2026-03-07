package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Turret  + Search")
public class Turret_Odometry_and_Limelight_Test extends OpMode {

    private DcMotor     turretMotor;
    private Limelight3A limelight;
    private ElapsedTime loopTimer = new ElapsedTime();

    // ---------- PID Tuning (now mutable so live tuner can change them) ----------
    private double kP             = 0.030;
    private double kD             = 0.002;  // high damping — prevents lurch-oscillation from herringbone friction
    private double kF             = 0.06;   // herringbone gears need ~0.10-0.15 to break static friction
    private double DEADZONE       = 3;    // wider deadzone — don't fight friction on small errors, let brake hold
    private double SCAN_PWR       = 0.25;

    private static final double MAX_POWER = 0.90;

    // ---------- Hardware Constants ----------
    private static final int LEFT_LIMIT  = -430;
    private static final int RIGHT_LIMIT =  430;

    // ---------- Target loss debounce ----------
    private static final int LOSS_DEBOUNCE_FRAMES = 8;
    private int framesWithoutTarget = 0;

    // ---------- State Variables ----------
    private double  lastTx        = 0;
    private boolean scanningRight = true;
    private boolean wasTracking   = false;

    // =====================================================================
    //  LIVE TUNER STATE
    //
    //  HOW TO USE:
    //    GP1 START         — toggle tuner ON / OFF
    //    GP1 dpad UP/DOWN  — select parameter (shown with >>> on telemetry)
    //    GP1 dpad RIGHT    — increase selected parameter
    //    GP1 dpad LEFT     — decrease selected parameter
    //    GP1 LEFT BUMPER   — hold for coarse steps (10x faster)
    //
    //  Parameters you can tune:
    //    kP       — proportional gain (main tracking force)
    //    kD       — derivative gain   (damps overshoot, ~10% of kP)
    //    kF       — feedforward       (minimum power to overcome friction)
    //    DEADZONE — degrees of error to ignore (hides Limelight jitter)
    //    SCAN     — search sweep power
    // =====================================================================
    private boolean tunerActive  = false;
    private int     tunerParam   = 0;           // which param is selected
    private static final int NUM_PARAMS = 5;

    // Fine step = 0.001, coarse (hold LB) = 0.010
    private static final double STEP_FINE   = 0.001;
    private static final double STEP_COARSE = 0.010;

    // Edge-detect booleans — prevents a single button hold registering as many presses
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

        telemetry.addLine("Turret Initialized.");
        telemetry.addLine("GP1 START = toggle live tuner");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        double  dt         = Math.max(0.001, loopTimer.seconds());
        loopTimer.reset();

        // -------------------------------------------------------
        // LIVE TUNER INPUT (runs every loop regardless of mode)
        // -------------------------------------------------------
        updateTuner();

        // -------------------------------------------------------
        // TURRET LOGIC
        // -------------------------------------------------------
        LLResult result      = limelight.getLatestResult();
        boolean  hasTarget   = result != null && result.isValid();
        int      currentPos  = turretMotor.getCurrentPosition();
        double   outputPower = 0;
        String   mode        = "IDLE";

        // 1. MANUAL OVERRIDE
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            mode            = "MANUAL";
            outputPower     = gamepad1.right_stick_x * 0.5;
            lastTx          = 0;
            wasTracking     = false;
            framesWithoutTarget = 0;
        }

        // 2. TRACKING
        else if (hasTarget) {
            double tx         = result.getTx();
            double derivative = wasTracking ? (tx - lastTx) / dt : 0;

            framesWithoutTarget = 0;
            wasTracking         = true;
            lastTx              = tx;

            if (Math.abs(tx) > DEADZONE) {
                mode        = Math.abs(tx) > 5.0 ? "SLEWING" : "TRACKING";
                outputPower = (kP * tx) + (kD * derivative);

                if (Math.abs(outputPower) > 0.001 && Math.abs(outputPower) < kF) {
                    outputPower = Math.signum(outputPower) * kF;
                }
            } else {
                mode        = "LOCKED";
                outputPower = 0;
            }
        }

        // 3. DEBOUNCE HOLD
        else if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
            framesWithoutTarget++;
            mode        = "HOLDING (" + framesWithoutTarget + "/" + LOSS_DEBOUNCE_FRAMES + ")";
            outputPower = 0;
        }

        // 4. SEARCH
        else {
            mode        = "SEARCHING";
            wasTracking = false;

            if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                scanningRight = lastTx > 0;
            }
            framesWithoutTarget++;

            if (currentPos >= RIGHT_LIMIT) scanningRight = false;
            else if (currentPos <= LEFT_LIMIT) scanningRight = true;

            outputPower  = scanningRight ? SCAN_PWR : -SCAN_PWR;
            mode        += scanningRight ? " ->" : " <-";
        }

        // 5. SAFETY
        outputPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, outputPower));
        if (currentPos <= LEFT_LIMIT  && outputPower < 0) outputPower = 0;
        if (currentPos >= RIGHT_LIMIT && outputPower > 0) outputPower = 0;

        turretMotor.setPower(outputPower);

        // -------------------------------------------------------
        // TELEMETRY
        // -------------------------------------------------------
        updateTelemetry(mode, hasTarget, result, currentPos, outputPower, dt);
    }

    // =====================================================================
    //  LIVE TUNER — call every loop()
    // =====================================================================
    private void updateTuner() {
        // Toggle tuner with START (edge detect)
        boolean start = gamepad1.start;
        if (start && !lastStart) tunerActive = !tunerActive;
        lastStart = start;

        if (!tunerActive) return; // nothing else to do if tuner is off

        boolean dU = gamepad1.dpad_up;
        boolean dD = gamepad1.dpad_down;
        boolean dL = gamepad1.dpad_left;
        boolean dR = gamepad1.dpad_right;
        boolean coarse = gamepad1.left_bumper; // hold for 10x step

        // Select parameter (edge detect on up/down)
        if (dU && !lastDpadUp)   tunerParam = (tunerParam + 1) % NUM_PARAMS;
        if (dD && !lastDpadDown) tunerParam = (tunerParam + NUM_PARAMS - 1) % NUM_PARAMS;

        // Adjust value (edge detect on left/right)
        double step = coarse ? STEP_COARSE : STEP_FINE;
        if (dR && !lastDpadRight) adjustParam(+step);
        if (dL && !lastDpadLeft)  adjustParam(-step);

        // Save edge state
        lastDpadUp    = dU;
        lastDpadDown  = dD;
        lastDpadLeft  = dL;
        lastDpadRight = dR;
    }

    private void adjustParam(double delta) {
        switch (tunerParam) {
            case 0: kP       = Math.max(0.000, kP       + delta);       break;
            case 1: kD       = Math.max(0.000, kD       + delta);       break;
            case 2: kF       = Math.max(0.000, kF       + delta);       break;
            case 3: DEADZONE = Math.max(0.100, DEADZONE + delta * 5);   break; // 5x scale feels natural for degrees
            case 4: SCAN_PWR = Math.max(0.050, Math.min(MAX_POWER, SCAN_PWR + delta * 2)); break;
        }
    }

    // =====================================================================
    //  TELEMETRY
    // =====================================================================
    @SuppressLint("DefaultLocale")
    private void updateTelemetry(String mode, boolean hasTarget, LLResult result,
                                 int currentPos, double outputPower, double dt) {
        // --- Turret status ---
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Mode",     mode);
        telemetry.addData("Target",   hasTarget ? "YES" : "no");
        telemetry.addData("TX (deg)", hasTarget
                ? String.format("%.2f", result.getTx()) : "N/A");
        telemetry.addData("Encoder",  String.format("%d  [limit: %d to %d]",
                currentPos, LEFT_LIMIT, RIGHT_LIMIT));
        telemetry.addData("Power",    String.format("%.3f", outputPower));
        telemetry.addData("Scan Dir", scanningRight ? "RIGHT ->" : "<- LEFT");
        telemetry.addData("dt (ms)",  String.format("%.1f", dt * 1000));

        // --- Live tuner panel ---
        telemetry.addLine("===== LIVE TUNER =====");
        telemetry.addData("GP1 START", "toggle tuner ON/OFF");
        telemetry.addData("Tuner", tunerActive
                ? "ON  — dpad UP/DN:select  L/R:adjust  LB:coarse"
                : "OFF");

        // Parameter list — selected one shows >>>
        String[] names  = { "kP",   "kD",     "kF",  "Deadzone(deg)", "ScanPower" };
        double[] vals   = {  kP,     kD,        kF,    DEADZONE,        SCAN_PWR   };
        for (int i = 0; i < NUM_PARAMS; i++) {
            String marker = (tunerActive && i == tunerParam) ? ">>>" : "   ";
            telemetry.addData(marker + " " + names[i], String.format("%.4f", vals[i]));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        turretMotor.setPower(0);
    }
}