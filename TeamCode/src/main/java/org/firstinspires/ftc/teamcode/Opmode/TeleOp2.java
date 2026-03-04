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




@TeleOp(name="TeleOp with Turret Tracking - Fixed", group="TeleOp")
public class TeleOp2 extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // ================= MECHANISMS =================
    private DcMotor middleTransfer;
    private DcMotorEx shooterLeft, shooterRight;
    private VoltageSensor voltageSensor;


    private Servo transferBlocker, Gate;

    // ================= TURRET TRACKING =================
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // DUAL-MODE PID: Aggressive Acquisition + Gentle Hold
    // ACQUISITION MODE (fast catch, accepts some overshoot)
    private static final double kP_acquire = 0.03;
    private static final double kD_acquire = 0.06;
    private static final double kF_acquire = 0.01;

    // HOLD MODE (smooth, no jitter)
    private static final double kP_hold = 0.018;
    private static final double kD_hold = 0.045;
    private static final double kF_hold = 0.0;

    private static final double ACQUIRE_THRESHOLD = 8.0; // Switch modes at 8° error
    private static final double HOLD_THRESHOLD = 2.0;    // Tight deadzone when locked
    private static final double MAX_TURRET_POWER = 0.8;

    // Fast sweep power when searching
    private static final double FAST_SEARCH_POWER = 0.6; // Scanner speed
    private static final double DEADZONE_DEGREES = 1.5;
    private static final int TARGET_LOSS_THRESHOLD = 3;  // Frames before declaring target lost (debounce)

    // ================= TURRET LIMITS =================
    private static final int LEFT_LIMIT = -347;
    private static final int RIGHT_LIMIT = 550;
    private static final int CENTER_POSITION = 93;

    private static final boolean AUTO_CENTER_ON_INIT = true;
    private static final double INIT_POWER = 0.3;
    private static final int INIT_TOLERANCE = 20;

    // Turret State
    private double lastError = 0;
    private boolean scanningRight = true;
    private boolean autoTrackEnabled = false;
    private boolean turretInitialized = false;
    private boolean wasSearching = false;
    private int framesOnTarget = 0;
    private int framesWithoutTarget = 0;  // Debounce target loss

    // ================= INTAKE =================
    private boolean intakeOn = false;
    private boolean lastCircle = false;

    // ================= SHOOTER =================
    private boolean shooterOn = false;
    private boolean shooterKilled = false;
    private boolean lastLeftBumper, lastRightBumper, lastTriangle;

    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double NOMINAL_VOLTAGE = 12.0; // nominal battery voltage (12V)
    private static final double MAX_SHOOTER_RPM = 4800.0; // maximum shooter speed

    // ================= PIDF (custom controller) =================
    // PIDF for 2000-3500 RPM range
    private double kP_shooter_low = 0.006;
    private double kI_shooter_low = 0.0005;
    private double kD_shooter_low = 0.0002;
    private double kF_shooter_low = 0.00620;

    // PIDF for 3600+ RPM range
    private double kP_shooter_high = 0.007;
    private double kI_shooter_high = 0.0008;
    private double kD_shooter_high = 0.0003;
    private double kF_shooter_high = 0.00520;

    // Current active PIDF values (selected based on targetRPM)
    private double kP_shooter;
    private double kI_shooter;
    private double kD_shooter;
    private double kF_shooter;

    private PIDFMotorController leftController = null;
    private PIDFMotorController rightController = null;

    // ================= SERVO POSITIONS =================
    private final double ServoStart = 0.5;
    private final double ServoDown = 0.0;

    @Override
    public void runOpMode() {
        // HARDWARE MAP
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight   = hardwareMap.get(DcMotorEx.class, "shooterRight");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        transferBlocker = hardwareMap.servo.get("transferBlocker");
        Gate = hardwareMap.servo.get("Gate");

        // TURRET MOTOR & LIMELIGHT
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // DIRECTIONS
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // Use RUN_WITHOUT_ENCODER so we can drive power directly from our custom PIDF
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INIT POSITIONS
        transferBlocker.setPosition(ServoStart);
        Gate.setPosition(0.5);

        telemetry.addLine("✅ Robot Initialized");
        telemetry.addLine("Press Gamepad2 A to reset turret encoder");
        telemetry.addLine("Press Gamepad2 X to toggle auto-tracking");
        telemetry.update();

        if (AUTO_CENTER_ON_INIT) centerTurret();

        waitForStart();

        // instantiate custom PIDF controllers for shooters
        leftController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);
        rightController = new PIDFMotorController(kP_shooter, kI_shooter, kD_shooter, kF_shooter, TICKS_PER_REV);

        while (opModeIsActive()) {
            // RESET TURRET ENCODER
            if (gamepad2.a && !turretInitialized) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretInitialized = true;
                sleep(200);
            }

            // DRIVE
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            // INTAKE

            if (gamepad1.circle && !lastCircle) intakeOn = !intakeOn;
            lastCircle = gamepad1.circle;

            if (intakeOn) {
                middleTransfer.setPower(1);
            } else {
                middleTransfer.setPower(0);
            }


            // SHOOTER INPUTS
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean tri = gamepad1.triangle;

            if (tri && !lastTriangle) {
                // kill shooter immediately
                shooterOn = false;
                shooterKilled = true;
                targetRPM = 0;
                Gate.setPosition(0.5);
            }
            if (lb && !lastLeftBumper) {
                // left bumper sets high speed
                shooterOn = true;
                shooterKilled = false;
                targetRPM = MAX_SHOOTER_RPM;  // 4800 RPM (max speed)
                Gate.setPosition(0.27);
            }
            if (rb && !lastRightBumper) {
                // right bumper sets lower speed
                shooterOn = true;
                shooterKilled = false;
                targetRPM = 3000;
                Gate.setPosition(0.27);
            }

            lastLeftBumper = lb;
            lastRightBumper = rb;
            lastTriangle = tri;

            // no longer using ramp logic; targetRPM is constant once set

            if (gamepad2.dpad_down) Gate.setPosition(0.27);
            if (gamepad2.dpad_up) Gate.setPosition(0.5);

            // Apply custom PIDF controller to shooter motors (powered 0..1) with voltage compensation
            if (!shooterOn || shooterKilled || targetRPM <= 0) {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                if (leftController != null) leftController.reset();
                if (rightController != null) rightController.reset();
            } else {
                // Select PIDF tuning based on target RPM
                if (targetRPM >= 3600) {
                    kP_shooter = kP_shooter_high;
                    kI_shooter = kI_shooter_high;
                    kD_shooter = kD_shooter_high;
                    kF_shooter = kF_shooter_high;
                } else {
                    kP_shooter = kP_shooter_low;
                    kI_shooter = kI_shooter_low;
                    kD_shooter = kD_shooter_low;
                    kF_shooter = kF_shooter_low;
                }

                // update tunings live with selected PIDF set
                if (leftController != null) leftController.setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);
                if (rightController != null) rightController.setTunings(kP_shooter, kI_shooter, kD_shooter, kF_shooter);

                // Get current battery voltage for compensation
                double currentVoltage = voltageSensor.getVoltage();

                // Compute power with voltage compensation
                double powerLeft = leftController.computePowerForTargetRPMWithVoltageCompensation(
                        targetRPM, shooterLeft.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);
                double powerRight = rightController.computePowerForTargetRPMWithVoltageCompensation(
                        targetRPM, shooterRight.getVelocity(), currentVoltage, NOMINAL_VOLTAGE);

                shooterLeft.setPower(powerLeft);
                shooterRight.setPower(powerRight);

                telemetry.addData("Shooter Power", String.format("%.2f / %.2f", powerLeft, powerRight));
                // show actual encoder-based RPM for each motor
                double rpmLeftActual = shooterLeft.getVelocity() * 60.0 / TICKS_PER_REV;
                double rpmRightActual = shooterRight.getVelocity() * 60.0 / TICKS_PER_REV;
                telemetry.addData("LeftRPM", (int)rpmLeftActual);
                telemetry.addData("RightRPM", (int)rpmRightActual);
                telemetry.addData("Battery Voltage", String.format("%.1f V", currentVoltage));
            }

            updateTurretTracking();
            updateTelemetry();
        }
        limelight.stop();
        turretMotor.setPower(0);
    }

    private void centerTurret() {
        int currentPos = 0;
        int error = CENTER_POSITION - currentPos;
        while (Math.abs(error) > INIT_TOLERANCE && opModeIsActive()) {
            currentPos = turretMotor.getCurrentPosition();
            error = CENTER_POSITION - currentPos;
            turretMotor.setPower(Math.signum(error) * INIT_POWER);
            sleep(20);
        }
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretInitialized = true;
    }

    private void updateTurretTracking() {
        if (gamepad2.cross) {
            autoTrackEnabled = !autoTrackEnabled;
            lastError = 0;
            framesWithoutTarget = 0;
            framesOnTarget = 0;
            wasSearching = false;
            sleep(200);
        }

        LLResult result = limelight.getLatestResult();
        int currentPos = turretMotor.getCurrentPosition();
        double outputPower = 0;
        String mode = "IDLE";

        // 1. MANUAL OVERRIDE
        if (Math.abs(gamepad2.right_stick_x) > 0.2) {
            mode = "MANUAL";
            outputPower = gamepad2.right_stick_x * 0.5;
            autoTrackEnabled = false;
            framesWithoutTarget = 0;
            framesOnTarget = 0;
            wasSearching = false;
        }
        // 2. AUTO TRACKING
        else if (autoTrackEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            double absTx = Math.abs(tx);
            framesWithoutTarget = 0;  // Reset debounce counter on valid target

            // SMOOTH transition on first detection (no harsh brake)
            if (wasSearching) {
                wasSearching = false;
                framesOnTarget = 0;
                // Let PID controller gradually reduce power instead of hard stop
            }

            // Choose PID gains based on error size
            double kP, kD, kF;
            if (absTx > ACQUIRE_THRESHOLD) {
                // FAR FROM TARGET: Aggressive gains
                kP = kP_acquire;
                kD = kD_acquire;
                kF = kF_acquire;
                mode = "TRACKING (ACQ)";
                framesOnTarget = 0;
            } else {
                // NEAR TARGET: Gentle gains
                kP = kP_hold;
                kD = kD_hold;
                kF = kF_hold;
                mode = "LOCKED 🎯";
                framesOnTarget++;
            }

            // Only apply power outside deadzone
            if (absTx > DEADZONE_DEGREES) {
                outputPower = -((kP * tx) + (kD * (tx - lastError)));

                // Feed-forward only in acquisition mode
                if (absTx > ACQUIRE_THRESHOLD && Math.abs(outputPower) < kF) {
                    outputPower = -Math.signum(tx) * kF;
                }
            } else {
                // Within deadzone AND stable for 5+ frames = perfect lock
                if (framesOnTarget > 5) {
                    outputPower = 0; // ZERO power when perfectly aligned
                    mode = "LOCKED ✅";
                }
            }

            lastError = tx;
        }
        // 3. FAST SEARCH (or debouncing target loss)
        else if (autoTrackEnabled) {
            framesWithoutTarget++;

            // If we just lost target, wait a few frames before searching
            if (framesWithoutTarget < TARGET_LOSS_THRESHOLD) {
                mode = "DEBOUNCE (lost target)";
                outputPower = 0; // Coast to a stop gently
            } else {
                mode = "SEARCHING";
                lastError = 0;
                wasSearching = true;
                framesOnTarget = 0;

                // Reverse at limits
                if (currentPos >= RIGHT_LIMIT) {
                    scanningRight = false;  // Switch direction when hitting limit
                } else if (currentPos <= LEFT_LIMIT) {
                    scanningRight = true;   // Switch direction when hitting limit
                }

                // FULL SPEED search
                double searchPower = FAST_SEARCH_POWER;
                outputPower = scanningRight ? searchPower : -searchPower;
                mode += scanningRight ? " →" : " ←";
            }
        }

        // Clamp power
        outputPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, outputPower));

        turretMotor.setPower(outputPower);
        telemetry.addData("Mode", mode);
        telemetry.addData("Error", result != null && result.isValid() ?
                String.format("%.2f°", result.getTx()) : "N/A");
        telemetry.addData("Frames Stable", framesOnTarget);
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Turret Mode", autoTrackEnabled ? "AUTO" : "MANUAL");
        telemetry.addData("Encoder", turretMotor.getCurrentPosition());
        telemetry.addData("Target Found", (result != null && result.isValid()));
        telemetry.addData("Shooter RPM", (int)targetRPM);
        telemetry.update();
    }
}