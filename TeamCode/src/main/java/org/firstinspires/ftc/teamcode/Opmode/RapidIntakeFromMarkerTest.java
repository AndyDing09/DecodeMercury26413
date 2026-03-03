package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * RapidIntakeFromMarkerTest
 *
 * Autonomous test program for Team 2.
 * Runs immediately on Start — no controller input needed.
 *
 * Sequence:
 *   1. Reset transfer servo to home position
 *   2. Start both intake motors
 *   3. Drive in the configured direction at the configured power
 *      for the configured duration (drive + intake simultaneously)
 *   4. Stop intake motors and drive motors
 *
 * Tune the constants in the CONFIGURATION block below.
 *
 * File location: Opmode/RapidIntakeFromMarkerTest.java
 */
@Autonomous(name = "Rapid Intake From Marker Test", group = "Testing")
public class RapidIntakeFromMarkerTest extends LinearOpMode {

    // ══════════════════════════════════════════════════════════════
    // CONFIGURATION — tune these without touching anything else
    // ══════════════════════════════════════════════════════════════

    /** Power applied to the drive motors while intaking (0.0 – 1.0) */
    private static final double DRIVE_POWER = 0.5;

    /** How long (seconds) to drive + intake before stopping */
    private static final double DRIVE_DURATION_SECS = 0.9;

    /**
     * Direction to move while intaking.
     * Options: FORWARD, STRAFE_LEFT, STRAFE_RIGHT, NONE
     */
    private static final DriveDirection DRIVE_DIRECTION = DriveDirection.FORWARD;

    /** Pause (ms) after resetting the transfer servo before starting intake */
    private static final long TRANSFER_RESET_DELAY_MS = 300;

    // ══════════════════════════════════════════════════════════════
    // SERVO POSITIONS (from Transfe_Intake.java)
    // ══════════════════════════════════════════════════════════════
    private static final double SERVO_HOME     = 0.5; // transferBlocker start position (TeleOp2: ServoStart)
    private static final double SERVO_EXTENDED = 0.0; // transferBlocker down position

    // ══════════════════════════════════════════════════════════════
    // DRIVE DIRECTION ENUM
    // ══════════════════════════════════════════════════════════════
    public enum DriveDirection {
        FORWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        NONE
    }

    // ══════════════════════════════════════════════════════════════
    // HARDWARE
    // ══════════════════════════════════════════════════════════════
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor middleTransfer;
    private Servo   transferBlocker;

    private ElapsedTime timer = new ElapsedTime();

    // ══════════════════════════════════════════════════════════════
    // MAIN
    // ══════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() throws InterruptedException {

        // ── Hardware map ───────────────────────────────────────────
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        transferBlocker = hardwareMap.servo.get("transferBlocker");

        // ── Motor directions (matching Transfe_Intake.java & TeleOp2.java) ──
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        middleTransfer.setDirection(DcMotor.Direction.FORWARD); // matches TeleOp2

        // ── Zero-power behavior ────────────────────────────────────
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Init position ──────────────────────────────────────────
        transferBlocker.setPosition(SERVO_HOME);

        telemetry.addLine("✅ Ready — press Start to run Rapid Intake");
        telemetry.addData("Direction", DRIVE_DIRECTION.name());
        telemetry.addData("Drive Power", DRIVE_POWER);
        telemetry.addData("Duration (s)", DRIVE_DURATION_SECS);
        telemetry.update();

        waitForStart();

        // ══════════════════════════════════════════════════════════
        // STEP 1: Reset transfer servo to home, then brief pause
        // ══════════════════════════════════════════════════════════
        telemetry.addLine("Step 1: Resetting transfer servo...");
        telemetry.update();

        transferBlocker.setPosition(SERVO_HOME);
        safeSleep(TRANSFER_RESET_DELAY_MS);

        // ══════════════════════════════════════════════════════════
        // STEP 2: Start intake motors
        // ══════════════════════════════════════════════════════════
        telemetry.addLine("Step 2: Starting intake...");
        telemetry.update();

        middleTransfer.setPower(1.0);

        // ══════════════════════════════════════════════════════════
        // STEP 3: Drive + intake simultaneously for DRIVE_DURATION_SECS
        // ══════════════════════════════════════════════════════════
        telemetry.addData("Step 3: Driving + intaking", DRIVE_DIRECTION.name());
        telemetry.update();

        setDrivePower(DRIVE_DIRECTION, DRIVE_POWER);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < DRIVE_DURATION_SECS) {
            telemetry.addData("Elapsed (s)", String.format("%.1f / %.1f", timer.seconds(), DRIVE_DURATION_SECS));
            telemetry.addData("Intake", "RUNNING");
            telemetry.addData("Drive", DRIVE_DIRECTION.name());
            telemetry.update();
        }

        // ══════════════════════════════════════════════════════════
        // STEP 4: Stop everything
        // ══════════════════════════════════════════════════════════
        stopDrive();
        middleTransfer.setPower(0);

        telemetry.addLine("✅ Done — intake and drive stopped.");
        telemetry.update();

        // Hold telemetry visible until OpMode is ended manually
        while (opModeIsActive()) {
            idle();
        }
    }

    // ══════════════════════════════════════════════════════════════
    // HELPERS
    // ══════════════════════════════════════════════════════════════

    /**
     * Sets mecanum drive powers for the given direction.
     *
     * Mecanum signs (robot-centric, with direction conventions from TeleOp2):
     *   FORWARD      FL+ BL+ FR+ BR+
     *   STRAFE_LEFT  FL- BL+ FR+ BR-
     *   STRAFE_RIGHT FL+ BL- FR- BR+
     */
    private void setDrivePower(DriveDirection direction, double power) {
        double fl, bl, fr, br;

        switch (direction) {
            case FORWARD:
                fl =  power; bl =  power;
                fr =  power; br =  power;
                break;
            case STRAFE_LEFT:
                fl = -power; bl =  power;
                fr =  power; br = -power;
                break;
            case STRAFE_RIGHT:
                fl =  power; bl = -power;
                fr = -power; br =  power;
                break;
            case NONE:
            default:
                fl = 0; bl = 0; fr = 0; br = 0;
                break;
        }

        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /** Sleep that exits early if the OpMode is stopped. */
    private void safeSleep(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
    }
}

