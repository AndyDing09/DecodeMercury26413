package org.firstinspires.ftc.teamcode.Opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

/**
 * RapidIntakeFromMarkerTest
 *
 * Autonomous test program for Team 2.
 * Runs immediately on Start — no controller input needed.
 *
 * Sequence:
 *   1. Reset transfer servo to home position
 *   2. Start intake motor
 *   3. Drive 24 inches in the positive X-direction using Pedro Pathing
 *      (intake runs the entire time)
 *   4. Stop intake motor
 *
 * File location: Opmode/RapidIntakeFromMarkerTest.java
 */
@Autonomous(name = "Rapid Intake From Marker Test", group = "Testing")
public class RapidIntakeFromMarkerTest extends LinearOpMode {

    // ══════════════════════════════════════════════════════════════
    // CONFIGURATION — tune these without touching anything else
    // ══════════════════════════════════════════════════════════════

    /** Starting pose of the robot (origin, facing 0°) */
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));

    /** End pose: 24 inches in the positive X-direction */
    private static final Pose END_POSE   = new Pose(24, 0, Math.toRadians(0));

    /** Pause (ms) after resetting the transfer servo before starting intake */
    private static final long TRANSFER_RESET_DELAY_MS = 300;

    // ══════════════════════════════════════════════════════════════
    // SERVO POSITIONS
    // ══════════════════════════════════════════════════════════════
    private static final double SERVO_HOME     = 0.5;
    private static final double SERVO_EXTENDED = 0.0;

    // ══════════════════════════════════════════════════════════════
    // HARDWARE
    // ══════════════════════════════════════════════════════════════
    private DcMotor middleTransfer;
    private Servo   transferBlocker;
    private Follower follower;

    // ══════════════════════════════════════════════════════════════
    // MAIN
    // ══════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {

        // ── Hardware map ───────────────────────────────────────────
        middleTransfer  = hardwareMap.get(DcMotor.class, "middleTransfer");
        transferBlocker = hardwareMap.servo.get("transferBlocker");

        middleTransfer.setDirection(DcMotor.Direction.FORWARD);

        // ── Pedro init ─────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // ── Build path ─────────────────────────────────────────────
        Path driveForward = new Path(new BezierLine(START_POSE, END_POSE));
        driveForward.setConstantHeadingInterpolation(START_POSE.getHeading());

        // ── Init position ──────────────────────────────────────────
        transferBlocker.setPosition(SERVO_HOME);

        telemetry.addLine("✅ Ready — press Start to run Rapid Intake");
        telemetry.addData("Drive distance", "24 inches (+X)");
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
        // STEP 2: Start intake
        // ══════════════════════════════════════════════════════════
        telemetry.addLine("Step 2: Starting intake...");
        telemetry.update();

        middleTransfer.setPower(1.0);

        // ══════════════════════════════════════════════════════════
        // STEP 3: Follow path 24 inches in +X while intake runs
        // ══════════════════════════════════════════════════════════
        telemetry.addLine("Step 3: Driving 24 inches + intaking...");
        telemetry.update();

        follower.followPath(driveForward);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X", String.format("%.2f", follower.getPose().getX()));
            telemetry.addData("Y", String.format("%.2f", follower.getPose().getY()));
            telemetry.addData("Intake", "RUNNING");
            telemetry.update();
        }

        // ══════════════════════════════════════════════════════════
        // STEP 4: Stop intake
        // ══════════════════════════════════════════════════════════
        middleTransfer.setPower(0);

        telemetry.addLine("✅ Done — intake stopped.");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    /** Sleep that exits early if the OpMode is stopped. */
    private void safeSleep(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
    }
}