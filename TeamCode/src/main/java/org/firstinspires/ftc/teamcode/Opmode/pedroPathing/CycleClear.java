package org.firstinspires.ftc.teamcode.Opmode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

@Autonomous(name = "CycleClear", group = "Testing")
public class CycleClear extends LinearOpMode {

    // =======================
    // Hardware
    // =======================
    private DcMotor middleTransfer;

    // =======================
    // Poses (exact values from ArnieautocloseredPathing)
    // =======================
    private final Pose shootPose         = new Pose(84,  84,  Math.toRadians(45));
    private final Pose PreclearPose      = new Pose(120, 72,  Math.toRadians(90));
    private final Pose clearPose         = new Pose(132, 72,  Math.toRadians(90));
    private final Pose PickfromclearPose = new Pose(134, 56,  Math.toRadians(30));

    // =======================
    // Pedro Pathing
    // =======================
    private Follower follower;

    // Paths
    private PathChain toClear;          // shoot -> preclear -> clear
    private PathChain toPickFromClear;  // clear -> pickfromclear
    private PathChain backToShoot;      // pickfromclear -> shoot

    // State machine
    private final Timer actionTimer = new Timer();
    private int state = 0;

    @Override
    public void runOpMode() {

        // =======================
        // Hardware init
        // =======================
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);

        // =======================
        // Pedro init
        // =======================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(shootPose);

        buildPaths();

        telemetry.addLine("✅ CycleClear Initialized");
        telemetry.addLine("Starting from Shoot Pose");
        telemetry.update();

        waitForStart();

        setState(0);

        while (opModeIsActive()) {
            follower.update();

            updateStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("H(deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Intake", middleTransfer.getPower() > 0 ? "ON" : "OFF");
            telemetry.update();
        }
    }

    private void buildPaths() {
        // shoot -> preclear -> clear
        toClear = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PreclearPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PreclearPose.getHeading())
                .addPath(new BezierLine(PreclearPose, clearPose))
                .setLinearHeadingInterpolation(PreclearPose.getHeading(), clearPose.getHeading())
                .build();

        // clear -> pickfromclear
        toPickFromClear = follower.pathBuilder()
                .addPath(new BezierLine(clearPose, PickfromclearPose))
                .setLinearHeadingInterpolation(clearPose.getHeading(), PickfromclearPose.getHeading())
                .build();

        // pickfromclear -> shoot
        backToShoot = follower.pathBuilder()
                .addPath(new BezierLine(PickfromclearPose, shootPose))
                .setLinearHeadingInterpolation(PickfromclearPose.getHeading(), shootPose.getHeading())
                .build();
    }

    private void updateStateMachine() {
        switch (state) {

            // ── Drive: shoot -> preclear -> clear ───────────────────────────
            case 0:
                follower.followPath(toClear, true);
                setState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setState(2);
                }
                break;

            // ── Drive: clear -> pickfromclear ───────────────────────────────
            case 2:
                follower.followPath(toPickFromClear, true);
                setState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    // Arrived at PickfromclearPose — turn on intake and start timer
                    middleTransfer.setPower(1.0);
                    actionTimer.resetTimer();
                    setState(4);
                }
                break;

            // ── Wait 5 seconds with intake running ──────────────────────────
            case 4:
                telemetry.addLine("Intaking... waiting 5s");
                if (actionTimer.getElapsedTimeSeconds() > 5.0) {
                    // Turn off intake and head back to shoot
                    middleTransfer.setPower(0);
                    setState(5);
                }
                break;

            // ── Drive: pickfromclear -> shoot ───────────────────────────────
            case 5:
                follower.followPath(backToShoot, true);
                setState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    setState(-1); // Done
                }
                break;

            default:
                // Idle / done
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
    }
}