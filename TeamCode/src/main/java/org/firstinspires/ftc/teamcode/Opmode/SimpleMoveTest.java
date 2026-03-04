package org.firstinspires.ftc.teamcode.Opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;

/**
 * SimpleMoveTest
 *
 * Diagnostic test — drives 12 inches in the positive X-direction from (0,0).
 * No intake, no servos, just pure pathing.
 */
@Autonomous(name = "Simple Move Tests", group = "Testing")
public class SimpleMoveTest extends LinearOpMode {

    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(0));
    private static final Pose END_POSE   = new Pose(84, 72, Math.toRadians(0));

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Path driveForward = new Path(new BezierLine(START_POSE, END_POSE));
        // driveForward.setConstantHeadingInterpolation(Math.toRadians(0));

        PathChain driveForward = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, END_POSE))
                .setConstantHeadingInterpolation(0)
                .build();


        telemetry.addLine("✅ Ready — press Start to drive 12 inches forward");
        telemetry.update();

        waitForStart();

        follower.followPath(driveForward);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X",   String.format("%.2f", follower.getPose().getX()));
            telemetry.addData("Y",   String.format("%.2f", follower.getPose().getY()));
            telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(follower.getPose().getHeading())));

            // To check if theres some issue with coefficients
            telemetry.addData("Drive Vector", follower.getDriveVector().toString());
            telemetry.addData("Corrective Vector", follower.getCorrectiveVector().toString());
            telemetry.addData("Heading Vector", follower.getHeadingVector().toString());

            telemetry.addData("Status", "DRIVING");
            telemetry.update();
        }

        // Stop all motors explicitly
        follower.breakFollowing();

        telemetry.addLine("✅ Done.");
        telemetry.addData("Final X", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Final Y", String.format("%.2f", follower.getPose().getY()));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}