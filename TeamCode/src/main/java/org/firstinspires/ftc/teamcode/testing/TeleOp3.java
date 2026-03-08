package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.testing.Turret;

@TeleOp(name = "TeleOp with Turret Tracking - Fixed", group = "TeleOp")
public class TeleOp3 extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Follower follower;
    private VoltageSensor voltageSensor;

    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(0));

    @Override
    public void runOpMode() {
        drivetrain    = new Drivetrain(hardwareMap);
        intake        = new Intake(hardwareMap);
        shooter       = new Shooter(hardwareMap);
        turret        = new Turret(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        telemetry.addLine("Ready. Auto-calc shooter from odometry.");
        telemetry.addData("Start Pose", String.format("(%.0f, %.0f) hdg=%.0f",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading())));
        telemetry.update();

        waitForStart();

        shooter.initControllers();

        while (opModeIsActive()) {
            // Update odometry (Pinpoint reads pods directly, no motor conflict)
            follower.update();

            // Drive
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Optional manual zeroing
            if (gamepad2.a) {
                turret.resetEncoder();
            }

            // Intake
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // Shooter input + outtake sequence
            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);

            // Auto-calc RPM and hood from odometry (overrides when shooter is on)
            shooter.updateFromOdometry(follower, telemetry);
            shooter.updatePIDF(voltageSensor, telemetry);

            // --- Clean Turret Integration ---
            // This single line handles all automated Limelight tracking, holding, and searching
            turret.update();

            // Telemetry
            turret.addTelemetry(telemetry);
            telemetry.addData("Shooter RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.update();
        }

        turret.stop();
    }
}