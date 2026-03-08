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
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TeleOp 3 - Odometry Shooter", group = "TeleOp")
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
        // Initialize subsystems
        drivetrain    = new Drivetrain(hardwareMap);
        intake        = new Intake(hardwareMap);
        shooter       = new Shooter(hardwareMap);
        turret        = new Turret(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        telemetry.addLine("Ready. Auto-calc shooter from odometry.");
        telemetry.update();

        waitForStart();

        shooter.initControllers();

        while (opModeIsActive()) {
            // 1. Update odometry (Reads Pinpoint/Encoders)
            follower.update();

            // 2. Obtain Robot Location from Follower
            Pose currentPose = follower.getPose();
            double robotX = currentPose.getX();
            double robotY = currentPose.getY();
            double robotHeading = currentPose.getHeading(); // Returns radians

            // 3. Drive Control
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // 4. Intake Control
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // 5. Shooter Control (Passing the Odometry data)
            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);

            // NOTE: You can pass the whole follower, OR update your shooter class to accept X and Y directly:
            shooter.updateFromOdometry(follower, telemetry);
            shooter.updatePIDF(voltageSensor, telemetry);

            // 6. Turret Control
            if (gamepad2.a) {
                turret.resetEncoder();
            }
            turret.update();

            // 7. Telemetry output
            turret.addTelemetry(telemetry);

            telemetry.addLine("===== ODOMETRY LOCATION =====");
            telemetry.addData("Robot X", String.format("%.1f inches", robotX));
            telemetry.addData("Robot Y", String.format("%.1f inches", robotY));
            telemetry.addData("Robot Heading", String.format("%.1f deg", Math.toDegrees(robotHeading)));

            telemetry.addLine("===== SHOOTER =====");
            telemetry.addData("Target RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Target Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));

            telemetry.update();
        }

        turret.stop();
    }
}