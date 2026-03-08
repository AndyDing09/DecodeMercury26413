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
import org.firstinspires.ftc.teamcode.subsystems.TurretRed;

@TeleOp(name = "TeleOp 3", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private TurretRed turret;
    private Follower follower;
    private VoltageSensor voltageSensor;

    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(0));

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new TurretRed(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        telemetry.addLine("Ready. LB=4500 RPM, RB=3000 RPM.");
        telemetry.update();

        waitForStart();

        shooter.initControllers();

        while (opModeIsActive()) {
            // 1. Update odometry (Reads Pinpoint/Encoders)
            follower.update();

            // 3. Drive Control
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // 4. Intake Control
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // 5. Shooter Control (fixed RPM from bumpers — LB=4500, RB=3000)
            shooter.handleHoodInput(gamepad1.y, gamepad1.x);
            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);
            shooter.updatePIDF(voltageSensor, telemetry);

            // 6. Turret Control
            if (gamepad2.a) {
                turret.resetEncoder();
            }
            turret.update();

            // 7. Telemetry output
            turret.addTelemetry(telemetry);

            telemetry.addLine("===== SHOOTER =====");
            telemetry.addData("Target RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.addData("Shooter Active", shooter.isShooterOn() ? "YES" : "no");

            telemetry.update();
        }

        turret.stop();
    }
}