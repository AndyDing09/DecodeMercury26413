package org.firstinspires.ftc.teamcode.Opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TurretInterface;
import org.firstinspires.ftc.teamcode.subsystems.TurretRed;
import org.firstinspires.ftc.teamcode.testing.TurretBlue;

@Config
@TeleOp(name = "TeleOp Unified 2", group = "TeleOp")
public class TeleOp2 extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private TurretInterface turret;
    private Follower follower;
    private VoltageSensor voltageSensor;

    // Toggle this variable in Dashboard to switch fields
    public static boolean IS_RED_ALLIANCE = true;

    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING = 0;

    @Override
    public void runOpMode() {
        drivetrain    = new Drivetrain(hardwareMap);
        intake        = new Intake(hardwareMap);
        shooter       = new Shooter(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize the correct Turret based on the alliance variable
        if (IS_RED_ALLIANCE) {
            turret = new TurretRed(hardwareMap);
        } else {
            turret = new TurretBlue(hardwareMap);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready. Auto-calc shooter from odometry.");
        telemetry.addData("Alliance", IS_RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Start Pose", String.format("(%.0f, %.0f) hdg=%.0f",
                START_X, START_Y, START_HEADING));
        telemetry.update();

        waitForStart();

        shooter.initControllers();

        while (opModeIsActive()) {
            // Update odometry (Pinpoint reads pods directly, no motor conflict)
            follower.update();

            // Drive
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad2.a) {
                turret.resetEncoder();
            }

            // Intake
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // Shooter input + outtake sequence
            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);

            // Auto-calc RPM and hood from odometry (overrides when shooter is on)
            shooter.updateFromOdometry(follower,telemetry);

            shooter.updatePIDF(voltageSensor, telemetry);

            // Turret update (includes manual control via right_stick_x)
            turret.update(gamepad2.right_stick_x);

            // Telemetry
            turret.addTelemetry(telemetry);
            telemetry.addData("Shooter RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.addData("Alliance", IS_RED_ALLIANCE ? "RED" : "BLUE");
            telemetry.update();
        }

        turret.stop();
    }
}