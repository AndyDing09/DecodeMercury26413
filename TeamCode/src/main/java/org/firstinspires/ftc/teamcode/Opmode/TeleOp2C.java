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
@TeleOp(name = "TeleOp Unified 2 controller", group = "TeleOp")
public class TeleOp2C extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private TurretInterface turret;
    private Follower follower;
    private VoltageSensor voltageSensor;

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

        if (IS_RED_ALLIANCE) {
            turret = new TurretRed(hardwareMap);
        } else {
            turret = new TurretBlue(hardwareMap);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Init loop: press dpad_up for RED, dpad_down for BLUE on gamepad1
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                IS_RED_ALLIANCE = true;
                turret.setAlliance(true);
            }
            if (gamepad1.dpad_down) {
                IS_RED_ALLIANCE = false;
                turret.setAlliance(false);
            }

            telemetry.addLine("Ready. Auto-calc shooter from odometry.");
            telemetry.addLine(">> DPAD UP = RED | DPAD DOWN = BLUE <<");
            telemetry.addData("Alliance", IS_RED_ALLIANCE ? "RED" : "BLUE");
            telemetry.addData("Start Pose", String.format("(%.0f, %.0f) hdg=%.0f",
                    START_X, START_Y, START_HEADING));
            telemetry.update();
        }

        shooter.initControllers();

        while (opModeIsActive()) {
            follower.update();

            // --- Gamepad 1: Drive + Turret recovery ---
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            turret.update(gamepad1.left_stick_x);  // Manual turret on GP1 left stick X

            if (gamepad1.a) {
                turret.resetEncoder();             // Reset turret encoder on GP1 A
            }

            // --- Gamepad 2: Intake + Shooter ---
            intake.update(gamepad2.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);
            shooter.handleShooterInput(gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);

            // Auto-calc RPM and hood from odometry
            shooter.updateFromOdometry(follower, telemetry);
            shooter.updatePIDF(voltageSensor, telemetry);

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