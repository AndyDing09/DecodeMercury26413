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
@TeleOp(name = "TeleOp Main 2", group = "TeleOp")
public class TeleOp2controller extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private TurretInterface turret;
    private Follower follower;
    private VoltageSensor voltageSensor;

    private volatile double manualTurretPower = 0;

    public static boolean IS_RED_ALLIANCE = true;
    private boolean lastShooterSpinUp = false;
    public static double START_X = 107;
    public static double START_Y = 47;
    public static double START_HEADING = -39.3;

    // Relocalization target pose
    private static final double RELOC_X       = 8.9;
    private static final double RELOC_Y       = 135.0;
    private static final double RELOC_HEADING = 0.0;

    private boolean lastRelocButton = false; // for edge detection

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

        // Turret runs on its own thread, unaffected by main loop speed
        Thread turretThread = new Thread(() -> {
            while (opModeIsActive()) {
                turret.update(manualTurretPower);
            }
        });
        turretThread.start();

        while (opModeIsActive()) {
            follower.update();

            // --- Relocalization (rising-edge on circle button) ---
            boolean relocPressed = gamepad1.circle;
            if (relocPressed && !lastRelocButton) {
                follower.setPose(new Pose(RELOC_X, RELOC_Y, Math.toRadians(RELOC_HEADING)));
            }
            lastRelocButton = relocPressed;

            double speed = 0.95;

            drivetrain.drive(-gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, gamepad1.right_stick_x * speed);

            manualTurretPower = gamepad1.right_stick_x;

            if (gamepad1.a) turret.resetEncoder();

            intake.setSlowMode(gamepad2.a);

            intake.update(gamepad2.right_bumper, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);
            intake.update(gamepad2.left_bumper, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE, gamepad2.dpad_down);

            shooter.startShooterOnly(gamepad2.left_bumper, lastShooterSpinUp);
            lastShooterSpinUp = gamepad2.left_bumper;

            shooter.handleShooterInput(gamepad2.left_bumper, gamepad2.a, gamepad2.b, intake);

            shooter.updateOuttakeSequence(intake, voltageSensor);
            shooter.updateFromOdometry(follower, telemetry);
            shooter.updatePIDF(voltageSensor, telemetry);

            turret.addTelemetry(telemetry);
            telemetry.addData("Shooter RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.addData("Alliance", IS_RED_ALLIANCE ? "RED" : "BLUE");
            telemetry.update();
        }

        turret.stop();
    }
}