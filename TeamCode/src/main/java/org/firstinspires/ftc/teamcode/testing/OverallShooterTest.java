package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name = "Shooter Tuner", group = "Testing")
public class OverallShooterTest extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private VoltageSensor voltageSensor;

    private static final double STARTING_RPM  = 2000;
    private static final double RPM_INCREMENT = 100;
    private static final double RPM_DECREMENT = 25;
    private static final double HOOD_STEP     = 0.05;

    private static final double GATE_OPEN   = 0.26;
    private static final double GATE_CLOSED = 0.1;

    private double targetRPM = STARTING_RPM;

    private boolean lastDpadUp      = false;
    private boolean lastDpadDown    = false;
    private boolean lastTriangle    = false;
    private boolean lastX           = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        drivetrain    = new Drivetrain(hardwareMap);
        intake        = new Intake(hardwareMap);
        shooter       = new Shooter(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Shooter Tuner Ready");
        telemetry.addLine("DPAD UP = Hood Up  |  DPAD DOWN = Hood Down");
        telemetry.addLine("Triangle = RPM +100  |  X = RPM -50");
        telemetry.addLine("R1 = Gate Open (hold)");
        telemetry.update();

        waitForStart();

        shooter.initControllers();

        // Turn shooter on immediately so it's ready from match start
        shooter.setShooterOn(true);
        shooter.setTargetRPM(targetRPM);

        while (opModeIsActive()) {

            // --- Drive ---
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // --- Intake ---
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // --- Hood Up ---
            if (gamepad1.dpad_up && !lastDpadUp) {
                double newHood = Math.min(1.0, shooter.getHoodAnglePos() + HOOD_STEP);
                shooter.setHoodAnglePos(newHood);
            }

            // --- Hood Down ---
            if (gamepad1.dpad_down && !lastDpadDown) {
                double newHood = Math.max(0.0, shooter.getHoodAnglePos() - HOOD_STEP);
                shooter.setHoodAnglePos(newHood);
            }

            // --- RPM Up (Triangle) ---
            if (gamepad1.triangle && !lastTriangle) {
                targetRPM += RPM_INCREMENT;
                shooter.setTargetRPM(targetRPM);
            }

            // --- RPM Down (X) ---
            if (gamepad1.cross && !lastX) {
                targetRPM = Math.max(0, targetRPM - RPM_DECREMENT);
                shooter.setTargetRPM(targetRPM);
            }

            // --- Gate: hold R1 to open, release to close ---
            if (gamepad1.right_bumper && !lastRightBumper) {
                shooter.getGate().setPosition(GATE_OPEN);
            } else if (!gamepad1.right_bumper && lastRightBumper) {
                shooter.getGate().setPosition(GATE_CLOSED);
            }

            // Save last states
            lastDpadUp      = gamepad1.dpad_up;
            lastDpadDown    = gamepad1.dpad_down;
            lastTriangle    = gamepad1.triangle;
            lastX           = gamepad1.cross;
            lastRightBumper = gamepad1.right_bumper;

            shooter.updatePIDF(voltageSensor, telemetry);

            telemetry.addData("Target RPM", (int) targetRPM);
            telemetry.addData("Hood Pos",   String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.addData("Gate",       gamepad1.right_bumper ? "OPEN" : "CLOSED");
            telemetry.update();
        }
    }
}