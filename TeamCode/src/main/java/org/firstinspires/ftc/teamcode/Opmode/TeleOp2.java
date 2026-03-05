package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TeleOp with Turret Tracking - Fixed", group = "TeleOp")
public class TeleOp2 extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() {
        drivetrain    = new Drivetrain(hardwareMap);
        intake        = new Intake(hardwareMap);
        shooter       = new Shooter(hardwareMap);
        turret        = new Turret(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Ready. GP1: Y/X=Hood Up/Down | GP2: UP/DOWN=cycle param, LEFT/RIGHT=adjust, LB=coarse, A=reset enc, X=track");
        telemetry.update();

        turret.centerTurret(this);

        waitForStart();

        shooter.initControllers();

        while (opModeIsActive()) {
            if (gamepad2.a) turret.resetEncoder(this);

            // Drive
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Intake
            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);

            // Hood angle
            shooter.handleHoodInput(gamepad1.y, gamepad1.x);

            // Shooter input + outtake sequence
            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
            shooter.updateOuttakeSequence(intake, voltageSensor);
            shooter.updatePIDF(voltageSensor, telemetry);

            // Turret
            turret.updateTuning(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.left_bumper);
            turret.updateTracking(gamepad2.cross, gamepad2.right_stick_x, voltageSensor, telemetry, this);

            // Telemetry
            turret.addTelemetry(telemetry);
            telemetry.addData("Shooter RPM", (int) shooter.getTargetRPM());
            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
            telemetry.update();
        }

        turret.stop();
    }
}
