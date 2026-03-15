package org.firstinspires.ftc.teamcode.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretInterface;
import org.firstinspires.ftc.teamcode.subsystems.TurretRed;
import org.firstinspires.ftc.teamcode.testing.TurretBlue;

@Config
@TeleOp(name = "Turret Test", group = "Testing")
public class TurretTest extends LinearOpMode {

    private TurretInterface turret;

    public static boolean IS_RED_ALLIANCE = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (IS_RED_ALLIANCE) {
            turret = new TurretRed(hardwareMap);
        } else {
            turret = new TurretBlue(hardwareMap);
        }

        // Init loop: pick alliance before starting
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                IS_RED_ALLIANCE = true;
                turret.setAlliance(true);
            }
            if (gamepad1.dpad_down) {
                IS_RED_ALLIANCE = false;
                turret.setAlliance(false);
            }

            telemetry.addLine(">> DPAD UP = RED | DPAD DOWN = BLUE <<");
            telemetry.addData("Alliance", IS_RED_ALLIANCE ? "RED" : "BLUE");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Right stick X for manual override, auto-tracks otherwise
            turret.update(gamepad1.right_stick_x);

            if (gamepad1.a) {
                turret.resetEncoder();
            }

            turret.addTelemetry(telemetry);
            telemetry.update();
        }

        turret.stop();
    }
}