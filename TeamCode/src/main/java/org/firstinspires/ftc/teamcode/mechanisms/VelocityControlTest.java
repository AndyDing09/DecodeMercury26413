package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Velocity Control Test", group="Testing")
public class VelocityControlTest extends LinearOpMode {

    // GoBilda motor
    private DcMotorEx shooterMotor;

    // RPM control
    private double targetRPM = 1000;      // start speed
    private final double RPM_INCREMENT = 50; // change step
    private final double MAX_RPM = 6000;  // depends on motor type
    private final double MIN_RPM = 0;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "motor1");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Velocity Control Initialized");
        telemetry.addLine("Use ▲/Δ (triangle) to increase RPM, X (square) to decrease RPM");
        telemetry.addLine("Press ▶ (circle) to stop motor");
        telemetry.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            // Adjust RPM using PS5 buttons
            if (gamepad1.triangle) {
                targetRPM += RPM_INCREMENT;
                sleep(200); // debounce
            } else if (gamepad1.square) {
                targetRPM -= RPM_INCREMENT;
                sleep(200);
            }

            // Clamp RPM
            targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));

            // Convert RPM to ticks per second (GoBilda 312RPM motor = 537.7 ticks/rev)
            double ticksPerSec = (targetRPM * 537.7) / 60.0;

            // Set velocity
            shooterMotor.setVelocity(ticksPerSec);

            // Stop motor on Circle
            if (gamepad1.circle) {
                shooterMotor.setPower(0);
                targetRPM = 0;
            }

            // Telemetry output
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual Velocity (ticks/s)", shooterMotor.getVelocity());
            telemetry.addData("Actual RPM", (shooterMotor.getVelocity() * 60) / 537.7);
            telemetry.addData("Elapsed Time", "%.2f s", timer.seconds());
            telemetry.update();
        }
    }
}
