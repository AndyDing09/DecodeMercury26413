package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="turntable", group="Testing")
public class turntable extends LinearOpMode {

    private Servo turntable;
    public static double position = 0.5;

    @Override
    public void runOpMode() {
        turntable = hardwareMap.servo.get("turntable");
        turntable.setPosition(position);

        boolean lastDpadUp   = false;
        boolean lastDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean currentDpadUp   = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // Rising-edge detection: 0.01 per button tap
            if (currentDpadUp && !lastDpadUp) {
                position += 0.01;
            } else if (currentDpadDown && !lastDpadDown) {
                position -= 0.01;
            }

            // Clamp position between 0.0 and 1.0
            position = Math.max(0.0, Math.min(1.0, position));

            turntable.setPosition(position);

            lastDpadUp   = currentDpadUp;
            lastDpadDown = currentDpadDown;

            telemetry.addData("Turntable Position", "%.2f", position);
            telemetry.update();
        }
    }
}