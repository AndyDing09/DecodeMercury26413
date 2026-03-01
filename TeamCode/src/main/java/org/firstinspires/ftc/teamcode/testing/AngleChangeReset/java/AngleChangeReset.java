package org.firstinspires.ftc.teamcode.testing.AngleChangeReset.java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AngleChangeReset", group = "Testing")
public class AngleChangeReset extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize servos from hardware map
        Servo angleChange1 = hardwareMap.get(Servo.class, "angleChange1");
        Servo angleChange2 = hardwareMap.get(Servo.class, "angleChange2");

        waitForStart();

        if (opModeIsActive()) {
            angleChange1.setPosition(0.5);
            angleChange2.setPosition(0.5);

            telemetry.addData("Servo 1 Position", angleChange1.getPosition());
            telemetry.addData("Servo 2 Position", angleChange2.getPosition());
            telemetry.update();

            // Hold position until stop is pressed
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}