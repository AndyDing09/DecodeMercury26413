package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "GateTest", group = "Linear Opmode")
public class GateTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware - "servo1" must match your Driver Station configuration
        // Declare the servo object
        Servo gate = hardwareMap.get(Servo.class, "gate");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // The joystick y-axis goes from -1 (up) to 1 (down).
            // We scale it so -1 becomes 0 and 1 becomes 1 for the servo.
            double stickPosition = (gamepad1.left_stick_y + 1.0) / 2.0;

            // Set the servo position
            gate.setPosition(stickPosition);

            // Send telemetry data to the Driver Station
            telemetry.addData("Joystick Raw", gamepad1.left_stick_y);
            telemetry.addData("Servo Target Position", stickPosition);
            telemetry.addData("Actual Servo Pos", gate.getPosition());
            telemetry.update();
        }
    }
}