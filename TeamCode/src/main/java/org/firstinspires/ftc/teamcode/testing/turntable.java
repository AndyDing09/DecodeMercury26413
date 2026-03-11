package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight Servo Tuner", group = "TeleOp")
public class turntable extends OpMode {

    private Servo axonServo;
    private Limelight3A limelight;

    private static final double GEAR_RATIO = 45.0 / 110.0;
    private static final double kP = 0.0001;
    private static final double kD = 0.001;

    private double lastError = 0;

    @Override
    public void init() {
        axonServo = hardwareMap.get(Servo.class, "turntable");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        axonServo.setPosition(0.5);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult(); // fixed

        if (result == null || !result.isValid()) {
            telemetry.addData("Target", "None");
            lastError = 0;
            telemetry.update();
            return;
        }

        double tx = result.getTx();

        double error = tx;
        double derivative = error - lastError;
        lastError = error;

        double rawCorrection = (kP * error) + (kD * derivative);
        double servoCorrection = rawCorrection / GEAR_RATIO;

        double newPosition = axonServo.getPosition() - servoCorrection;
        newPosition = Math.max(0.0, Math.min(1.0, newPosition));

        axonServo.setPosition(newPosition);

        telemetry.addData("TX (error)", tx);
        telemetry.addData("Servo Position", newPosition);
        telemetry.addData("Correction", servoCorrection);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        axonServo.setPosition(0.5);
    }
}