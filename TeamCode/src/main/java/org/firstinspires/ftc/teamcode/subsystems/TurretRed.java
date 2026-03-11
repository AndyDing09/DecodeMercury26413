package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretRed implements TurretInterface {

    private Servo axonServo;
    private Limelight3A limelight;

    private static final double GEAR_RATIO = 45.0 / 110.0;
    private static final double kP = 0.0001;
    private static final double kD = 0.001;

    private double lastError = 0;

    public TurretRed(HardwareMap hardwareMap) {
        axonServo = hardwareMap.get(Servo.class, "turntable");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        axonServo.setPosition(0.5);
    }

    @Override
    public void update(double manualPower) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            lastError = 0;
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
    }

    @Override
    public void setAlliance(boolean isRed) {
        limelight.pipelineSwitch(isRed ? 1 : 0);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== TURRET RED =====");
        telemetry.addData("Servo Position", String.format("%.4f", axonServo.getPosition()));
        telemetry.addData("kP / kD", String.format("%.5f / %.4f", kP, kD));
    }

    @Override
    public void resetEncoder() {
        axonServo.setPosition(0.5);
    }

    @Override
    public void stop() {
        limelight.stop();
        axonServo.setPosition(0.5);
    }
}