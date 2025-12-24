package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="FlyWheelTuner (RPM Targets)", group="Tuning")
public class FlyWheelTunerTutorial extends OpMode {

    private DcMotorEx flywheelMotor;
    private VoltageSensor voltageSensor;

    // goBILDA Yellow Jacket 6000 RPM (1:1) encoder: 28 ticks per output revolution
    private static final double TICKS_PER_REV = 28.0;

    // Targets in RPM (wheel RPM if direct drive)
    private double highRPM = 3350;
    private double lowRPM  = 0;
    private double targetRPM = highRPM;

    // PIDF to tune (I and D left at 0 for now)
    private double P = 180;
    private double F = 14.52; // decent starting ballpark for 6000rpm YJ

    // Step sizes for tuning (more realistic for REV PIDF than 10,1,0.1,...)
    private final double[] stepSizes = {5.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        telemetry.addLine("Init complete");
        telemetry.addLine("Triangle: toggle target RPM | Square: step size");
        telemetry.addLine("Dpad U/D: P +/- | Dpad L/R: F +/- | Circle: auto-guess F");
    }

    @Override
    public void loop() {
        // Toggle target RPM
        if (gamepad1.triangleWasPressed()) {
            targetRPM = (Math.abs(targetRPM - highRPM) < 1e-6) ? lowRPM : highRPM;
        }

        // Cycle step size
        if (gamepad1.squareWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Tune F
        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];

        // Tune P
        if (gamepad1.dpadUpWasPressed())   P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed()) P -= stepSizes[stepIndex];

        // Auto-guess F from current velocity (press Circle when near steady-state)
        // F ≈ 32767 / v (v in ticks/sec)
        if (gamepad1.circleWasPressed()) {
            double v = Math.max(1.0, flywheelMotor.getVelocity());
            F = 32767.0 / v;
            telemetry.addLine("Auto-guessed F from current velocity");
        }

        // Keep values sane
        if (P < 0) P = 0;
        if (F < 0) F = 0;

        applyPIDF();

        // Convert RPM -> ticks/sec for SDK velocity control
        double targetTicksPerSec = rpmToTicksPerSec(targetRPM);
        flywheelMotor.setVelocity(targetTicksPerSec);

        // Read actual velocity
        double curTicksPerSec = flywheelMotor.getVelocity();
        double curRPM = ticksPerSecToRpm(curTicksPerSec);

        double errorRPM = targetRPM - curRPM;

        telemetry.addData("Battery (V)", "%.2f", voltageSensor.getVoltage());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", curRPM);
        telemetry.addData("Error RPM", "%.0f", errorRPM);
        telemetry.addLine("---------------------------");
        telemetry.addData("Target (ticks/s)", "%.1f", targetTicksPerSec);
        telemetry.addData("Current (ticks/s)", "%.1f", curTicksPerSec);
        telemetry.addLine("---------------------------");
        telemetry.addData("P (Dpad U/D)", "%.4f", P);
        telemetry.addData("F (Dpad L/R)", "%.4f", F);
        telemetry.addData("Step (Square)", "%.4f", stepSizes[stepIndex]);
    }

    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }
}
