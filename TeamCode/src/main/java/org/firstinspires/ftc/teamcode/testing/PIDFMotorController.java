package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFMotorController {

    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;

    // Anti-windup: clamp integral accumulator
    private static final double MAX_INTEGRAL = 300.0;

    // Only accumulate integral when error is within this zone (RPM)
    // Normalized from ticks/sec so gains are human-tunable
    private static final double INTEGRAL_ZONE_RPM = 50.0;

    // Low-pass filter coefficient for derivative (0 = no filtering, 1 = full filtering)
    private static final double DERIVATIVE_FILTER = 0.7;

    // Cap dt to prevent derivative spikes from slow loop iterations (e.g. Pedro path computation)
    private static final double MAX_DT = 0.05;

    private final double ticksPerRev;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDFMotorController(double kP, double kI, double kD, double kF, double ticksPerRev) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.ticksPerRev = ticksPerRev;
        timer.reset();
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastDerivative = 0;
        timer.reset();
    }

    public void setTunings(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double computePowerForTargetRPM(double targetRPM, double currentTicksPerSec) {
        double targetTicksPerSec = (targetRPM * ticksPerRev) / 60.0;
        return computePowerForTargetTicksPerSec(targetTicksPerSec, currentTicksPerSec);
    }

    public double computePowerForTargetTicksPerSec(double targetTicksPerSec, double currentTicksPerSec) {
        if (targetTicksPerSec == 0) {
            reset();
            return 0.0;
        }

        double dt = Math.min(MAX_DT, Math.max(1e-6, timer.seconds()));
        timer.reset();

        // Normalize error to RPM so gains are human-tunable
        // kP = 0.001 means: 1 RPM error -> +0.001 power contribution
        double targetRPM = (targetTicksPerSec / ticksPerRev) * 60.0;
        double currentRPM = (currentTicksPerSec / ticksPerRev) * 60.0;
        double errorRPM = targetRPM - currentRPM;

        // Only accumulate integral when close to setpoint (prevents ramp-up windup)
        boolean inIntegralZone = Math.abs(errorRPM) < INTEGRAL_ZONE_RPM;
        if (inIntegralZone) {
            integralSum += errorRPM * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        } else {
            // Outside integral zone — reset integral so it doesn't carry stale values
            integralSum = 0;
        }

        // Low-pass filtered derivative to reduce noise/jitter
        double rawDerivative = (errorRPM - lastError) / dt;
        double derivative = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * rawDerivative;
        lastDerivative = derivative;
        lastError = errorRPM;

        // Feedforward operates on ticks/sec (unchanged), PID operates on RPM-normalized error
        double ff = kF * targetTicksPerSec;
        double pid = (kP * errorRPM) + (kI * integralSum) + (kD * derivative);
        double power = ff + pid;

        power = Math.max(0, Math.min(1.0, power));

        return power;
    }

    public double applyToMotor(DcMotorEx motor, double targetRPM) {
        double currentTicksPerSec = motor.getVelocity();
        double power = computePowerForTargetRPM(targetRPM, currentTicksPerSec);
        motor.setPower(power);
        return power;
    }

    public static double compensateForVoltage(double rawPower, double currentVoltage, double nominalVoltage) {
        if (currentVoltage <= 0 || nominalVoltage <= 0) return rawPower;
        double compensated = rawPower * (nominalVoltage / currentVoltage);
        return Math.min(1.0, Math.max(0.0, compensated));
    }

    /**
     * Compute motor power with voltage compensation baked into feedforward.
     * Instead of scaling the entire output (which fails at saturation),
     * we scale the feedforward directly so it accounts for voltage drop.
     */
    public double computePowerForTargetRPMWithVoltageCompensation(
            double targetRPM, double currentTicksPerSec, double currentVoltage, double nominalVoltage) {
        double targetTicksPerSec = (targetRPM * ticksPerRev) / 60.0;

        if (targetTicksPerSec == 0) {
            reset();
            return 0.0;
        }

        double dt = Math.min(MAX_DT, Math.max(1e-6, timer.seconds()));
        timer.reset();

        // Normalize error to RPM so gains are human-tunable
        double currentRPM = (currentTicksPerSec / ticksPerRev) * 60.0;
        double errorRPM = targetRPM - currentRPM;

        boolean inIntegralZone = Math.abs(errorRPM) < INTEGRAL_ZONE_RPM;
        if (inIntegralZone) {
            integralSum += errorRPM * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        } else {
            integralSum = 0;
        }

        double rawDerivative = (errorRPM - lastError) / dt;
        double derivative = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * rawDerivative;
        lastDerivative = derivative;
        lastError = errorRPM;

        // Voltage-compensated feedforward: scale FF by voltage ratio so the motor
        // receives the same effective voltage regardless of battery level
        double voltageScale = (currentVoltage > 0) ? (nominalVoltage / currentVoltage) : 1.0;
        double ff = kF * targetTicksPerSec * voltageScale;
        double pid = (kP * errorRPM) + (kI * integralSum) + (kD * derivative);
        double power = ff + pid;

        power = Math.max(0, Math.min(1.0, power));

        return power;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}