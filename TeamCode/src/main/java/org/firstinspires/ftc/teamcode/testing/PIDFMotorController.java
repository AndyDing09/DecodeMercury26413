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

    // Only accumulate integral when error is within this zone (ticks/sec)
    // Prevents windup during ramp-up
    private static final double INTEGRAL_ZONE_TICKS = 400.0;

    // Low-pass filter coefficient for derivative (0 = no filtering, 1 = full filtering)
    private static final double DERIVATIVE_FILTER = 0.7;

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

        double dt = Math.max(1e-6, timer.seconds());
        timer.reset();

        double error = targetTicksPerSec - currentTicksPerSec;

        // Only accumulate integral when close to setpoint (prevents ramp-up windup)
        if (Math.abs(error) < INTEGRAL_ZONE_TICKS) {
            integralSum += error * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        } else {
            // Decay integral when far from target to prevent stale buildup
            integralSum *= 0.8;
        }

        // Low-pass filtered derivative to reduce noise/jitter
        double rawDerivative = (error - lastError) / dt;
        double derivative = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * rawDerivative;
        lastDerivative = derivative;
        lastError = error;

        // Feedforward + PID
        double ff = kF * targetTicksPerSec;
        double pid = (kP * error) + (kI * integralSum) + (kD * derivative);
        double power = ff + pid;

        // Back-calculation anti-windup: if output saturates, undo the integral that caused it
        if (power > 1.0) {
            integralSum -= (power - 1.0) / Math.max(kI, 1e-9) * dt;
            power = 1.0;
        } else if (power < 0) {
            integralSum -= power / Math.max(kI, 1e-9) * dt;
            power = 0;
        }

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

        double dt = Math.max(1e-6, timer.seconds());
        timer.reset();

        double error = targetTicksPerSec - currentTicksPerSec;

        if (Math.abs(error) < INTEGRAL_ZONE_TICKS) {
            integralSum += error * dt;
            integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        } else {
            integralSum *= 0.8;
        }

        double rawDerivative = (error - lastError) / dt;
        double derivative = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * rawDerivative;
        lastDerivative = derivative;
        lastError = error;

        // Voltage-compensated feedforward: scale FF by voltage ratio so the motor
        // receives the same effective voltage regardless of battery level
        double voltageScale = (currentVoltage > 0) ? (nominalVoltage / currentVoltage) : 1.0;
        double ff = kF * targetTicksPerSec * voltageScale;
        double pid = (kP * error) + (kI * integralSum) + (kD * derivative);
        double power = ff + pid;

        if (power > 1.0) {
            integralSum -= (power - 1.0) / Math.max(kI, 1e-9) * dt;
            power = 1.0;
        } else if (power < 0) {
            integralSum -= power / Math.max(kI, 1e-9) * dt;
            power = 0;
        }

        return power;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}
