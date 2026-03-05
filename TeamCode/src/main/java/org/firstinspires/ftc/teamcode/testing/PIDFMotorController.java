package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple reusable PIDF controller for a shooter motor.
 *
 * Usage:
 * - Create one instance per motor (left/right) with tuned gains and encoder ticks per revolution.
 * - Call {@link #applyToMotor(DcMotorEx, double)} in a loop with the desired target RPM.
 * - Or call {@link #computePowerForTargetRPM(double, double)} if you only want the computed power.
 */
public class PIDFMotorController {

    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;

    // Anti-windup: clamp integral accumulator to prevent runaway when output is saturated
    private static final double MAX_INTEGRAL = 500.0;

    // encoder ticks per motor revolution (common values: 28 for many motors)
    private final double ticksPerRev;

    private final ElapsedTime timer = new ElapsedTime();

    // No built-in defaults; caller must provide tunings explicitly.

    /**
     * Create a PIDF controller for a motor.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF feedforward gain (applied to target velocity in ticks/s)
     * @param ticksPerRev encoder ticks per revolution
     */
    public PIDFMotorController(double kP, double kI, double kD, double kF, double ticksPerRev) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.ticksPerRev = ticksPerRev;
        timer.reset();
    }


    /** Reset integral and derivative history. Call when stopping motor or changing setpoints drastically. */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void setTunings(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Compute motor power [0..1] for a given target RPM and current motor velocity (ticks/s).
     * This does not apply the power to the motor.
     *
     * @param targetRPM desired RPM
     * @param currentTicksPerSec current measured velocity in ticks per second
     * @return clamped power in range [0,1]
     */
    public double computePowerForTargetRPM(double targetRPM, double currentTicksPerSec) {
        double targetTicksPerSec = (targetRPM * ticksPerRev) / 60.0;
        return computePowerForTargetTicksPerSec(targetTicksPerSec, currentTicksPerSec);
    }

    /**
     * Compute motor power [0..1] for a given target velocity in ticks/s.
     */
    public double computePowerForTargetTicksPerSec(double targetTicksPerSec, double currentTicksPerSec) {
        double dt = Math.max(1e-6, timer.seconds());
        timer.reset();

        double error = targetTicksPerSec - currentTicksPerSec;
        integralSum += error * dt;
        // Anti-windup: clamp integral to prevent saturation buildup
        integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetTicksPerSec);

        // If target is zero, reset state and return zero
        if (targetTicksPerSec == 0) {
            integralSum = 0;
            lastError = 0;
            return 0.0;
        }

        // clamp
        if (power < 0) power = 0;
        if (power > 1) power = 1;
        return power;
    }

    /**
     * Convenience: read motor velocity, compute power for target RPM and apply it to the motor.
     *
     * @param motor motor to apply power to
     * @param targetRPM desired RPM
     * @return applied power
     */
    public double applyToMotor(DcMotorEx motor, double targetRPM) {
        double currentTicksPerSec = motor.getVelocity();
        double power = computePowerForTargetRPM(targetRPM, currentTicksPerSec);
        motor.setPower(power);
        return power;
    }

    /**
     * Apply voltage compensation to raw power to maintain consistent motor output across voltage variations.
     * Scales power inversely with voltage to keep RPM stable as battery drains.
     *
     * @param rawPower unadjusted power [0..1] from PID calculation
     * @param currentVoltage actual measured voltage in volts
     * @param nominalVoltage expected/nominal voltage (e.g., 12V for a 12V battery)
     * @return voltage-compensated power, clamped to [0..1]
     */
    public static double compensateForVoltage(double rawPower, double currentVoltage, double nominalVoltage) {
        if (currentVoltage <= 0 || nominalVoltage <= 0) return rawPower; // safety check
        double compensated = rawPower * (nominalVoltage / currentVoltage);
        // clamp to [0, 1]
        return Math.min(1.0, Math.max(0.0, compensated));
    }

    /**
     * Compute motor power with voltage compensation already applied.
     *
     * @param targetRPM desired RPM
     * @param currentTicksPerSec current measured velocity in ticks per second
     * @param currentVoltage actual measured voltage in volts
     * @param nominalVoltage expected/nominal voltage (e.g., 12V)
     * @return voltage-compensated power in range [0,1]
     */
    public double computePowerForTargetRPMWithVoltageCompensation(
            double targetRPM, double currentTicksPerSec, double currentVoltage, double nominalVoltage) {
        double rawPower = computePowerForTargetRPM(targetRPM, currentTicksPerSec);
        return compensateForVoltage(rawPower, currentVoltage, nominalVoltage);
    }

    // getters for tuning/debugging
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}
