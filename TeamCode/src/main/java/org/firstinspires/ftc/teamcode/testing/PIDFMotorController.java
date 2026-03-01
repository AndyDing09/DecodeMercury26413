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
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetTicksPerSec);

        // If target is zero, return zero to avoid holding power
        if (targetTicksPerSec == 0) return 0.0;

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

    // getters for tuning/debugging
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}
