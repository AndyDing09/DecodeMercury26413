package org.firstinspires.ftc.teamcode.testing;

/**
 * Simple value object representing the outcome of a launcher physics calculation.
 *
 * Keeping it as a top-level class avoids nested layers and makes imports easier
 * when other opmodes reference the result.
 */
public class LauncherSolution {
    public final double velocityMs;   // required launch speed (meters per second)
    public final double hoodAngleDeg; // recommended hood angle (degrees)

    public LauncherSolution(double velocityMs, double hoodAngleDeg) {
        this.velocityMs = velocityMs;
        this.hoodAngleDeg = hoodAngleDeg;
    }

    public boolean isValid() {
        return !Double.isNaN(velocityMs);
    }

    @Override
    public String toString() {
        return String.format("LauncherSolution(v=%.2f m/s, angle=%.1f°)", velocityMs, hoodAngleDeg);
    }
}