package org.firstinspires.ftc.teamcode.testing;

/**
 * Utility class for shooter calculations.
 *
 * Uses empirically measured lookup tables (distance → RPM, distance → hood servo pos)
 * with linear interpolation. Physics solver has been removed in favour of real-world data.
 */
public final class MathLib {
    private MathLib() {}

    // ==================== PHYSICAL CONSTANTS ====================
    public static final double TARGET_HEIGHT = 1.22;

    // ==================== GOAL POSITIONS (field inches) ====================
    public static final double GOAL_CENTER_X = 132.0;
    public static final double GOAL_CENTER_Y = 12.0;

    // ==================== UNIT CONVERSION ====================
    public static final double INCHES_TO_METERS = 0.0254;

    // ==================== MOTOR CONSTANTS ====================
    public static final double TICKS_PER_REV = 28.0;

    // ==================== EMPIRICAL LOOKUP TABLES ====================
    // Sorted by distance (meters). Measured on real robot.
    private static final double[] DISTANCES = { 1.1,  1.5,  2.08, 2.4,  2.75, 3.08, 3.34 };
    private static final double[] RPMS      = { 3000, 3300, 3500, 3600, 3675, 3900, 3975 };

    // ==================== HOOD THRESHOLD ====================
    private static final double HOOD_CLOSE_THRESHOLD = 1.3; // meters
    private static final double HOOD_CLOSE_POS       = 0.75;
    private static final double HOOD_FAR_POS         = 0.90;

    // ==========================================================================
    //  MAIN ENTRY POINT
    // ==========================================================================

    /**
     * Returns a LauncherSolution (RPM + hood angle) for a given distance in meters.
     * Clamps to the nearest known value if outside the measured range.
     */
    public static LauncherSolution distanceToLauncherValues(double distance) {
        double rpm     = interpolate(DISTANCES, RPMS, distance);
        double hoodPos = distance < HOOD_CLOSE_THRESHOLD ? HOOD_CLOSE_POS : HOOD_FAR_POS;
        return new LauncherSolution(rpm, hoodPos);
    }

    // ==========================================================================
    //  CONVERSION HELPERS
    // ==========================================================================

    /**
     * Returns target RPM directly from a LauncherSolution.
     * NOTE: solution.velocityMs now stores RPM directly (not m/s).
     */
    public static double solutionToRPM(LauncherSolution solution) {
        if (!solution.isValid()) return 0;
        return solution.velocityMs; // repurposed field — holds RPM
    }

    /**
     * Returns hood servo position directly from a LauncherSolution.
     * NOTE: solution.hoodAngleDeg now stores servo pos directly (not degrees).
     */
    public static double solutionToServoPos(LauncherSolution solution) {
        if (!solution.isValid()) return 0.5;
        return solution.hoodAngleDeg; // repurposed field — holds servo pos
    }

    // ==========================================================================
    //  INTERPOLATION
    // ==========================================================================

    /**
     * Linear interpolation (or clamping) across a sorted x/y table.
     */
    private static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0])                return ys[0];
        if (x >= xs[xs.length - 1])    return ys[ys.length - 1];

        for (int i = 0; i < xs.length - 1; i++) {
            if (x >= xs[i] && x <= xs[i + 1]) {
                double fraction = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + fraction * (ys[i + 1] - ys[i]);
            }
        }
        return ys[0]; // fallback, should never be reached
    }
}