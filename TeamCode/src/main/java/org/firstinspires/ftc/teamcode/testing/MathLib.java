package org.firstinspires.ftc.teamcode.testing;

/**
 * Utility class containing physics calculations for a shooter launcher.
 *
 * The original helper methods lived inside a TeleOp opmode; this standalone
 * class makes them reusable from any opmode (TeleOp2, shooter tests, etc.).
 *
 * All members are static so callers can simply reference
 * `MathLib.distanceToLauncherValues(...)` and
 * `MathLib.interpolateToTicks(...)`.
 */
public final class MathLib {
    // prevent instantiation
    private MathLib() {}

    // physical constants (must match robot dimensions)
    public static final double GRAVITY = 9.81;
    public static final double TARGET_HEIGHT = 1.22;    // goal height in meters
    public static final double LAUNCHER_HEIGHT = 0.2;    // shooter height in meters
    public static final double MIN_HOOD_ANGLE = 16.0;
    public static final double MAX_HOOD_ANGLE = 50.0;
    public static final double LAUNCHER_MAX_BALL_VELOCITY = 15.65;
    public static final double MAX_DRIVE_VELOCITY = 15.0;

    // interpolation tables originally captured from launcher testing
    private static final double [] INPUT_MS = {-0.01, 0.0, 7.376, 8.9408, 10.2819, 12.07, 13.4112, 14.5288, 15.42288};
    private static final double[] OUTPUT_TICKS = {-0.01, 0.0, 933.33, 1166.67, 1400.0, 1633.33, 1866.667, 2100, 2240.0};
    /**
     * Given a horizontal {@code distance} in meters, compute the minimum launcher
     * velocity (m/s) and optimal hood angle (vertical degrees) required to hit the
     * target.  If the shot is impossible (angle/velocity limits or physics failures)
     * the {@link LauncherSolution#velocityMs} field will be {@code NaN}.
     *
     * @param distance horizontal distance to the goal, in meters
     * @return a {@link LauncherSolution} containing the velocity and hood angle
     */
    public static LauncherSolution distanceToLauncherValues(double distance) {
        if (distance <= 0) {
            return new LauncherSolution(Double.NaN, Double.NaN);
        }

        double x = distance;
        double deltaY = TARGET_HEIGHT - LAUNCHER_HEIGHT;

        double minVelocitySquared = GRAVITY * (deltaY +
                Math.sqrt(Math.pow(deltaY, 2) + Math.pow(x, 2)));
        double minVelocity = Math.sqrt(minVelocitySquared);

        double tanThetaMin = minVelocitySquared / (GRAVITY * x);
        double optimalAngleHoriz = Math.toDegrees(Math.atan(tanThetaMin));
        double optimalAngleVert = 90.0 - optimalAngleHoriz;

        double finalAngleVert;
        double finalAngleHoriz;
        boolean forceOverride = false;

        if (distance <= 1) {
            finalAngleVert = MIN_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE;
            forceOverride = true;
        } else if (distance >= 4) {
            finalAngleVert = MAX_HOOD_ANGLE;
            finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
            forceOverride = true;
        } else {
            finalAngleVert = 0;
            finalAngleHoriz = 0;
        }

        if (!forceOverride) {
            if (optimalAngleVert >= MIN_HOOD_ANGLE && optimalAngleVert <= MAX_HOOD_ANGLE) {
                finalAngleVert = optimalAngleVert;
                finalAngleHoriz = optimalAngleHoriz;
                if (minVelocity > LAUNCHER_MAX_BALL_VELOCITY) {
                    return new LauncherSolution(Double.NaN, Double.NaN);
                }
                return new LauncherSolution(minVelocity, finalAngleVert);
            } else if (optimalAngleVert < MIN_HOOD_ANGLE) {
                finalAngleVert = MIN_HOOD_ANGLE;
                finalAngleHoriz = 90.0 - MIN_HOOD_ANGLE;
            } else {
                finalAngleVert = MAX_HOOD_ANGLE;
                finalAngleHoriz = 90.0 - MAX_HOOD_ANGLE;
            }
        }

        double angleToUseRad = Math.toRadians(finalAngleHoriz);
        double tanTheta = Math.tan(angleToUseRad);
        double cosTheta = Math.cos(angleToUseRad);

        double denominator = 2 * (cosTheta * cosTheta) * (x * tanTheta - deltaY);
        if (denominator <= 0) {
            return new LauncherSolution(Double.NaN, Double.NaN);
        }

        double requiredVelocity = Math.sqrt((GRAVITY * x * x) / denominator);
        if (requiredVelocity > MAX_DRIVE_VELOCITY) {
            return new LauncherSolution(Double.NaN, Double.NaN);
        }

        return new LauncherSolution(requiredVelocity, finalAngleVert);
    }

    /**
     * Convert a launcher velocity (m/s) into motor ticks per second using linear
     * interpolation of calibration data.<br>
     * Values outside the table bounds are clamped to the nearest endpoint.
     */
    public static double interpolateToTicks(double velocityMs) {
        if (velocityMs <= INPUT_MS[0]) return OUTPUT_TICKS[0];
        if (velocityMs >= INPUT_MS[INPUT_MS.length - 1])
            return OUTPUT_TICKS[OUTPUT_TICKS.length - 1];

        for (int i = 0; i < INPUT_MS.length - 1; i++) {
            if (velocityMs >= INPUT_MS[i] && velocityMs <= INPUT_MS[i + 1]) {
                double fraction = (velocityMs - INPUT_MS[i]) /
                        (INPUT_MS[i + 1] - INPUT_MS[i]);
                return OUTPUT_TICKS[i] + fraction *
                        (OUTPUT_TICKS[i + 1] - OUTPUT_TICKS[i]);
            }
        }
        return 0.0;
    }
}
