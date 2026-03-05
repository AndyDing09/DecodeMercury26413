package org.firstinspires.ftc.teamcode.testing;

/**
 * Utility class containing physics calculations for a shooter launcher.
 *
 * Uses a full ballistics solver with lip clearance, backboard priority,
 * and constraint-based angle selection (from ShooterControl).
 *
 * All members are static so callers can simply reference
 * {@code MathLib.distanceToLauncherValues(...)} and
 * {@code MathLib.interpolateToTicks(...)}.
 */
public final class MathLib {
    // prevent instantiation
    private MathLib() {}

    // physical constants (must match robot dimensions)
    public static final double GRAVITY = 9.81;
    public static final double TARGET_HEIGHT = 1.22;    // goal height in meters
    public static final double LAUNCHER_HEIGHT = 0.32385;    // shooter height in meters
    public static final double MIN_HOOD_ANGLE = 26.0;
    public static final double MAX_HOOD_ANGLE = 39.5;
    public static final double LAUNCHER_MAX_BALL_VELOCITY = 15.0;
    public static final double MAX_DRIVE_VELOCITY = 15.0;

    // goal geometry for lip clearance and backboard targeting
    public static final double GOAL_LIP = 0.45;           // rim horizontal offset (meters)
    public static final double BACKBOARD_Y_OFFSET = 0.1;  // meters above goal center
    public static final double LIP_BUFFER = 8 * 0.0254;   // clearance above lip (~0.2032m)
    public static final double DISTANCE_OFFSET = 0.0;   // no distance sensor — input is true distance

    // interpolation tables originally captured from launcher testing
    private static final double[] INPUT_MS = {-0.01, 0.0, 4.29,   4.49,   4.76,   5.22,   5.65,   6.06,   6.44,   6.86,   7.2,    10.0};
    private static final double[] OUTPUT_TICKS = {-0.01, 0.0, 1040.0, 1100.0, 1180.0, 1320.0, 1480.0, 1620.0, 1780.0, 1940.0, 1980.0, 2100.0}; // output: ticks/s

    /**
     * Full ballistics solver. Given a horizontal distance to the goal, computes
     * the required ball velocity and hood angle.
     *
     * Priority: tries backboard shot first (flat, fast), falls back to
     * goal center if backboard is impossible.
     *
     * @param distance horizontal distance to the goal, in meters
     * @return a {@link LauncherSolution} containing the velocity and hood angle
     */
    public static LauncherSolution distanceToLauncherValues(double distance) {
        distance += DISTANCE_OFFSET;
        if (distance <= 0) {
            return new LauncherSolution(Double.NaN, Double.NaN);
        }

        double g = GRAVITY;
        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // Attempt 1: Backboard shot (priority - faster, flatter)
        double[] result = calculateBestShot(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET, xLip, deltaYLip, g);

        // Attempt 2: Goal center fallback
        if (Double.isNaN(result[0])) {
            result = calculateBestShot(x, TARGET_HEIGHT, xLip, deltaYLip, g);
        }

        return new LauncherSolution(result[0], result[1]);
    }

    /**
     * Finds the flattest valid shot (lowest angle that still clears everything).
     *
     * Enforces three constraints:
     *   1. Line-of-sight: angle must be steep enough to reach the target
     *   2. Lip clearance: trajectory must clear the goal rim by LIP_BUFFER
     *   3. Hood mechanical limits: angle must be within [MIN, MAX]_HOOD_ANGLE
     */
    private static double[] calculateBestShot(double x, double targetY, double xLip, double deltaYLip, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double minPhysAngleH = 90.0 - MAX_HOOD_ANGLE;  // flattest the hood allows (from horizontal)
        double maxPhysAngleH = 90.0 - MIN_HOOD_ANGLE;  // steepest the hood allows (from horizontal)

        // Constraint 1: Line-of-sight
        double minGeomAngle = Math.toDegrees(Math.atan(deltaY / x)) + 0.1;

        // Constraint 2: Lip clearance
        double minLipH = 0.0;
        if (xLip > 0) {
            double num = (deltaY * xLip * xLip) - (deltaYLip * x * x);
            double den = (x * xLip * xLip) - (xLip * x * x);
            if (Math.abs(den) > 1e-5) {
                minLipH = Math.toDegrees(Math.atan(num / den));
            } else {
                minLipH = 89.9;
            }
        }

        // The angle floor: must satisfy all three constraints
        double targetAngleH = Math.max(minPhysAngleH, Math.max(minLipH, minGeomAngle));

        // If the floor is steeper than hood allows, impossible
        if (targetAngleH > maxPhysAngleH) {
            return new double[]{Double.NaN, Double.NaN};
        }

        // Calculate velocity for this flattest legal angle
        double vReq = calculateVelocity(x, deltaY, targetAngleH, g);

        if (!Double.isNaN(vReq) && vReq <= LAUNCHER_MAX_BALL_VELOCITY) {
            return new double[]{vReq, 90.0 - targetAngleH};  // return as hood angle (from horiz)
        }

        // Velocity-limited fallback: max power, solve for best angle
        double v = LAUNCHER_MAX_BALL_VELOCITY;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;
        double disc = B * B - 4 * A * C;

        if (disc < 0) return new double[]{Double.NaN, Double.NaN};

        double sqrtD = Math.sqrt(disc);
        double tan1 = (-B - sqrtD) / (2 * A);  // flatter solution
        double tan2 = (-B + sqrtD) / (2 * A);  // steeper solution

        double a1 = Math.toDegrees(Math.atan(tan1));
        double a2 = Math.toDegrees(Math.atan(tan2));

        // Prefer flat, then lob
        if (a1 >= targetAngleH && a1 <= maxPhysAngleH) return new double[]{v, 90.0 - a1};
        if (a2 >= targetAngleH && a2 <= maxPhysAngleH) return new double[]{v, 90.0 - a2};

        return new double[]{Double.NaN, Double.NaN};
    }

    /**
     * Projectile velocity from the kinematic equation:
     *   v = sqrt( g * x^2 / (2 * cos^2(theta) * (x*tan(theta) - deltaY)) )
     */
    private static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 1e-9) return Double.NaN;

        return Math.sqrt((g * x * x) / denom);
    }

    /**
     * Convert a launcher velocity (m/s) into motor ticks per second using linear
     * interpolation of calibration data.
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
