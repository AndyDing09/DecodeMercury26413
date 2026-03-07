package org.firstinspires.ftc.teamcode.testing;

/**
 * Utility class containing physics calculations for a shooter launcher.
 *
 * Uses a full ballistics solver with lip clearance, backboard priority,
 * and constraint-based angle selection.
 *
 * All members are static so callers can simply reference
 * {@code MathLib.solveFromPosition(...)} etc.
 */
public final class MathLib {
    private MathLib() {}

    // ==================== PHYSICAL CONSTANTS ====================
    public static final double GRAVITY = 9.81;
    public static final double TARGET_HEIGHT = 1.22;         // goal height in meters
    public static final double LAUNCHER_HEIGHT = 0.32385;    // shooter height in meters
    public static final double MIN_HOOD_ANGLE = 26.0;
    public static final double MAX_HOOD_ANGLE = 39.5;
    public static final double LAUNCHER_MAX_BALL_VELOCITY = 15.0;

    // ==================== GOAL POSITIONS (field inches) ====================
    public static final double GOAL_LIP_X = 120.0;
    public static final double GOAL_LIP_Y = 120.0;
    public static final double GOAL_CENTER_X = 132.0;
    public static final double GOAL_CENTER_Y = 132.0;
    public static final double GOAL_BACK_X = 144.0;
    public static final double GOAL_BACK_Y = 144.0;

    // ==================== GOAL GEOMETRY (meters) ====================
    public static final double GOAL_LIP = 0.45;
    public static final double BACKBOARD_Y_OFFSET = 0.1;    // meters above goal center height
    public static final double LIP_BUFFER = 8 * 0.0254;     // clearance above lip (~0.2032m)
    public static final double DISTANCE_OFFSET = 0.0;

    // ==================== HOOD SERVO CONVERSION ====================
    public static final double SMALL_GEAR_DIAMETER = 57.25;
    public static final double LARGE_GEAR_DIAMETER = 375.0;
    public static final double GEAR_RATIO = LARGE_GEAR_DIAMETER / SMALL_GEAR_DIAMETER;
    public static final double SERVO_START_POS = 0.5;
    public static final double SERVO_UNITS_PER_HOOD_DEGREE = GEAR_RATIO / 180.0;
    public static final double MAX_REACHABLE_HOOD_ANGLE =
            MIN_HOOD_ANGLE + (1.0 - SERVO_START_POS) * 180.0 / GEAR_RATIO;

    // ==================== MOTOR CONSTANTS ====================
    public static final double TICKS_PER_REV = 28.0;

    // ==================== UNIT CONVERSION ====================
    public static final double INCHES_TO_METERS = 0.0254;

    // ==================== INTERPOLATION TABLES ====================
    private static final double[] INPUT_MS =
            {-0.01, 0.0, 4.29, 4.49, 4.76, 5.22, 5.65, 6.06, 6.44, 6.86, 7.2, 10.0};
    private static final double[] OUTPUT_TICKS =
            {-0.01, 0.0, 1040.0, 1100.0, 1180.0, 1320.0, 1480.0, 1620.0, 1780.0, 1940.0, 1980.0, 2100.0};

    // ==========================================================================
    //  POSITION-BASED SOLVER (uses odometry)
    // ==========================================================================

    /**
     * Full solver using robot field position. Computes actual distances to the
     * goal lip, center, and backboard from the robot's (x, y) position.
     *
     * @param robotX robot X position in field inches
     * @param robotY robot Y position in field inches
     * @return LauncherSolution with velocity and hood angle
     */
    public static LauncherSolution solveFromPosition(double robotX, double robotY) {
        double xLip    = fieldDistanceMeters(robotX, robotY, GOAL_LIP_X, GOAL_LIP_Y);
        double xCenter = fieldDistanceMeters(robotX, robotY, GOAL_CENTER_X, GOAL_CENTER_Y);
        double xBack   = fieldDistanceMeters(robotX, robotY, GOAL_BACK_X, GOAL_BACK_Y);

        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        // Attempt 1: Backboard shot (priority — faster, flatter)
        double[] result = calculateBestShot(xBack, TARGET_HEIGHT + BACKBOARD_Y_OFFSET,
                xLip, deltaYLip, GRAVITY);

        // Attempt 2: Goal center fallback
        if (Double.isNaN(result[0])) {
            result = calculateBestShot(xCenter, TARGET_HEIGHT, xLip, deltaYLip, GRAVITY);
        }

        return new LauncherSolution(result[0], result[1]);
    }

    /**
     * Compute Euclidean distance between two field points, returned in meters.
     */
    public static double fieldDistanceMeters(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy) * INCHES_TO_METERS;
    }

    // ==========================================================================
    //  DISTANCE-BASED SOLVER (legacy — takes raw distance in meters)
    // ==========================================================================

    /**
     * Full ballistics solver given a horizontal distance to the goal in meters.
     */
    public static LauncherSolution distanceToLauncherValues(double distance) {
        distance += DISTANCE_OFFSET;
        if (distance <= 0) {
            return new LauncherSolution(Double.NaN, Double.NaN);
        }

        double x = distance;
        double xLip = x - GOAL_LIP;
        double deltaYLip = (TARGET_HEIGHT + LIP_BUFFER) - LAUNCHER_HEIGHT;

        double[] result = calculateBestShot(x, TARGET_HEIGHT + BACKBOARD_Y_OFFSET,
                xLip, deltaYLip, GRAVITY);

        if (Double.isNaN(result[0])) {
            result = calculateBestShot(x, TARGET_HEIGHT, xLip, deltaYLip, GRAVITY);
        }

        return new LauncherSolution(result[0], result[1]);
    }

    // ==========================================================================
    //  CONVERSION HELPERS
    // ==========================================================================

    /**
     * Convert hood angle (degrees) to servo position.
     * Servo 0.5 = MIN_HOOD_ANGLE. Servo increases to raise the hood.
     */
    public static double hoodAngleToServoPos(double angleDeg) {
        double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
        double clamped = Math.max(MIN_HOOD_ANGLE, Math.min(effectiveMax, angleDeg));
        double servoPos = SERVO_START_POS + (clamped - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    /**
     * Convert a LauncherSolution's velocity to target RPM via interpolation.
     */
    public static double solutionToRPM(LauncherSolution solution) {
        if (!solution.isValid()) return 0;
        double ticks = interpolateToTicks(solution.velocityMs);
        return ticks * 60.0 / TICKS_PER_REV;
    }

    /**
     * Convert a LauncherSolution's hood angle to servo position.
     */
    public static double solutionToServoPos(LauncherSolution solution) {
        if (!solution.isValid() || Double.isNaN(solution.hoodAngleDeg)) {
            return SERVO_START_POS;
        }
        return hoodAngleToServoPos(solution.hoodAngleDeg);
    }

    // ==========================================================================
    //  BALLISTICS INTERNALS
    // ==========================================================================

    /**
     * Finds the flattest valid shot (lowest angle that clears everything).
     */
    private static double[] calculateBestShot(double x, double targetY,
                                              double xLip, double deltaYLip, double g) {
        double deltaY = targetY - LAUNCHER_HEIGHT;
        double minPhysAngleH = 90.0 - MAX_HOOD_ANGLE;
        double maxPhysAngleH = 90.0 - MIN_HOOD_ANGLE;

        double minGeomAngle = Math.toDegrees(Math.atan(deltaY / x)) + 0.1;

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

        double targetAngleH = Math.max(minPhysAngleH, Math.max(minLipH, minGeomAngle));

        if (targetAngleH > maxPhysAngleH) {
            return new double[]{Double.NaN, Double.NaN};
        }

        double vReq = calculateVelocity(x, deltaY, targetAngleH, g);

        if (!Double.isNaN(vReq) && vReq <= LAUNCHER_MAX_BALL_VELOCITY) {
            return new double[]{vReq, 90.0 - targetAngleH};
        }

        double v = LAUNCHER_MAX_BALL_VELOCITY;
        double A = (g * x * x) / (2.0 * v * v);
        double B = -x;
        double C = deltaY + A;
        double disc = B * B - 4 * A * C;

        if (disc < 0) return new double[]{Double.NaN, Double.NaN};

        double sqrtD = Math.sqrt(disc);
        double tan1 = (-B - sqrtD) / (2 * A);
        double tan2 = (-B + sqrtD) / (2 * A);

        double a1 = Math.toDegrees(Math.atan(tan1));
        double a2 = Math.toDegrees(Math.atan(tan2));

        if (a1 >= targetAngleH && a1 <= maxPhysAngleH) return new double[]{v, 90.0 - a1};
        if (a2 >= targetAngleH && a2 <= maxPhysAngleH) return new double[]{v, 90.0 - a2};

        return new double[]{Double.NaN, Double.NaN};
    }

    private static double calculateVelocity(double x, double deltaY, double angleHoriz, double g) {
        double angleRad = Math.toRadians(angleHoriz);
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);
        double denom = 2 * cosTheta * cosTheta * (x * tanTheta - deltaY);

        if (denom <= 1e-9) return Double.NaN;

        return Math.sqrt((g * x * x) / denom);
    }

    /**
     * Convert velocity (m/s) to motor ticks/sec via interpolation table.
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
