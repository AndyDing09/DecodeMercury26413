package org.firstinspires.ftc.teamcode.testing;

/**
 * Utility class containing physics calculations for a shooter launcher.
 *
 * Uses a full ballistics solver with lip clearance, backboard priority,
 * and constraint-based angle selection.
 *
 * All members are static so callers can simply reference
 * {@code MathLib.distanceToLauncherValues(...)} etc.
 */
public final class MathLib {
    private MathLib() {}

    // ==================== PHYSICAL CONSTANTS ====================
    public static final double GRAVITY = 9.810;
    public static final double TARGET_HEIGHT = 1.22;         // goal height in meters
    public static final double LAUNCHER_HEIGHT = 0.32385;    // shooter height in meters
    public static final double MIN_HOOD_ANGLE = 26.0;
    public static final double MAX_HOOD_ANGLE = 45.0;
    public static final double LAUNCHER_MAX_BALL_VELOCITY = 15.0;

    // ==================== GOAL POSITIONS (field inches) ====================
    public static final double GOAL_LIP_X = 120.0;
    public static final double GOAL_LIP_Y = 120.0;
    public static final double GOAL_CENTER_X = 132.0;
    public static final double GOAL_CENTER_Y = 12.0;
    public static final double GOAL_BACK_X = 144.0;
    public static final double GOAL_BACK_Y = 0.0;

    // ==================== GOAL GEOMETRY (meters) ====================
    public static final double GOAL_LIP = 0.45;
    public static final double BACKBOARD_Y_OFFSET = 0.1;    // meters above goal center height
    public static final double LIP_BUFFER = 8 * 0.0254;     // clearance above lip (~0.2032m)
    public static final double DISTANCE_OFFSET = 0.0;

    // ==================== HOOD SERVO CONVERSION ====================
    public static final double SMALL_GEAR_DIAMETER = 104.0;
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
            {-0.01, 0.0, 1220.0, 1280.0, 1360.0, 1500.0, 1660.0, 1800.0, 1960.0, 2120.0, 2160.0, 2280.0};

    // The distances your code calculates or asks the system to move.
    // IMPORTANT: These values must be sorted from lowest to highest!
    private static final double[] REAL_DISTANCES = {
            1.7,   // Point 0
            1.9,  // Point 1
            2.5,  // Point 2
    };

    // The actual, physical distances measured in the real world.
    // These must correspond directly to the index of the theoretical distances above.
    private static final double[] THEORETICAL_DISTANCES = {
            1.6,   // Real distance when 0.0 is requested
            2,  // Real distance when 10.0 is requested (overshot slightly)
            3.2,  // Real distance when 20.0 is requested (undershot slightly)
    };


    // ==========================================================================
    //  DISTANCE-BASED SOLVER (legacy — takes raw distance in meters)
    // ==========================================================================

    /**
     * Full ballistics solver given a horizontal distance to the goal in meters.
     */
    public static LauncherSolution distanceToLauncherValues(double distance) {
        distance = interpolateToShootingDistance(distance);
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
     * Convert hood angle (degrees) to base servo position (before flip/offsets).
     * Servo 0.5 = MIN_HOOD_ANGLE. Servo increases to raise the hood.
     */
    public static double hoodAngleToServoPos(double angleDeg) {
        double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
        double clamped = Math.max(MIN_HOOD_ANGLE, Math.min(effectiveMax, angleDeg));
        double servoPos = SERVO_START_POS + (clamped - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    /** Servo1 position: normal (matching hoodtest) */
    public static double servo1Position(double angleDeg) {
        return hoodAngleToServoPos(angleDeg);
    }

    /** Servo2 position: flipped (matching hoodtest) */
    public static double servo2Position(double angleDeg) {
        return 1.0 - hoodAngleToServoPos(angleDeg);
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
    // Make sure to define these arrays in your class, sorted from lowest to highest!
    // private static final double[] THEORETICAL_DISTANCES = { ... };
    // private static final double[] REAL_DISTANCES = { ... };

    public static double interpolateToShootingDistance(double theoreticalDistance) {
        // Handle out-of-bounds inputs by clamping to the lowest or highest known values
        if (theoreticalDistance <= THEORETICAL_DISTANCES[0]) {
            return REAL_DISTANCES[0];
        }
        if (theoreticalDistance >= THEORETICAL_DISTANCES[THEORETICAL_DISTANCES.length - 1]) {
            return REAL_DISTANCES[REAL_DISTANCES.length - 1];
        }

        // Loop through the intervals to find where the input theoretical distance falls
        for (int i = 0; i < THEORETICAL_DISTANCES.length - 1; i++) {
            if (theoreticalDistance >= THEORETICAL_DISTANCES[i] && theoreticalDistance <= THEORETICAL_DISTANCES[i + 1]) {

                // Calculate how far along the input interval our value is (from 0.0 to 1.0)
                double fraction = (theoreticalDistance - THEORETICAL_DISTANCES[i]) /
                        (THEORETICAL_DISTANCES[i + 1] - THEORETICAL_DISTANCES[i]);

                // Apply that same fraction to the output interval
                return REAL_DISTANCES[i] + fraction *
                        (REAL_DISTANCES[i + 1] - REAL_DISTANCES[i]);
            }
        }
        return 0.0; // Fallback (should theoretically never be reached due to clamping above)
    }
}
