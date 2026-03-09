package org.firstinspires.ftc.teamcode.Storedvalues;

/**
 * RobotState — static storage for passing the final autonomous pose into TeleOp.
 *
 * At the end of autonomous, write:
 *   RobotState.endX       = follower.getPose().getX();
 *   RobotState.endY       = follower.getPose().getY();
 *   RobotState.endHeading = follower.getPose().getHeading();
 *   RobotState.hasData    = true;
 *
 * In TeleOp init, read RobotState.hasData — if true, use the stored pose
 * as the starting pose. If false (e.g. no auto was run), fall back to a
 * hardcoded default pose.
 */
public class RobotPose {

    // Final pose written by autonomous
    public static double endX       = 0;
    public static double endY       = 0;
    public static double endHeading = 0;

    // Flag — only true if auto actually wrote a pose
    public static boolean hasData = false;

    // Default fallback pose if teleop is run without auto (for testing)
    public static final double DEFAULT_X       = 72;
    public static final double DEFAULT_Y       = 8.325;
    public static final double DEFAULT_HEADING = Math.toRadians(90);
}