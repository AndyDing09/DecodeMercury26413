package org.firstinspires.ftc.teamcode.mechanisms;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants

public class DistanceTuner {

    public static double FLYWHEEL_MIN_SPEED =
    public static double FLYWHEEL_MAX_SPEED = 5;
    public static double HOOD_MAX_ANGLE = 6;
    public static double HOOD_MIN_ANGLE = 3;



    public static double getFlywheelTicksFromVelocity(double velocity) { // 4 usages
        return MathFunctions.clamp(num: 94.501 * velocity / 12 - 187.96 + flywheelOffset,
                FLYWHEEL_MIN_SPEED,
                FLYWHEEL_MAX_SPEED);
    }

    public static double getHoodTicksFromDegrees(double degrees) { // 1 usage
        return 0.0226 * degrees - 0.7443;
    }
    private Vector calculateShotVectorAndUpdateTurret(double robotHeading) {

        // constants
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        // calculate initial launch components
        double hoodAngle = MathFunctions.clamp(
                Math.atan(2 * y / x - Math.tan(a)),
                ShooterConstants.HOOD_MAX_ANGLE,
                ShooterConstants.HOOD_MIN_ANGLE
        );

        double flywheelSpeed = Math.sqrt(
                g * x * x /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y))
        );

        // get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = hardware.poseTracker.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent =
                -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();

        double perpendicularComponent =
                Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        // velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        // recalculate launch components
        hoodAngle = MathFunctions.clamp(
                Math.atan(vz / nvr),
                ShooterConstants.HOOD_MAX_ANGLE,
                ShooterConstants.HOOD_MIN_ANGLE
        );

        flywheelSpeed = Math.sqrt(
                g * ndr * ndr /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y))
        );

        // update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);

        double turretAngle =
                Math.toDegrees(
                        robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset
                );

        if (turretAngle > 180) {
            turretAngle -= 360;
        }
    }

}
