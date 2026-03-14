package org.firstinspires.ftc.teamcode.Storedvalues;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.75)
            .strafePodX(-6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("imu")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.009)
            .yVelocity(60.41)

            ;


    // ================================================================
    // THIS MIGHT BE THE ISSUE: NO TRANSLATIONAL OR LATERAL HEADING COEFFICIENTS HERE
    // ================================================================
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.4)
            .forwardZeroPowerAcceleration(-30.703)
            .lateralZeroPowerAcceleration(-64.97)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.108403, 0.001407))
            .headingPIDFCoefficients(new PIDFCoefficients(2.25, 0, 0.23, 0.02)) // 1. increase f until oscillations then decrease  2. increase p(push it high - 2) and d(matching p so that when robot corrects itself it doesn overshoot)   3. i=0
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
//}
//}