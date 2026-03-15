//package org.firstinspires.ftc.teamcode.Opmode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.Storedvalues.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.testing.LauncherSolution;
//import org.firstinspires.ftc.teamcode.testing.MathLib;
//
//@TeleOp(name = "TeleOp with Turret Tracking - Fixed", group = "TeleOp")
//public class TeleOp2 extends LinearOpMode {
//
//    private Drivetrain drivetrain;
//    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;
//    private VoltageSensor voltageSensor;
//    private Follower follower;
//
//    // Odometry starting position (inches, radians)
//    private static final Pose START_POSE = new Pose(84, 12, Math.toRadians(0));
//
//    // Goal positions (inches) — toggle with GP1 B
//    private static final double GOAL_RED_X = 132, GOAL_RED_Y = 132;
//    private static final double GOAL_BLUE_X = 12, GOAL_BLUE_Y = 132;
//    private boolean useRedGoal = true;
//    private boolean lastG1B = false;
//
//    // Hood angle-to-servo conversion (must match launchermathtest / ShooterConstants)
//    private static final double MIN_HOOD_ANGLE = 26.0;
//    private static final double MAX_HOOD_ANGLE = 39.5;
//    private static final double GEAR_RATIO = 375.0 / 57.25;
//    private static final double SERVO_START_POS = 0.5;
//    private static final double SERVO_UNITS_PER_HOOD_DEGREE = GEAR_RATIO / 180.0;
//    private static final double MAX_REACHABLE_HOOD_ANGLE = MIN_HOOD_ANGLE
//            + (1.0 - SERVO_START_POS) * 180.0 / GEAR_RATIO;
//    private static final double TICKS_PER_REV = 28.0;
//
//    @Override
//    public void runOpMode() {
//        drivetrain    = new Drivetrain(hardwareMap);
//        intake        = new Intake(hardwareMap);
//        shooter       = new Shooter(hardwareMap);
//        turret        = new Turret(hardwareMap);
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(START_POSE);
//
//        telemetry.addLine("Ready. GP1 B=Toggle Goal Side | Bumpers=Shooter On | Triangle=Kill");
//        telemetry.update();
//
//        turret.centerTurret(this);
//
//        waitForStart();
//
//        shooter.initControllers();
//
//        while (opModeIsActive()) {
//            // Update odometry
//            follower.update();
//
//            if (gamepad2.a) turret.resetEncoder(this);
//
//            // Drive
//            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//
//            // Intake
//            intake.update(gamepad1.circle, voltageSensor, shooter.getOuttakeState() == Shooter.OuttakeState.IDLE);
//
//            // Shooter input (bumpers to start, triangle to kill)
//            shooter.handleShooterInput(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle, intake);
//
//            // Toggle goal side with B
//            boolean currentG1B = gamepad1.b;
//            if (currentG1B && !lastG1B) useRedGoal = !useRedGoal;
//            lastG1B = currentG1B;
//
//            // Auto-calculate distance, RPM, and hood from odometry
//            Pose currentPose = follower.getPose();
//            double goalX = useRedGoal ? GOAL_RED_X : GOAL_BLUE_X;
//            double goalY = useRedGoal ? GOAL_RED_Y : GOAL_BLUE_Y;
//            double dx = goalX - currentPose.getX();
//            double dy = goalY - currentPose.getY();
//            double distanceInches = Math.sqrt(dx * dx + dy * dy);
//            double distanceMeters = distanceInches * 0.0254;
//
//            LauncherSolution solution = MathLib.distanceToLauncherValues(distanceMeters);
//
//            if (shooter.isShooterOn() && solution.isValid()) {
//                double targetTicks = MathLib.interpolateToTicks(solution.velocityMs);
//                double targetRPM = targetTicks * 60.0 / TICKS_PER_REV;
//                shooter.setTargetRPM(targetRPM);
//
//                if (!Double.isNaN(solution.hoodAngleDeg)) {
//                    double effectiveMax = Math.min(MAX_HOOD_ANGLE, MAX_REACHABLE_HOOD_ANGLE);
//                    double clampedAngle = Math.max(MIN_HOOD_ANGLE,
//                            Math.min(effectiveMax, solution.hoodAngleDeg));
//                    double servoPos = SERVO_START_POS
//                            + (clampedAngle - MIN_HOOD_ANGLE) * SERVO_UNITS_PER_HOOD_DEGREE;
//                    servoPos = Math.max(0.0, Math.min(1.0, servoPos));
//                    shooter.setHoodAnglePos(servoPos);
//                }
//            }
//
//            // Outtake sequence + PIDF (uses the RPM we just set)
//            shooter.updateOuttakeSequence(intake, voltageSensor);
//            shooter.updatePIDF(voltageSensor, telemetry);
//
//            // Turret
//            turret.updateTuning(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.left_bumper);
//            turret.update(gamepad2.cross, gamepad2.right_stick_x, voltageSensor, telemetry, this);
//
//            // Telemetry
//            turret.addTelemetry(telemetry);
//            telemetry.addData("Goal", useRedGoal ? "RED (132,132)" : "BLUE (12,132)");
//            telemetry.addData("Robot Pos", String.format("(%.1f, %.1f)", currentPose.getX(), currentPose.getY()));
//            telemetry.addData("Distance", String.format("%.2f m (%.1f in)", distanceMeters, distanceInches));
//            if (solution.isValid()) {
//                double calcTicks = MathLib.interpolateToTicks(solution.velocityMs);
//                telemetry.addData("Calc Velocity", String.format("%.2f m/s", solution.velocityMs));
//                telemetry.addData("Calc Hood Angle", String.format("%.1f deg", solution.hoodAngleDeg));
//                telemetry.addData("Calc RPM", (int) (calcTicks * 60.0 / TICKS_PER_REV));
//            } else {
//                telemetry.addData("Calc", "IMPOSSIBLE SHOT");
//            }
//            telemetry.addData("Shooter RPM", (int) shooter.getTargetRPM());
//            telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodAnglePos()));
//            telemetry.update();
//        }
//
//        turret.stop();
//    }
//}
