package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Transfe_Intake")
public class Transfe_Intake extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor Intake1;
    private DcMotor Intake2;
    private DcMotor Shooter;
    private Servo Transfer;

    // Individual toggle states
    private boolean intakeOn = false;
    private boolean lastCircle = false;

    private boolean shooterOn = false;
    private boolean lastTriangle = false;
    // Servo positions
    private double servoHome = 0.075;
    private double servoExtendedPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Intake1 = hardwareMap.dcMotor.get("Intake1");
        Intake2 = hardwareMap.dcMotor.get("Intake2");
        Transfer = hardwareMap.servo.get("Transfer");
        Shooter = hardwareMap.dcMotor.get("Shooter");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        // Reverse one intake motor
        Intake2.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        Transfer.setPosition(servoHome);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }
            // --------- INTAKE TOGGLE (Circle) ----------
            boolean circle = gamepad1.circle;

            if (circle && !lastCircle) {
                intakeOn = !intakeOn;
            }
            lastCircle = circle;

            if (intakeOn) {
                Intake1.setPower(1.0);
                Intake2.setPower(1.0);
            } else {
                Intake1.setPower(0);
                Intake2.setPower(0);
            }


            // --------- SERVO TOGGLE (X) ----------
            if (gamepad1.left_bumper) {
                Transfer.setPosition(servoHome);
            } else if (gamepad1.right_bumper) {
                Transfer.setPosition(servoExtendedPos);
            }



            // --------- SHOOTER TOGGLE (Triangle) ----------
            boolean triangle = gamepad1.triangle;

            if (triangle && !lastTriangle) {
                shooterOn = !shooterOn;
            }
            lastTriangle = triangle;

            if (shooterOn) {
                Shooter.setPower(-0.5);
            } else {
                Shooter.setPower(0);
            }


            // --------- TELEMETRY ----------
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Servo Position", Transfer.getPosition());
            telemetry.update();
        }
    }
}
