package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp1")
public class TeleOp1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor Intake1;
    private DcMotor Intake2;
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor1;
    private Servo Transfer;
    private Servo TurnTable2;

    // Individual toggle states
    private boolean intakeOn = false;
    private boolean lastCircle = false;
    private boolean lastleftBumper = false;
    private boolean lastrightBumper = false;
    private boolean shooterOn = false;
    // Servo positions
    private double servoHome = 0.075;
    private double servoExtendedPos = 0;
    private double turntableStart = 0.1;
    private double targetRPM = 0;      // start speed// change step
    private final double MAX_RPM = 6000;  // depends on motor type
    private final double MIN_RPM = 0;
    private double fastRPM = 6000;
    private double slowRPM = 3000;




    @Override
    public void runOpMode() throws InterruptedException {

        Intake1 = hardwareMap.dcMotor.get("Intake1");
        Intake2 = hardwareMap.dcMotor.get("Intake2");
        Transfer = hardwareMap.servo.get("Transfer");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        TurnTable2 = hardwareMap.servo.get("TurnTable");
        // Reverse one intake motor
        Intake2.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setPosition(servoHome);
        TurnTable2.setPosition(turntableStart);
        waitForStart();

        while (opModeIsActive()) {
            double max;
//ALL RELATED TO DRIVING  DON'T TOUCH
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 0.7) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
//END OF DON'T TOUCH DRIVING

            // --------- TURNTABLE SERVO INCREMENT CONTROL (DPAD UP/DOWN) ----------

            if (gamepad2.left_bumper) {
                TurnTable2.setPosition(turntableStart);
            } else if (gamepad2.right_bumper) {
                TurnTable2.setPosition(servoExtendedPos);
            }

            // --------- Intake on/off --------
            // Gamepad1.circle turn intake on (click once) and off (click again)
            boolean circle = gamepad1.circle;

            if (circle && !lastCircle) {
                intakeOn = !intakeOn;
            }
            lastCircle = circle;

            if (intakeOn) {
                Intake1.setPower(1);
                Intake2.setPower(1);
            } else {
                Intake1.setPower(0);
                Intake2.setPower(0);
            }

            // --------- BACKUP Button for Transfer from Intake to shooter ----------
            // dpad_up and dpad_down to toggle the transfer servo
            // servo initial position down
            if (gamepad1.dpad_up) {
                Transfer.setPosition(servoHome);
            } else if (gamepad1.dpad_down) {
                Transfer.setPosition(servoExtendedPos);
            }

            // --------- Control motors for shooter ----------
            // Left_bumper turn on (click once) and off (click again) motors at fastRPM (for shooting from far)
            // Right_bumper turn on (click once) and off (click again) motors at slowRPM (for shooting from closeby)
            boolean leftBumper = gamepad1.left_bumper;

            if (leftBumper && lastleftBumper) {
                shooterOn = !shooterOn;
            }

            lastleftBumper = leftBumper;

            boolean rightBumper = gamepad1.right_bumper;

            if (rightBumper && lastrightBumper) {
                shooterOn = !shooterOn;
            }

            lastrightBumper = rightBumper;

//            if (shooterOn) {
//                shooterMotor.setPower(0.8);
//                shooterMotor1.setPower(0.8);
//            } else {
//                shooterMotor.setPower(0);
//                shooterMotor1.setPower(0);
//            }
             if (gamepad1.left_bumper) {
                 targetRPM = fastRPM;
                 sleep(200); // debounce
             } else if (gamepad1.right_bumper) {
                 targetRPM = slowRPM;
                 sleep(200);
             }

             //Clamp RPM
             targetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, targetRPM));

            // Convert RPM to ticks per second (GoBilda 312RPM motor = 537.7 ticks/rev)
            double ticksPerSec = (targetRPM * 537.7) / 60.0;

            // Set velocity
            shooterMotor.setVelocity(ticksPerSec);
            shooterMotor1.setVelocity(ticksPerSec);

            // -------- REVERSE Intake direction to spit out ball --------
            // Gamepad1.cross button click once to reserver intake if intake is off
            // Gamepad1.cross button hold to reverse intake if intake is on
            if (gamepad1.cross) {
                //Turn intake OFF
                Intake1.setPower(-1.0);
                Intake2.setPower(-1.0);
            }
            // --------- MAIN Transfer code: Transfer Macro ----------
            // Gamepad1.square button initiates the transfer process by first turn off intake, then turn on transfer servo
            if (gamepad1.square) {

                // Save previous intake state
                boolean wasIntakeOn = intakeOn;

                // Turn intake OFF
                Intake1.setPower(0);
                Intake2.setPower(0);

                // Move servo forward
                Transfer.setPosition(servoExtendedPos);

                // Wait 0.5 seconds
                sleep(500);

                // Move servo back
                Transfer.setPosition(servoHome);

                // If intake was ON before, turn it back on
                if (wasIntakeOn) {
                    Intake1.setPower(1);
                    Intake2.setPower(1);
                }

                // Small debounce so holding Square doesn't spam the sequence
                sleep(250);
            }


            // --------- TELEMETRY for driving and shooter calculation ----------
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Servo Position", Transfer.getPosition());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual Velocity (ticks/s)", shooterMotor.getVelocity());
            telemetry.addData("Actual RPM", (shooterMotor.getVelocity() * 60) / 537.7);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual Velocity (ticks/s)", shooterMotor1.getVelocity());
            telemetry.addData("Actual RPM", (shooterMotor1.getVelocity() * 60) / 537.7);
            telemetry.update();
        }
    }
}



