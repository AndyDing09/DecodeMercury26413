package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name="LimelightServoTuner", group="Test")
public class limelight extends LinearOpMode {

    private Servo TurnTable;
    private Limelight3A limelight;

    // ================= CR SERVO CONTROL =================
    private static final double SERVO_STOP = 0.5;  // CR servo neutral
    private double kP = 0.006;  // proportional gain
    private double kD = 0.001;   // derivative gain
    private double DEADZONE_DEG = 0.5;

    private double lastTx = 0;
    private double lastTime = 0;

    // Step sizes for tuning
    private final double[] stepSizes = {0.0001, 0.0005, 0.001, 0.005, 0.01};
    private int stepIndex = 2;  // default starting step size

    // Edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastSquare = false;

    // Servo direction fix
    private int servoDirection = 1; // 1 = normal, -1 = reversed

    @Override
    public void runOpMode() {

        // ================= HARDWARE =================
        TurnTable = hardwareMap.servo.get("TurnTable");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // ================= INIT =================
        TurnTable.setPosition(SERVO_STOP);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight + CR Servo Ready");
        telemetry.addLine("Dpad U/D: P +/- | Dpad L/R: D +/- | Square: step size");
        telemetry.update();

        waitForStart();
        lastTime = getRuntime();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            double now = getRuntime();
            double dt = Math.max(now - lastTime, 0.01); // prevent divide by zero
            lastTime = now;

            // ================= PD CONTROLLER =================
            LLResult result = limelight.getLatestResult();
            double servoCmd = SERVO_STOP;

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                // flip the sign for correct robot direction
                double error = servoDirection * -tx;

                double dError = (error - lastTx) / dt;
                lastTx = error;

                if (Math.abs(error) > DEADZONE_DEG) {
                    servoCmd = SERVO_STOP + kP * error + kD * dError;
                    servoCmd = clamp(servoCmd, 0.0, 1.0);
                } else {
                    servoCmd = SERVO_STOP;
                }

                telemetry.addData("Target Found", "YES");
                telemetry.addData("tx", "%.2f", tx);

            } else {
                servoCmd = SERVO_STOP;
                lastTx = 0;
                telemetry.addData("Target Found", "NO");
            }

            TurnTable.setPosition(servoCmd);

            // ================= CONTROLLER TUNING =================
            // Edge detection for single press increment
            if (gamepad1.dpad_up && !lastDpadUp)    kP += stepSizes[stepIndex];
            if (gamepad1.dpad_down && !lastDpadDown)  kP = Math.max(0, kP - stepSizes[stepIndex]);
            if (gamepad1.dpad_right && !lastDpadRight) kD += stepSizes[stepIndex];
            if (gamepad1.dpad_left && !lastDpadLeft)  kD = Math.max(0, kD - stepSizes[stepIndex]);

            // Cycle step size
            if (gamepad1.square && !lastSquare) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
                sleep(200); // debounce
            }

            // save last state for edge detection
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;
            lastSquare = gamepad1.square;

            // ================= TELEMETRY =================
            telemetry.addData("Servo Cmd", servoCmd);
            telemetry.addData("kP (Dpad U/D)", kP);
            telemetry.addData("kD (Dpad L/R)", kD);
            telemetry.addData("Step (Square)", stepSizes[stepIndex]);
            telemetry.addData("Deadzone", DEADZONE_DEG);
            telemetry.addData("Servo Direction", servoDirection);
            telemetry.update();
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
