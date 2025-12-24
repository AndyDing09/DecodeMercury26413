package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="limelight", group="Vision")
public class limelight extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    private Limelight3A limelight;
    private Servo turnTable2;

    // Servo position (0..1)
    private double servoPos = 0.5;

    // ---------- TUNING ----------
    private static final double kP = 0.160;
    private static final double kD = 0.050;
    private static final double kI= 0.030;
    private static final double MAX_STEP = 0.010;

    // Hysteresis (kills oscillation)
    private static final double TX_STOP  = 0.7;
    private static final double TX_START = 1.3;

    private double time = 0.0;
    private double prevTime = 0.0;

    private double dt = 0.0;
    private double cdt = 0.0; // time difference for control system

    // tx low-pass filter
    private static final double ALPHA = 0.90;
    private double txFiltered = 0.0;
    private double lastTxFiltered = 0.0;

    // Protect mechanism
    private static final double SERVO_MIN = 0.05;
    private static final double SERVO_MAX = 0.95;

    // Flip if direction is wrong
    private double d_error = 0.0;
    private double i_error = 0.0;

    private boolean moving = false;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turnTable2 = hardwareMap.get(Servo.class, "TurnTable");

        // ---- Limelight setup ----
        limelight.setPollRateHz(100);
        limelight.start();

        // 🔴 FORCE PIPELINE 0 (AprilTag pipeline)
        limelight.pipelineSwitch(0);

        turnTable2.setPosition(servoPos);

        telemetry.addLine("Limelight forced to pipeline 0");
        telemetry.addLine("Press PLAY");
        telemetry.update();

        waitForStart();
        time = timer.milliseconds();
        prevTime = time;
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                // Low-pass filter
                txFiltered = (1.0 - ALPHA) * txFiltered + ALPHA * tx;

                // Hysteresis logic
                if (moving) {
                    if (Math.abs(txFiltered) < TX_STOP) moving = false;
                } else {
                    if (Math.abs(txFiltered) > TX_START) moving = true;
                }

                time = timer.milliseconds();
                dt = time - prevTime;
                prevTime = time;
                cdt = cdt + dt;

                if (moving &&  (cdt > 18)) {
                    d_error = (txFiltered - lastTxFiltered) * 1000.0/ (cdt);
                    d_error = clamp(d_error, -300.0, 300.0);
                    i_error = (i_error + txFiltered) * cdt / 1000.0;
                    i_error = clamp(i_error, -3.0, 3.0);

                    double delta = (txFiltered * kP) + (i_error * 0.0) + (d_error * kD);
                    delta = clamp(delta, -MAX_STEP, MAX_STEP);

                    lastTxFiltered = txFiltered;

                    servoPos = clamp(servoPos + delta, SERVO_MIN, SERVO_MAX);
                    turnTable2.setPosition(servoPos);
                    cdt = 0.0;
                }

                telemetry.addData("Target", "YES");
                telemetry.addData("tx raw", "%.2f", tx);
                telemetry.addData("tx filt", "%.2f", txFiltered);
                telemetry.addData("d_error", "%.2f", d_error);
                telemetry.addData("moving", moving);
                telemetry.addData("servo", "%.3f", servoPos);


            } else {
                telemetry.addData("Target", "NO");
                telemetry.addData("servo", "%.3f", servoPos);
            }

            telemetry.update();
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
