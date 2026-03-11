package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Limelight Servo Tuner", group = "TeleOp")
public class turntable extends OpMode {

    private Servo axonServo;
    private Limelight3A limelight;

    // Tunable via FTC Dashboard
    public static double GEAR_RATIO = 45.0 / 110.0;
    public static double kP = 0.00007;
    public static double kD = 0.00;
    public static double DEADZONE = 2.0;
    public static double TX_SMOOTHING = 1; // 0.0 (max smooth) to 1.0 (no smooth)

    // Search & Tracking constants
    public static int LOSS_DEBOUNCE_FRAMES = 10;
    public static double SEARCH_STEP = 0.005; // Amount to move servo per loop during search

    // State variables
    private int framesWithoutTarget = 0;
    private boolean searchDirectionPositive = true; // true = moving towards 1.0, false = moving towards 0.0
    private double lastTx = 0;
    private double lastError = 0;
    private double filteredTx = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        axonServo = hardwareMap.get(Servo.class, "turntable");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        axonServo.setPosition(0.5);
        framesWithoutTarget = LOSS_DEBOUNCE_FRAMES; // Force filter reset on first acquisition
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Connect to FTC Dashboard to tune variables.");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult(); // fixed

        boolean hasTarget = (result != null && result.isValid());
        String mode;

        if (hasTarget) {
            // ===========================
            // TRACKING MODE
            // ===========================
            mode = "TRACKING";
            double tx = result.getTx();

            // Low-pass filter to smooth out oscillating signals
            // If we just re-acquired the target, reset the filter to avoid lag
            if (framesWithoutTarget > 0) filteredTx = tx;
            else filteredTx = (TX_SMOOTHING * tx) + ((1.0 - TX_SMOOTHING) * filteredTx);

            // Update state for search logic
            lastTx = filteredTx;
            framesWithoutTarget = 0;

            double error = filteredTx;
            double derivative = error - lastError;
            lastError = error;

            if (Math.abs(tx) > DEADZONE) {
                double rawCorrection = (kP * error) + (kD * derivative);
                double servoCorrection = rawCorrection / GEAR_RATIO;

                double newPosition = axonServo.getPosition() - servoCorrection;
                newPosition = Math.max(0.0, Math.min(1.0, newPosition));

                axonServo.setPosition(newPosition);
                telemetry.addData("Correction", servoCorrection);
            } else {
                mode = "LOCKED";
            }

            telemetry.addData("TX (raw)", tx);
            telemetry.addData("TX (smooth)", filteredTx);
        } else {
            // ===========================
            // NO TARGET (DEBOUNCE -> SEARCH)
            // ===========================
            framesWithoutTarget++;

            if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
                mode = "HOLDING";
                // Hold current position
            } else {
                mode = "SEARCHING";

                // If we just entered search mode, pick a direction based on where we last saw the target
                if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                    // If lastTx > 0 (right), we likely need to decrease servo pos (based on tracking math) to follow it
                    // If lastTx < 0 (left), we likely need to increase servo pos
                    searchDirectionPositive = (lastTx < 0);
                }

                double currentPos = axonServo.getPosition();
                // Sweep back and forth
                if (currentPos >= 1.0) searchDirectionPositive = false;
                else if (currentPos <= 0.0) searchDirectionPositive = true;

                double nextPos = currentPos + (searchDirectionPositive ? SEARCH_STEP : -SEARCH_STEP);
                axonServo.setPosition(Math.max(0.0, Math.min(1.0, nextPos)));
            }
        }

        telemetry.addData("Mode", mode);
        telemetry.addData("Servo Position", axonServo.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        axonServo.setPosition(0.5);
    }
}