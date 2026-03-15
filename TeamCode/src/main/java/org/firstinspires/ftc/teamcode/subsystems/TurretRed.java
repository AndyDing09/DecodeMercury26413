package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.TurretInterface;

@Config
public class TurretRed implements TurretInterface {

    private Servo axonServo;
    private Limelight3A limelight;

    // Tunable via FTC Dashboard
    public static double GEAR_RATIO = 45.0 / 110.0;
    public static double kP = 0.00007;
    public static double kD = 0.00;
    public static double DEADZONE = 2;
    public static double TX_SMOOTHING = 1; // 0.0 (max smooth) to 1.0 (no smooth)

    // Search & Tracking constants
    public static int LOSS_DEBOUNCE_FRAMES = 10;
    public static double SEARCH_STEP = 0.005;

    // State variables
    private int framesWithoutTarget = 0;
    private boolean searchDirectionPositive = true;
    private double lastTx = 0;
    private double lastError = 0;
    private double filteredTx = 0;

    // Telemetry state
    private String currentMode = "IDLE";
    private double currentTx = 0;
    private boolean currentlyHasTarget = false;

    public TurretRed(HardwareMap hardwareMap) {
        axonServo = hardwareMap.get(Servo.class, "TurnTable");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        axonServo.setPosition(0.5);
        framesWithoutTarget = LOSS_DEBOUNCE_FRAMES;
    }

    @Override
    public void update(double manualPower) {
        // 1. MANUAL OVERRIDE
        if (Math.abs(manualPower) > 0.1) {
            currentMode = "MANUAL";
            double newPosition = axonServo.getPosition() + manualPower * 0.01;
            newPosition = Math.max(0.0, Math.min(1.0, newPosition));
            axonServo.setPosition(newPosition);

            lastError = 0;
            framesWithoutTarget = 0;
            return;
        }

        // 2. AUTO-TRACKING LOGIC
        LLResult result = limelight.getLatestResult();
        currentlyHasTarget = (result != null && result.isValid());

        if (currentlyHasTarget) {
            // ===========================
            // TRACKING MODE
            // ===========================
            currentTx = result.getTx();

            // Low-pass filter — matches turntable: TX_SMOOTHING * tx + (1 - TX_SMOOTHING) * filteredTx
            if (framesWithoutTarget > 0) {
                filteredTx = currentTx; // Just re-acquired, reset filter to avoid lag
            } else {
                filteredTx = (TX_SMOOTHING * currentTx) + ((1.0 - TX_SMOOTHING) * filteredTx);
            }

            lastTx = filteredTx;
            framesWithoutTarget = 0;

            double error = filteredTx;
            double derivative = error - lastError;
            lastError = error;

            if (Math.abs(currentTx) > DEADZONE) {
                currentMode = "TRACKING";
                double rawCorrection = (kP * error) + (kD * derivative);
                double servoCorrection = rawCorrection / GEAR_RATIO;

                double newPosition = axonServo.getPosition() - servoCorrection;
                newPosition = Math.max(0.0, Math.min(1.0, newPosition));
                axonServo.setPosition(newPosition);
            } else {
                currentMode = "LOCKED";
            }
        } else {
            // ===========================
            // NO TARGET (DEBOUNCE -> SEARCH)
            // ===========================
            framesWithoutTarget++;
            currentTx = 0;

            if (framesWithoutTarget < LOSS_DEBOUNCE_FRAMES) {
                currentMode = "HOLDING";
            } else {
                currentMode = "SEARCHING";

                if (framesWithoutTarget == LOSS_DEBOUNCE_FRAMES) {
                    searchDirectionPositive = (lastTx < 0);
                }

                double currentPos = axonServo.getPosition();
                if (currentPos >= 1.0) searchDirectionPositive = false;
                else if (currentPos <= 0.0) searchDirectionPositive = true;

                double nextPos = currentPos + (searchDirectionPositive ? SEARCH_STEP : -SEARCH_STEP);
                axonServo.setPosition(Math.max(0.0, Math.min(1.0, nextPos)));
            }
        }
    }

    @Override
    public void setAlliance(boolean isRed) {
        limelight.pipelineSwitch(isRed ? 1 : 0);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== TURRET (SERVO) =====");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Target", currentlyHasTarget ? String.format("YES (tx: %.2f)", currentTx) : "NO");
        if (currentlyHasTarget) {
            telemetry.addData("TX (smooth)", String.format("%.2f", filteredTx));
        }
        telemetry.addData("Servo Position", String.format("%.4f", axonServo.getPosition()));
    }

    @Override
    public void resetEncoder() {
        axonServo.setPosition(0.5);
    }

    @Override
    public void stop() {
        limelight.stop();
        axonServo.setPosition(0.5);
    }
}