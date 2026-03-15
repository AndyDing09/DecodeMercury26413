package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Intake {
    private final DcMotor middleTransfer;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double BASE_POWER = 0.5;

    private boolean intakeOn = false;
    private boolean lastCircle = false;
    public boolean isHighSpeed = true;
    private boolean lastSpeedToggle = false;

    public Intake(HardwareMap hardwareMap) {
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        middleTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update(boolean toggleIntake, boolean toggleSpeed, VoltageSensor voltageSensor, boolean outtakeIdle) {
        if (toggleIntake && !lastCircle) intakeOn = !intakeOn;
        lastCircle = toggleIntake;

        if (toggleSpeed && !lastSpeedToggle) isHighSpeed = !isHighSpeed;
        lastSpeedToggle = toggleSpeed;

        double basePower = isHighSpeed ? 1.0 : BASE_POWER;
        double intakePower = 1.0;
        
        if (intakeOn) {
            middleTransfer.setPower(intakePower);
        } else if (outtakeIdle) {
            middleTransfer.setPower(0);
        } else if (middleTransfer.getPower() > 0) {
            // If outtake sequence started the transfer, ensure it respects voltage compensation
            middleTransfer.setPower(basePower);
        }
    }

    public void runTransfer(VoltageSensor voltageSensor) {
        double scale = isHighSpeed ? 1.0 : 0.55;
        runTransfer(voltageSensor, scale);
    }

    public void runTransfer(VoltageSensor voltageSensor, double scale) {
        middleTransfer.setPower(Math.min(1.0, (NOMINAL_VOLTAGE / voltageSensor.getVoltage()) * scale));
    }

    public void stopIfNotIntaking() {
        if (!intakeOn) middleTransfer.setPower(0);
    }

    public boolean isOn() { return intakeOn; }
}
