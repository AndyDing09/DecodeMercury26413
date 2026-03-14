package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Intake {
    private final DcMotor middleTransfer;
    private static final double NOMINAL_VOLTAGE = 12.0;

    private boolean intakeOn = false;
    private boolean lastCircle = false;
    private boolean slowMode = false;
    private boolean lastA = false;

    public Intake(HardwareMap hardwareMap) {
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        middleTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSlowMode(boolean aPressed) {
        if (aPressed && !lastA) slowMode = !slowMode;
        lastA = aPressed;
    }

    public void update(boolean circlePressed, VoltageSensor voltageSensor, boolean outtakeIdle) {
        if (circlePressed && !lastCircle) intakeOn = !intakeOn;
        lastCircle = circlePressed;

        if (intakeOn) {
            double intakePower = Math.min(1.0, NOMINAL_VOLTAGE / voltageSensor.getVoltage()) * (slowMode ? 0.65 : 1.0);
            middleTransfer.setPower(intakePower);
        } else if (outtakeIdle) {
            middleTransfer.setPower(0);
        }
    }

    public void update(boolean circlePressed, VoltageSensor voltageSensor, boolean outtakeIdle, boolean reverse) {
        if (circlePressed && !lastCircle) intakeOn = !intakeOn;
        lastCircle = circlePressed;

        if (reverse) {
            double reversePower = -Math.min(1.0, NOMINAL_VOLTAGE / voltageSensor.getVoltage());
            middleTransfer.setPower(reversePower);
        } else if (intakeOn) {
            double intakePower = Math.min(1.0, NOMINAL_VOLTAGE / voltageSensor.getVoltage()) * (slowMode ? 0.65 : 1.0);
            middleTransfer.setPower(intakePower);
        } else if (outtakeIdle) {
            middleTransfer.setPower(0);
        }
    }

    public void runTransfer(VoltageSensor voltageSensor) {
        middleTransfer.setPower(Math.min(1.0, NOMINAL_VOLTAGE / voltageSensor.getVoltage()) * 0.65);
    }

    public void stopIfNotIntaking() {
        if (!intakeOn) middleTransfer.setPower(0);
    }

    public boolean isOn() { return intakeOn; }
}