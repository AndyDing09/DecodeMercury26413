package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Intake {
    private final DcMotor middleTransfer;
    private static final double NOMINAL_VOLTAGE = 12.0;

    private boolean intakeOn = false;
    private boolean lastCircle = false;
    private boolean isHighSpeed = true;
    private boolean lastSpeedToggle = false;

    public Intake(HardwareMap hardwareMap) {
        middleTransfer = hardwareMap.get(DcMotor.class, "middleTransfer");
        middleTransfer.setDirection(DcMotor.Direction.FORWARD);
        middleTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update(boolean toggleIntake, boolean toggleSpeed, boolean reverse, VoltageSensor voltageSensor, boolean outtakeIdle) {
        if (toggleIntake && !lastCircle) intakeOn = !intakeOn;
        lastCircle = toggleIntake;

        if (toggleSpeed && !lastSpeedToggle) isHighSpeed = !isHighSpeed;
        lastSpeedToggle = toggleSpeed;

        if (intakeOn) {
            double intakePower = 1.0;
            if (!isHighSpeed) {
                intakePower = 0.55; // low speed scale
            }
            if (reverse) {
                intakePower = -intakePower;
            }
            middleTransfer.setPower(intakePower);
        } else if (outtakeIdle) {
            middleTransfer.setPower(0);
        }
    }

    public void update(boolean circlePressed, VoltageSensor voltageSensor, boolean outtakeIdle) {
        update(circlePressed, false, false, voltageSensor, outtakeIdle);
    }

    public void update(boolean circlePressed, VoltageSensor voltageSensor, boolean outtakeIdle, boolean reverse) {
        update(circlePressed, false, reverse, voltageSensor, outtakeIdle);
    }

    public void runTransfer(VoltageSensor voltageSensor) {
        runTransfer(voltageSensor, 1.0);
    }

    public void runTransfer(VoltageSensor voltageSensor, double scale) {
        middleTransfer.setPower(Math.min(1.0, (NOMINAL_VOLTAGE / voltageSensor.getVoltage()) * scale));
    }

    public void stopIfNotIntaking() {
        if (!intakeOn) middleTransfer.setPower(0);
    }

    public boolean isOn() { return intakeOn; }

    public boolean isHighSpeed() { return isHighSpeed; }
}
