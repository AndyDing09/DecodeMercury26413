package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private final DcMotor frontLeft, backLeft, frontRight, backRight;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft .setDirection(DcMotor.Direction.REVERSE);
        backLeft  .setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight .setDirection(DcMotor.Direction.FORWARD);
    }

    public void drive(double axial, double lateral, double yaw) {
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeft .setPower(fl);
        frontRight.setPower(fr);
        backLeft  .setPower(bl);
        backRight .setPower(br);
    }
}
