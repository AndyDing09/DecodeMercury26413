package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Transfer Encoder Test")
public class TransferEncoderTest extends OpMode {

    private DcMotorEx transferMotor;
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "middleTransfer");
        telemetry.addLine("Transfer Encoder Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.circle) {
            transferMotor.setPower(1);
        }
        double velocityTicks = transferMotor.getVelocity();
        double rpm = velocityTicks * 60.0 / TICKS_PER_REV;
        int position = transferMotor.getCurrentPosition();

        telemetry.addData("Encoder Pos", position);
        telemetry.addData("Velocity (ticks/s)", String.format("%.1f", velocityTicks));
        telemetry.addData("RPM", String.format("%.0f", rpm));
        telemetry.update();
    }
}
