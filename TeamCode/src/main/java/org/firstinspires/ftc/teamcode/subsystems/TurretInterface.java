package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface TurretInterface {
    void update(double manualPower);

    //    public void update(double manualPower) {

    void setAlliance(boolean isRed);
    void addTelemetry(Telemetry telemetry);
    void resetEncoder();
    void stop();
}