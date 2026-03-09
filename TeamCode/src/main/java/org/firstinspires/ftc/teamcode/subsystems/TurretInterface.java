package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Interface for a turret subsystem to allow for easy switching between
 * different implementations (e.g., Red and Blue alliance versions).
 */
public interface TurretInterface {
    /**
     * Updates the turret's state, including tracking and manual control.
     * @param manualPower Power for manual override, from -1.0 to 1.0.
     */
    void update(double manualPower);

    /**
     * Adds telemetry data for the turret to the provided telemetry object.
     * @param telemetry The telemetry object to add data to.
     */
    void addTelemetry(Telemetry telemetry);

    /**
     * Resets the turret's encoder position to zero.
     */
    void resetEncoder();

    /**
     * Stops the turret motor and any related processes.
     */
    void stop();
}