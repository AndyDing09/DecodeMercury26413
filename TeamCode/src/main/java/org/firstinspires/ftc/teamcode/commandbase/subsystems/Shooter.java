package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Shooter {
    private final double[] distances = {
            24.508, 39.477, 65.4485, 87.6797, 112.16, 118.97, 128.7672 ,155.791
    };
    private final double[] velocities = {
            1200, 1300, 1340, 1540, 1700, 1740, 1800, 2000
    };
    private final double[] hoodAngles = {
            29, 38, 45, 50, 52, 52.5, 52.5, 52.5
    };

    private final InterpLUT velocityLut = new InterpLUT();
    private final InterpLUT hoodAngleLut = new InterpLUT();
}
