
package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Various constants used throughout the program are defined here. They're
 * defined here instead of elsewhere just for ease of changing them.
 */
public final class Constants {
    public static final boolean simReplay = false;
    public static final double controllerDeadband = 0.1;
    public static final int pigeonId = 10;

    public static final int testSampleCount = 1024;
    public static final int testStepCount = 5;
    public static final double testMaxVoltage = 0.2;
    public static final double testMinVoltage = 0.05;
    public static final double testMaxDistance = 2;
    public static final String testMeasurementFName = "test_measurement.csv";
    public static final String testResidualsFName = "test_residuals.csv";

    public static final DriverStation.Alliance defaultAlliance = DriverStation.Alliance.Blue;
}
