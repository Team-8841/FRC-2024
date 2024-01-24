package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.SwerveConstants;

public class SimConstants {
    public static final double steeringKP = 0.3;
    public static final double steeringKI = 0;
    public static final double steeringKD = 0.01;

    public static final double driveKV = 0.115;
    public static final double driveKP = 0.02;
    public static final double driveKI = 0;
    public static final double driveKD = 0;

    public static final DCMotor steeringGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor driveGearbox = DCMotor.getFalcon500(1);

    public static final double steeringGearRatio = SwerveConstants.chosenModule.angleGearRatio;
    public static final double driveGearRatio = SwerveConstants.chosenModule.driveGearRatio;

    // Modeled as solid cylinders (kg*m^2)
    public static final double steeringInertia = 0.5 * 10 * Math.pow(0.1, 2);
    public static final double driveInertia = 0.5 * 10 * Math.pow(0.1, 2);

    public static final double wheelCircumference = SwerveConstants.chosenModule.wheelCircumference;
    public static final double wheelDiameter = SwerveConstants.chosenModule.wheelDiameter;
}
