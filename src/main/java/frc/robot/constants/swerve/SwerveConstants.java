package frc.robot.constants.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;


public class SwerveConstants {
    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
            .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    //public static final double trackWidth = Units.inchesToMeters(22.75); // TODO: This must be tuned to specific
                                                                            // robot
    public static final double compTrackWidth = Units.inchesToMeters(25);
    //public static final double wheelBase = Units.inchesToMeters(24.5); // TODO: This must be tuned to specific
                                                                        // robot
    public static final double compWheelBase = Units.inchesToMeters(23.25);
    public static final double driveBaseTrackWidth = Units.inchesToMeters(26.75);
    public static final double driveBaseWheelBase = Units.inchesToMeters(18);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
        * Swerve Kinematics
        * No need to ever change this unless you are not doing a traditional
        * rectangular/square 4 module swerve
        */

    public static final Translation2d[] compModulePositions = {
        new Translation2d(compWheelBase / 2.0, compTrackWidth / 2.0),
        new Translation2d(compWheelBase / 2.0, -compTrackWidth / 2.0),
        new Translation2d(-compWheelBase / 2.0, compTrackWidth / 2.0),    
        new Translation2d(-compWheelBase / 2.0, -compTrackWidth / 2.0)
    };

    public static final Translation2d[] driveBaseModulePositions = {
        new Translation2d(driveBaseWheelBase / 2.0, driveBaseTrackWidth / 2.0),
        new Translation2d(driveBaseWheelBase / 2.0, -driveBaseTrackWidth / 2.0),
        new Translation2d(-driveBaseWheelBase / 2.0, driveBaseTrackWidth / 2.0),    
        new Translation2d(-driveBaseWheelBase / 2.0, -driveBaseTrackWidth / 2.0)
    };

    public static final SwerveDriveKinematics compSwerveKinematics = new SwerveDriveKinematics(compModulePositions);
    public static final SwerveDriveKinematics driveBaseSwerveKinematics = new SwerveDriveKinematics(driveBaseModulePositions);

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderDir = chosenModule.canCoderDir == SensorDirectionValue.Clockwise_Positive ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;

    /* Swerve Current Limiting */
    /*
        * When using the NEO, only *PeakCurrentLimit and *ContinousCurrentLimit are
        * used
        */
    public static final int angleContinuousCurrentLimit = 10;
    public static final int angleInfCurrentThreshold = 10;
    public static final double angleInfCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int driveInfCurrentThreshold = 35;
    public static final double driveInfCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
        * These values are used by the drive falcon to ramp in open loop and closed
        * loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
        */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3; // m/s
    public static final double maxAcceleration = 3; // m/s^2
    public static final double maxJerk = 30; // m/s^3

    public static final double maxAngularVelocity = 3; // rad/s
}
