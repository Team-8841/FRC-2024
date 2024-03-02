package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.util.Gains;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {

        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-78.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-49.04);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(41.48);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-164.79);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {

        /* Motor IDs */
        public static final int kIntakeMotor = 14;
        public static final int kIndexMotor = 15;

        /* Sensor IDs */
        public static final int indexSensor = 0;

        /* Intake Motor Speeds */
        public static final double kIntakeInSpeed = 0.7f;
        public static final double kIntakeOutSpeed = -0.7;

        public static final double kIndexInSpeed = 0.9f;
        public static final double kIndexOutSpeed = -0.7f;
    }

    public static final class ShooterConstants {
        
        /* Motor IDs */
        public static final int kShooterMotor = 16;
        public static final int kFollowerMotor = 17;

        public static final int kEndeffectorID = 18;
        public static final int kEndEffectorRollerID = 19;

        /* Shooter PID and limits */
        public static final int kShooterCurrentLimit = 40;
        public static final double kShooterAllowedError = 200;

        /* Shooter Setpoints */
        public static final double kSubShotSpeed = 1000;
        public static final double kFarrShotSpeed1 = 2500;
        public static final double kFarShotSpeed2 = 3000;
        public static final double kAmpShotSpeed = 500;

        /* End Effector PID */
        public static final double kEE_kP = 0.013;
        public static final double kEE_kI = 0;
        public static final double kEE_kD = 0;

        /* End Effector Setpoints */
        public static final double kEEHome = 0;
        public static final double kEEDeployed = 64;

        /* Sensors */
        public static final int kEEInSensor = 1;
        public static final int kEEOutSensor = 2;

        


    }

    public static final class ElevatorConstants {
        public static final int kElevatorMain = 20;
        public static final int kElevatorFollower = 21;

        public static final int kElevatorBottomSensor = 3;
        public static final int kElevatorTopSensor = 4;

        public static final int kBreaksPort = 2;
    }

    public static final class CandleConstants {
        public static final int kCandleID = 22;

        public static final int kLEDCount = 300;

        public static final double kMaxBrightness = 0.7;
    }

    public static final class VisionConstants {
        public static final double kAssistkP = 0.085;

        public static final double kMinAssistSpeed = 0.2;
    }

    public static class OIConstants {

        /*-------------------------------- Devices --------------------------------*/
        public static final int gamepadPort = 0; // Xbox controller
        public static final int copilotPort = 1; // Copilot controller
    
        /*-------------------------------- Gamepad variables --------------------------------*/
        public static final int buttonA     = 1;    // Not used
        public static final int buttonB     = 2;    // Not used
        public static final int buttonX     = 3;    // Not used
        public static final int buttonY     = 4;    // Not used
        public static final int buttonLB    = 5;    // Not used
        public static final int buttonRB    = 6;    // Not used
        public static final int buttonSel   = 7;    // Zero Heading
        public static final int buttonSrt   = 8;    // Not used
        
        public static final int axisLX      = 0;    // Not used
        public static final int axisLY      = 1;    // Not used
        public static final int triggerL    = 2;    // Not used
        public static final int triggerR    = 3;    // Not used
        public static final int axisRX      = 4;    // Not used
        public static final int axisRY      = 5;    // Not used
    
    
        /*-------------------------------- Copilot variables --------------------------------*/
    
        public static final int button1         = 1;    // Compressor Override
        public static final int button2         = 2;    // Deploy Hood
        public static final int button3         = 3;    // Hood Override
        public static final int button4         = 4;    // Intake In
        public static final int button5         = 5;    // Intake Out
        public static final int button6         = 6;    // Shooter Angle
        public static final int button7         = 7;    // Climber Breaks
        public static final int button8         = 8;    // Not used
        public static final int button9         = 9;    // Not used
        public static final int button10        = 10;   // Not used
        public static final int button11        = 11;   // Not used
        public static final int button12        = 12;   // Not used
        public static final int button13        = 13;   // Not used
        public static final int button14        = 14;   // Not used
        public static final int button15        = 15;   // Not used
        public static final int button16        = 16;   // Not used
        public static final int button17        = 17;   // Not used
        public static final int button18        = 18;   // Not used
        public static final int button19        = 19;   // Not used
        public static final int button20        = 20;   // Not used
        public static final int button21        = 21;   // Not used
        public static final int button22        = 22;   // Not used
    
        public static final int analog1         = 0;    // Not used
        public static final int analog2         = 1;    // Climber Up/Down
        public static final int analog3         = 2;    // Not used
        public static final int analog4         = 3;    // Shooter Speed
        public static final int analog5         = 4;    // Not used
        public static final int analog6         = 5;    // Not used
        public static final int analog7         = 6;    // Not used
        public static final int analog8         = 7;    // Not used
        
    }

}
