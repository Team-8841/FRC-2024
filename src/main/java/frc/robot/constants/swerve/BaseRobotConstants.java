package frc.robot.constants.swerve;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;


public class BaseRobotConstants {
    public static final TalonFXConfiguration angleMotorConfigs = new TalonFXConfiguration();

    /* Angle Motor PID Values */
    public static final double angleKP = SwerveConstants.chosenModule.angleKP;
    public static final double angleKI = SwerveConstants.chosenModule.angleKI;
    public static final double angleKD = SwerveConstants.chosenModule.angleKD;
    public static final double angleKF = SwerveConstants.chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKS = 0; 
    public static final double driveKV = 0.1;
    public static final double driveKA = 0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = 19;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.728515625);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 0);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1  {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 5;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(235.01953124999997);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 1);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 8;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-41.39648438);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 2);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 9;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.185546875);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 3);
    }

    static {
        angleMotorConfigs.Slot0 = new Slot0Configs();
        angleMotorConfigs.Slot0.kP = angleKP;
        angleMotorConfigs.Slot0.kI = angleKI;
        angleMotorConfigs.Slot0.kD = angleKD;
        angleMotorConfigs.Slot0.kV = angleKF;

        angleMotorConfigs.CurrentLimits = new CurrentLimitsConfigs();
        angleMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        angleMotorConfigs.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleContinuousCurrentLimit;
        angleMotorConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.angleInfCurrentThreshold;
        angleMotorConfigs.CurrentLimits.SupplyTimeThreshold = SwerveConstants.angleInfCurrentDuration;

        angleMotorConfigs.ClosedLoopGeneral = new ClosedLoopGeneralConfigs();
        angleMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotorConfigs.MotorOutput = new MotorOutputConfigs();
        angleMotorConfigs.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
        angleMotorConfigs.MotorOutput.NeutralMode = angleNeutralMode;
    }
}
