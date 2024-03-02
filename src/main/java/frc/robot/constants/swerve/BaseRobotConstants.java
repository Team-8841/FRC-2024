package frc.robot.constants.swerve;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;


public class BaseRobotConstants {
    public static final TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    /* Angle Motor PID Values */
    public static final double angleKP = 1;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.010009775171065494; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKS = 0.32 / 12;
    public static final double driveKV = 1.51 / 12;
    public static final double driveKA = 0.27 / 12;

    /* Neutral Modes */
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-348.92578125);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 0);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { 
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 6;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-298.65234375);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 1);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-76.025390625);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 2);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 10;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-60.29296875000001);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 3);
    }

    static {
        /* Drive motor */

        driveMotorConfigs.Slot0 = new Slot0Configs();
        driveMotorConfigs.Slot0.kP = driveKP;
        driveMotorConfigs.Slot0.kI = driveKI;
        driveMotorConfigs.Slot0.kD = driveKD;
        driveMotorConfigs.Slot0.kS = driveKS;
        driveMotorConfigs.Slot0.kV = driveKV;
        driveMotorConfigs.Slot0.kA = driveKA;

        driveMotorConfigs.CurrentLimits = new CurrentLimitsConfigs();
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimit;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveContinuousCurrentLimit;
        driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.driveInfCurrentThreshold;
        driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = SwerveConstants.driveInfCurrentDuration;

        driveMotorConfigs.OpenLoopRamps = new OpenLoopRampsConfigs();
        driveMotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;

        driveMotorConfigs.ClosedLoopRamps = new ClosedLoopRampsConfigs();
        driveMotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

        driveMotorConfigs.MotorOutput = new MotorOutputConfigs();
        driveMotorConfigs.MotorOutput.Inverted = SwerveConstants.driveMotorInvert == InvertedValue.Clockwise_Positive ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveMotorConfigs.MotorOutput.NeutralMode = driveNeutralMode;

        driveMotorConfigs.MotionMagic = new MotionMagicConfigs();
        driveMotorConfigs.MotionMagic.MotionMagicAcceleration = Conversions.metersToRots(SwerveConstants.maxAcceleration,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        driveMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Conversions.metersToRots(SwerveConstants.maxJerk,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
    }
}
