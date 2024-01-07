package frc.robot.constants.swerve;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;


public class MixedMotorConstants {
    public static final TalonFXConfiguration angleMotorConfigs = new TalonFXConfiguration();

    /* Angle Motor PID Values */
    //public static final double angleKP = SwerveConstants.chosenModule.angleKP;
    public static final double angleKP = SwerveConstants.chosenModule.angleKP;
    public static final double angleKI = SwerveConstants.chosenModule.angleKI;
    public static final double angleKD = SwerveConstants.chosenModule.angleKD;
    public static final double angleKF = SwerveConstants.chosenModule.angleKF;

    /* Drive Motor PID Values */
    //public static final double driveKP = 0.010009775171065494; // TODO: This must be tuned to specific robot
    //public static final double driveKI = 0.0;
    //public static final double driveKD = 0.0;
    public static final double driveKP = 0.0; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    // TODO: Check if these need to be converted.
    //public static final double driveKS = (0.32 / 12); 
    //public static final double driveKV = (1.51 / 12);
    //public static final double driveKA = (0.27 / 12);
    public static final double driveKS = 0; 
    public static final double driveKV = 0.1;
    public static final double driveKA = 0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    //public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = 19;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 12;
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(189.932);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.728515625);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 0);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1  {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 5;
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(173.1445);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(235.01953124999997);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 1);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 8;
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(50.0977);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-41.39648438);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 2);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 9;
        public static final int canCoderID = 11;
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(31.1133);
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.185546875);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 3);
    }

    static {
        /* Angle/Steering motor */
        Slot0Configs anglePIDConfigs = new Slot0Configs();
        anglePIDConfigs.kP = angleKP;
        anglePIDConfigs.kI = angleKI;
        anglePIDConfigs.kD = angleKD;
        anglePIDConfigs.kV = angleKF;
        angleMotorConfigs.Slot0 = anglePIDConfigs;

        CurrentLimitsConfigs angleCurrentConfigs = new CurrentLimitsConfigs();
        angleCurrentConfigs.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        angleCurrentConfigs.SupplyCurrentLimit = SwerveConstants.angleContinuousCurrentLimit;
        angleCurrentConfigs.SupplyCurrentThreshold = SwerveConstants.anglePeakCurrentLimit;
        angleCurrentConfigs.SupplyTimeThreshold = SwerveConstants.anglePeakCurrentDuration;
        angleMotorConfigs.CurrentLimits = angleCurrentConfigs;

        ClosedLoopGeneralConfigs angleClosedGeneralConfigs = new ClosedLoopGeneralConfigs();
        angleClosedGeneralConfigs.ContinuousWrap = true;
        angleMotorConfigs.ClosedLoopGeneral = angleClosedGeneralConfigs;

        MotorOutputConfigs angleMotorOutConfigs = new MotorOutputConfigs();
        angleMotorOutConfigs.Inverted = SwerveConstants.angleMotorInvert;
        angleMotorOutConfigs.NeutralMode = angleNeutralMode;
        angleMotorConfigs.MotorOutput = angleMotorOutConfigs;
    }
}
