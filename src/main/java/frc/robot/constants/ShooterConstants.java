package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

    public static final boolean tuneMode = true; // Change when tuning the shooter to allow for on the fly tuning;

    public static final int shooterMotorID = 16;
    public static final int followerMotorID = 17;

    public static final int endEffectorROTID = 17;
    public static final int endEffectorRollerID = 18;

    public static final int shooterCurrentLimit = 40; // Note: This is PER MOTOR
    public static final NeutralModeValue shooterNeutralMode = NeutralModeValue.Brake;
    public static final double shooterMaxDutyCycle = 1;
    public static final double shooterMinDutyCycle = 0;

    public static final int shooterEncoderCPR = 2048; // CANCoder CPR

    public static final double shooter_kS = 0.3; // Tune
    public static final double shooter_kV = 0.108; // Tune
    public static final double shooter_kA = 0; // Tune
    public static final double shooter_kP = 0.15; // Tune
    public static final double shooter_kI = 0; // Tune
    public static final double shooter_kD = 0; // Tune
    public static final double shooter_kIZone = 0; // Tune
    public static final double shooter_kFF = 0; // Tune
    public static final double shooter_maxAccel = 25;
    public static final double shooter_maxOutput = 1; // Tune
    public static final double shooter_minOutput = 0; // Tune
    public static final double shooter_allowedError = 200; // Tune

    public static final int endEffectorEncoderCPR = 4096; // CANCoder CPR
    public static final double endEffector_kS = 0; // Tune
    public static final double endEffector_kG = 0; // Tune
    public static final double endEffector_kV = 0; // Tune
    public static final double endEffector_kA = 0; // Tune
    public static final double endEffector_kP = 0; // Tune
    public static final double endEffector_kI = 0; // Tune
    public static final double endEffector_kD = 0; // Tune
    public static final double endEffector_kIZone = 0; // Tune
    public static final double endEffector_maxOutput = 0.4; // Tune
    public static final double endEffector_minOutput = 0; // Tune
    public static final Rotation2d endEffector_allowedError = Rotation2d.fromDegrees(5); // Tune

    public static final double shooterOffSpeed = 0; // Dont change this
    public static final double subShotSpeed = 0; // Tune
    public static final double farShotSpeed1 = 0; // Tune
    public static final double farShotSpeed2 = 0; // Tune
    public static final double ampShotSpeed = 0; // Tune

    public static final Rotation2d endEffectorHome = Rotation2d.fromDegrees(0); // Setpoint of end effector at home position
    public static final Rotation2d endEffectorDeployed = Rotation2d.fromDegrees(0); // Setpoint of end effector deployed

    public static final double rollerOutSpeed = 0.6; // Tune
    public static final double rollerInSpeed = 0.0; // Tune
    public static final double rollerOffSpeed = 0.0; // Tune

    public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    static {
        // Shooter Configs
        shooterConfig.Slot0 = new Slot0Configs();
        shooterConfig.Slot0.kS = ShooterConstants.shooter_kS;
        shooterConfig.Slot0.kV = ShooterConstants.shooter_kV;
        shooterConfig.Slot0.kA = ShooterConstants.shooter_kA;
        shooterConfig.Slot0.kP = ShooterConstants.shooter_kP;
        shooterConfig.Slot0.kI = ShooterConstants.shooter_kI;
        shooterConfig.Slot0.kD = ShooterConstants.shooter_kD;

        shooterConfig.MotionMagic = new MotionMagicConfigs();
        shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooter_maxAccel;

        shooterConfig.MotorOutput = new MotorOutputConfigs();
        shooterConfig.MotorOutput.NeutralMode = ShooterConstants.shooterNeutralMode;
        shooterConfig.MotorOutput.PeakForwardDutyCycle = ShooterConstants.shooterMaxDutyCycle;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = ShooterConstants.shooterMinDutyCycle;

        shooterConfig.CurrentLimits = new CurrentLimitsConfigs();
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
}
