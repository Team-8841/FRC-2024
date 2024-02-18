package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
    // Shooter

    public static final int shooterMotorID = 15;
    public static final int followerMotorID = 16;

    public static final int shooterCurrentLimit = 80; // May need to change

    public static final int shooterEncoderCPR = 2048; // CANCoder CPR

    public static final double subShotSpeed = 0; // Tune
    public static final double farShotSpeed1 = 0; // Tune
    public static final double farShotSpeed2 = 0; // Tune
    public static final double ampShotSpeed = 0; // Tune

    public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // End Effector Hinge

    public static final int endEffectorROTID = 17;

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

    public static final Rotation2d endEffectorHome = Rotation2d.fromDegrees(0); // Setpoint of end effector at home position
    public static final Rotation2d endEffectorDeployed = Rotation2d.fromDegrees(0); // Setpoint of end effector deployed

    // End Effector Roller

    public static final int endEffectorRollerID = 18;

    public static final double rollerOutSpeed = 0.6; // Tune
    public static final double rollerInSpeed = 0.0; // Tune

    static {
        // Shooter Motor Configs

        shooterConfig.Slot0 = new Slot0Configs();
        shooterConfig.Slot0.kS = 0;
        shooterConfig.Slot0.kV = 0;
        shooterConfig.Slot0.kA = 0;
        shooterConfig.Slot0.kP = 0;
        shooterConfig.Slot0.kI = 0;
        shooterConfig.Slot0.kD = 0;

        shooterConfig.MotorOutput = new MotorOutputConfigs();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.PeakForwardDutyCycle = 0;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = -1;

        // Per motor current limit. Total max subsystem current input is 2x this
        shooterConfig.CurrentLimits = new CurrentLimitsConfigs();
        shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40; 
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;

        shooterConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
        shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;
    }
}
