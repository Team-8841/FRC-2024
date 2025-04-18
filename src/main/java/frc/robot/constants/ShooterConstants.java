package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

    public static final boolean tuneMode = true; // Change when tuning the shooter to allow for on the fly tuning;

    public static final int shooterMotorID = 15;
    public static final int followerMotorID = 16;

    public static final int endEffectorROTID = 17;
    public static final int endEffectorRollerID = 18;

    public static final int shooterCurrentLimit = 80; // May need to change

    public static final int shooterEncoderCPR = 2048; // CANCoder CPR
    public static final double shooter_kS = 0; // Tune
    public static final double shooter_kV = 0; // Tune
    public static final double shooter_kA = 0; // Tune
    public static final double shooter_kP = 0; // Tune
    public static final double shooter_kI = 0; // Tune
    public static final double shooter_kD = 0; // Tune
    public static final double shooter_kIZone = 0; // Tune
    public static final double shooter_kFF = 0; // Tune
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
    }
}
