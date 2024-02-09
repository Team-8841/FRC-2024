package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.ShooterConstants;

public class RealShooterIO implements ShooterIO {
    // Motors
    private final TalonFX m_shooter = new TalonFX(ShooterConstants.shooterMotorID),
            m_follower = new TalonFX(ShooterConstants.followerMotorID);
    // private final CANSparkMax m_endEffectorROT = new
    // CANSparkMax(ShooterConstants.endEffectorROTID, MotorType.kBrushed),
    // m_endEffectorRoller = new CANSparkMax(ShooterConstants.endEffectorRollerID,
    // MotorType.kBrushed);
    // private final SparkPIDController m_endEffectorPID =
    // this.m_endEffectorROT.getPIDController();
    // private final ArmFeedforward endEffectorFeedforward = new
    // ArmFeedforward(ShooterConstants.endEffector_kS,
    // ShooterConstants.endEffector_kG, ShooterConstants.endEffector_kV,
    // ShooterConstants.endEffector_kA);

    // Sensors
    // private final RelativeEncoder m_endEffectorEncoder =
    // this.m_endEffectorROT.getEncoder(),
    // m_endEffectorRollerEncoder = this.m_endEffectorRoller.getEncoder();
    private final StatusSignal<Double> shooterVel = this.m_shooter.getVelocity(),
            shooterSupCur = this.m_shooter.getSupplyCurrent(),
            shooterStaCur = this.m_shooter.getStatorCurrent(), 
            shooterAppliedOut = this.m_shooter.getClosedLoopOutput(),
            followerSupCur = this.m_follower.getSupplyCurrent(),
            followerStaCur = this.m_follower.getStatorCurrent(),
            followerAppliedOut = this.m_follower.getClosedLoopOutput();

    private double shooterSP;
    private Rotation2d endEffectorSP = new Rotation2d();

    public RealShooterIO() {
        // Shooter and follower motor
        this.m_shooter.getConfigurator().apply(ShooterConstants.shooterConfig);
        this.m_follower.getConfigurator().apply(ShooterConstants.shooterConfig);

        // End effector
        // this.m_endEffectorPID.setP(ShooterConstants.endEffector_kP);
        // this.m_endEffectorPID.setI(ShooterConstants.endEffector_kI);
        // this.m_endEffectorPID.setD(ShooterConstants.endEffector_kD);
        // this.m_endEffectorPID.setOutputRange(ShooterConstants.endEffector_minOutput,
        // ShooterConstants.endEffector_maxOutput);
    }

    @Override
    public void setShooter(double targetRPS) {
        //this.m_follower.setControl(new Follower(this.m_shooter.getDeviceID(), false));
        this.shooterSP = targetRPS;
        this.m_shooter.setControl(new MotionMagicVelocityVoltage(-targetRPS));
        this.m_follower.setControl(new Follower(this.m_shooter.getDeviceID(), false));
    }

    @Override
    public void setEndEffector(Rotation2d targetAngle) {
        this.endEffectorSP = targetAngle;
        // double ff = this.endEffectorFeedforward.calculate(targetAngle.getRadians(),
        // 0);
        // m_endEffectorPID.setReference(targetAngle.getRotations(),
        // ControlType.kPosition, 0, ff);
    }

    @Override
    public void setRollerSpeed(double dcycle) {
        // this.m_endEffectorRoller.set(dcycle);
    }

    @Override
    public double getShooterRPS() {
        return this.shooterVel.refresh().getValue();
    }

    @Override
    public Rotation2d getEndEffector() {
        // return Rotation2d.fromRotations(this.m_endEffectorEncoder.getPosition());
        return new Rotation2d();
    }

    LinearFilter filter = LinearFilter.movingAverage(5);

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {
        // Shooter
        inputs.setShooterRPS = this.shooterSP;
        inputs.actualShooterRPS = -this.shooterVel.refresh().getValue();

        // End Effector
        inputs.setEndEffectorDeg = this.endEffectorSP.getDegrees();
        inputs.actualEndEffectorDeg = this.getEndEffector().getDegrees();

        // Roller
        // inputs.setRollerDCycle = this.m_endEffectorRoller.get();
        // inputs.rollerSpeedRPS = this.m_endEffectorRollerEncoder.getVelocity() / 60;

        // Current readings
        Logger.recordOutput("Shooter/shooterSupCur", this.shooterSupCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterStaCur", this.shooterStaCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterOutput", this.shooterAppliedOut.refresh().getValue());
        Logger.recordOutput("Shooter/shooterFollowerSupCur", this.followerSupCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterFollowerStaCur", this.followerStaCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterFollowerOutput", this.followerAppliedOut.refresh().getValue());
        // double endEffectorOutCur = this.m_endEffectorROT.getOutputCurrent();
        // Logger.recordOutput("Shooter/endEffectorOutCur", endEffectorOutCur);
        // double rollerOutCur = this.m_endEffectorRoller.getOutputCurrent();
        // Logger.recordOutput("Shooter/rollerOutCur", rollerOutCur);
        // Logger.recordOutput("Shooter/totalCur", this.shooterSupCur.getValue() +
        // this.followerSupCur.getValue() +
        // endEffectorOutCur + rollerOutCur);
        Logger.recordOutput("Shooter/totalCur", this.shooterSupCur.getValue() + this.followerSupCur.getValue());
        Logger.recordOutput("Shooter/movingAverageVel", this.filter.calculate(inputs.actualShooterRPS));
    }
}
