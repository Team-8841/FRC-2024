package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.ShooterConstants;

public class RealShooterIO implements ShooterIO {
    // Motors
    private final TalonFX m_shooter = new TalonFX(ShooterConstants.shooterMotorID),
            m_follower = new TalonFX(ShooterConstants.followerMotorID);
    private final CANSparkMax m_endEffectorROT = new CANSparkMax(ShooterConstants.endEffectorROTID, MotorType.kBrushless);
            //m_endEffectorRoller = new CANSparkMax(ShooterConstants.endEffectorRollerID, MotorType.kBrushless);
    //private final SparkPIDController m_endEffectorPID = this.m_endEffectorROT.getPIDController();
    private final ArmFeedforward endEffectorFeedforward = new ArmFeedforward(0.1,
            ShooterConstants.endEffector_kG, ShooterConstants.endEffector_kV, ShooterConstants.endEffector_kA);

    // Sensors
    // 36/16
    private final RelativeEncoder m_endEffectorEncoder = this.m_endEffectorROT.getEncoder();
    private final StatusSignal<Double> shooterVel = this.m_shooter.getVelocity(),
            shooterSupCur = this.m_shooter.getSupplyCurrent(),
            shooterStaCur = this.m_shooter.getStatorCurrent(), followerSupCur = this.m_follower.getSupplyCurrent(),
            followerStaCur = this.m_follower.getStatorCurrent();
    private final DigitalInput upLimitSwitch = new DigitalInput(1), downLimitSwitch = new DigitalInput(2);

    private final Solenoid shooterAngleSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    private final static boolean SHOOTER_UP_VAL = false;
    private final static boolean SHOOTER_DOWN_VAL = true;

    private double shooterSP;
    private Rotation2d endEffectorSP = new Rotation2d();

    public RealShooterIO() {
        // Shooter and follower motor
        this.m_shooter.getConfigurator().apply(ShooterConstants.shooterConfig);

        this.m_follower.getConfigurator().apply(ShooterConstants.shooterConfig);

        // End effector
        this.m_endEffectorEncoder.setPositionConversionFactor(1.0 / (100 * 36.0/16.0));
        //this.m_endEffectorPID.setP(ShooterConstants.endEffector_kP);
        //this.m_endEffectorPID.setI(ShooterConstants.endEffector_kI);
        //this.m_endEffectorPID.setD(ShooterConstants.endEffector_kD);
        //this.m_endEffectorPID.setOutputRange(ShooterConstants.endEffector_minOutput,
        //        ShooterConstants.endEffector_maxOutput);
        //this.m_endEffectorPID.setSmartMotionAllowedClosedLoopError(10, 0);

        this.m_endEffectorROT.setSmartCurrentLimit(10);
        //this.m_endEffectorRoller.setSmartCurrentLimit(10);

        this.shooterAngleSolenoid.set(SHOOTER_UP_VAL);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void setShooter(double dcycle) {
        this.shooterSP = dcycle;
        this.m_shooter.setControl(new VoltageOut(-12 * dcycle));
        this.m_follower.setControl(new Follower(this.m_shooter.getDeviceID(), false));
    }

    @Override
    public void setShooterAngle(ShooterAngle state) {
        //this.shooterAngleSolenoid.set(state == ShooterAngle.UP ? SHOOTER_UP_VAL : SHOOTER_DOWN_VAL);
    }

    @Override
    public void setEndEffector(double dcycle) {
        this.m_endEffectorROT.set(dcycle);

        // Ignore everything below this comment

        // I inform the cop that "we're on a mission, sorry to burst your bubble"
        // Then it's like yeah, pedal to the metal, yeah, whatever (Bling!)

        // double ff = this.endEffectorFeedforward.calculate(targetAngle.getRadians(), 0);

        // if (this.getUpLimitSwitch() && targetAngle.getDegrees() > 20) {
        //     this.endEffectorSP = targetAngle;
        //     System.out.println("Setting up");
        //     Logger.recordOutput("Shooter/testVal", 1);
        //     m_endEffectorPID.setReference(targetAngle.getRotations(), ControlType.kPosition, 0, ff);
        // }
        // else if (this.getDownLimitSwitch() && targetAngle.getDegrees() <= 20) {
        //     this.endEffectorSP = targetAngle;
        //     Logger.recordOutput("Shooter/testVal", -1);
        //     m_endEffectorPID.setReference(targetAngle.getRotations(), ControlType.kPosition, 0, ff);
        // }
        // else if (!this.getDownLimitSwitch() && !this.getUpLimitSwitch()) {
        //     this.endEffectorSP = targetAngle;
        //     Logger.recordOutput("Shooter/testVal", 5);
        //     m_endEffectorPID.setReference(targetAngle.getRotations(), ControlType.kPosition, 0, ff);
        // }
        // else {
        //     Logger.recordOutput("Shooter/testVal", 0);
        // }
        // m_endEffectorPID.setReference(targetAngle.getRotations(), ControlType.kPosition, 0, ff);

        // Logger.recordOutput("Shooter/setPoint", this.endEffectorSP.getDegrees());

        // if (this.getDownLimitSwitch() && targetAngle.getDegrees() >= 30) {
        //     this.endEffectorSP = targetAngle;
        //     this.m_endEffectorROT.set(0.4);
        // }

        // else if (this.getUpLimitSwitch() && targetAngle.getDegrees() < 30) {
        //     this.endEffectorSP = targetAngle;
        //     this.m_endEffectorROT.set(-0.4);
        // }
        // else if (!this.getDownLimitSwitch() && !this.getUpLimitSwitch()) {
        //     if (this.endEffectorSP.getDegrees() >= 30) {
        //         this.m_endEffectorROT.set(0.4);
        //     }
        //     else {
        //         this.m_endEffectorROT.set(-0.4);
        //     }
        // }
        // else if (this.getDownLimitSwitch() || this.getUpLimitSwitch()) {
        //     this.m_endEffectorROT.set(0);
        // }

        //this.m_endEffectorROT.set(0.2);
    }

    @Override
    public void setRollerSpeed(double dcycle) {
        //this.m_endEffectorRoller.set(dcycle);
    }

    @Override
    public double getShooterRPS() {
        return this.shooterVel.getValue();
    }

    @Override
    public Rotation2d getEndEffector() {
        return Rotation2d.fromRotations(this.m_endEffectorEncoder.getPosition());
        //return Rotation2d.fromDegrees(0);
    }

    @Override
    public boolean getUpLimitSwitch() {
        return !this.upLimitSwitch.get();
    }

    @Override
    public boolean getDownLimitSwitch() {
        return !this.downLimitSwitch.get();
    }

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {
        // Shooter
        inputs.setShooterRPS = this.shooterSP;
        inputs.actualShooterRPS = this.shooterVel.refresh().getValue();
        inputs.isShooterUp = this.shooterAngleSolenoid.get() == SHOOTER_UP_VAL;


        // End Effector
        inputs.setEndEffectorDeg = this.endEffectorSP.getDegrees();
        inputs.actualEndEffectorDeg = this.getEndEffector().getDegrees();

        // Roller
        // inputs.setRollerDCycle = this.m_endEffectorRoller.get();
        // inputs.rollerSpeedRPS = this.m_endEffectorRollerEncoder.getVelocity() / 60;

        inputs.upLimitSwitch = this.getUpLimitSwitch();
        inputs.downLimitSwitch = this.getDownLimitSwitch();

        // Current readings
        Logger.recordOutput("Shooter/shooterSupCur", this.shooterSupCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterStaCur", this.shooterStaCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterFollowerSupCur", this.followerSupCur.refresh().getValue());
        Logger.recordOutput("Shooter/shooterFollowerStaCur", this.followerStaCur.refresh().getValue());
        double endEffectorOutCur = this.m_endEffectorROT.getOutputCurrent();
        Logger.recordOutput("Shooter/endEffectorOutCur", endEffectorOutCur);
        //double rollerOutCur = this.m_endEffectorRoller.getOutputCurrent();
        // Logger.recordOutput("Shooter/rollerOutCur", rollerOutCur);
        // Logger.recordOutput("Shooter/totalCur", this.shooterSupCur.getValue() + this.followerSupCur.getValue() +
        //         endEffectorOutCur + rollerOutCur);

        Logger.recordOutput("Shooter/endEffectorDCycle", this.m_endEffectorROT.get());
        Logger.recordOutput("Shooter/endEffectorAppliedOutput", this.m_endEffectorROT.getAppliedOutput());

    }
}
