package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*-------------------------------- Private Instance Variables --------------------------------*/
    // Motors
    private TalonFX m_shooter = new TalonFX(ShooterConstants.kShooterMotor);
    private TalonFX m_follower = new TalonFX(ShooterConstants.kFollowerMotor);

    private CANSparkMax m_endEffector = new CANSparkMax(ShooterConstants.kEndeffectorID, MotorType.kBrushless);
    private CANSparkMax m_endEffectorRoller = new CANSparkMax(ShooterConstants.kEndEffectorRollerID, MotorType.kBrushless);

    private Solenoid m_angle = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    private DigitalInput m_EEInSensor = new DigitalInput(ShooterConstants.kEEInSensor);
    private DigitalInput m_EEOutSensor = new DigitalInput(ShooterConstants.kEEOutSensor);

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private double m_shooterSetPoint, m_EESetPoint;

    private PIDController m_EEPIDController = new PIDController(ShooterConstants.kEE_kP, ShooterConstants.kEE_kI, ShooterConstants.kEE_kD);

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public ShooterSubsystem() {
        /* Shooter */
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = 0.011;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.12;

        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 40;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 40;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = m_shooter.getConfigurator().apply(configs);
            if(status.isOK()) break;
        }

        if(!status.isOK()){
            System.out.println("Could not apply configs, Error code: " + status.toString());
        }

        m_shooter.setInverted(true);
        m_follower.setControl(new Follower(m_shooter.getDeviceID(), false));

        m_shooterSetPoint = 0;

        /* End Effector */
        ConfigureSparkMax(m_endEffector, 30, IdleMode.kBrake);
        ConfigureSparkMax(m_endEffectorRoller, 40, IdleMode.kBrake);

        m_endEffector.setInverted(true);



    }
    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        updateStatus();
    }

    /*-------------------------------- Custom Public Functions --------------------------------*/
    
    public void setShooterSpeed(double d) {
        // velocity is set in RPS
        m_shooterSetPoint = d / 60;
        m_shooter.setControl(m_velocityVoltage.withVelocity(d / 60));
        //m_follower.setControl(m_velocityVoltage.withVelocity(d / 60));
    }

    public double getShooterSpeed() {
        return m_shooter.getVelocity().getValueAsDouble() * 60;
    }

    public boolean isShooterAtSpeed() {
        double curVel = m_shooter.getVelocity().getValueAsDouble();
        return curVel > m_shooterSetPoint - ShooterConstants.kShooterAllowedError;
    }
    
    public void setShooterAngle(boolean extended){
        m_angle.set(extended);
    }

    public boolean getShooterAngle(){
        return m_angle.get();
    }

    public double getShooterSetpoint() {
        return m_shooterSetPoint;
    }

    public void setEERoller(double speed) {
        m_endEffectorRoller.set(speed);
    }

    public void setEEAngle(double angle) {
        m_EESetPoint = angle;
        double val = m_EEPIDController.calculate(getEEAngle() - angle);

        SmartDashboard.putNumber("[EE]: PID output", val);
        SmartDashboard.putNumber("[EE]: Angle Diff", getEEAngle() - angle);

        if (getEEHomeLimit()) {
            zeroEEAngle();
        }

        if(getEEHomeLimit() && val < 0) {
            setEESpeed(0);
        } else if (getEEDeployedLimit() && val > 0){
            setEESpeed(0);
        } else {
            setEESpeed(val);
        }
    }
    
    public void setEESpeed(double speed) {
        m_endEffector.set(speed);
    }

    public double getEEAngle() {
        return m_endEffector.getEncoder().getPosition();
    }

    public boolean getEEHomeLimit() {
        return !m_EEInSensor.get();
    }

    public boolean getEEDeployedLimit() {
        return !m_EEOutSensor.get();
    }

    public void zeroEEAngle() {
        m_endEffector.getEncoder().setPosition(0);
    }
    /*-------------------------------- Custom Private Functions --------------------------------*/
    private void updateStatus(){
        SmartDashboard.putNumber("[EE]: Angle", getEEAngle());
        SmartDashboard.putNumber("[Shooter]: Setpoint", getShooterSetpoint() * 60);
        SmartDashboard.putNumber("[Shooter]: Velocity", getShooterSpeed());
        SmartDashboard.putBoolean("[Shooter]: Angle", getShooterAngle());

    }

    private static void ConfigureSparkMax(CANSparkMax spark, int currentLimit, IdleMode idleMode) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(currentLimit);
        spark.setIdleMode(idleMode);
    }


}
