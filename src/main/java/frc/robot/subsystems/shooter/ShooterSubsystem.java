package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Shooter.ShooterConstants;
import frc.robot.util.SparkConfigs;

public class ShooterSubsystem extends SubsystemBase {

    /*-------------------------------- Private Instance Variables --------------------------------*/

    // Motors
    private CANSparkMax m_shooter = new CANSparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private CANSparkMax m_follower = new CANSparkMax(ShooterConstants.followerMotorID, MotorType.kBrushless);

    private CANSparkMax m_endEffectorROT = new CANSparkMax(ShooterConstants.endEffectorROTID, MotorType.kBrushed);
    private CANSparkMax m_endEffectorRoller = new CANSparkMax(ShooterConstants.endEffectorRollerID, MotorType.kBrushed);

    // PID Controller
    private final SparkPIDController m_shooterPID, m_endEffectorPID;

    // Shooter Encoder
    private final RelativeEncoder m_shooterEncoder, m_endEffectorEncoder;

    // PID values for on the fly turning for the shooter
    private double m_kP, m_kI, m_kD, m_kIZone, m_kFF, m_setPoint;

    // Shooter State variable
    private ShooterState curState;

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public static enum ShooterState {
        /*ENUM(shooter Setpoint, End effector setpoint, End effector roller speed ,Shooter Aimed Up) */
        OFF(ShooterConstants.shooterOffSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, true),
        SUBSHOT(ShooterConstants.subShotSpeed, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, true),
        FARSHOT(ShooterConstants.farShotSpeed1, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, false),
        AMP(ShooterConstants.ampShotSpeed, ShooterConstants.endEffectorDeployed, ShooterConstants.rollerOutSpeed, true),
        FIELDTOSS(ShooterConstants.farShotSpeed2, ShooterConstants.endEffectorHome, ShooterConstants.rollerOffSpeed, false);

        private final boolean m_shooterAimedUP;
        private final double m_shooterSP, m_endEffectorSP, m_endEffectorRollerSpeed;

        private ShooterState(double shooterSP, double endEffectorSP, double eeRollerSpeed, boolean shooterAimedUP){
            this.m_shooterSP = shooterSP;
            this.m_endEffectorSP = endEffectorSP;
            this.m_endEffectorRollerSpeed = eeRollerSpeed;
            this.m_shooterAimedUP = shooterAimedUP;
        }

    }

    public ShooterSubsystem() {

        // Configure Sparkmax controllers
        SparkConfigs.configureSparkMax(m_shooter, ShooterConstants.shooterCurrentLimit, IdleMode.kCoast);
        SparkConfigs.configureSparkMax(m_follower, ShooterConstants.shooterCurrentLimit, IdleMode.kCoast);

        // Invert rotation direction
        m_shooter.setInverted(false);   // Change if needed
        m_follower.setInverted(false);  // Change if needed

        // Follow the main shooter motor
        m_follower.follow(m_shooter);

        // Set the PID controller to the shooter
        m_shooterPID = m_shooter.getPIDController();

        // Set the PID controller for the end effector
        m_endEffectorPID = m_endEffectorROT.getPIDController();

        // Get the main shooter motor encoder
        m_shooterEncoder = m_shooter.getEncoder();

        // Get the end effector encoder
        m_endEffectorEncoder = m_endEffectorROT.getEncoder();

        // Set shooter PID values
        m_shooterPID.setP(ShooterConstants.shooter_kP);
        m_shooterPID.setI(ShooterConstants.shooter_kI);
        m_shooterPID.setD(ShooterConstants.shooter_kD);
        m_shooterPID.setFF(ShooterConstants.shooter_kFF);
        m_shooterPID.setOutputRange(ShooterConstants.shooter_minOutput, ShooterConstants.shooter_maxOutput);

        // Set end effector PId values
        m_endEffectorPID.setP(ShooterConstants.endEffector_kP);
        m_endEffectorPID.setI(ShooterConstants.endEffector_kI);
        m_endEffectorPID.setD(ShooterConstants.endEffector_kD);
        m_endEffectorPID.setFF(ShooterConstants.endEffector_kFF);
        m_endEffectorPID.setOutputRange(ShooterConstants.endEffector_minOutput, ShooterConstants.endEffector_maxOutput);


        // If tune mode is on populate the dashboard with the pid coefficients
        if(ShooterConstants.tuneMode) {
            SmartDashboard.putNumber("[Shooter] P Gain", ShooterConstants.shooter_kP);
            SmartDashboard.putNumber("[Shooter] I Gain", ShooterConstants.shooter_kI);
            SmartDashboard.putNumber("[Shooter] D Gain", ShooterConstants.shooter_kD);
            SmartDashboard.putNumber("[Shooter] I Zone", ShooterConstants.shooter_kIZone);
            SmartDashboard.putNumber("[Shooter] Feed Forward", ShooterConstants.shooter_kFF);
            SmartDashboard.putNumber("[Shooter] Setpoint", 0);
        }
    }
    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        updateStatus();
    }

    /*-------------------------------- Custom Public Functions --------------------------------*/
    
    public void shooter(ShooterState state) {
        setShooterSetPoint(state.m_shooterSP);
        setEndEffector(state.m_endEffectorSP);
        setRollerSpeed(state.m_endEffectorRollerSpeed);
        shooterIsUP(state.m_shooterAimedUP);
        curState = state;
    }

    /* ========= Shooter Functions ========= */
    public void setShooterSetPoint(double targetRPM) {
        m_shooterPID.setReference(targetRPM, ControlType.kVelocity);
    }

    public void shooterIsUP(boolean up){
        // Retruns if the shooter is facing up
        // Add later if we add multi position shooting
    }

    // Get the target setpoint from the current state and check if we are within a tolerance of that value
    public boolean atTarget() {
        double curVel = m_shooterEncoder.getVelocity();
        return curVel > curState.m_shooterSP - ShooterConstants.shooter_allowedError;
    }

    /* ========= End Effector Functions ========= */
    public void setEndEffector(double targetAngle){
        m_endEffectorPID.setReference(targetAngle, ControlType.kPosition);
    }

    // Get the target setpoint from the current state and check if we are within a tolerance of that value
    public boolean endEffectorAtTarget() {
        double curAngle = m_endEffectorEncoder.getPosition();
        return curAngle > curState.m_endEffectorSP - ShooterConstants.endEffector_allowedError;
    }

    public void setRollerSpeed(double speed) {
        m_endEffectorRoller.set(speed);
    }


    // On the fly tuning for the shooter PID loop
    public void tuneShooter() {
        double p = SmartDashboard.getNumber("[Shooter]: P Gain", ShooterConstants.shooter_kP);
        double i = SmartDashboard.getNumber("[Shooter] I Gain", ShooterConstants.shooter_kI);
        double d = SmartDashboard.getNumber("[Shooter] D Gain", ShooterConstants.shooter_kD);
        double iz = SmartDashboard.getNumber("[Shooter] I Zone", ShooterConstants.shooter_kIZone);
        double ff = SmartDashboard.getNumber("[Shooter] Feed Forward", ShooterConstants.shooter_kFF);
        double sp = SmartDashboard.getNumber("[Shooter] Setpoint", 0);

        if (p != m_kP) {
            m_shooterPID.setP(p);
            m_kP = p;
        }

        if (i != m_kI) {
            m_shooterPID.setI(i);
            m_kI = i;
        }

        if (d != m_kD) {
            m_shooterPID.setD(d);
            m_kD = d;
        }

        if (iz != m_kIZone) {
            m_shooterPID.setIZone(iz);
            m_kIZone = iz;
        }

        if (ff != m_kFF) {
            m_shooterPID.setFF(ff);
            m_kFF = ff;
        }

        if(sp != m_setPoint) {
            m_shooterPID.setReference(sp, ControlType.kVelocity);
            m_setPoint = sp;
        }
    }

    
    /*-------------------------------- Custom Private Functions --------------------------------*/
    private void updateStatus(){
        SmartDashboard.putNumber("[Shooter] Velocity", m_shooterEncoder.getVelocity());
        SmartDashboard.putBoolean("[Shooter] At Speed", atTarget());

        SmartDashboard.putNumber("[End Effector] Angle", m_endEffectorEncoder.getPosition());
        SmartDashboard.putBoolean("[End Effector] At Target", endEffectorAtTarget());

    }

}
