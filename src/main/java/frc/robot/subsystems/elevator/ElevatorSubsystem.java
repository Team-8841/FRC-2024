package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase{

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private CANSparkMax m_main = new CANSparkMax(ElevatorConstants.kElevatorMain, MotorType.kBrushless);
    private CANSparkMax m_follower = new CANSparkMax(ElevatorConstants.kElevatorFollower, MotorType.kBrushless);

    private DigitalInput m_TopLimmit = new DigitalInput(ElevatorConstants.kElevatorTopSensor);
    private DigitalInput m_BottomLimit = new DigitalInput(ElevatorConstants.kElevatorBottomSensor);

    private Solenoid m_breaks = new Solenoid(PneumaticsModuleType.CTREPCM, 0);



    /*-------------------------------- Public Instance Variables --------------------------------*/

    public ElevatorSubsystem() {
        ConfigureSparkMax(m_main, 10, IdleMode.kBrake);
        ConfigureSparkMax(m_follower, 10, IdleMode.kBrake);

        m_follower.follow(m_main, true);
        m_main.setInverted(false);

        this.setBreaks(true);
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        if (this.getBreaksEnabled()) {
            this.setSpeed(0);
        }
        updateStatus();
    }


    /*-------------------------------- Custom Public Functions --------------------------------*/

    public void setSpeed(double speed){
        if (!this.getBreaksEnabled()) {
            Logger.recordOutput("elevatorSpeed", speed);
            m_main.set(speed);
        }
    }

    public boolean getTopLimit() {
        return m_TopLimmit.get();
    }

    public boolean getBottomLimit() {
        return !m_BottomLimit.get();
    }

    public void setBreaks(boolean enabled) {
        m_breaks.set(!enabled);

        if (!enabled) {
            this.setSpeed(0);
        }
    }

    public boolean getBreaksEnabled() {
        return !m_breaks.get();
    }


    /*-------------------------------- Custom Private Functions --------------------------------*/
    private static void ConfigureSparkMax(CANSparkMax spark, int currentLimit, IdleMode idleMode) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(currentLimit);
        spark.setIdleMode(idleMode);
    }

    private void updateStatus() {
        SmartDashboard.putBoolean("[Elevator]: Top Limit", getTopLimit());
        SmartDashboard.putBoolean("[Elevator]: Bottom Limit", getBottomLimit());
        Logger.recordOutput("elevatorBrake", this.getBreaksEnabled());
        Logger.recordOutput("elevatorLower", this.getBottomLimit());
        Logger.recordOutput("elevatorUpper", this.getTopLimit());
    }
}
