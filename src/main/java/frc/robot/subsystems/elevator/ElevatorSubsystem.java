package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase{

    private CANSparkMax mainMotor = new CANSparkMax(ElevatorConstants.kElevatorMain, MotorType.kBrushless);
    private CANSparkMax followerMotor = new CANSparkMax(ElevatorConstants.kElevatorFollower, MotorType.kBrushless);

    private DigitalInput TopLimit = new DigitalInput(ElevatorConstants.kElevatorTopSensor);
    private DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.kElevatorBottomSensor);

    private Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private static final boolean BRAKES_APPLIED = false;
    private static final boolean BRAKES_RELINQUISHED = true;

    public ElevatorSubsystem() {
        mainMotor.restoreFactoryDefaults();
        mainMotor.setSmartCurrentLimit(60);
        mainMotor.setIdleMode(IdleMode.kBrake);

        followerMotor.restoreFactoryDefaults();
        followerMotor.setSmartCurrentLimit(60);
        followerMotor.setIdleMode(IdleMode.kBrake);

        followerMotor.follow(mainMotor, true);
        mainMotor.setInverted(true);

        this.setBreaks(true);

        Shuffleboard.getTab("Robot").addBoolean("Bottom Elevator Sensor", this::getBottomLimit);
        Shuffleboard.getTab("Robot").addBoolean("Top Elevator Sensor", this::getTopLimit);
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        if (this.isBraking()) {
            this.setSpeed(0);
        }
        updateStatus();

    }


    /*-------------------------------- Custom Public Functions --------------------------------*/

    public void setSpeed(double speed){
        if (!this.isBraking()) {
            mainMotor.set(speed);
        }
    }

    public boolean getTopLimit() {
        return TopLimit.get();
    }

    public boolean getBottomLimit() {
        return !bottomLimit.get();
    }

    public void setBreaks(boolean isBraking) {
        brakeSolenoid.set(isBraking ? BRAKES_APPLIED : BRAKES_RELINQUISHED);

        if (isBraking) {
            this.setSpeed(0);
        }
    }

    public boolean isBraking() {
        return brakeSolenoid.get() == BRAKES_APPLIED;
    }


    private void updateStatus() {
        SmartDashboard.putBoolean("[Elevator]: Top Limit", getTopLimit());
        SmartDashboard.putBoolean("[Elevator]: Bottom Limit", getBottomLimit());

        Logger.recordOutput("elevator/brake", this.isBraking());
        Logger.recordOutput("elevator/lowerLimit", this.getBottomLimit());
        Logger.recordOutput("elevator/upperLimit", this.getTopLimit());
        Logger.recordOutput("elevator/motorDcycle", this.mainMotor.get());
        Logger.recordOutput("elevator/motorAppOut", this.mainMotor.getAppliedOutput());
        Logger.recordOutput("elevator/folMotorAppOut", this.followerMotor.getAppliedOutput());
        Logger.recordOutput("elevator/motorOutCur", this.mainMotor.getOutputCurrent());
        Logger.recordOutput("elevator/folMotorOutCur", this.followerMotor.getOutputCurrent());
    }
}
