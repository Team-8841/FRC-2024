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

    private CANSparkMax mainMotor = new CANSparkMax(ElevatorConstants.kElevatorMain, MotorType.kBrushless);
    private CANSparkMax followerMotor = new CANSparkMax(ElevatorConstants.kElevatorFollower, MotorType.kBrushless);

    private DigitalInput TopLimit = new DigitalInput(ElevatorConstants.kElevatorTopSensor);
    private DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.kElevatorBottomSensor);

    private Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private static final boolean BRAKES_APPLIED = false;
    private static final boolean BRAKES_RELINQUISHED = true;

    public ElevatorSubsystem() {
        mainMotor.restoreFactoryDefaults();
        mainMotor.setSmartCurrentLimit(10);
        mainMotor.setIdleMode(IdleMode.kBrake);

        followerMotor.restoreFactoryDefaults();
        followerMotor.setSmartCurrentLimit(10);
        followerMotor.setIdleMode(IdleMode.kBrake);

        followerMotor.follow(mainMotor, true);
        mainMotor.setInverted(false);

        this.setBreaks(true);
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
        if (this.isBraking()) {
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

        if (isBraking == BRAKES_APPLIED) {
            this.setSpeed(0);
        }
    }

    public boolean isBraking() {
        return brakeSolenoid.get() == BRAKES_APPLIED;
    }


    private void updateStatus() {
        SmartDashboard.putBoolean("[Elevator]: Top Limit", getTopLimit());
        SmartDashboard.putBoolean("[Elevator]: Bottom Limit", getBottomLimit());
        Logger.recordOutput("elevatorBrake", this.isBraking());
        Logger.recordOutput("elevatorLower", this.getBottomLimit());
        Logger.recordOutput("elevatorUpper", this.getTopLimit());
        Logger.recordOutput("elevatorSet", this.mainMotor.get());
        Logger.recordOutput("elevatorAppOut", this.mainMotor.getAppliedOutput());
        Logger.recordOutput("elevatorFolAppOut", this.followerMotor.getAppliedOutput());
        Logger.recordOutput("elevatorOutCur", this.mainMotor.getOutputCurrent());
        Logger.recordOutput("elevatorFolOutCur", this.followerMotor.getOutputCurrent());
    }
}
