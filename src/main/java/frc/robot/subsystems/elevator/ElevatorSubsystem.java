package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.elevator.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private final ElevatorIO hwImpl;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public ElevatorSubsystem(ElevatorIO hwImpl) {
        this.hwImpl = hwImpl;
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        this.hwImpl.updateInputs(this.inputs);
        Logger.processInputs("/Elevator", this.inputs);
    }


    /*-------------------------------- Custom Public Functions --------------------------------*/

    public void set(double dcycle) {
        this.hwImpl.set(dcycle);
    }

    /*-------------------------------- Custom Private Functions --------------------------------*/


    
}
