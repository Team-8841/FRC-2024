package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.BrakeState;

public class ElevatorSubsystem extends SubsystemBase{

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private final ElevatorIO hwImpl;
    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private boolean lowerLimitSwitchPrev, upperLimitSwitchPrev;

    /*-------------------------------- Public Instance Variables --------------------------------*/

    public ElevatorSubsystem(ElevatorIO hwImpl) {
        this.hwImpl = hwImpl;
        this.hwImpl.setBrake(BrakeState.BRAKE_DISENGAGE);
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
        this.hwImpl.updateInputs(this.inputs);
        Logger.processInputs("Elevator", this.inputs);

        boolean lowerLimitSwitch = this.hwImpl.getLowerLimitSwitch();
        boolean upperLimitSwitch = this.hwImpl.getUpperLimitSwitch();

        if (this.hwImpl.isBraking()) {
            this.hwImpl.set(0);
        }

        if (lowerLimitSwitch || upperLimitSwitch) {
            //this.hwImpl.stopMotors();
            //this.hwImpl.setBrake(BrakeState.BRAKE_ENGAGE);
        }

        if (lowerLimitSwitch && !this.lowerLimitSwitchPrev) {
            System.out.println("Lower elevator limit switch engaged");
        }
        else if (!lowerLimitSwitch && this.lowerLimitSwitchPrev) {
            System.out.println("Lower elevator limit switch disengaged");
        }

        if (upperLimitSwitch && !this.upperLimitSwitchPrev) {
            System.out.println("Upper elevator limit switch engaged");
        }
        else if (!upperLimitSwitch && this.upperLimitSwitchPrev) {
            System.out.println("Upper elevator limit switch disengaged");
        }

        this.lowerLimitSwitchPrev = lowerLimitSwitch;
        this.upperLimitSwitchPrev = upperLimitSwitch;
    }


    /*-------------------------------- Custom Public Functions --------------------------------*/

    public void set(double dcycle) {
        if (!this.hwImpl.isBraking()) {
            this.hwImpl.set(dcycle);
        }
    }

    public void setBrake(BrakeState state) {
        this.hwImpl.setBrake(state);
    }

    /*-------------------------------- Custom Private Functions --------------------------------*/


    
}
