package frc.robot.subsystems.elevator;

public class DummyElevatorIO implements ElevatorIO {
    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public void set(double dcycle) { 
        // Do nothing
    }
    
    public void setBrake(BrakeState state) {
        // Do nothing
    }
    
    public void stopMotors() {
        // Do nothing
    }

    public boolean isBraking() {
        return this.inputs == null ? true : this.inputs.isBraking;
    }

    public boolean getUpperLimitSwitch() {
        return this.inputs.upperLimitSwitch;
    }

    public boolean getLowerLimitSwitch() {
        return this.inputs.lowerLimitSwitch;
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        if (inputs != null) {
            this.inputs = inputs;
        }
    }
}
