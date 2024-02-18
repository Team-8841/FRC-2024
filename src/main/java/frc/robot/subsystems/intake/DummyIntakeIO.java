package frc.robot.subsystems.intake;

public class DummyIntakeIO implements IntakeIO {
    private IntakeInputsAutoLogged inputs;

    public void setIndexSpeed(double dcycle) {
        // Do nothing
    }

    public void setIntakeSpeed(double dcycle) {
        // Do nothing
    }

    public void feed() {
        // Do nothing
    }

    public boolean getIntakeSensor() {
        return inputs.intakeSensor;
    }

    public boolean getIndexSensor() {
        return inputs.indexSensor;
    }

    public void updateInputs(IntakeInputsAutoLogged inputs) {
        this.inputs = inputs;
    }
}
