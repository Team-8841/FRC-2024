package frc.robot.subsystems.intake;

public class DummyIntakeIO implements IntakeIO {
    private IntakeInputsAutoLogged inputs;

    @Override
    public void setIndexSpeed(double dcycle) {
        // Do nothing
    }

    @Override
    public void setIntakeSpeed(double dcycle) {
        // Do nothing
    }

    @Override
    public boolean getIndexSensor() {
        return inputs.indexSensor;
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        this.inputs = inputs;
    }
}
