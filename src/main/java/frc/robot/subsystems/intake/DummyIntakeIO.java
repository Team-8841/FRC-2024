package frc.robot.subsystems.intake;

public class DummyIntakeIO implements IntakeIO {
    private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

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
        return this.inputs.indexSensor;
    }

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        if (inputs != null) {
            this.inputs = inputs;
        }
    }
}
