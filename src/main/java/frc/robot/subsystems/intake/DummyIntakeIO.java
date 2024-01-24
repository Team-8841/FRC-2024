package frc.robot.subsystems.intake;

public class DummyIntakeIO implements IntakeIO {
    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    @Override
    public void updateInputs(IntakeInputsAutoLogged inputs) {
        this.inputs = inputs;
    }

    @Override
    public void setIntakeSpeed(double speed) {
        // do nothing
    }

    @Override
    public void setFeedSpeed(double speed) {
        // do nothing
    }

    @Override
    public double getIntakeSpeed() {
        return this.inputs.intakeSpeed;
    }

    @Override
    public double getFeedSpeed() {
        return this.inputs.feedSpeed;
    }

    @Override
    public boolean getIntakeSensor() {
        return this.inputs.intakeSensor;
    }

    @Override
    public boolean getFeedSensor() {
        return this.inputs.feedSensor;
    }
}
