package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO impl;
    private IntakeInputsAutoLogged implInputs = new IntakeInputsAutoLogged();

    public IntakeSubsystem(IntakeIO implementation) {
        this.impl = implementation;
        this.initializeShuffleBoardWidgets();
    }

    @Override
    public void periodic() {
        this.impl.updateInputs(implInputs);
        Logger.processInputs("/IntakeInputs", implInputs);
    }

    public void setIntakeSpeed(double speed) {
        this.impl.setIntakeSpeed(speed);
    }

    public void setFeedSpeed(double speed) {
        this.impl.setFeedSpeed(speed);
    }

    public double getIntakeSpeed() {
        return this.impl.getIntakeSpeed();
    }

    public double getFeedSpeed() {
        return this.impl.getFeedSpeed();
    }

    // Conditions for sensor
    public boolean getIntakeSensor() {
        return this.impl.getIntakeSensor();
    }

    public boolean getFeedSensor() {
        return this.impl.getFeedSensor();
    }

    private void initializeShuffleBoardWidgets() {
        ShuffleboardLayout layout = Shuffleboard.getTab("Robot").getLayout("Intake");
        layout.addBoolean("Intake Sensor", () -> this.getIntakeSensor());
        layout.addBoolean("Feed Sensor", () -> this.getFeedSensor());
    }
}