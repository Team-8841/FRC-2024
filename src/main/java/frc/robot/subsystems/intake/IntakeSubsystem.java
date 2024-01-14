package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeSubsystemIO impl;
    private IntakeInputsAutoLogged implInputs = new IntakeInputsAutoLogged();
    
    public IntakeSubsystem(IntakeSubsystemIO implementation) {
        this.impl = implementation;

        this.initializeShuffleBoardWidgets();
    }

    @Override
    public void periodic() {
        this.impl.updateInputs(implInputs);
    }

    public void setIntakeSpeed(double speed) {
        this.impl.setIntakeSpeed(speed);
    }

    public void setFeedSpeed(double speed) {
        this.impl.setFeedSpeed(speed);
    }
    // Conditions for sensor
    public boolean getIntakeSensor() {
        return this.impl.getIntakeSensor();
    }

    public boolean getFeedSensor() {
        return this.impl.getFeedSensor();
    }

    private void initializeShuffleBoardWidgets() {
      ShuffleboardLayout intakeLayout = Shuffleboard.getTab("Robot").getLayout("Intake");
      intakeLayout.addBoolean("Intake Sensor", () -> this.getIntakeSensor());
      intakeLayout.addBoolean("Feed Sensor", () -> this.getFeedSensor());
    }
}