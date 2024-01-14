package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double intakeSpeed, feedSpeed;
    public boolean intakeSensor, feedSensor;
  }

  public default void updateInputs(IntakeInputsAutoLogged inputs) {
    inputs.intakeSpeed = this.getIntakeSpeed();
    inputs.feedSpeed = this.getIntakeSpeed();
    inputs.intakeSensor = this.getIntakeSensor();
    inputs.feedSensor = this.getFeedSensor();
  }

  public void setIntakeSpeed(double speed);
  public void setFeedSpeed(double speed);
  public double getIntakeSpeed();
  public double getFeedSpeed();
  // Conditions for sensor
  public boolean getIntakeSensor();
  public boolean getFeedSensor();
}