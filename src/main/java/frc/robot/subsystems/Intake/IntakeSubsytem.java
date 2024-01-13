package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Intake.IntakeConstants;

public class IntakeSubsytem extends SubsystemBase {
  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushed);
  private final CANSparkMax feedMotor =
      new CANSparkMax(IntakeConstants.feedMotor, MotorType.kBrushed);

  // Sensor
  private DigitalInput intakeSenor = new DigitalInput(IntakeConstants.intakeSensor);
  private DigitalInput feedSenor = new DigitalInput(IntakeConstants.feedSensor);

  public IntakeSubsytem() {}

  @Override
  public void periodic() {
    updateStatus();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setFeedSpeed(double speed) {
    feedMotor.set(speed);
  }

  // Conditions for sensor
  public boolean getIntakeSensor() {
    return intakeSenor.get();
  }

  public boolean getFeedSensor() {
    return feedSenor.get();
  }

  private void updateStatus() {
    SmartDashboard.putBoolean("[Intake] Intake Sensor", getIntakeSensor());
    SmartDashboard.putBoolean("[Intake] Feed Sensor", getFeedSensor());
  }
}
