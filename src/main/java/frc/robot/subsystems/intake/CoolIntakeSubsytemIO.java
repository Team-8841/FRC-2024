package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Intake.IntakeConstants;

public class CoolIntakeSubsytemIO implements IntakeSubsystemIO {
  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushed);
  private final CANSparkMax feedMotor =
      new CANSparkMax(IntakeConstants.feedMotor, MotorType.kBrushed);

    // Sensor
    private DigitalInput intakeSenor = new DigitalInput(IntakeConstants.intakeSensor);
    private DigitalInput feedSenor = new DigitalInput(IntakeConstants.feedSensor);

  public CoolIntakeSubsytemIO() {
  }

  @Override
  public void setIntakeSpeed(double speed) {
    this.intakeMotor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return this.intakeMotor.get();
  }

  @Override
  public void setFeedSpeed(double speed) {
    this.feedMotor.set(speed);
  }

  @Override
  public double getFeedSpeed() {
    return this.feedMotor.get();
  }

  // Conditions for sensor
  @Override
  public boolean getIntakeSensor() {
    return this.intakeSenor.get();
  }

  @Override
  public boolean getFeedSensor() {
    return this.feedSenor.get();
  }
}