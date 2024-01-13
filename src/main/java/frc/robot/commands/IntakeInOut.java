package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsytem;

public class IntakeInOut extends Command {
  private IntakeSubsytem intakeSubsystem;

  private boolean m_in, m_out, m_intakeSensor, m_feedSensor;

  private IntakeInOut(
      boolean in, boolean out, boolean intakeSensor, boolean feedSensor, IntakeSubsytem intake) {
    intakeSubsystem = intake;
    m_in = in;
    m_out = out;
    m_feedSensor = feedSensor;
    m_intakeSensor = intakeSensor;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_out) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed);
      intakeSubsystem.setIntakeSpeed(IntakeConstants.kFeederOutSpeed);
    } else if (m_intakeSensor && m_in) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed);
      if (!m_feedSensor) {
        intakeSubsystem.setFeedSpeed(IntakeConstants.kFeederInSpeed);
      } else {
        intakeSubsystem.setFeedSpeed(0);
      }
    } else if (!m_intakeSensor && m_in) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeInSpeed);
      if (!m_feedSensor) {
        intakeSubsystem.setFeedSpeed(IntakeConstants.kFeederInSpeed);
      } else {
        intakeSubsystem.setFeedSpeed(0);
      }
    } else {
      intakeSubsystem.setIntakeSpeed(0);
      intakeSubsystem.setFeedSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setFeedSpeed(0);
    intakeSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
