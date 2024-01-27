package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsytem;
import frc.robot.subsystems.Intake.IntakeSubsytem.IntakeState;

public class IntakeInOut extends Command {
  private IntakeSubsytem intakeSubsystem;

  private int m_intakeInput;

  public IntakeInOut(int intakeInput, IntakeSubsytem intake) {
    intakeSubsystem = intake;
    m_intakeInput = intakeInput;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (m_intakeInput) {

      case -1:
        intakeSubsystem.intake(IntakeState.OUTAKE);
        break;

      case 0:
        intakeSubsystem.intake(IntakeState.OFF);
        break;
      
      case 1:

        if(intakeSubsystem.getIndexSensor()) {
          intakeSubsystem.intake(IntakeState.INTAKEANDHOLD);
        } else {
          intakeSubsystem.intake(IntakeState.INTAKE);
        }
        break;
      default:
        intakeSubsystem.intake(IntakeState.OFF);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intake(IntakeState.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
