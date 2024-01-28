package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class IntakeInOut extends Command {
  private IntakeSubsystem intakeSubsystem;

  private int m_intakeInput;

  public IntakeInOut(int intakeInput, IntakeSubsystem intake) {
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
        intakeSubsystem.setIntakeState(IntakeState.OUTAKE);
        break;

      case 0:
        intakeSubsystem.setIntakeState(IntakeState.OFF);
        break;
      
      case 1:

        if(intakeSubsystem.getIndexSensor()) {
          intakeSubsystem.setIntakeState(IntakeState.INTAKEANDHOLD);
        } else {
          intakeSubsystem.setIntakeState(IntakeState.INTAKE);
        }
        break;
      default:
        intakeSubsystem.setIntakeState(IntakeState.OFF);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeState(IntakeState.OFF);
  }
}