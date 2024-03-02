package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.IntakeSubsytem.IntakeState;

public class SetIntakeStateCommand extends Command {

    private IntakeSubsytem m_intake;

    private IntakeState m_newState, m_oldState;

    public SetIntakeStateCommand(IntakeState state, IntakeSubsytem intake) {
        m_intake = intake;
        m_newState = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_oldState = m_intake.getState();
    }

    @Override
    public void execute() {
        m_intake.setState(m_newState);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_newState != m_oldState;
    }
    
}
