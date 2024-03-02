package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.IntakeSubsytem.IntakeState;

public class IntakeCommand extends Command {
    private IntakeSubsytem m_intake;
    private boolean m_in, m_out, m_indexSensor;


    public IntakeCommand(boolean in, boolean out, boolean indexSensor, IntakeSubsytem intake) {
        m_in = in;
        m_out = out;
        m_indexSensor = indexSensor;
        m_intake = intake;
        

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(m_in && !m_out) {
            if(m_indexSensor) {
                m_intake.setState(IntakeState.INTAKEANDHOLD);
            } else {
                m_intake.setState(IntakeState.INTAKE);
            }
        } else if(m_in && m_out) {
            m_intake.setState(IntakeState.OUTAKE);
        } else if (!m_in && m_out) {
            m_intake.setState(IntakeState.OUTAKE);
        }else {
            m_intake.setState(IntakeState.OFF);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
