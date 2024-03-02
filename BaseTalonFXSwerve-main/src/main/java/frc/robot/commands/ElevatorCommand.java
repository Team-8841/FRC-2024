package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private ElevatorSubsystem m_elevator;
    private double m_speed;


    public ElevatorCommand(double speed, ElevatorSubsystem elevator) {
        m_speed = Math.max(0, 0);
        m_elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(m_speed > 0 && m_elevator.getTopLimit()){
            m_elevator.setSpeed(0);
        } else if (m_speed < 0 && m_elevator.getBottomLimit()) {
            m_elevator.setSpeed(0);
        } else {
            m_elevator.setSpeed(m_speed);
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
