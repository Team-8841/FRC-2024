package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private ElevatorSubsystem m_elevator;
    private DoubleSupplier joyStickSupplier;

    public ElevatorCommand(DoubleSupplier speedSupplier, ElevatorSubsystem elevator) {
        this.joyStickSupplier = speedSupplier;
        m_elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double m_speed = 0;
        double stick = this.joyStickSupplier.getAsDouble();

        if (stick < -0.5) {
            m_speed = -0.9;
        }
        else if (stick > 0.5) {
            m_speed = 0.9;
        }

        //m_elevator.setSpeed(m_speed);
        if(m_speed < 0 && m_elevator.getTopLimit()){
           m_elevator.setSpeed(0);
        } else if (m_speed > 0 && m_elevator.getBottomLimit()) {
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
