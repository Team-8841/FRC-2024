package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.elevator.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    /*-------------------------------- Private Instance Variables --------------------------------*/

    private CANSparkMax m_elevator = new CANSparkMax(ElevatorConstants.elevatorMain, MotorType.kBrushed);
    private CANSparkMax m_elevatorFollower = new CANSparkMax(ElevatorConstants.elevatorFollower, MotorType.kBrushed);


    /*-------------------------------- Public Instance Variables --------------------------------*/


    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void periodic() {
    }


    /*-------------------------------- Custom Public Functions --------------------------------*/


    /*-------------------------------- Custom Private Functions --------------------------------*/


    
}
