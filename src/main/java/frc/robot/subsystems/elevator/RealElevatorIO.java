package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.constants.elevator.ElevatorConstants;

public class RealElevatorIO implements ElevatorIO {
    private final CANSparkMax m_elevator = new CANSparkMax(ElevatorConstants.elevatorMain, MotorType.kBrushed);
    private final CANSparkMax m_elevatorFollower = new CANSparkMax(ElevatorConstants.elevatorFollower,
            MotorType.kBrushed);
    private final RelativeEncoder elevatorEncoder = this.m_elevator.getEncoder();

    public RealElevatorIO() {
        this.m_elevatorFollower.follow(this.m_elevator);
    }

    public void set(double dcycle) {
        this.m_elevator.set(dcycle);
    }

    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.setElevatorDCycle = this.m_elevator.get();
        inputs.actualElevatorRPS = this.elevatorEncoder.getVelocity() / 60;

        double elevatorOutCur = this.m_elevator.getOutputCurrent();
        Logger.recordOutput("Elevator/elevatorMotorOutCur", elevatorOutCur);
        double followerOutCur = this.m_elevatorFollower.getOutputCurrent();
        Logger.recordOutput("Elevator/elevatorFollowerMotorOutCur", followerOutCur);
        Logger.recordOutput("Elevator/totalCur", elevatorOutCur + followerOutCur);
    }
}
