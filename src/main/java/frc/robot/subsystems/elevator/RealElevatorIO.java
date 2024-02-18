package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.util.SafeCANSparkMax;
import frc.robot.constants.elevator.ElevatorConstants;

public class RealElevatorIO implements ElevatorIO {
    private final SafeCANSparkMax m_elevator = new SafeCANSparkMax(ElevatorConstants.elevatorMain, MotorType.kBrushed);
    private final SafeCANSparkMax m_elevatorFollower = new SafeCANSparkMax(ElevatorConstants.elevatorFollower,
            MotorType.kBrushed);
    private final RelativeEncoder elevatorEncoder = this.m_elevator.getEncoder();

    public RealElevatorIO() {
        this.m_elevatorFollower.follow(this.m_elevator);
    }

    public void set(double dcycle) {
        this.m_elevator.set(dcycle);
        this.feed();
    }

    public void feed() {
        this.m_elevator.feed();
        this.m_elevatorFollower.feed();
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
