package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        double setElevatorDCycle, actualElevatorRPS;
    }

    public void set(double dcycle);

    public void feed();

    public void updateInputs(ElevatorInputsAutoLogged inputs);
}
