package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        double setElevatorDCycle, actualElevatorRPS;
        boolean lowerLimitSwitch, upperLimitSwitch, isBraking;
    }

    public static enum BrakeState { BRAKE_ENGAGE, BRAKE_DISENGAGE; }

    public default void periodic() {}

    public void set(double dcycle);
    
    public void setBrake(BrakeState state);
    
    public void stopMotors();

    public boolean isBraking();

    public boolean getUpperLimitSwitch();

    public boolean getLowerLimitSwitch();

    public void updateInputs(ElevatorInputsAutoLogged inputs);
}
