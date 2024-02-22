package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        double setIndexDCycle, actualIndexRPS, setIntakeDCycle, actualIntakeRPS;
        boolean indexSensor;
    }

    public void setIntakeSpeed(double dcycle);
    public void setIndexSpeed(double dcycle);
    public boolean getIndexSensor();
    public void updateInputs(IntakeInputsAutoLogged inputs);
}
