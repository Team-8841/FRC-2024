package frc.lib;

import edu.wpi.first.wpilibj.MotorSafety;

//public class RunnableMotorSafety extends MotorSafety {
public class RunnableMotorSafety {
    private Runnable stopMotorRunnable;
    private String description;

    public RunnableMotorSafety(Runnable stopMotorRunnable, String description) {
        this.stopMotorRunnable = stopMotorRunnable;
        this.description = description;
    }

    public void feed() {}

    // @Override
    public String getDescription() {
        return this.description;
    }

    // @Override
    public void stopMotor() {
        this.stopMotorRunnable.run();
    }
}
