package frc.lib;

import edu.wpi.first.wpilibj.MotorSafety;

public class RunnableMotorSafety extends MotorSafety {
    private Runnable stopMotorRunnable;
    private String description;

    public RunnableMotorSafety(Runnable stopMotorRunnable, String description) {
        this.stopMotorRunnable = stopMotorRunnable;
        this.description = description;
    }

    @Override
    public String getDescription() {
        return this.description;
    }

    @Override
    public void stopMotor() {
        this.stopMotorRunnable.run();
    }
}
