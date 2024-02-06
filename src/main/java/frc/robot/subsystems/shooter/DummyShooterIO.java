package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class DummyShooterIO implements ShooterIO {
    private ShooterInputsAutoLogged inputs;

    public void setShooter(double targetRPS) {
        // Do nothing
    }

    public void setEndEffector(Rotation2d targetAngle) {
        // Do nothing
    }

    public void setRollerSpeed(double dcycle) {
        // Do nothing
    }

    public double getShooterRPS() {
        return inputs.actualShooterRPS;
    }

    public Rotation2d getEndEffector() {
        return Rotation2d.fromDegrees(inputs.actualEndEffectorDeg);
    }

    public void updateInputs(ShooterInputsAutoLogged inputs) {
        this.inputs = inputs;
    }
}
