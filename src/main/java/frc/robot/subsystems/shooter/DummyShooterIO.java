package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class DummyShooterIO implements ShooterIO {
    private ShooterInputsAutoLogged inputs;

    @Override
    public void setShooter(double targetRPS) {
        // Do nothing
    }

    @Override
    public void setShooterAngle(ShooterAngle state) {
        // Do nothing
    }

    @Override
    public void setEndEffector(double dcycle) {
        // Do nothing
    }

    @Override
    public void setRollerSpeed(double dcycle) {
        // Do nothing
    }

    @Override
    public double getShooterRPS() {
        return inputs.actualShooterRPS;
    }

    @Override
    public Rotation2d getEndEffector() {
        return Rotation2d.fromDegrees(inputs.actualEndEffectorDeg);
    }

    @Override
    public boolean getUpLimitSwitch() {
        return inputs.upLimitSwitch;
    }

    @Override
    public boolean getDownLimitSwitch() {
        return inputs.downLimitSwitch;
    }

    @Override
    public void updateInputs(ShooterInputsAutoLogged inputs) {
        this.inputs = inputs;
    }
}
