package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        double setShooterRPS, actualShooterRPS, setEndEffectorDeg, actualEndEffectorDeg, setRollerDCycle, rollerSpeedRPS;
    }

    public void setShooter(double targetRPS);

    public void setEndEffector(Rotation2d targetAngle);

    public void setRollerSpeed(double dcycle);

    public double getShooterRPS();

    public Rotation2d getEndEffector();

    public void updateInputs(ShooterInputsAutoLogged inputs);
}