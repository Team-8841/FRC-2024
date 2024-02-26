package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        double setShooterRPS, actualShooterRPS, setEndEffectorDeg, actualEndEffectorDeg, setRollerDCycle, rollerSpeedRPS;
        boolean isShooterUp, upLimitSwitch, downLimitSwitch;
    }

    public static enum ShooterAngle {
        UP, DOWN;
    }

    public default void periodic() {}

    public void setShooter(double targetRPS);

    public void setShooterAngle(ShooterAngle state);

    public void setEndEffector(double dcycle);

    public void setRollerSpeed(double dcycle);

    public double getShooterRPS();

    public Rotation2d getEndEffector();

    public boolean getUpLimitSwitch();

    public boolean getDownLimitSwitch();

    public void updateInputs(ShooterInputsAutoLogged inputs);
}