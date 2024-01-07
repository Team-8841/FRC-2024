package frc.robot.sensors.imu;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.SimManager;

/**
 * Simulated IMU used during simulations.
 */
public class SimIMU extends IMU {
    private Supplier<ChassisSpeeds> speedSupplier;
    private double accumulatedYaw;
    private long lastTimestamp;

    public SimIMU() {
        SimManager.getInstance().registerIMU(this);
        lastTimestamp = Logger.getInstance().getTimestamp();
    }

    public void registerOrientationSuppliers(Supplier<ChassisSpeeds> speedSupplier) {
        this.speedSupplier = speedSupplier;
    }

    @Override
    public void periodic() {
        super.periodic();

        long lastTimestamp = this.lastTimestamp;
        long currentTimestamp = Logger.getInstance().getTimestamp();
        this.lastTimestamp = currentTimestamp;

        double dt = (currentTimestamp - lastTimestamp) / 1000000.0;

        this.accumulatedYaw += this.speedSupplier.get().omegaRadiansPerSecond * dt;
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(0, 0, this.accumulatedYaw);
    }

    @Override
    public Rotation2d getHeading() {
        return this.getYaw();
    }

    @Override
    public boolean isSensorPresent() {
        return true;
    }

    @Override
    public boolean isInitialized() {
        return true;
    }
}
