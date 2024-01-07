package frc.robot.sensors.imu;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;

/**
 * IMU Container for the NavX2 IMU.
 */
public class NavX2 extends IMU {
    private AHRS ahrs;

    public NavX2() {
        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.ahrs.zeroYaw();
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger logger = Logger.getInstance();
        logger.recordOutput("/IMU/Displacement/x", this.ahrs.getDisplacementX());
        logger.recordOutput("/IMU/Displacement/y", this.ahrs.getDisplacementY());
        logger.recordOutput("/IMU/Displacement/z", this.ahrs.getDisplacementZ());

        logger.recordOutput("/IMU/Velocity/x", this.ahrs.getVelocityX());
        logger.recordOutput("/IMU/Velocity/y", this.ahrs.getVelocityY());
        logger.recordOutput("/IMU/Velocity/z", this.ahrs.getVelocityZ());

        logger.recordOutput("/IMU/Calibrating", this.ahrs.isCalibrating());
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(new Quaternion(this.ahrs.getQuaternionW(), this.ahrs.getQuaternionX(),
                this.ahrs.getQuaternionY(), this.ahrs.getQuaternionZ()));
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(this.ahrs.getFusedHeading());
    }

    @Override
    public boolean isInitialized() {
        return this.ahrs.isCalibrating();
    }

    @Override
    public boolean isSensorPresent() {
        return this.ahrs.isConnected();
    }
}
