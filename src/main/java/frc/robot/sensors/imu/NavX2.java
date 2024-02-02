package frc.robot.sensors.imu;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
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

        Logger.recordOutput("IMU/displacementX", this.ahrs.getDisplacementX());
        Logger.recordOutput("IMU/displacementY", this.ahrs.getDisplacementY());
        Logger.recordOutput("IMU/displacementZ", this.ahrs.getDisplacementZ());

        Logger.recordOutput("IMU/velocityX", this.ahrs.getVelocityX());
        Logger.recordOutput("IMU/velocityY", this.ahrs.getVelocityY());
        Logger.recordOutput("IMU/velocityZ", this.ahrs.getVelocityZ());

        Logger.recordOutput("IMU/rawGyroX", this.ahrs.getRawGyroX());
        Logger.recordOutput("IMU/rawGyroY", this.ahrs.getRawGyroY());
        Logger.recordOutput("IMU/rawGyroZ", this.ahrs.getRawGyroZ());

        Logger.recordOutput("IMU/rawAccelX", this.ahrs.getRawAccelX());
        Logger.recordOutput("IMU/rawAccelY", this.ahrs.getRawAccelY());
        Logger.recordOutput("IMU/rawAccelZ", this.ahrs.getRawAccelZ());

        Logger.recordOutput("IMU/rawMagX", this.ahrs.getRawMagX());
        Logger.recordOutput("IMU/rawMagY", this.ahrs.getRawMagY());
        Logger.recordOutput("IMU/rawMagZ", this.ahrs.getRawMagZ());

        Logger.recordOutput("IMU/worldLinearAccelX", this.ahrs.getWorldLinearAccelX());
        Logger.recordOutput("IMU/worldLinearAccelY", this.ahrs.getWorldLinearAccelY());
        Logger.recordOutput("IMU/worldLinearAccelZ", this.ahrs.getWorldLinearAccelZ());

        Logger.recordOutput("IMU/calibrating", this.ahrs.isCalibrating());
        Logger.recordOutput("IMU/rate", this.ahrs.getRate());
        Logger.recordOutput("IMU/totalAngle", this.ahrs.getAngle());
        Logger.recordOutput("IMU/isMoving", this.ahrs.isMoving());
        Logger.recordOutput("IMU/isRotating", this.ahrs.isRotating());
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(new Quaternion(this.ahrs.getQuaternionW(), this.ahrs.getQuaternionX(),
                this.ahrs.getQuaternionY(), this.ahrs.getQuaternionZ()));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.ahrs.getAngle());
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(360-this.ahrs.getFusedHeading());
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
