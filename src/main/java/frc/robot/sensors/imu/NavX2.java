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

        Logger.recordOutput("NavX2/displacementX", this.ahrs.getDisplacementX());
        Logger.recordOutput("NavX2/displacementY", this.ahrs.getDisplacementY());
        Logger.recordOutput("NavX2/displacementZ", this.ahrs.getDisplacementZ());

        Logger.recordOutput("NavX2/velocityX", this.ahrs.getVelocityX());
        Logger.recordOutput("NavX2/velocityY", this.ahrs.getVelocityY());
        Logger.recordOutput("NavX2/velocityZ", this.ahrs.getVelocityZ());

        Logger.recordOutput("NavX2/rawGyroX", this.ahrs.getRawGyroX());
        Logger.recordOutput("NavX2/rawGyroY", this.ahrs.getRawGyroY());
        Logger.recordOutput("NavX2/rawGyroZ", this.ahrs.getRawGyroZ());

        Logger.recordOutput("NavX2/rawAccelX", this.ahrs.getRawAccelX());
        Logger.recordOutput("NavX2/rawAccelY", this.ahrs.getRawAccelY());
        Logger.recordOutput("NavX2/rawAccelZ", this.ahrs.getRawAccelZ());

        Logger.recordOutput("NavX2/rawMagX", this.ahrs.getRawMagX());
        Logger.recordOutput("NavX2/rawMagY", this.ahrs.getRawMagY());
        Logger.recordOutput("NavX2/rawMagZ", this.ahrs.getRawMagZ());

        Logger.recordOutput("NavX2/worldLinearAccelX", this.ahrs.getWorldLinearAccelX());
        Logger.recordOutput("NavX2/worldLinearAccelY", this.ahrs.getWorldLinearAccelY());
        Logger.recordOutput("NavX2/worldLinearAccelZ", this.ahrs.getWorldLinearAccelZ());

        Logger.recordOutput("NavX2/calibrating", this.ahrs.isCalibrating());
        Logger.recordOutput("NavX2/rate", this.ahrs.getRate());
        Logger.recordOutput("NavX2/totalAngle", this.ahrs.getAngle());
        Logger.recordOutput("NavX2/isMoving", this.ahrs.isMoving());
        Logger.recordOutput("NavX2/isRotating", this.ahrs.isRotating());
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(new Quaternion(this.ahrs.getQuaternionW(), this.ahrs.getQuaternionX(),
                this.ahrs.getQuaternionY(), this.ahrs.getQuaternionZ()));
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

    @Override
    public void reset() {
        this.ahrs.zeroYaw();
    }
}
